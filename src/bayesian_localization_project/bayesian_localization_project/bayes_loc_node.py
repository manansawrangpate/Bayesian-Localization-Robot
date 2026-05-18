#!/usr/bin/env python3
"""
Bayesian localisation node for the Galbraith Memorial Mail Robot.

Phase 1 – Exploration: traverse the full closed loop once (all 11 office patches
          visited) so the belief converges before any delivery is attempted.
Phase 2 – Delivery: deliver mail only when the belief has been stably high at the
          same office state for MIN_CONFIDENT_FRAMES consecutive frames.
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray, String, UInt32


# ── Map ───────────────────────────────────────────────────────────────────────
# States 0-10: offices 2-12 in track order (index 0 = office 2, index 10 = office 12)
# State 11: traversal (any inter-office segment)
COLOUR_MAP = [
    'yellow',    # 0: office  2  (-1.1, -1.6) bottom
    'green',     # 1: office  3  ( 1.0, -1.6) bottom-right
    'blue',      # 2: office  4  ( 2.05,-0.95) right-lower
    'orange',    # 3: office  5  ( 2.05,-0.15) right-middle
    'orange',    # 4: office  6  ( 2.05, 0.70) right-upper
    'green',     # 5: office  7  ( 1.2,  1.6)  top-right
    'blue',      # 6: office  8  (-0.15, 1.6)  top-middle
    'orange',    # 7: office  9  (-0.95, 1.6)  top-left
    'yellow',    # 8: office 10  (-1.8,  0.70) left-upper
    'green',     # 9: office 11  (-1.8, -0.20) left-middle
    'blue',      # 10: office 12  (-1.8, -1.0)  left-lower
    'traversal', # 11: between offices
]
NUM_STATES        = len(COLOUR_MAP)       # 12
NUM_OFFICE_STATES = NUM_STATES - 1        # 11

STATE_LABELS = [f'office{i + 2}' for i in range(NUM_OFFICE_STATES)] + ['traversal']

_COL_IDX = {'blue': 0, 'green': 1, 'yellow': 2, 'orange': 3, 'traversal': 4}

# ── Bayesian models ───────────────────────────────────────────────────────────
_TRANS_WEIGHTS = {
    +1: (0.05, 0.10, 0.85),
     0: (0.05, 0.90, 0.05),
    -1: (0.85, 0.10, 0.05),
}

# p(z | true colour)  — columns: [blue, green, yellow, orange, traversal]
# Each column sums to 1.0.
_MEAS = {
    'blue':    [0.65, 0.15, 0.05, 0.10, 0.05],
    'green':   [0.15, 0.65, 0.05, 0.10, 0.05],
    'yellow':  [0.05, 0.05, 0.70, 0.15, 0.05],
    'orange':  [0.10, 0.10, 0.15, 0.60, 0.05],
    'nothing': [0.05, 0.05, 0.05, 0.05, 0.80],
}

# ── Colour classifier ─────────────────────────────────────────────────────────
def classify_colour(rgb: np.ndarray) -> str:
    r, g, b = float(rgb[0]), float(rgb[1]), float(rgb[2])
    if r > 200 and g > 200 and abs(r - g) < 25 and r > b + 3:
        return 'yellow'
    if r > 160 and b < 80 and r > g * 1.12:
        return 'orange'
    if g > 130 and g > r * 1.10 and g > b * 1.10:
        return 'green'
    if b > 130 and b > r * 1.20 and b > g * 1.20:
        return 'blue'
    return 'nothing'


# ── Controller / filter parameters ───────────────────────────────────────────
IMG_WIDTH   = 640
LINE_CENTRE = IMG_WIDTH // 2
LINEAR_VEL  = 0.08

KP          = 0.002

# Belief floor: no state ever drops to 0.  Prevents the filter from locking into
# a wrong hypothesis and also ensures belief never reaches 1.0 (always uncertain).
# With 12 states and floor=0.01, the maximum any one state can reach is ~89%.
BELIEF_FLOOR = 0.01

# Delivery gate: MAP estimate must be stable for this many consecutive frames.
# No probability threshold — whatever state is most likely is trusted once stable.
MIN_CONFIDENT_FRAMES = 8      # consecutive frames (~0.8 s at 10 Hz) at same state

# Exploration: count 'nothing → colour' transitions (one per office patch entered).
# After EXPLORATION_PATCHES entries the robot has been around the full loop once.
# Fallback: also complete exploration after MAX_EXPLORE_SECONDS regardless of count
# (handles rare missed patches due to camera angle on corners).
EXPLORATION_PATCHES  = NUM_OFFICE_STATES   # 11
MAX_EXPLORE_SECONDS  = 240                 # ~1.5 laps at 0.08 m/s safety net


class BayesLocNode(Node):

    def __init__(self):
        super().__init__('bayes_loc_node')

        self.belief   = np.ones(NUM_STATES) / NUM_STATES
        self.cur_rgb  = None
        self.cur_line = LINE_CENTRE
        self.last_u   = +1
        self._lock    = threading.Lock()

        self.target_offices = [4, 6, 8]

        self.create_subscription(
            Float64MultiArray, '/mean_img_rgb', self._rgb_cb, 10)
        self.create_subscription(
            UInt32, '/line_idx', self._line_cb, 10)

        self.cmd_pub    = self.create_publisher(TwistStamped,      '/cmd_vel',      10)
        self.belief_pub = self.create_publisher(Float64MultiArray, '/belief',       10)
        self.status_pub = self.create_publisher(String,            '/bayes_status', 10)

        self._running = True
        self._thread  = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'BayesLoc started | targets: offices {self.target_offices} | '
            f'exploration: {EXPLORATION_PATCHES} patches required')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _rgb_cb(self, msg: Float64MultiArray):
        with self._lock:
            self.cur_rgb = np.array(msg.data)

    def _line_cb(self, msg: UInt32):
        with self._lock:
            self.cur_line = int(msg.data)

    # ── Bayesian filter ───────────────────────────────────────────────────────
    def _predict(self, u: int):
        p_left, p_stay, p_right = _TRANS_WEIGHTS[u]
        new_belief = np.zeros(NUM_STATES)

        # Office states (0-10) form their own circular chain mod 11.
        # Traversal (11) is NOT in this circle — it sits beside it.
        # This reflects the physical track: offices are always in the same
        # cyclic order, and any office can be preceded/followed by traversal.
        for i in range(NUM_OFFICE_STATES):
            new_belief[(i - 1) % NUM_OFFICE_STATES] += self.belief[i] * p_left
            new_belief[i]                            += self.belief[i] * p_stay
            new_belief[(i + 1) % NUM_OFFICE_STATES] += self.belief[i] * p_right

        # Traversal state: staying keeps us in traversal; leaving spreads
        # uniformly across all 11 offices because we don't know which corridor
        # segment we're in.  The next measurement update resolves the office.
        t = self.belief[NUM_OFFICE_STATES]
        new_belief[NUM_OFFICE_STATES] += t * p_stay
        spread = t * (p_left + p_right) / NUM_OFFICE_STATES
        for j in range(NUM_OFFICE_STATES):
            new_belief[j] += spread

        self.belief = new_belief

    def _update(self, observed: str):
        likelihoods = np.array(_MEAS[observed])
        for i in range(NUM_STATES):
            col_idx = _COL_IDX[COLOUR_MAP[i]]
            self.belief[i] *= likelihoods[col_idx]

        # Floor: no state can be fully eliminated.  Repeated updates on the same
        # patch would otherwise drive all non-matching states to exactly 0 and
        # lock the filter into the first hypothesis it found.
        self.belief = np.maximum(self.belief, BELIEF_FLOOR)

        total = self.belief.sum()
        self.belief /= total

    # ── Motion helpers ────────────────────────────────────────────────────────
    def _publish_twist(self, linear: float, angular: float):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.twist.linear.x  = linear
        t.twist.angular.z = angular
        self.cmd_pub.publish(t)

    def _stop(self):
        self.cmd_pub.publish(TwistStamped())

    def _rotate(self, angle_rad: float, speed: float = 0.3):
        duration = abs(angle_rad) / speed
        sign     = math.copysign(1.0, angle_rad)
        end      = time.time() + duration
        while time.time() < end and self._running:
            self._publish_twist(0.0, sign * speed)
            time.sleep(0.05)
        self._stop()

    def _deliver_mail(self, office: int):
        self.get_logger().info(f'*** DELIVERING to office {office} — line-following 4s to centre ***')
        end = time.time() + 4.0
        while time.time() < end and self._running:
            with self._lock:
                line = self.cur_line
            angular = KP * float(LINE_CENTRE - line)
            self._publish_twist(LINEAR_VEL, angular)
            time.sleep(0.05)
        self._stop()
        time.sleep(2.0)
        self._rotate(math.pi / 2)
        time.sleep(1.0)
        self._rotate(-math.pi / 2)
        end = time.time() + 3.0
        while time.time() < end and self._running:
            self._publish_twist(LINEAR_VEL, 0.0)
            time.sleep(0.05)
        self._stop()

    # ── Main control loop ─────────────────────────────────────────────────────
    def _control_loop(self):
        rate = 10
        dt   = 1.0 / rate

        prev_observed        = 'nothing'
        patches_visited      = 0
        exploration_complete = False
        explore_start        = time.time()

        confident_frames    = 0
        prev_best_idx       = -1

        while self._running and rclpy.ok():
            with self._lock:
                rgb  = self.cur_rgb.copy() if self.cur_rgb is not None else None
                line = self.cur_line

            if rgb is None:
                time.sleep(dt)
                continue

            observed = classify_colour(rgb)

            # ── Exploration lap counter ───────────────────────────────────
            # Every nothing→colour edge is one new office patch entered.
            if prev_observed == 'nothing' and observed != 'nothing':
                patches_visited += 1
                self.get_logger().info(
                    f'Patch entry {patches_visited}/{EXPLORATION_PATCHES}')

            elapsed = time.time() - explore_start
            if not exploration_complete and (
                    patches_visited >= EXPLORATION_PATCHES or
                    elapsed >= MAX_EXPLORE_SECONDS):
                exploration_complete = True
                reason = (f'{patches_visited} patches'
                          if patches_visited >= EXPLORATION_PATCHES
                          else f'timeout {elapsed:.0f}s')
                self.get_logger().info(
                    f'=== Exploration complete ({reason}) — delivery mode active ===')

            # ── Bayesian filter ───────────────────────────────────────────
            just_entered_nothing = (prev_observed != 'nothing' and observed == 'nothing')
            just_exited_nothing  = (prev_observed == 'nothing' and observed != 'nothing')

            if just_entered_nothing:
                # Entering a corridor: predict ONCE (advance from last office),
                # then update ONCE with 'nothing' (traversal gets its signal here).
                # Belief is then frozen for the duration of the corridor.
                self._predict(self.last_u)
                self._update('nothing')
            elif just_exited_nothing:
                # Exiting a corridor: update only.
                # Predict already fired on corridor entry — firing again would
                # over-advance the belief past the correct next office.
                self._update(observed)
            elif observed != 'nothing' and observed != prev_observed:
                # Direct colour-to-colour transition (offices with no grey gap)
                self._predict(self.last_u)
                self._update(observed)
            elif observed != 'nothing':
                # Sustained colour: keep reinforcing the current office estimate
                self._update(observed)
            # Sustained 'nothing' (mid-corridor): freeze — no predict, no update.
            # The belief set on corridor entry is preserved intact until the
            # next office colour appears.

            prev_observed = observed

            best_idx  = int(np.argmax(self.belief))
            best_prob = float(self.belief[best_idx])

            # ── Sustained-confidence counter ──────────────────────────────
            if best_idx == prev_best_idx:
                confident_frames += 1
            else:
                confident_frames = 0
            prev_best_idx = best_idx

            # ── Logging ───────────────────────────────────────────────────
            if exploration_complete:
                mode = 'DELIVER'
            else:
                mode = f'EXPLORE {patches_visited:2d}/{EXPLORATION_PATCHES}'
            self.get_logger().info(
                f'[{mode}] obs={observed:9s}  '
                f'est={STATE_LABELS[best_idx]:12s}  p={best_prob:.2f}  '
                f'conf={confident_frames:3d}  '
                f'[{" ".join(f"{v:.2f}" for v in self.belief)}]')

            # Publish belief + status for external visualiser
            bm = Float64MultiArray()
            bm.data = [float(v) for v in self.belief]
            self.belief_pub.publish(bm)

            sm = String()
            sm.data = (f'obs={observed}  est={STATE_LABELS[best_idx]}  '
                       f'p={best_prob:.2f}  conf={confident_frames}  '
                       f'mode={mode}  targets={self.target_offices}')
            self.status_pub.publish(sm)

            # ── Delivery gate ─────────────────────────────────────────────
            if (exploration_complete
                    and observed != 'nothing'
                    and best_idx < NUM_OFFICE_STATES
                    and confident_frames >= MIN_CONFIDENT_FRAMES):
                best_office = best_idx + 2
                if best_office in self.target_offices:
                    self._deliver_mail(best_office)
                    self.target_offices.remove(best_office)
                    confident_frames = 0
                    prev_observed    = 'nothing'

                    if not self.target_offices:
                        self.get_logger().info('All mail delivered! Stopping.')
                        self._stop()
                        return

            # ── Line following ────────────────────────────────────────────
            error      = LINE_CENTRE - line
            angular    = KP * float(error)
            self.last_u = +1 if LINEAR_VEL > 0 else 0
            self._publish_twist(LINEAR_VEL, angular)

            time.sleep(dt)

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BayesLocNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
