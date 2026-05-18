#!/usr/bin/env python3
"""
Live bar-chart visualiser for the Bayesian belief state.

Subscribes to /belief (Float64MultiArray, 12 values) and /bayes_status (String)
published by bayes_loc_node, and draws an updating matplotlib bar chart.

Run in a separate terminal after launching the main nodes:
    ros2 run bayesian_localization_project belief_visualizer
"""
import threading

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


# Must match bayes_loc_node.py
NUM_STATES = 12
STATE_LABELS = [f'office{i + 2}' for i in range(11)] + ['traversal']

# Bar fill colours — reflect each state's physical patch colour
_BASE_COLOURS = [
    '#d4d400',  # office 2  yellow
    '#22bb22',  # office 3  green
    '#4444ff',  # office 4  blue
    '#ff8800',  # office 5  orange
    '#ff8800',  # office 6  orange
    '#22bb22',  # office 7  green
    '#4444ff',  # office 8  blue
    '#ff8800',  # office 9  orange
    '#d4d400',  # office 10 yellow
    '#22bb22',  # office 11 green
    '#4444ff',  # office 12 blue
    '#999999',  # traversal  grey
]



class BeliefSubscriber(Node):
    def __init__(self):
        super().__init__('belief_visualizer')
        self._lock   = threading.Lock()
        self.belief  = np.ones(NUM_STATES) / NUM_STATES
        self.status  = 'waiting for data…'

        self.create_subscription(Float64MultiArray, '/belief',       self._belief_cb, 10)
        self.create_subscription(String,            '/bayes_status', self._status_cb, 10)

    def _belief_cb(self, msg: Float64MultiArray):
        with self._lock:
            data = list(msg.data)
            if len(data) == NUM_STATES:
                self.belief = np.array(data)

    def _status_cb(self, msg: String):
        with self._lock:
            self.status = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = BeliefSubscriber()

    # Spin ROS in a background thread so matplotlib owns the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── Build figure ──────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(15, 5))
    fig.patch.set_facecolor('#1e1e1e')
    ax.set_facecolor('#2a2a2a')

    x = np.arange(NUM_STATES)
    bars = ax.bar(x, node.belief, color=_BASE_COLOURS,
                  edgecolor='#555555', linewidth=0.8, width=0.7)

    ax.set_xlim(-0.6, NUM_STATES - 0.4)
    ax.set_ylim(0, 1.05)
    ax.set_xticks(x)
    ax.set_xticklabels(STATE_LABELS, rotation=40, ha='right',
                       fontsize=9, color='white')
    ax.set_ylabel('Probability', color='white')
    ax.tick_params(colors='white')
    for spine in ax.spines.values():
        spine.set_edgecolor('#555555')

    title  = ax.set_title('', color='white', fontsize=16, fontweight='bold', pad=12)

    # Probability labels above each bar (updated each frame)
    bar_labels = [
        ax.text(xi, 0, '', ha='center', va='bottom',
                fontsize=7, color='white', fontweight='bold')
        for xi in x
    ]

    plt.tight_layout()

    # ── Animation update ──────────────────────────────────────────────────────
    def update(_frame):
        with node._lock:
            belief = node.belief.copy()
            status = node.status

        max_idx = int(np.argmax(belief))

        for i, (bar, val, lbl) in enumerate(zip(bars, belief, bar_labels)):
            bar.set_height(val)

            # Highlight MAP estimate with bright white border
            if i == max_idx:
                bar.set_edgecolor('white')
                bar.set_linewidth(2.5)
                alpha = 1.0
            else:
                bar.set_edgecolor('#555555')
                bar.set_linewidth(0.8)
                alpha = 0.55 + 0.45 * val   # dim low-probability bars

            bar.set_alpha(alpha)

            # Probability label (hide if tiny to avoid clutter)
            if val >= 0.03:
                lbl.set_text(f'{val:.2f}')
                lbl.set_position((bar.get_x() + bar.get_width() / 2, val + 0.01))
            else:
                lbl.set_text('')

        title.set_text(status)
        return bars

    ani = animation.FuncAnimation(
        fig, update, interval=120, blit=False, cache_frame_data=False)

    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
