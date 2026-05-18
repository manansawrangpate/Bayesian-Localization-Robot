from setuptools import setup
from glob import glob
import os

package_name = 'bayesian_localization_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lordmanan',
    maintainer_email='lordmanan@example.com',
    description='Bayesian localization TurtleBot3 project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node     = bayesian_localization_project.perception_node:main',
            'bayes_loc_node      = bayesian_localization_project.bayes_loc_node:main',
            'perception_test     = bayesian_localization_project.perception_test_node:main',
            'belief_visualizer   = bayesian_localization_project.belief_visualizer:main',
        ],
    },
)