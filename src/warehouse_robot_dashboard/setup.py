# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'warehouse_robot_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'warehouse_robot_dashboard', 'static'),
            glob('warehouse_robot_dashboard/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanishk',
    maintainer_email='tanishk@example.com',
    description='Real-time ROS 2 <-> web dashboard bridge for the warehouse robot.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_bridge = warehouse_robot_dashboard.dashboard_bridge:main',
        ],
    },
)
