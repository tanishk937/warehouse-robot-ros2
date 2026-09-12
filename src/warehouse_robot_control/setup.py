# Copyright 2026 Tanishk Patidar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'warehouse_robot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanishk',
    maintainer_email='tanishk@example.com',
    description='Core autonomy, safety and diagnostics nodes for the warehouse_robot system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_heartbeat_monitor = warehouse_robot_control.sensor_heartbeat_monitor:main',
            'battery_manager = warehouse_robot_control.battery_manager:main',
            'localization_monitor = warehouse_robot_control.localization_monitor:main',
            'safety_controller = warehouse_robot_control.safety_controller:main',
            'e_stop_manager = warehouse_robot_control.e_stop_manager:main',
            'health_aggregator = warehouse_robot_control.health_aggregator:main',
            'sensor_fault_injector = warehouse_robot_control.sensor_fault_injector:main',
        ],
    },
)
