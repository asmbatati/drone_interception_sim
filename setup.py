# Copyright 2026 Abdulrahman S. Al-Batati
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

package_name = 'drone_interception_sim'


def recursive_data_files(directory):
    return [
        (os.path.join('share', package_name, root), [os.path.join(root, file) for file in files])
        for root, _, files in os.walk(directory)
        if files
    ]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *recursive_data_files('config'),
        *recursive_data_files('launch'),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        *(
            [(os.path.join('share', package_name, 'worlds'), glob('worlds/*'))]
            if os.path.exists('worlds') and glob('worlds/*')
            else []
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdulrahman S. Al-Batati',
    maintainer_email='asmalbatati@hotmail.com',
    description='Drone-to-drone interception multi-drone simulation assets built on uavros2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_trajectory = drone_interception_sim.target_trajectory_node:main',
            'interception_metrics = drone_interception_sim.interception_metrics:main',
            'drone_markers = drone_interception_sim.drone_markers:main',
            'offboard_manager = drone_interception_sim.offboard_manager:main',
            'pipeline_fsm = drone_interception_sim.pipeline_fsm:main',
            'pipeline_viz = drone_interception_sim.pipeline_viz:main',
        ],
    },
)
