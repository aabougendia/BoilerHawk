from setuptools import setup
import os
from glob import glob

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aabougen',
    maintainer_email='aabugendia@gmail.com',
    description='Planning module for path planning with occupancy grid',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning_node = planning.planning_node:main',
            'mock_perception_node = planning.mock_perception_node:main',
        ],
    },
)
