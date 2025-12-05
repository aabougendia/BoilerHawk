from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BoilerHawk',
    maintainer_email='boilerhawk@purdue.edu',
    description='Perception package for point cloud processing and occupancy grid generation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = perception.perception:main',
            'mock_pointcloud_publisher = perception.mock_pointcloud_publisher:main',
        ],
    },
)
