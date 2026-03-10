from setuptools import find_packages, setup

package_name = 'sensors_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aabougen',
    maintainer_email='aabugendia@gmail.com',
    description='Depth camera simulation and visualization package using ROS 2 Jazzy and Gazebo Harmonic.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_camera_node = sensors_interface.camera_listener:main',
            'drone_teleop_key = sensors_interface.drone_teleop_key:main',
            'mavros_tf_broadcaster = sensors_interface.mavros_tf_broadcaster:main',
        ],
    },
)
