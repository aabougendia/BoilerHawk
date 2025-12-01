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
        
        # depth camera
        # ('share/' + package_name + '/launch', ['launch/depth_camera_sim.launch.py']),
        # ('share/' + package_name + '/urdf', ['urdf/depth_camera.urdf.xacro']),
        # ('share/' + package_name + '/rviz', ['rviz/depth_camera.rviz']),
        # ('share/' + package_name +'/worlds', ['worlds/obstacles.world']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aabougen',
    maintainer_email='aabugendia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_camera_node = sensors_interface.depth_camera_node:main',
            'drone_teleop_key = sensors_interface.drone_teleop_key:main',
        ],
    },
)
