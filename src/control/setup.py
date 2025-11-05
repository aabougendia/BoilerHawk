from setuptools import setup

setup(
    name='control',
    version='0.0.0',
    packages=['control'],
    package_data={
        'control': [],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/control']),
        ('share/control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zizo2004',
    maintainer_email='zizo2004@example.com',
    description='Control node for autonomous drone',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'control_node=control.control_node:main',
        ],
    },
)

