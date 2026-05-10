from setuptools import find_packages, setup

package_name = 'fastlio_px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lingzhilab',
    maintainer_email='lingzhilab@example.com',
    description='Bridge FAST-LIO Odometry to PX4 VehicleVisualOdometry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = fastlio_px4_bridge.bridge_node:main',
        ],
    },
)
