from setuptools import find_packages, setup

package_name = 'vins_px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vins_px4_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/vins_px4_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lingzhilab',
    maintainer_email='lingzhilab@example.com',
    description='Bridge VINS-Fusion Odometry to PX4 VehicleVisualOdometry',
    license='MIT',
    # tests_require is deprecated; use extras_require instead if needed
    entry_points={
        'console_scripts': [
            'bridge_node = vins_px4_bridge.bridge_node:main',
        ],
    },
)
