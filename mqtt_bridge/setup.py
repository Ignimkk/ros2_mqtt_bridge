from setuptools import find_packages, setup

package_name = 'mqtt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ros_to_mqtt.yaml']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='mk',
    maintainer_email='kmingi98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_to_mqtt = mqtt_bridge.ros_to_mqtt:main',
            'mqtt_to_ros = mqtt_bridge.mqtt_to_ros:main',
            'test_ros_to_mqtt = mqtt_bridge.test_ros_to_mqtt:main',
            'test_mqtt_to_ros = mqtt_bridge.test_mqtt_to_ros:main',
        ],
    },
)
