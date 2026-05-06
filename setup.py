from setuptools import setup
from glob import glob
import os

package_name = 'camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package to publish camera images using ROS2.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'raspicam = camera_publisher.raspicam_publisher:main',
            'webcam = camera_publisher.webcam_publisher:main',
            'oak = camera_publisher.oak_d:main',
            'jetson = camera_publisher.imx_219:main'
        ],
    },
)
