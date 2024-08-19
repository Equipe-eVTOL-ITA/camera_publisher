from setuptools import setup

package_name = 'camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
        ],
    },
)
