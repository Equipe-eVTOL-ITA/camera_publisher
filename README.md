# Raspicam Publisher

This repository contains the ROS2 package `raspicam_publisher`, which captures images from a Raspberry Pi camera and publishes them as compressed image messages.

## Prerequisites

Before you begin, ensure you have met the following requirements:

- **Operating System:** Raspberry Pi OS or Ubuntu 20.04/22.04 for Raspberry Pi
- **ROS2:** ROS2 Humble
- **Python:** Python 3.8 or later
- **Camera:** Raspberry Pi Camera Module (e.g., RaspCam V1.3)

### Install ROS2

Follow the official ROS2 installation guide for [Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or [Raspberry Pi OS](https://docs.ros.org/en/humble/Installation/Raspberry-Pi-Install.html).

### Install Additional Dependencies

1. **Install `cv_bridge` and other ROS2 dependencies:**

   ```sh
   sudo apt-get update
   sudo apt-get install ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport
   ```

2. **Install OpenCV for Python:**

   ```sh
   pip install opencv-python
   ```

## Installation

1. **Clone the Repository:**

   ```sh
   git clone https://github.com/Equipe-eVTOL-ITA/raspicam_publisher.git
   cd raspicam_publisher
   ```

2. **Build the Package:**

   ```sh
   colcon build
   source install/setup.bash
   ```

## Usage

1. **Run the Camera Publisher Node:**

   Note: remember to `source /opt/ros/humble/setup.bash` and `source install/setup.bash`

   ```sh
   ros2 run raspicam_publisher camera_publisher
   ```

2. **View the Published Images:**

   You can use `rqt_image_view` to view the published images:

   ```sh
   rqt_image_view /camera/image/compressed
   ```

## Explanation

The `raspicam_publisher` package captures images from the Raspberry Pi camera and publishes them as compressed image messages on the `/camera/image/compressed` topic. This helps in reducing the bandwidth and improving the performance when transmitting images over a network.

## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Raspberry Pi Camera Module](https://www.raspberrypi.com/documentation/accessories/camera.html)

Note: README.md wrote with the help of ChatGPT 4o.
