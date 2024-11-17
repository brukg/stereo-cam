# Stereo Camera ROS 2 Node

A ROS 2 node for capturing and publishing synchronized stereo images using dual IMX219-83 cameras with libcamera. This project provides real-time stereo image capture and publishing capabilities, suitable for robotics and computer vision applications.

## Features

- Dual camera synchronization
- Configurable image resolution and frame rate
- ROS 2 image publishing
- Support for various exposure modes
- Real-time performance monitoring
- Buffer management for continuous capture

## Prerequisites

### Hardware

- 2x IMX219-83 cameras
- Compatible hardware platform (e.g., Raspberry Pi)

### Software Dependencies

- ROS 2 (tested with Humble)
- libcamera
- OpenCV 4
- libevent
- Boost
- cv_bridge
- sensor_msgs

## Installation

1. Install system dependencies:

```bash
sudo apt install -y \
    libcamera-dev \
    libevent-dev \
    libopencv-dev \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs
```

2. Clone the repository:

```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

3. Build the project:

```bash
cd ~/ros2_ws
colcon build
```

## Usage

1. Source your ROS 2 workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

2. Run the stereo camera node:

```bash
ros2 run stereo_camera stereo_cam
```

### Command Line Options

- `-d`: Enable/disable dual cameras (default: 1)
- `-w`: Image width (default: 640)
- `-h`: Image height (default: 480)
- `-p`: Preview window position and size (format: x,y,width,height)
- `-f`: Frame rate in fps (default: 100)
- `-s`: Shutter speed in nanoseconds
- `-e`: Exposure mode (normal/sport/short/long/custom)
- `-t`: Timeout in seconds (default: 100)
- `-b`: Buffer count (default: 4)

## Topics

The node publishes images on the following topics:

- `/camera/left/image_raw`: Left camera images
- `/camera/right/image_raw`: Right camera images

Both topics publish `sensor_msgs/msg/Image` messages.

## Performance

The node includes performance monitoring features:

- Frame rate monitoring
- Dropped frame counting
- Buffer management statistics

## License

This project is licensed under GPL-2.0-or-later.

## Authors

- Bruk G.
- Peyton Howe

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Troubleshooting

### Common Issues

1. No cameras detected:
   - Check camera connections
   - Verify camera permissions
   - Ensure libcamera is properly installed

2. Poor performance:
   - Adjust buffer count
   - Reduce resolution or frame rate
   - Check system resources

## References

- [libcamera Documentation](https://libcamera.org/docs.html)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [IMX219-83 Camera Documentation](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)
