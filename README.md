# RN_Sport Arduino Library

A comprehensive Arduino library for controlling RN Sport robot with MPU6050 gyroscope integration. This library provides easy-to-use functions for precise robot movement control, including gyroscope-based heading control and predefined movement sequences.

## Features

- **Gyroscope Integration**: Built-in support for MPU6050 gyroscope for accurate heading control
- **Motor Control**: Easy control of DC motors with speed and direction management
- **Movement Sequences**: Predefined movement patterns for autonomous operation
- **I2C Device Management**: Built-in I2C scanner and device management tools
- **Configurable Parameters**: Adjustable speed, gyro gain, and movement parameters

## Installation

1. Download this repository as a ZIP file
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file
4. The library will be installed and ready to use

## Dependencies

This library requires the following dependencies:
- Wire (built-in Arduino library)
- AFMotor (Adafruit Motor Shield library)
- Adafruit MPU6050
- Adafruit Unified Sensor

You can install the required libraries through the Arduino Library Manager:
1. Open Arduino IDE
2. Go to Tools > Manage Libraries
3. Search for and install:
   - "Adafruit Motor Shield library"
   - "Adafruit MPU6050"
   - "Adafruit Unified Sensor"

## Hardware Requirements

- Arduino board (compatible with Adafruit Motor Shield)
- Adafruit Motor Shield
- MPU6050 Gyroscope
- RN Sport robot chassis
- DC Motors (compatible with Motor Shield)

## Available Functions

### Basic Movement
- `moveForward()`: Move robot forward
- `moveBackward()`: Move robot backward
- `stopMotors()`: Stop all motors
- `rotateLeft()`: Rotate robot left
- `rotateRight()`: Rotate robot right

### Gyro-Controlled Movement
- `moveForwardWithGyro()`: Move forward while maintaining heading
- `moveBackwardWithGyro()`: Move backward while maintaining heading
- `rotateLeftWithGyro(float targetAngle)`: Rotate left to specific angle
- `rotateRightWithGyro(float targetAngle)`: Rotate right to specific angle

### Configuration
- `setBaseSpeed(int speed)`: Set base motor speed
- `setGyroGain(float gain)`: Adjust gyroscope sensitivity
- `setGyroRange(mpu6050_gyro_range_t range)`: Configure gyroscope range
- `setFilterBandwidth(mpu6050_bandwidth_t bandwidth)`: Set gyroscope filter bandwidth

### Status and Information
- `getYaw()`: Get current yaw angle
- `getTargetYaw()`: Get target yaw angle
- `getLeftSpeed()`: Get left motor speed
- `getRightSpeed()`: Get right motor speed
- `printStatus()`: Print current robot status

## Examples

Check the `examples` folder for complete usage examples:
- Basic movement control
- Gyroscope calibration
- Movement sequences
- I2C device scanning

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For support, please open an issue in the GitHub repository or contact the maintainer.