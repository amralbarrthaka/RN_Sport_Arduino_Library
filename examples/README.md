# RN_Sport Library Examples

This folder contains example sketches demonstrating how to use the RN_Sport library.

## Examples

### 1. RN_Sport_Example
Basic example showing how to initialize and use the RN_Sport robot.

### 2. GyroMPU6050MoveTest
This example replicates the functionality of the original gyro_MPU6050_move_test.ino sketch. It demonstrates:

- MPU6050 gyroscope initialization and calibration
- Movement sequence:
  1. Forward for 3 seconds
  2. Stop for 1 second
  3. Backward for 0.5 seconds
  4. Rotate left 180 degrees
  5. Forward for 3 seconds
  6. Stop for 1 second
  7. Backward for 0.5 seconds
  8. Rotate right 180 degrees
  9. Repeat

Features:
- Automatic recovery from stuck rotations
- Gyroscope-based heading control
- Status monitoring via Serial

### 3. I2CScanner
A utility example to scan for I2C devices connected to your Arduino. This is useful for:

- Verifying MPU6050 connection
- Finding I2C device addresses
- Troubleshooting I2C communication issues

Features:
- Scans all possible I2C addresses (1-127)
- Reports found devices with their addresses
- Repeats scan every 5 seconds
- Shows unknown errors at specific addresses

## Usage

1. Install the RN_Sport library in your Arduino IDE
2. Open the example sketch
3. Upload to your Arduino
4. Open Serial Monitor at 115200 baud to see status updates

## Hardware Requirements

- Arduino board
- Adafruit Motor Shield
- MPU6050 gyroscope
- Two DC motors
- Power supply

## Connections

- MPU6050: SDA and SCL pins
- Motors: Connected to Motor Shield
- Power: 7-12V to Motor Shield

## I2C Scanner Usage

1. Upload the I2CScanner example
2. Open Serial Monitor at 115200 baud
3. The scanner will:
   - Search for I2C devices
   - Display found devices with their addresses
   - Show any communication errors
   - Repeat scan every 5 seconds

Expected MPU6050 address: 0x68 