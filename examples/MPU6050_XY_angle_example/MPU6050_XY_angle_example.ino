/*
 * MPU6050 XY Angle Measurement Example using RN_Sport Library
 * 
 * This example demonstrates how to use the RN_Sport library to measure
 * angles using the MPU6050 sensor.
 * 
 * Hardware Connections:
 * - MPU6050 VCC to 5V or 3.3V
 * - MPU6050 GND to GND
 * - MPU6050 SCL to Arduino SCL
 * - MPU6050 SDA to Arduino SDA
 */

#include <RN_Sport.h>

// Create RN_Sport object with default settings
RN_Sport robot;

// Variables for timing
unsigned long lastMicros = 0;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("MPU6050 XY Angle Measurement Example");
  
  // Initialize the robot and MPU6050
  if (!robot.begin()) {
    Serial.println("Failed to initialize RN_Sport!");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("RN_Sport initialized successfully!");
  
  // Configure MPU6050
  robot.setGyroRange(MPU6050_RANGE_250_DEG);
  robot.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Wait for sensor to stabilize
  delay(1000);
  
  // Calibrate the gyroscope
  robot.calibrateGyro();
  
  // Initialize timing
  lastMicros = micros();
}

void loop() {
  // Update the yaw angle
  robot.updateYaw();
  
  // Get the current yaw angle
  float yaw = robot.getYaw();
  
  // Print the angle
  Serial.print("Yaw (Heading Estimate): ");
  Serial.print(yaw, 2);
  Serial.println("°");
  
  // Small delay to avoid spamming Serial
  delay(10);
}
