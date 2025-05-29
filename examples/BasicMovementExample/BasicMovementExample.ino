/*
 * Basic Movement Example using RN_Sport Library
 * 
 * This example demonstrates basic movement functions:
 * - Forward movement
 * - Backward movement
 * - Left rotation
 * - Right rotation
 * - Stop
 * 
 * Hardware Connections:
 * - MPU6050 VCC to 5V or 3.3V
 * - MPU6050 GND to GND
 * - MPU6050 SCL to Arduino SCL
 * - MPU6050 SDA to Arduino SDA
 * - Motor connections as per RN_Sport library
 */

#include <RN_Sport.h>

// Create RN_Sport object with default settings
RN_Sport robot;



void setup() {
    // Initialize Serial communication
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Basic Movement Example");
    
    // Initialize the robot and MPU6050
    if (!robot.begin()) {
        Serial.println("Failed to initialize RN_Sport!");
        while (1) {
            delay(10);
        }
    }
    
    Serial.println("RN_Sport initialized successfully!");
    
    // Set base speed
    robot.setMovementSpeed(150);  // Set to 150 (0-255)
    
    // Wait for sensor to stabilize
    delay(1000);
    
}

void loop() {
    // Get current time
    robot.moveForward();
    delay(5000);
    robot.moveBackward();
    delay(5000);
    robot.rotateLeft();
    delay(5000);
    robot.rotateRight();
    delay(5000);
    robot.stopMotors();
    delay(5000);
}  