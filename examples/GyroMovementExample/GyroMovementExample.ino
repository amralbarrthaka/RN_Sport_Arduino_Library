/*
 * Gyro Movement Example using RN_Sport Library
 * 
 * This example demonstrates how to use the RN_Sport library's gyro-controlled
 * movement functions to maintain heading while moving.
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

// Movement sequence states
enum MovementState {
    FORWARD_WITH_GYRO,
    STOP_1S,
    BACKWARD_WITH_GYRO,
    STOP_1S_2,
    ROTATE_LEFT_90,
    STOP_1S_3,
    ROTATE_RIGHT_90,
    STOP_1S_4
};

MovementState currentState = FORWARD_WITH_GYRO;
unsigned long lastStateChange = 0;

void setup() {
    // Initialize Serial communication
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Gyro Movement Example");
    
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
    
    // Set movement parameters
    robot.setBaseSpeed(150);  // Set base speed
    robot.setGyroGain(2.0);   // Set gyro correction gain
    
    // Wait for sensor to stabilize
    delay(1000);
    
    // Calibrate the gyroscope
    robot.calibrateGyro();
    
    // Start the movement sequence
    lastStateChange = millis();
}

void loop() {
    // Update the yaw angle
    robot.updateYaw();
    
    // Get current time
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastStateChange;
    
    // State machine for movement sequence
    switch(currentState) {
        case FORWARD_WITH_GYRO:
            if (elapsedTime >= 3000) {  // Move forward for 3 seconds
                robot.stopMotors();
                currentState = STOP_1S;
                lastStateChange = currentTime;
            } else {
                robot.moveForwardWithGyro();
            }
            break;
            
        case STOP_1S:
            if (elapsedTime >= 1000) {  // Stop for 1 second
                currentState = BACKWARD_WITH_GYRO;
                lastStateChange = currentTime;
            }
            break;
            
        case BACKWARD_WITH_GYRO:
            if (elapsedTime >= 3000) {  // Move backward for 3 seconds
                robot.stopMotors();
                currentState = STOP_1S_2;
                lastStateChange = currentTime;
            } else {
                robot.moveBackwardWithGyro();
            }
            break;
            
        case STOP_1S_2:
            if (elapsedTime >= 1000) {  // Stop for 1 second
                currentState = ROTATE_LEFT_90;
                lastStateChange = currentTime;
            }
            break;
            
        case ROTATE_LEFT_90:
            if (elapsedTime >= 1000) {  // Rotate left for 1 second
                robot.stopMotors();
                currentState = STOP_1S_3;
                lastStateChange = currentTime;
            } else {
                robot.rotateLeftWithGyro(90.0);  // Rotate 90 degrees
            }
            break;
            
        case STOP_1S_3:
            if (elapsedTime >= 1000) {  // Stop for 1 second
                currentState = ROTATE_RIGHT_90;
                lastStateChange = currentTime;
            }
            break;
            
        case ROTATE_RIGHT_90:
            if (elapsedTime >= 1000) {  // Rotate right for 1 second
                robot.stopMotors();
                currentState = STOP_1S_4;
                lastStateChange = currentTime;
            } else {
                robot.rotateRightWithGyro(90.0);  // Rotate 90 degrees
            }
            break;
            
        case STOP_1S_4:
            if (elapsedTime >= 1000) {  // Stop for 1 second
                currentState = FORWARD_WITH_GYRO;  // Restart sequence
                lastStateChange = currentTime;
            }
            break;
    }
    
    // Print status every 100ms
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 100) {
        Serial.print("Yaw: ");
        Serial.print(robot.getYaw(), 2);
        Serial.print("° Target: ");
        Serial.print(robot.getTargetYaw(), 2);
        Serial.print("° Left: ");
        Serial.print(robot.getLeftSpeed());
        Serial.print(" Right: ");
        Serial.println(robot.getRightSpeed());
        lastPrintTime = currentTime;
    }
    
    // Small delay to avoid overwhelming the system
    delay(10);
} 