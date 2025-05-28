/*

 */

#include <RN_Sport.h>

// Create RN_Sport object with default settings
RN_Sport robot;

// Movement sequence timing
const unsigned long MOVE_DURATION = 3000;    // 3 seconds
const unsigned long STOP_DURATION = 1000;    // 1 second
const unsigned long BACK_DURATION = 3000;     // 0.5 seconds
const float ROTATION_ANGLE = 180.0;         // 180 degrees
const float ROTATION_TOLERANCE = 2.0;       // 2 degrees tolerance
const unsigned long ROTATION_TIMEOUT = 3000; // 3 seconds timeout

void setup() {
    // Initialize Serial communication
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Gyro Movement Example");
    Serial.println("---------------------");
    
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
    
    // Set rotation timeout (0 to disable automatic direction switching)
    robot.setLeftRightRotationTimeout(3000);  // 3 seconds timeout
    
    // Wait for sensor to stabilize
    Serial.println("Calibrating gyroscope...");
    robot.correctGyro();
    delay(1000);
    
    Serial.println("Starting movement sequence...");
    Serial.println();
}

void loop() {
    // Forward movement with gyro control
    Serial.println("Moving forward with gyro control...");
    robot.moveForwardWithGyro();
    robot.printGyroMotorStatus();
    delay(MOVE_DURATION);
    
    // Stop
    Serial.println("Stopping...");
    robot.stopMotors();
    robot.printGyroMotorStatus();
    delay(STOP_DURATION);
    
    // Backward movement with gyro control
    Serial.println("Moving backward with gyro control...");
    robot.moveBackwardWithGyro();
    robot.printGyroMotorStatus();
    delay(BACK_DURATION);

    // Stop
    Serial.println("Stopping...");
    robot.stopMotors();
    robot.printGyroMotorStatus();
    delay(STOP_DURATION);
    
    // // Rotate left with precise angle control
    // Serial.println("Rotating left 180 degrees...");
    // if (robot.handleRotation(&RN_Sport::rotateLeftWithGyro, ROTATION_ANGLE, ROTATION_TOLERANCE, ROTATION_TIMEOUT)) {
    //     Serial.println("Left rotation completed successfully!");
    // } else {
    //     Serial.println("Left rotation timed out!");
    // }
    // robot.printGyroMotorStatus();
    // delay(STOP_DURATION);
    
    // // Forward movement again
    // Serial.println("Moving forward with gyro control...");
    // robot.moveForwardWithGyro();
    // robot.printGyroMotorStatus();
    // delay(MOVE_DURATION);
    
    // // Stop
    // Serial.println("Stopping...");
    // robot.stopMotors();
    // robot.printGyroMotorStatus();
    // delay(STOP_DURATION);
    
    // // Backward movement
    // Serial.println("Moving backward with gyro control...");
    // robot.moveBackwardWithGyro();
    // robot.printGyroMotorStatus();
    // delay(BACK_DURATION);
    
    // // Rotate right with precise angle control
    // Serial.println("Rotating right 180 degrees...");
    // if (robot.handleRotation(&RN_Sport::rotateRightWithGyro, ROTATION_ANGLE, ROTATION_TOLERANCE, ROTATION_TIMEOUT)) {
    //     Serial.println("Right rotation completed successfully!");
    // } else {
    //     Serial.println("Right rotation timed out!");
    // }
    // robot.printGyroMotorStatus();
    // delay(STOP_DURATION);
    
    // Serial.println("Sequence complete! Starting over...");
    // Serial.println();
}  