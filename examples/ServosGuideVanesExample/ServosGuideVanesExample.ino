#include <RN_Sport.h>

// Create robot instance
RN_Sport robot;

// Servo pins
const int TOP_SERVO_PIN = 2;  // Top guide vane servo pin
const int LOW_SERVO_PIN = 7;  // Low guide vane servo pin

// Servo configuration
const int TOP_SERVO_DEFAULT = 40;  // Center position
const int TOP_SERVO_MIN = 40;      // Minimum angle
const int TOP_SERVO_MAX = 75;      // Maximum angle

const int LOW_SERVO_DEFAULT = 30;  // Center position
const int LOW_SERVO_MIN = 20;      // Minimum angle
const int LOW_SERVO_MAX = 180;     // Maximum angle

// Servo speed control
const int SERVO_SPEED = 220;    // Servo speed 0 to 255

void setup() {
    Serial.begin(115200);
    Serial.println("Servos Guide Vanes Example");

    // Set servo speed BEFORE initializing servos
    robot.setServoSpeed(SERVO_SPEED);
    
    // Initialize servos with all parameters
    robot.initializeTopServo(TOP_SERVO_PIN, TOP_SERVO_DEFAULT, TOP_SERVO_MIN, TOP_SERVO_MAX);
    delay(500);  // Short delay between initializations
    robot.initializeLowServo(LOW_SERVO_PIN, LOW_SERVO_DEFAULT, LOW_SERVO_MIN, LOW_SERVO_MAX);
    delay(1000);  // Wait for servos to initialize
}

void loop() {
    // Test top servo
    Serial.println("Moving top servo to minimum position");
    robot.setTopServoAngle(TOP_SERVO_MIN);
    delay(1000);
    
    Serial.println("Moving top servo to center position");
    robot.setTopServoAngle(TOP_SERVO_DEFAULT);
    delay(1000);
    
    Serial.println("Moving top servo to maximum position");
    robot.setTopServoAngle(TOP_SERVO_MAX);
    delay(1000);
    
    // Test low servo
    Serial.println("Moving low servo to minimum position");
    robot.setLowServoAngle(LOW_SERVO_MIN);
    delay(1000);
    
    Serial.println("Moving low servo to center position");
    robot.setLowServoAngle(LOW_SERVO_DEFAULT);
    delay(1000);
    
    Serial.println("Moving low servo to maximum position");
    robot.setLowServoAngle(LOW_SERVO_MAX);
    delay(1000);
    
    // Reset both servos
    Serial.println("Resetting servos to default positions");
    robot.resetServos();
    delay(2000);
}
