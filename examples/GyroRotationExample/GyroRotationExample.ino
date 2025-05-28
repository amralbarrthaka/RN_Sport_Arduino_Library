#include <RN_Sport.h>

// Create RN_Sport instance with default motor pins and base speed
RN_Sport robot;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("RN_Sport Gyro Movement Example");
    
    // Initialize the robot
    if (!robot.begin()) {
        Serial.println("Failed to initialize robot!");
        while (1) delay(10);
    }
    
    // Initialize gyroscope
    robot.initializeGyro();
    
    // Set movement speed (0-255)
    robot.setMovementSpeed(100);
    
    delay(1000);
}

void loop() {
    robot.rotateLeftWithGyro(90);
    delay(3000);
    robot.rotateRightWithGyro(180);
    delay(3000);
    robot.rotateLeftWithGyro(90);
    delay(3000);
}