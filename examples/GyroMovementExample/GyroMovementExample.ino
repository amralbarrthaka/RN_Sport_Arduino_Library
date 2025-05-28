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
    
    delay(2000);
}

void loop() {
    static unsigned long lastStateChange = 0;
    static int state = 0;  // 0: forward, 1: stop1, 2: backward, 3: stop2
    unsigned long currentTime = millis();

    switch (state) {
        case 0:  // Forward
            if (currentTime - lastStateChange >= 3000) {  // After 3 seconds
                robot.stopMotors();
                state = 1;
                lastStateChange = currentTime;
            } else {
                robot.moveForwardWithGyro();
            }
            break;
            
        case 1:  // First stop
            if (currentTime - lastStateChange >= 1000) {  // After 1 second
                state = 2;
                lastStateChange = currentTime;
            }
            break;
            
        case 2:  // Backward
            if (currentTime - lastStateChange >= 3000) {  // After 3 seconds
                robot.stopMotors();
                state = 3;
                lastStateChange = currentTime;
            } else {
                robot.moveBackwardWithGyro();
            }
            break;
            
        case 3:  // Second stop
            if (currentTime - lastStateChange >= 1000) {  // After 1 second
                state = 0;  // Reset to forward
                lastStateChange = currentTime;
            }
            break;
    }
}