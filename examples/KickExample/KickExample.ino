/*

 */

#include <RN_Sport.h>

// Create RN_Sport object with default settings
RN_Sport robot;



void setup() {
    // Initialize Serial communication
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // Initialize the robot and MPU6050
    if (!robot.begin()) {
        Serial.println("Failed to initialize RN_Sport!");
        while (1) {
            delay(10);
        }
    }
    
    // Set Kick Speed
    robot.setKickSpeed(255);
    delay(1000);

}

void loop() {
    robot.kickForward();
    delay(5000);
    robot.stopKick();
    delay(5000);
    robot.kickBackward();
    delay(5000);
    robot.stopKick();
    delay(5000);
}  