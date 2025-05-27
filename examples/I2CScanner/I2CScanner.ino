#include <Wire.h>
#include <RN_Sport.h>

RN_Sport robot;

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for Serial to be ready
    
    Serial.println("\nI2C Scanner Example");
    Wire.begin();
}

void loop() {
    // Use the library's built-in I2C scanner
    robot.scanI2CDevices();
    
    // Example of checking a specific device (MPU6050 at address 0x68)
    Serial.println("\nChecking MPU6050...");
    robot.printI2CDeviceInfo(0x68);
    
    delay(5000);  // Wait 5 seconds for next scan
} 