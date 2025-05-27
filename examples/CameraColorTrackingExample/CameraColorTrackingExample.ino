#include <RN_Sport.h>

// Create RN_Sport instance
RN_Sport robot;

// Tracking variables
bool colorFound = false;
bool track = false;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000; // Print status every second

void setup() {
    Serial.begin(115200);
    
    // Initialize the camera
    if (!robot.beginCamera()) {
        Serial.println("Failed to initialize camera!");
        while (1);
    }
    
    // Set camera to color recognition mode
    robot.setCameraAlgorithm(ALGORITHM_COLOR_RECOGNITION);
    
    Serial.println("Setup complete! Ready to detect objects.");
    Serial.println("Press the learn button on HUSKYLENS and show an object to track.");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check if we need to print status
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
        if (!robot.isColorLearned()) {
            Serial.println("Color not learned!");
            Serial.println("Please press the learn button on HUSKYLENS and show an object.");
        } else if (!robot.isObjectDetected()) {
            Serial.println("No object detected!");
            Serial.println("Please show the learned object to the camera.");
        }
        lastPrintTime = currentTime;
    }
    
    // If object is detected, print its information
    if (robot.isObjectDetected()) {
        if (!colorFound) {
            colorFound = true;
            Serial.println("Object detected!");
        }
        
        Serial.print("Object position - X: ");
        Serial.print(robot.getObjectX());
        Serial.print(", Y: ");
        Serial.print(robot.getObjectY());
        Serial.print(", Width: ");
        Serial.print(robot.getObjectWidth());
        Serial.print(", Height: ");
        Serial.println(robot.getObjectHeight());
        
        // Add a small delay to prevent too rapid updates
        delay(100);
    } else {
        colorFound = false;
    }
} 