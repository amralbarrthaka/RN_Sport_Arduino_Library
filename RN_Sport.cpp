#include "RN_Sport.h"
#include <Arduino.h>
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "HUSKYLENS.h"

RN_Sport::RN_Sport(int leftMotorPin, int rightMotorPin, int baseSpeed) 
    : motorRight(rightMotorPin), motorLeft(leftMotorPin), baseSpeed(baseSpeed) {
    // Initialize member variables
    yaw = 0.0;
    correctionValueGyro = 0.0;
    lastMicros = 0;
    lastGyroZ = 0.0;
    targetYaw = 0.0;
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
    Kp = 10.0;
    isForward = false;
    isBackward = false;
    isRotating = false;
    lastPrintTime = 0;
    lastUpdateTime = 0;
    lastStateChange = 0;
    totalCorrectionValueGyro = 0;
    currentDirection = DIR_STOP;  // Initialize to stopped
}

bool RN_Sport::begin() {
    // Initialize I2C
    Wire.begin();
    
    // Initialize motors
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    motorLeft.run(RELEASE);
    motorRight.run(RELEASE);
    
    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050");
        return false;
    }
    
    // Configure MPU6050 settings
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Initialize timing variables
    lastMicros = micros();
    lastPrintTime = millis();
    lastUpdateTime = millis();
    lastStateChange = millis();
    
    // Initialize camera flag
    cameraInitialized = false;
    
    return true;
}

void RN_Sport::setGyroRange(mpu6050_gyro_range_t range) {
    mpu.setGyroRange(range);
}

void RN_Sport::setFilterBandwidth(mpu6050_bandwidth_t bandwidth) {
    mpu.setFilterBandwidth(bandwidth);
}

void RN_Sport::initializeGyro() {
    Serial.println("Initializing MPU6050...");
    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050");
        return;
    }
    
    // Configure MPU6050 settings
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Wait for sensor to stabilize
    delay(1000);
    
    // Initialize timing and target values
    lastMicros = micros();
    targetYaw = yaw;
    
    Serial.println("MPU6050 initialized successfully");
    correctGyro();
}

void RN_Sport::correctGyro() {
    Serial.println("Correcting gyroscope (keep sensor still)...");
    float sum = 0.0;
    const int samples = 500;  // Increased samples for better calibration

    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;
        delay(5);
    }

    correctionValueGyro = sum/samples;
    Serial.print("Gyro Correction Value : ");
    Serial.println(correctionValueGyro, 6);
    delay(1000);
}

void RN_Sport::updateGyro() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - lastMicros) / 1000000.0; // convert to seconds
    lastMicros = currentMicros;

    // Get gyro reading and apply drift compensation
    float gyroZ = g.gyro.z - correctionValueGyro;
    

    // Integrate gyro.z to estimate yaw
    yaw += gyroZ * (180.0 / PI) * deltaTime;

    // Normalize yaw to [0, 360)
    if (yaw >= 360.0) yaw -= 360.0;
    if (yaw < 0.0) yaw += 360.0;
}

void RN_Sport::adjustMotorSpeeds() {
    // Skip adjustment if robot is rotating
    if (isRotating) return;

    // Calculate angle error and normalize to [-180, 180]
    float angleError = yaw - targetYaw;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    // Calculate proportional correction
    int correction = Kp * angleError;
    
    // Log correction value for debugging
    // Serial.print("Correction: ");
    // Serial.println(correction);

    // Apply correction based on movement direction
    if (isForward) {
        leftSpeed = baseSpeed - correction;  // Reduce left speed if turning right
        rightSpeed = baseSpeed + correction; // Increase right speed if turning right
    } else if (isBackward) {
        leftSpeed = baseSpeed + correction;  // Increase left speed if turning right
        rightSpeed = baseSpeed - correction; // Reduce right speed if turning right
    }

    // Ensure speeds stay within valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Apply calculated speeds to motors
    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
}

void RN_Sport::moveForward() {
    isForward = true;
    isBackward = false;
    isRotating = false;
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
}

void RN_Sport::moveBackward() {
    isForward = false;
    isBackward = true;
    isRotating = false;
    motorLeft.run(BACKWARD);
    motorRight.run(FORWARD);
}

void RN_Sport::stopMotors() {
    isForward = false;
    isBackward = false;
    isRotating = false;
    motorLeft.run(RELEASE);
    motorRight.run(RELEASE);

}

void RN_Sport::rotateLeft() {
    isForward = false;
    isBackward = false;
    isRotating = true;
    motorLeft.run(BACKWARD);
    motorRight.run(BACKWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
}

void RN_Sport::rotateRight() {
    isForward = false;
    isBackward = false;
    isRotating = true;
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
}

void RN_Sport::printGyroMotorStatus() {
    
    // Only print if moving with gyro control
    if (!isForward && !isBackward) {
        return;
    }


    // Check if it's time to print (every 1 second)
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime < 1000) {
        return;
    }
    lastPrintTime = currentTime;

    // Print status information
    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print("° Target: ");
    Serial.print(targetYaw, 2);
    Serial.print("° Left: ");
    Serial.print(leftSpeed);
    Serial.print(" Right: ");
    Serial.print(rightSpeed);
    Serial.print(" Direction: ");
    Serial.println(isForward ? "Forward" : "Backward");
}

void RN_Sport::scanI2CDevices() {
    byte error, address;
    int deviceCount = 0;

    Serial.println("\nScanning I2C devices...");

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            deviceCount++;
        }
    }

    if (deviceCount == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("Scan complete\n");
    }
}

bool RN_Sport::isI2CDevicePresent(byte address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    return (error == 0);
}

void RN_Sport::printI2CDeviceInfo(byte address) {
    if (isI2CDevicePresent(address)) {
        Serial.print("Device found at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
        
        // Try to read device ID or other information
        Wire.beginTransmission(address);
        Wire.write(0x00);  // Usually the first register
        Wire.endTransmission();
        
        Wire.requestFrom(address, (byte)1);
        if (Wire.available()) {
            byte data = Wire.read();
            Serial.print("First register value: 0x");
            if (data < 16) Serial.print("0");
            Serial.println(data, HEX);
        }
    } else {
        Serial.print("No device at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
    }
}

void RN_Sport::setMovementSpeed(int speed) {
    baseSpeed = constrain(speed, 0, 255);
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
}

bool RN_Sport::directChange() {
    MovementDirection newDirection;
    
    if (isForward) {
        newDirection = DIR_FORWARD;
    } else if (isBackward) {
        newDirection = DIR_BACKWARD;
    } else {
        newDirection = DIR_STOP;
    }
    
    if (newDirection != currentDirection) {
        currentDirection = newDirection;
        Serial.print("Direction changed to: ");
        switch(currentDirection) {
            case DIR_STOP:
                Serial.println("STOP");
                break;
            case DIR_FORWARD:
                Serial.println("FORWARD");
                break;
            case DIR_BACKWARD:
                Serial.println("BACKWARD");
                break;
            case DIR_LEFT:
                Serial.println("LEFT");
                break;
            case DIR_RIGHT:
                Serial.println("RIGHT");
                break;
        }
        return true;
    }
    return false;
}

void RN_Sport::moveForwardWithGyro() {
        isForward = true;
        isBackward = false;
        isRotating = false;
    if (directChange()) {
        targetYaw = yaw;
        motorLeft.run(FORWARD);
        motorRight.run(BACKWARD);
    }
    
    updateGyro();  // Update current yaw
    adjustMotorSpeeds();  // Adjust speeds to maintain straight path
    printGyroMotorStatus();  // Print status information
}

void RN_Sport::moveBackwardWithGyro() {
    isForward = false;
    isBackward = true;
    isRotating = false;
    if (directChange()) {
        targetYaw = yaw;
        motorLeft.run(BACKWARD);
        motorRight.run(FORWARD);
    }
    
    updateGyro();  // Update current yaw
    adjustMotorSpeeds();  // Adjust speeds to maintain straight path
    printGyroMotorStatus();  // Print status information
}

void RN_Sport::rotateLeftWithGyro(float targetAngle) {
    // Set movement state
    isForward = false;
    isBackward = false;
    isRotating = true;
    
    // Calculate target angle with proper normalization
    // For left rotation, we subtract the angle (clockwise)
    targetYaw = fmod(yaw - targetAngle + 360.0, 360.0);
    
    // Set motor directions and speeds
    motorLeft.run(BACKWARD);
    motorRight.run(BACKWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
    
    // Execute rotation with timeout and error handling
    bool rotationComplete = handleRotation(targetAngle, DIR_LEFT);
    
    // Update state and provide feedback
    isRotating = false;
    Serial.print("Left rotation ");
    Serial.println(rotationComplete ? "completed successfully" : "timed out with error");
}

void RN_Sport::rotateRightWithGyro(float targetAngle) {
    // Set movement state
    isForward = false;
    isBackward = false;
    isRotating = true;
    
    // Calculate target angle with proper normalization
    // For right rotation, we add the angle (counter-clockwise)
    targetYaw = fmod(yaw + targetAngle + 360.0, 360.0);
    
    // Set motor directions and speeds
    // For right rotation, left motor forward, right motor backward
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
    
    // Execute rotation with timeout and error handling
    bool rotationComplete = handleRotation(targetAngle, DIR_RIGHT);
    
    // Update state and provide feedback
    isRotating = false;
    Serial.print("Right rotation ");
    Serial.println(rotationComplete ? "completed successfully" : "timed out with error");
}

bool RN_Sport::handleRotation(float angle, MovementDirection direction) {
    const unsigned long ROTATION_TIMEOUT = 5000;  // 5 second timeout
    const float ROTATION_TOLERANCE = 2.0;        // 2 degree tolerance
    
    unsigned long startTime = millis();
    bool rotationComplete = false;
    
    // Monitor rotation until complete or timeout
    while (!rotationComplete && (millis() - startTime < ROTATION_TIMEOUT)) {
        updateGyro();
        rotationComplete = checkRotationComplete(ROTATION_TOLERANCE);

    }
    
    // Stop motors if rotation is complete or timed out
    stopMotors();
    isRotating = false;
    
    return rotationComplete;
}

bool RN_Sport::checkRotationComplete(float tolerance) {
    float angleError = yaw - targetYaw;
    
    // Normalize angle error to [-180, 180]
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;
    
    // Check if we're within tolerance and log the error for debugging
    bool isComplete = abs(angleError) <= tolerance;
    if (!isComplete) {
        Serial.print("Angle error: ");
        Serial.println(angleError);
    }
    
    return isComplete;
}

bool RN_Sport::beginCamera() {
    Wire.begin();
    if (!huskylens.begin(Wire)) {
        return false;
    }
    cameraInitialized = true;
    return true;
}

bool RN_Sport::updateCameraData() {
    if (!cameraInitialized) return false;
    
    if (!huskylens.request()) {
        return false;
    }
    
    if (!huskylens.available()) {
        return false;
    }
    
    lastResult = huskylens.read();
    return true;
}

bool RN_Sport::isObjectDetected() {
    return updateCameraData();
}

int RN_Sport::getObjectX() {
    if (updateCameraData()) {
        return lastResult.xCenter;
    }
    return -1;
}

int RN_Sport::getObjectY() {
    if (updateCameraData()) {
        return lastResult.yCenter;
    }
    return -1;
}

int RN_Sport::getObjectWidth() {
    if (updateCameraData()) {
        return lastResult.width;
    }
    return -1;
}

int RN_Sport::getObjectHeight() {
    if (updateCameraData()) {
        return lastResult.height;
    }
    return -1;
}

bool RN_Sport::isColorLearned() {
    if (!cameraInitialized) return false;
    return huskylens.isLearned();
}

void RN_Sport::setCameraAlgorithm(int algorithm) {
    if (cameraInitialized) {
        huskylens.writeAlgorithm(algorithm);
    }
}



