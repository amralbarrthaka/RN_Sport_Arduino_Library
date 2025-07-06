#include "RN_Sport.h"
#include <Arduino.h>
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "HUSKYLENS.h"
#include <Servo.h>
#include <SoftwareSerial.h>

RN_Sport::RN_Sport(int leftMotorPin, int rightMotorPin, int kickMotorPin, int baseSpeed) 
    : motorRight(rightMotorPin), motorLeft(leftMotorPin), motorKick(kickMotorPin), baseSpeed(baseSpeed) {
    // Initialize member variables
    yaw = 0.0;
    correctionValueGyro = 0.0;
    lastMicros = 0;
    lastGyroZ = 0.0;
    targetYaw = 0.0;
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
    kickSpeed = baseSpeed;  // Initialize kick speed
    Kp = 10.0;
    isForward = false;
    isBackward = false;
    isRotating = false;
    lastPrintTime = 0;
    lastUpdateTime = 0;
    lastStateChange = 0;
    totalCorrectionValueGyro = 0;
    currentDirection = DIR_STOP;  // Initialize to stopped
    
    // Initialize servo variables
    topServoInitialized = false;
    lowServoInitialized = false;
    servoSpeed = 255;  // Default to fastest speed
    currentTopAngle = 0;
    currentLowAngle = 0;
    bool leftMotorReverse = false;  // Reverse left motor direction
    bool rightMotorReverse = false; // Reverse right motor direction
}

bool RN_Sport::begin() {
    // Initialize I2C
    Wire.begin();
    
    // Initialize motors
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    motorKick.setSpeed(0);
    motorLeft.run(RELEASE);
    motorRight.run(RELEASE);
    motorKick.run(RELEASE);
    
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
        if (leftMotorReverse){
            leftSpeed = baseSpeed - correction; 
        }else{
            leftSpeed = baseSpeed + correction; 
        }
        if (rightMotorReverse){
            rightSpeed = baseSpeed + correction;
        }else{
                rightSpeed = baseSpeed - correction; 
        }
   
    } else if (isBackward) {
        if (leftMotorReverse){
            leftSpeed = baseSpeed + correction; 
        }else{
            leftSpeed = baseSpeed - correction; 
        }
        if (rightMotorReverse){
            rightSpeed = baseSpeed - correction;
        }else{
                rightSpeed = baseSpeed + correction; 
        }
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
    this->process_motorDirections("forward");
}

void RN_Sport::moveBackward() {
    isForward = false;
    isBackward = true;
    isRotating = false;
    this->process_motorDirections("backward");
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
    this->process_motorDirections("left");
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
}

void RN_Sport::rotateRight() {
    isForward = false;
    isBackward = false;
    isRotating = true;
    this->process_motorDirections("right");
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
    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
    Serial.print("Moving Speed changed to: ");
    Serial.println(baseSpeed);
}


void RN_Sport::setKickSpeed(int speed) {
    kickSpeed = constrain(speed, 0, 255);
    motorKick.setSpeed(kickSpeed); // Update the kick motor speed immediately
    Serial.print("Kick Speed changed to: ");
    Serial.println(kickSpeed);
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

void RN_Sport::process_motorDirections(String direction) {
    if (direction == "forward") {
        if (leftMotorReverse && rightMotorReverse) {
            motorLeft.run(BACKWARD);
            motorRight.run(BACKWARD);
        } else if (leftMotorReverse) {
            motorLeft.run(BACKWARD);
            motorRight.run(FORWARD);
        } else if (rightMotorReverse) {
            motorLeft.run(FORWARD);
            motorRight.run(BACKWARD);
        } else {
            // Default case, both motors run forward
            motorLeft.run(FORWARD);
            motorRight.run(FORWARD);
        }
    } else if (direction == "backward") {
        if (leftMotorReverse && rightMotorReverse) {
            motorLeft.run(FORWARD);
            motorRight.run(FORWARD);
        } else if (leftMotorReverse) {
            motorLeft.run(FORWARD);
            motorRight.run(BACKWARD);
        } else if (rightMotorReverse) {
            motorLeft.run(BACKWARD);
            motorRight.run(FORWARD);
        } else {
            // Default case, both motors run backward
            motorLeft.run(BACKWARD);
            motorRight.run(BACKWARD);
        }

    }else if (direction == "left") {
        if (leftMotorReverse && rightMotorReverse) {
            motorLeft.run(FORWARD);
            motorRight.run(BACKWARD);
        } else if (leftMotorReverse) {
            motorLeft.run(FORWARD);
            motorRight.run(FORWARD);
        } else if (rightMotorReverse) {
            motorLeft.run(BACKWARD);
            motorRight.run(BACKWARD);
        } else {
            // Default case, left motor backward, right motor forward
            motorLeft.run(BACKWARD);
            motorRight.run(FORWARD);
        }
    } else if (direction == "right") {
        if (leftMotorReverse && rightMotorReverse) {
            motorLeft.run(BACKWARD);
            motorRight.run(FORWARD);
        } else if (leftMotorReverse) {
            motorLeft.run(BACKWARD);
            motorRight.run(BACKWARD);
        } else if (rightMotorReverse) {
            motorLeft.run(FORWARD);
            motorRight.run(FORWARD);
        } else {
            // Default case, left motor forward, right motor backward
            motorLeft.run(FORWARD);
            motorRight.run(BACKWARD);
        }
    } else {
        // Stop motors for any other direction
        stopMotors();
    }
}

void RN_Sport::moveForwardWithGyro() {
    isForward = true;
    isBackward = false;
    isRotating = false;
    if (directChange()) {
        targetYaw = yaw;
    }
    this->process_motorDirections("forward");  // Process motor directions based on reverse flags

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
    }
    this->process_motorDirections("backward");  // Process motor directions based on reverse flags
    
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
    if (leftMotorReverse){
            targetYaw = fmod(yaw - targetAngle + 360.0, 360.0);
    }else{
         targetYaw = fmod(yaw + targetAngle + 360.0, 360.0);
    }
   
    
    // Set motor directions and speeds
    this->process_motorDirections("left");
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
    if (leftMotorReverse){
            targetYaw = fmod(yaw + targetAngle + 360.0, 360.0);
    }else{
         targetYaw = fmod(yaw - targetAngle + 360.0, 360.0);
    }
    
    // Set motor directions and speeds
    // For right rotation, left motor forward, right motor backward
    this->process_motorDirections("right");
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
    const unsigned long ROTATION_TIMEOUT = 300000;  // 5 second timeout
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
    if (angleError > 180) angleError -= 180;
    if (angleError < -180) angleError += 180;
    
    // Check if we're within tolerance and log the error for debugging
    bool isComplete = abs(angleError) <= tolerance;
    if (!isComplete) {
        Serial.print("Angle error: ");
        Serial.print(angleError);
        Serial.print("  Target yaw: ");
        Serial.print(targetYaw);
        Serial.print("  Current yaw: ");
        Serial.println(yaw);
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

void RN_Sport::kickForward() {
    // Set kick motor direction and speed
    motorKick.run(FORWARD);
    motorKick.setSpeed(kickSpeed);
}

void RN_Sport::kickBackward() {
    // Set kick motor direction and speed
    motorKick.run(BACKWARD);
    motorKick.setSpeed(kickSpeed);
}

void RN_Sport::stopKick() {
    // Stop kick motor
    motorKick.run(RELEASE);
    motorKick.setSpeed(0);
}

void RN_Sport::setServoSpeed(int speed) {
    // Constrain speed to valid range (0-255)
    servoSpeed = constrain(speed, 0, 255);
    Serial.print("Servo speed set to: ");
    Serial.println(servoSpeed);
}

void RN_Sport::setTopServoAngle(int angle) {
    if (!topServoInitialized) {
        Serial.println("Top servo not initialized!");
        return;
    }
    
    // Constrain angle to valid range
    angle = constrain(angle, topServoMin, topServoMax);
    
    // Calculate delay based on speed (faster speed = shorter delay)
    int delayTime = map(servoSpeed, 0, 255, 50, 5);  // 50ms at slowest, 5ms at fastest
    
    // Move servo gradually to target angle
    while (currentTopAngle != angle) {
        if (currentTopAngle < angle) {
            currentTopAngle++;
        } else {
            currentTopAngle--;
        }
        topServo.write(currentTopAngle);
        delay(delayTime);
    }
}

void RN_Sport::setLowServoAngle(int angle) {
    if (!lowServoInitialized) {
        Serial.println("Low servo not initialized!");
        return;
    }
    
    // Constrain angle to valid range
    angle = constrain(angle, lowServoMin, lowServoMax);
    
    // Calculate delay based on speed (faster speed = shorter delay)
    int delayTime = map(servoSpeed, 0, 255, 50, 5);  // 50ms at slowest, 5ms at fastest
    
    // Move servo gradually to target angle
    while (currentLowAngle != angle) {
        if (currentLowAngle < angle) {
            currentLowAngle++;
        } else {
            currentLowAngle--;
        }
        lowServo.write(currentLowAngle);
        delay(delayTime);
    }
}

void RN_Sport::initializeTopServo(int pin, int defaultAngle, int minAngle, int maxAngle) {
    // Attach top servo to its pin
    topServo.attach(pin);
    
    // Store angle limits
    topServoMin = constrain(minAngle, 0, 180);
    topServoMax = constrain(maxAngle, 0, 180);
    
    // Ensure min is less than max
    if (topServoMin > topServoMax) {
        int temp = topServoMin;
        topServoMin = topServoMax;
        topServoMax = temp;
    }
    
    // Store and set default angle
    topServoDefault = constrain(defaultAngle, topServoMin, topServoMax);
    currentTopAngle = topServoDefault;
    topServo.write(topServoDefault);
    
    topServoInitialized = true;
    Serial.print("Top servo initialized - Pin: ");
    Serial.print(pin);
    Serial.print(", Default: ");
    Serial.print(topServoDefault);
    Serial.print(", Range: ");
    Serial.print(topServoMin);
    Serial.print("-");
    Serial.println(topServoMax);
}

void RN_Sport::initializeLowServo(int pin, int defaultAngle, int minAngle, int maxAngle) {
    // Attach low servo to its pin
    lowServo.attach(pin);
    
    // Store angle limits
    lowServoMin = constrain(minAngle, 0, 180);
    lowServoMax = constrain(maxAngle, 0, 180);
    
    // Ensure min is less than max
    if (lowServoMin > lowServoMax) {
        int temp = lowServoMin;
        lowServoMin = lowServoMax;
        lowServoMax = temp;
    }
    
    // Store and set default angle
    lowServoDefault = constrain(defaultAngle, lowServoMin, lowServoMax);
    currentLowAngle = lowServoDefault;
    lowServo.write(lowServoDefault);
    
    lowServoInitialized = true;

}

void RN_Sport::resetServos() {
    if (topServoInitialized) {
        setTopServoAngle(topServoDefault);
    }
    if (lowServoInitialized) {
        setLowServoAngle(lowServoDefault);
    }
}

void RN_Sport::initializeDistanceSensor(int trigPin, int echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
    
    // Initialize pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    // Initialize state
    distanceSensorInitialized = true;
    errorState = false;
    errorCode = 0;
    
    Serial.println("Distance sensor initialized");
}

float RN_Sport::getDistance() {
    if (!distanceSensorInitialized) {
        Serial.println("Distance sensor not initialized!");
        return -1;
    }
    
    // Send trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure echo duration
    float duration = pulseIn(echoPin, HIGH);
    
    // Calculate distance using exact same formula as reference
    float distance = (duration * 0.0343) / 2;
    
    return distance;
}



