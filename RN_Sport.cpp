#include "RN_Sport.h"

RN_Sport::RN_Sport(int leftMotorPin, int rightMotorPin, int baseSpeed) 
    : motorRight(rightMotorPin), motorLeft(leftMotorPin), baseSpeed(baseSpeed) {
    // Initialize member variables
    yaw = 0.0;
    gyroZOffset = 0.0;
    lastMicros = 0;
    lastGyroZ = 0.0;
    targetYaw = 0.0;
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
    Kp = 2.0;
    isForward = false;
    isBackward = false;
    isRotating = false;
    lastPrintTime = 0;
    lastUpdateTime = 0;
    lastStateChange = 0;
    currentState = FORWARD_3S;
}

bool RN_Sport::begin() {
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);

    if (!mpu.begin()) {
        return false;
    }

    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(1000); // Stabilize
    calibrateGyro();
    lastMicros = micros();
    targetYaw = yaw;
    
    lastPrintTime = millis();
    lastUpdateTime = millis();
    lastStateChange = millis();
    
    return true;
}

void RN_Sport::setGyroRange(mpu6050_gyro_range_t range) {
    mpu.setGyroRange(range);
}

void RN_Sport::setFilterBandwidth(mpu6050_bandwidth_t bandwidth) {
    mpu.setFilterBandwidth(bandwidth);
}

void RN_Sport::calibrateGyro() {
    Serial.println("Calibrating gyroscope (keep sensor still)...");
    float sum = 0.0;
    const int samples = 500;  // Increased samples for better calibration

    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;
        delay(5);
    }

    gyroZOffset = sum / samples;
    Serial.print("Gyro Z offset: ");
    Serial.println(gyroZOffset, 6);
}

void RN_Sport::updateYaw() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - lastMicros) / 1000000.0; // convert to seconds
    lastMicros = currentMicros;

    // Get gyro reading and apply offset
    float gyroZ = g.gyro.z - gyroZOffset;

    // Drift compensation
    if (abs(gyroZ) < 0.001) {  // If rotation is very small
        gyroZ = 0;  // Ignore tiny movements
    }

    // Integrate gyro.z to estimate yaw
    yaw += gyroZ * (180.0 / PI) * deltaTime;

    // Normalize yaw to [0, 360)
    if (yaw >= 360.0) yaw -= 360.0;
    if (yaw < 0.0) yaw += 360.0;
}

void RN_Sport::adjustMotorSpeeds() {
    if (isRotating) return;

    float angleError = yaw - targetYaw;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    int rawCorrection = Kp * angleError;
    int maxCorrection = min(baseSpeed, 255 - baseSpeed);
    int correction = constrain(rawCorrection, -maxCorrection, maxCorrection);
    
    if (isForward) {
        leftSpeed = baseSpeed - correction;
        rightSpeed = baseSpeed + correction;
    } else if (isBackward) {
        leftSpeed = baseSpeed + correction;
        rightSpeed = baseSpeed - correction;
    }

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
}

void RN_Sport::moveForward() {
    targetYaw = yaw;
    isForward = true;
    isBackward = false;
    isRotating = false;
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
}

void RN_Sport::moveBackward() {
    targetYaw = yaw;
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

void RN_Sport::startMovementSequence() {
    currentState = FORWARD_3S;
    lastStateChange = millis();
    moveForward();
}

void RN_Sport::updateMovementState() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastStateChange;

    switch(currentState) {
        case FORWARD_3S:
            if (elapsedTime >= 3000) {
                stopMotors();
                currentState = STOP_1S;
                lastStateChange = currentTime;
            }
            break;

        case STOP_1S:
            if (elapsedTime >= 1000) {
                moveBackward();
                currentState = BACKWARD_0_5S;
                lastStateChange = currentTime;
            }
            break;

        case BACKWARD_0_5S:
            if (elapsedTime >= 500) {
                rotateLeft();
                currentState = ROTATE_LEFT_180;
                lastStateChange = currentTime;
                targetYaw = fmod(yaw + 180.0, 360.0);
            }
            break;

        case ROTATE_LEFT_180:
            if (abs(yaw - targetYaw) < 5.0) {
                moveForward();
                currentState = FORWARD_3S_2;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                rotateRight();
                currentState = ROTATE_RIGHT_180_FROM_LEFT;
                lastStateChange = currentTime;
                targetYaw = fmod(yaw + 180.0, 360.0);
            }
            break;

        case ROTATE_RIGHT_180_FROM_LEFT:
            if (abs(yaw - targetYaw) < 5.0) {
                moveForward();
                currentState = FORWARD_3S_2;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                moveForward();
                currentState = FORWARD_3S_2;
                lastStateChange = currentTime;
            }
            break;

        case FORWARD_3S_2:
            if (elapsedTime >= 3000) {
                stopMotors();
                currentState = STOP_1S_2;
                lastStateChange = currentTime;
            }
            break;

        case STOP_1S_2:
            if (elapsedTime >= 1000) {
                moveBackward();
                currentState = BACKWARD_0_5S_2;
                lastStateChange = currentTime;
            }
            break;

        case BACKWARD_0_5S_2:
            if (elapsedTime >= 500) {
                rotateRight();
                currentState = ROTATE_RIGHT_180;
                lastStateChange = currentTime;
                targetYaw = fmod(yaw + 180.0, 360.0);
            }
            break;

        case ROTATE_RIGHT_180:
            if (abs(yaw - targetYaw) < 5.0) {
                moveForward();
                currentState = FORWARD_3S;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                rotateLeft();
                currentState = ROTATE_LEFT_180_FROM_RIGHT;
                lastStateChange = currentTime;
                targetYaw = fmod(yaw + 180.0, 360.0);
            }
            break;

        case ROTATE_LEFT_180_FROM_RIGHT:
            if (abs(yaw - targetYaw) < 5.0) {
                moveForward();
                currentState = FORWARD_3S;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                moveForward();
                currentState = FORWARD_3S;
                lastStateChange = currentTime;
            }
            break;
    }
}

void RN_Sport::printStatus() {
    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print("° Target: ");
    Serial.print(targetYaw, 2);
    Serial.print("° Left: ");
    Serial.print(leftSpeed);
    Serial.print(" Right: ");
    Serial.print(rightSpeed);
    Serial.print(" State: ");
    switch(currentState) {
        case FORWARD_3S: Serial.println("Forward 3s"); break;
        case STOP_1S: Serial.println("Stop 1s"); break;
        case BACKWARD_0_5S: Serial.println("Backward 0.5s"); break;
        case ROTATE_LEFT_180: Serial.println("Rotate Left 180°"); break;
        case ROTATE_RIGHT_180_FROM_LEFT: Serial.println("Rotate Right 180° (recovery)"); break;
        case FORWARD_3S_2: Serial.println("Forward 3s (2)"); break;
        case STOP_1S_2: Serial.println("Stop 1s (2)"); break;
        case BACKWARD_0_5S_2: Serial.println("Backward 0.5s (2)"); break;
        case ROTATE_RIGHT_180: Serial.println("Rotate Right 180°"); break;
        case ROTATE_LEFT_180_FROM_RIGHT: Serial.println("Rotate Left 180° (recovery)"); break;
    }
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

void RN_Sport::setBaseSpeed(int speed) {
    baseSpeed = constrain(speed, 0, 255);
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
}

void RN_Sport::setGyroGain(float gain) {
    Kp = gain;
}

void RN_Sport::moveForwardWithGyro() {
    targetYaw = yaw;  // Set target heading to current heading
    isForward = true;
    isBackward = false;
    isRotating = false;
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
    adjustMotorSpeeds();  // Apply initial correction
}

void RN_Sport::moveBackwardWithGyro() {
    targetYaw = yaw;  // Set target heading to current heading
    isForward = false;
    isBackward = true;
    isRotating = false;
    motorLeft.run(BACKWARD);
    motorRight.run(FORWARD);
    adjustMotorSpeeds();  // Apply initial correction
}

void RN_Sport::rotateLeftWithGyro(float targetAngle) {
    isForward = false;
    isBackward = false;
    isRotating = true;
    targetYaw = fmod(yaw + targetAngle, 360.0);  // Calculate target angle
    motorLeft.run(BACKWARD);
    motorRight.run(BACKWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
}

void RN_Sport::rotateRightWithGyro(float targetAngle) {
    isForward = false;
    isBackward = false;
    isRotating = true;
    targetYaw = fmod(yaw - targetAngle + 360.0, 360.0);  // Calculate target angle
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
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