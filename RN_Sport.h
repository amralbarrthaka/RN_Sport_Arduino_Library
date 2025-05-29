#ifndef RN_SPORT_H
#define RN_SPORT_H

#include <Arduino.h>
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "HUSKYLENS.h"
#include <Servo.h>

class RN_Sport {
public:
    // Direction enum
    enum MovementDirection {
        DIR_STOP,
        DIR_FORWARD,
        DIR_BACKWARD,
        DIR_LEFT,
        DIR_RIGHT
    };

    // Constructor
    RN_Sport(int leftMotorPin = 2, int rightMotorPin = 1, int kickMotorPin = 3, int baseSpeed = 150);

    // Setup and initialization
    bool begin();
    bool beginCamera();  // New function for camera initialization

    // MPU6050 Configuration
    void setGyroRange(mpu6050_gyro_range_t range);
    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);

    // Camera functions
    bool isObjectDetected();
    int getObjectX();
    int getObjectY();
    int getObjectWidth();
    int getObjectHeight();
    bool isColorLearned();
    void setCameraAlgorithm(int algorithm);

    // I2C Scanner functions
    void scanI2CDevices();
    bool isI2CDevicePresent(byte address);
    void printI2CDeviceInfo(byte address);

    // Basic Movement functions
    void moveForward();
    void moveBackward();
    void stopMotors();
    void rotateLeft();
    void rotateRight();
    void kickForward();
    void kickBackward();
    void stopKick();
    void setKickSpeed(int speed);

    void initializeGyro();
    void updateGyro();

    void moveForwardWithGyro();
    void moveBackwardWithGyro();
    void printGyroMotorStatus();

    void rotateLeftWithGyro(float targetAngle);
    void rotateRightWithGyro(float targetAngle);
   
    void setMovementSpeed(int speed);

    bool checkRotationComplete(float tolerance);
    bool isRotating;
    bool handleRotation(float angle, MovementDirection direction);

    // Movement sequence
    void startMovementSequence();

    // Getter functions
    float getYaw() { return yaw; }
    float getTargetYaw() { return targetYaw; }
    int getLeftSpeed() { return leftSpeed; }
    int getRightSpeed() { return rightSpeed; }
    MovementDirection getCurrentDirection() { return currentDirection; }

    // Servo functions
    void initializeTopServo(int pin, int defaultAngle, int minAngle, int maxAngle);
    void initializeLowServo(int pin, int defaultAngle, int minAngle, int maxAngle);
    void setTopServoAngle(int angle);
    void setLowServoAngle(int angle);
    void resetServos();
    void setServoSpeed(int speed);  // Set speed for both servos (0-255)
    bool isTopServoInitialized() { return topServoInitialized; }
    bool isLowServoInitialized() { return lowServoInitialized; }

private:
    // MPU6050 setup
    Adafruit_MPU6050 mpu;
    float yaw;
    float correctionValueGyro;
    unsigned long lastMicros;
    float lastGyroZ;
    float targetYaw;
    float totalCorrectionValueGyro;

    // Camera setup
    HUSKYLENS huskylens;
    bool cameraInitialized;
    HUSKYLENSResult lastResult;

    // Motor setup
    AF_DCMotor motorRight;
    AF_DCMotor motorLeft;
    AF_DCMotor motorKick;

    // Speed and direction control
    int baseSpeed;
    int leftSpeed;
    int rightSpeed;
    int kickSpeed;
    float Kp;
    bool isForward;
    bool isBackward;
    MovementDirection currentDirection;  // Current movement direction

    // Timing variables
    unsigned long lastPrintTime;
    const unsigned long PRINT_INTERVAL = 100;
    unsigned long lastUpdateTime;
    const unsigned long UPDATE_INTERVAL = 10;
    unsigned long lastStateChange;

    // Helper functions
    void adjustMotorSpeeds();
    bool updateCameraData();  // Helper function for camera
    void correctGyro();  // Private calibration function
    bool directChange();  // Check if direction has changed

    // Servo setup
    Servo topServo;
    Servo lowServo;
    bool topServoInitialized;
    bool lowServoInitialized;
    int topServoDefault;
    int lowServoDefault;
    int topServoMin;
    int topServoMax;
    int lowServoMin;
    int lowServoMax;
    int servoSpeed;  // Speed for both servos
    unsigned long lastTopServoMove;
    unsigned long lastLowServoMove;
    int currentTopAngle;
    int currentLowAngle;
};

#endif 