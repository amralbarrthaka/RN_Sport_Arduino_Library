#ifndef RN_SPORT_H
#define RN_SPORT_H

#include <Arduino.h>
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "HUSKYLENS.h"

class RN_Sport {
public:
    // Constructor
    RN_Sport(int leftMotorPin = 2, int rightMotorPin = 1, int baseSpeed = 150);

    // Setup and initialization
    bool begin();
    void calibrateGyro();
    bool beginCamera();  // New function for camera initialization

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

    // Movement functions with gyro control
    void moveForwardWithGyro();
    void moveBackwardWithGyro();
    void rotateLeftWithGyro(float targetAngle);
    void rotateRightWithGyro(float targetAngle);
    void setBaseSpeed(int speed);
    void setGyroGain(float gain);

    // Movement sequence
    void startMovementSequence();
    void updateMovementState();
    void printStatus();

    // Getter functions
    float getYaw() { return yaw; }
    float getTargetYaw() { return targetYaw; }
    int getLeftSpeed() { return leftSpeed; }
    int getRightSpeed() { return rightSpeed; }

    // MPU6050 Configuration
    void setGyroRange(mpu6050_gyro_range_t range);
    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);
    
    // Yaw update function
    void updateYaw();

private:
    // MPU6050 setup
    Adafruit_MPU6050 mpu;
    float yaw;
    float gyroZOffset;
    unsigned long lastMicros;
    float lastGyroZ;
    float targetYaw;

    // Camera setup
    HUSKYLENS huskylens;
    bool cameraInitialized;
    HUSKYLENSResult lastResult;

    // Motor setup
    AF_DCMotor motorRight;
    AF_DCMotor motorLeft;

    // Speed and direction control
    int baseSpeed;
    int leftSpeed;
    int rightSpeed;
    float Kp;
    bool isForward;
    bool isBackward;
    bool isRotating;

    // Timing variables
    unsigned long lastPrintTime;
    const unsigned long PRINT_INTERVAL = 100;
    unsigned long lastUpdateTime;
    const unsigned long UPDATE_INTERVAL = 10;
    unsigned long lastStateChange;
    const unsigned long ROTATION_TIMEOUT = 3000;

    // Movement states
    enum MovementState {
        FORWARD_3S,
        STOP_1S,
        BACKWARD_0_5S,
        ROTATE_LEFT_180,
        ROTATE_RIGHT_180_FROM_LEFT,
        FORWARD_3S_2,
        STOP_1S_2,
        BACKWARD_0_5S_2,
        ROTATE_RIGHT_180,
        ROTATE_LEFT_180_FROM_RIGHT
    };

    MovementState currentState;

    // Helper functions
    void adjustMotorSpeeds();
    bool updateCameraData();  // New helper function for camera
};

#endif 