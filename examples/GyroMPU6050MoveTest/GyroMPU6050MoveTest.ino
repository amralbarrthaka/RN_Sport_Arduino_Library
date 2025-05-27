#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU6050 setup
Adafruit_MPU6050 mpu;

float yaw = 0.0;               // Estimated yaw angle in degrees
float gyroZOffset = 0.0;       // Gyroscope Z-axis bias
unsigned long lastMicros = 0;  // For precise timing
float lastGyroZ = 0.0;        // For drift compensation
float targetYaw = 0.0;        // Target heading to maintain

// Motor calibration
int leftMotorOffset = 0;   // Adjust this to compensate for motor differences
int rightMotorOffset = 0;  // Adjust this to compensate for motor differences
float Kp = 2.0;           // Proportional gain for angle correction

// Motor setup with Adafruit Motor Shield
AF_DCMotor motorRight(1); // Right motor on M1
AF_DCMotor motorLeft(2);  // Left motor on M2

// Direction and speed control
int baseSpeed = 150;      // Base motor speed
int leftSpeed = baseSpeed;
int rightSpeed = baseSpeed;
bool isForward = false;   // Direction flag
bool isBackward = false;  // Direction flag
bool isRotating = false;  // Rotation flag

// Timing variables
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100;  // Print every 100ms
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 10;  // Update every 10ms
unsigned long lastStateChange = 0;
unsigned long currentStateDuration = 0;
const unsigned long ROTATION_TIMEOUT = 3000;  // 3 seconds timeout for rotation

// Movement sequence states
enum MovementState {
    FORWARD_3S,
    STOP_1S,
    BACKWARD_0_5S,
    ROTATE_LEFT_180,
    ROTATE_RIGHT_180_FROM_LEFT,  // New state for timeout recovery
    FORWARD_3S_2,
    STOP_1S_2,
    BACKWARD_0_5S_2,
    ROTATE_RIGHT_180,
    ROTATE_LEFT_180_FROM_RIGHT   // New state for timeout recovery
};

MovementState currentState = FORWARD_3S;

// Calibrate gyro.z bias while sensor is stationary
void calibrateGyro() {
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

void setup() {
    Serial.begin(115200);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }

    Serial.println("MPU6050 found!");
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(1000); // Stabilize
    calibrateGyro();  // Important: Must be done on a stable surface
    lastMicros = micros();
    targetYaw = yaw;  // Set initial target heading
    
    // Initialize timing variables
    lastPrintTime = millis();
    lastUpdateTime = millis();
    lastStateChange = millis();
}

// Update yaw angle using MPU6050
void updateYaw() {
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

// Adjust motor speeds to maintain heading
void adjustMotorSpeeds() {
    if (isRotating) return; // Skip correction during rotation

    float angleError = yaw - targetYaw;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    int rawCorrection = Kp * angleError;

    // Find max possible correction without exceeding motor limits
    int maxCorrection = min(baseSpeed, 255 - baseSpeed);

    // Limit correction to maxCorrection
    int correction = constrain(rawCorrection, -maxCorrection, maxCorrection);
    
    // Apply correction based on direction
    if (isForward) {
        leftSpeed = baseSpeed - correction;
        rightSpeed = baseSpeed + correction;
    } else if (isBackward) {
        leftSpeed = baseSpeed + correction;
        rightSpeed = baseSpeed - correction;
    }

    // Constrain speeds to valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
}

// Movement functions
void moveForward() {
    targetYaw = yaw;
    isForward = true;
    isBackward = false;
    isRotating = false;
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
}

void moveBackward() {
    targetYaw = yaw;
    isForward = false;
    isBackward = true;
    isRotating = false;
    motorLeft.run(BACKWARD);
    motorRight.run(FORWARD);
}

void stopMotors() {
    isForward = false;
    isBackward = false;
    isRotating = false;
    motorLeft.run(RELEASE);
    motorRight.run(RELEASE);
}

void rotateLeft() {
    isForward = false;
    isBackward = false;
    isRotating = true;
    motorLeft.run(BACKWARD);
    motorRight.run(BACKWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
}

void rotateRight() {
    isForward = false;
    isBackward = false;
    isRotating = true;
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
}

// Print status information
void printStatus() {
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

void updateMovementState() {
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
            if (abs(yaw - targetYaw) < 5.0) {  // Within 5 degrees of target
                moveForward();
                currentState = FORWARD_3S_2;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                // If rotation takes too long, try rotating right instead
                rotateRight();
                currentState = ROTATE_RIGHT_180_FROM_LEFT;
                lastStateChange = currentTime;
                targetYaw = fmod(yaw + 180.0, 360.0);
            }
            break;

        case ROTATE_RIGHT_180_FROM_LEFT:
            if (abs(yaw - targetYaw) < 5.0) {  // Within 5 degrees of target
                moveForward();
                currentState = FORWARD_3S_2;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                // If still stuck, try moving forward anyway
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
            if (abs(yaw - targetYaw) < 5.0) {  // Within 5 degrees of target
                moveForward();
                currentState = FORWARD_3S;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                // If rotation takes too long, try rotating left instead
                rotateLeft();
                currentState = ROTATE_LEFT_180_FROM_RIGHT;
                lastStateChange = currentTime;
                targetYaw = fmod(yaw + 180.0, 360.0);
            }
            break;

        case ROTATE_LEFT_180_FROM_RIGHT:
            if (abs(yaw - targetYaw) < 5.0) {  // Within 5 degrees of target
                moveForward();
                currentState = FORWARD_3S;
                lastStateChange = currentTime;
            } else if (elapsedTime >= ROTATION_TIMEOUT) {
                // If still stuck, try moving forward anyway
                moveForward();
                currentState = FORWARD_3S;
                lastStateChange = currentTime;
            }
            break;
    }
}

void loop() {
    unsigned long currentTime = millis();

    // Update yaw and motor speeds at regular intervals
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = currentTime;
        updateYaw();
        adjustMotorSpeeds();
    }

    // Update movement state
    updateMovementState();

    // Print status at regular intervals
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
        lastPrintTime = currentTime;
        printStatus();
    }
}
 
