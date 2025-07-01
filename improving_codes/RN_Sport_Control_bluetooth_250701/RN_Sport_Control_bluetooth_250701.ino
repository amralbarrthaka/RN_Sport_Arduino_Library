#include <RN_Sport.h>

#include <QueueArray.h>

QueueArray<String> commandQueue(10);

SoftwareSerial Bluetooth(10, 8); // Arduino (RX, TX) - HC-05 Bluetooth (TX, RX)

// Create RN_Sport instance with default motor pins and base speed
RN_Sport robot;


int Speed = 200; // Initial speed
int Speed_C = 255; //rotation

String direction = "d,s";

// Servo pins
const int TOP_SERVO_PIN = 2;  // Top guide vane servo pin
const int LOW_SERVO_PIN = 7;  // Low guide vane servo pin

// Servo configuration
const int TOP_SERVO_DEFAULT = 40;  // Center position
const int TOP_SERVO_MIN = 40;      // Minimum angle
const int TOP_SERVO_MAX = 75;      // Maximum angle

const int LOW_SERVO_DEFAULT = 30;  // Center position
const int LOW_SERVO_MIN = 20;      // Minimum angle
const int LOW_SERVO_MAX = 180;     // Maximum angle

// Servo speed control
const int SERVO_SPEED = 220;    // Servo speed 0 to 255

int TOP_SERVO_CurrentAngle = TOP_SERVO_DEFAULT;
int LOW_SERVO_CurrentAngle = LOW_SERVO_DEFAULT;

String mode = "m";
int auto_program = 0;



#define BUFFER_SIZE 10
String commandBuffer[BUFFER_SIZE];
int bufferStart = 0;
int bufferEnd = 0;
bool bufferFull = false;
//String mode = "a"
void enqueueCommand(String command) {
    commandBuffer[bufferEnd] = command;
    bufferEnd = (bufferEnd + 1) % BUFFER_SIZE;
    if (bufferFull) {
        bufferStart = (bufferStart + 1) % BUFFER_SIZE;
    }
    bufferFull = bufferEnd == bufferStart;
}

String dequeueCommand() {
    if (bufferStart == bufferEnd && !bufferFull) {
        return ""; // Buffer is empty
    }
    String command = commandBuffer[bufferStart];
    bufferStart = (bufferStart + 1) % BUFFER_SIZE;
    bufferFull = false;
    return command;
}


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
    Bluetooth.begin(9600);

    
    // Initialize gyroscope
    robot.initializeGyro();
    delay(500);
    
    // Set movement speed (0-255)
    robot.setMovementSpeed(Speed);
    robot.setKickSpeed(Speed_C);

    // Set servo speed BEFORE initializing servos
    robot.setServoSpeed(SERVO_SPEED);

    // Initialize servos with all parameters
    robot.initializeTopServo(TOP_SERVO_PIN, TOP_SERVO_DEFAULT, TOP_SERVO_MIN, TOP_SERVO_MAX);
    delay(500);  // Short delay between initializations
    robot.initializeLowServo(LOW_SERVO_PIN, LOW_SERVO_DEFAULT, LOW_SERVO_MIN, LOW_SERVO_MAX);
    delay(1000);  // Wait for servos to initialize
    Serial.println("RN_Sport initialized successfully!");
}

void loop() {
    // Check for available Bluetooth data and add to command buffer
    if (Bluetooth.available()) {
        String command = Bluetooth.readStringUntil('\n');
        Serial.print(command);
        enqueueCommand(command);
    }

    // Process commands from the buffer
    String command = dequeueCommand();
    if (command != "") {
        command.trim();
        Serial.print("Received: ");
        Serial.println(command);

        if (command.startsWith("d,")) {
            direction = command;
            Serial.print("Setting direction to ");
            Serial.println(direction);
        }else if (command.startsWith("mo,")) {
            mode = command.substring(3);
            Serial.print("Setting mode to ");
            Serial.println(mode);
            if(mode == "m"){
              auto_program = 0;
            }
        }else if (command.startsWith("pr,")) {
            auto_program = command.substring(3).toInt();
            Serial.print("Setting program to ");
            Serial.println(auto_program);
        } else if (command.startsWith("s1,")) {
            int speedValue = command.substring(3).toInt();
            if (speedValue >= 0 && speedValue <= 100) {
              Speed = map(speedValue, 0, 100, 0, 255);
              robot.setMovementSpeed(Speed);
            } else {
                Serial.println("Invalid speed value");
            }
        } else if (command.startsWith("s2,")) {
            int speedValue = command.substring(3).toInt();
            if (speedValue >= 0 && speedValue <= 100) {
                Speed_C = map(speedValue, 0, 100, 0, 255);
                robot.setKickSpeed(Speed_C);
                // delay(50);
            } else {
                Serial.println("Invalid speed value");
            }
        } else if (command.startsWith("t1,")) {
            int targetValue1 = command.substring(3).toInt();
            if (targetValue1 >= 0 && targetValue1 <= 100) {
                TOP_SERVO_CurrentAngle = map(targetValue1, 0, 100, TOP_SERVO_MIN, TOP_SERVO_MAX);
                Serial.print("Setting target1 to ");
                Serial.println(TOP_SERVO_CurrentAngle);
                robot.setTopServoAngle(TOP_SERVO_CurrentAngle);
            } else {
                Serial.println("Invalid target1 value");
            }
        } else if (command.startsWith("t2,")) {
            int targetValue2 = command.substring(3).toInt();
            if (targetValue2 >= 0 && targetValue2 <= 100) {
                LOW_SERVO_CurrentAngle = map(targetValue2, 0, 100, LOW_SERVO_MIN, LOW_SERVO_MAX);
                Serial.print("Setting target2 to ");
                Serial.println(LOW_SERVO_CurrentAngle);
                robot.setLowServoAngle(LOW_SERVO_CurrentAngle);
            } else {
                Serial.println("Invalid target2 value");
            }
        } else if (command.startsWith("k1,")) {
            String commandbtn = command.substring(3);
            if (commandbtn == "k") {
              Serial.println("kick");
              robot.kickForward();
            } else {
              Serial.println("stop_kick");
              robot.stopKick();
            }
        } else {
            Serial.println("Invalid command");
        }
    }

    // Existing direction handling code...
    if (direction == "d,f") {
        robot.moveForwardWithGyro();
        Serial.println("Forward");
    } else if (direction == "d,b") {
        robot.moveBackwardWithGyro();
        Serial.println("Backward");
    } else if (direction == "d,l") {
        robot.rotateLeftWithGyro(90);
        Serial.println("Left");
    } else if (direction == "d,r") {
        robot.rotateRightWithGyro(90);
        Serial.println("Right");
    } else if (direction == "d,fl") {
        robot.rotateLeftWithGyro(10);
        Serial.println("fl");
    } else if (direction == "d,fr") {
        robot.rotateRightWithGyro(10);
        Serial.println("fr");
    } else if (direction == "d,bl") {
        robot.rotateRightWithGyro(10);
        Serial.println("bl");
    } else if (direction == "d,br") {
        robot.rotateLeftWithGyro(10);
        Serial.println("br");
    } else if (direction == "d,s") {
        robot.stopMotors();
    }

}
