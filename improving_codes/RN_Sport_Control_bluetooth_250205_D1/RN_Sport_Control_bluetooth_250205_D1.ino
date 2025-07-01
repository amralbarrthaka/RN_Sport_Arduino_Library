//5 6 3 11 
//2 7 
#include <SoftwareSerial.h>
#include <Servo.h>
#include <QueueArray.h>
#include <AFMotor.h>
#include "HUSKYLENS.h"

QueueArray<String> commandQueue(10);

AF_DCMotor motorA(1);
AF_DCMotor motorB(2);
AF_DCMotor motorC(3);


SoftwareSerial Bluetooth(10, 8); // Arduino (RX, TX) - HC-05 Bluetooth (TX, RX)

int Speed = 255; // Initial speed
int Speed_min = 80; 
int Speed_max = 255;
int calibrate_value_F = 45;//left and right calibration
int calibrate_value_B = 58;//left and right calibration

int Speed_C = 255; //rotation
int Speed_C_min = 80; 
int Speed_C_max = 255;


String direction = "d,s";

Servo myservo;
Servo myservo2;
int servoPin = 2;
int servoPin2 = 7;
int stepSize = 1;
int delayTime = 10;

int target_servo1 = 30;
int target_servo1_min = 40;
int target_servo1_max = 75;

int target_servo2 = 30;
int target_servo2_Kick = 30;
int target_servo2_min = 20;
int target_servo2_max = 180;

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


void setting_calibrate_F(){
  int Speed_A = Speed;
  int Speed_B = Speed;
if (calibrate_value_F >= 50){
  Speed_A = Speed * (100 - (2 * (calibrate_value_F - 50)))/100;
}else if (calibrate_value_F < 50){
  Speed_B = Speed * (100 - (2 * (50 - calibrate_value_F)))/100;
}
Serial.println(Speed_A );
Serial.println(Speed_B );
motorA.setSpeed(Speed_A);
motorB.setSpeed(Speed_B); 
delay(50);
}
void setting_calibrate_B(){
  int Speed_A = Speed;
  int Speed_B = Speed;
if (calibrate_value_B >= 50){
  Speed_A = Speed * (100 - (2 * (calibrate_value_B - 50)))/100;
}else if (calibrate_value_B < 50){
  Speed_B = Speed * (100 - (2 * (50 - calibrate_value_B)))/100;
}
Serial.println(Speed_A );
Serial.println(Speed_B );
motorA.setSpeed(Speed_A);
motorB.setSpeed(Speed_B); 
delay(50);
}


void backword() {
  setting_calibrate_F();
	motorA.run(FORWARD);
  motorB.run(FORWARD);
}

void forword() {
  setting_calibrate_B();
	motorA.run(BACKWARD);
  motorB.run(BACKWARD);
}

void turnLeft() {
	motorA.run(BACKWARD);
  motorB.run(FORWARD);
}

void turnRight() {
	motorA.run(FORWARD);
  motorB.run(BACKWARD);
}

void fRight() {
	motorB.run(BACKWARD);
}

void fLeft() {
  motorA.run(BACKWARD);
}

void bRight() {
  motorB.run(FORWARD);
}

void bLeft() {
  motorA.run(FORWARD);
}

void StopMove() {
  motorA.run(RELEASE);
  motorB.run(RELEASE);
  //motorA.run(BRAKE);
}

void kick() { 
  // moveToTarget_servo2(target_servo2_Kick);
  motorC.run(FORWARD);
}

void stop_Kick() { 
  motorC.run(RELEASE);
  motorC.run(BRAKE);
}

void moveToTarget_servo1(int target) {
    int currentPos = myservo.read();
    if (target > currentPos) {
        for (int d = currentPos; d <= target; d += stepSize) {
            myservo.write(d);
            delay(delayTime);
        }
    } else {
        for (int d = currentPos; d >= target; d -= stepSize) {
            myservo.write(d);
            delay(delayTime);
        }
    }
    delay(50); // Added delay to maintain the target position briefly
}

void moveToTarget_servo2(int target) {
    int currentPos = myservo2.read();
    if (target > currentPos) {
        for (int d = currentPos; d <= target; d += stepSize) { 
            myservo2.write(d);
            delay(delayTime);
        }
    } else {
        for (int d = currentPos; d >= target; d -= stepSize) {
            myservo2.write(d);
            delay(delayTime);
        }
    }
    delay(50); // Added delay to maintain the target position briefly
}

void setup() {
    Serial.begin(9600);
    Bluetooth.begin(9600); // Default baud rate of the Bluetooth module
    myservo.attach(servoPin);
    myservo2.attach(servoPin2);
    moveToTarget_servo1(target_servo1);
    moveToTarget_servo2(target_servo2);
    setting_calibrate_F();
    motorC.setSpeed(Speed_C);
    delay(1000);
}

void loop() {
    // Check for available Bluetooth data and add to command buffer
    if (Bluetooth.available()) {
        String command = Bluetooth.readStringUntil('\n');
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
                Speed = map(speedValue, 0, 100, Speed_min, Speed_max);
                Serial.print("Setting movement speed to ");
                Serial.println(Speed );
                setting_calibrate_F();
            } else {
                Serial.println("Invalid speed value");
            }
        } else if (command.startsWith("s2,")) {
            int speedValue = command.substring(3).toInt();
            if (speedValue >= 0 && speedValue <= 100) {
                Speed_C = map(speedValue, 0, 100, Speed_C_min, Speed_C_max);
                Serial.print("Setting rotation speed to ");
                Serial.println(Speed_C);
                motorC.setSpeed(Speed_C);
                delay(50);
            } else {
                Serial.println("Invalid speed value");
            }
        } else if (command.startsWith("c1,")) {
            calibrate_value_F = command.substring(3).toInt();
            if (calibrate_value_F >= 0 && calibrate_value_F <= 100) {
                setting_calibrate_F();
            } else {
                Serial.println("Invalid value");
            }
        }else if (command.startsWith("c2,")) {
            calibrate_value_B = command.substring(3).toInt();
            if (calibrate_value_F >= 0 && calibrate_value_F <= 100) {
                setting_calibrate_B();
            } else {
                Serial.println("Invalid value");
            }
        } else if (command.startsWith("t1,")) {
            int targetValue1 = command.substring(3).toInt();
            if (targetValue1 >= 0 && targetValue1 <= 100) {
                target_servo1 = map(targetValue1, 0, 100, target_servo1_min, target_servo1_max);
                Serial.print("Setting target1 to ");
                Serial.println(target_servo1);
                moveToTarget_servo1(target_servo1);
            } else {
                Serial.println("Invalid target1 value");
            }
        } else if (command.startsWith("t2,")) {
            int targetValue2 = command.substring(3).toInt();
            if (targetValue2 >= 0 && targetValue2 <= 100) {
                target_servo2 = map(targetValue2, 0, 100, target_servo2_min, target_servo2_max);
                Serial.print("Setting target2 to ");
                Serial.println(target_servo2);
                moveToTarget_servo2(target_servo2);
            } else {
                Serial.println("Invalid target2 value");
            }
        } else if (command.startsWith("k1,")) {
            String commandbtn = command.substring(3);
            if (commandbtn == "k") {
              Serial.println("kick");
              kick();
            } else {
              Serial.println("stop_kick");
              stop_Kick();
            }
        } else {
            Serial.println("Invalid command");
        }
    }

    // Existing direction handling code...
    if (direction == "d,f") {
        forword();
        Serial.println("Forward");
    } else if (direction == "d,b") {
        backword();
        Serial.println("Backward");
    } else if (direction == "d,l") {
        turnLeft();
        Serial.println("Left");
    } else if (direction == "d,r") {
        turnRight();
        Serial.println("Right");
    } else if (direction == "d,fl") {
        fLeft();
        Serial.println("fl");
    } else if (direction == "d,fr") {
        fRight();
        Serial.println("fr");
    } else if (direction == "d,bl") {
        bLeft();
        Serial.println("bl");
    } else if (direction == "d,br") {
        bRight();
        Serial.println("br");
    } else if (direction == "d,s") {
        StopMove();
    }
    if (mode = "a"){
      if (auto_program == 1){
        program_1();
      }else if (auto_program == 2){
        program_2();
      }else if (auto_program == 3){
        program_3();
      }else if (auto_program == 4){
        program_4();
      }
      
    }

}

void program_1() {

}

void program_2() {
moveToTarget_servo2(target_servo2_Kick); 
delay(500);
forword();
kick();
delay(1000);
StopMove();

delay(3000);
stop_Kick();
moveToTarget_servo2(target_servo2_max);
delay(500);
backword();
delay(1000);
StopMove();
delay(500);
}


void program_3() {

}
void program_4() {

}

