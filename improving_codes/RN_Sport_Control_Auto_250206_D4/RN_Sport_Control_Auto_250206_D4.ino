#include <Wire.h>
#include "HUSKYLENS.h"
#include <AFMotor.h>
#include <Servo.h>
 
// Ultrasonic sensor pins
const int trigPin = 22;
const int echoPin = 23;
 
// Motor setup with Adafruit Motor Shield
AF_DCMotor motorLeft(1);  // Left motor on M1
AF_DCMotor motorRight(2); // Right motor on M2
AF_DCMotor motorC(3);
 
Servo myservo;
Servo myservo2;
int servoPin = 2;
int servoPin2 = 7;
 
int target_servo1 = 40;
int target_servo1_min = 40;
int target_servo1_max = 75;
int stepSize = 1;
int delayTime = 10;

int target_servo2 = 30;
int target_servo2_Kick = 30;
int target_servo2_min = 20;
int target_servo2_max = 180;
 
HUSKYLENS huskylens;
bool colorFound = false;  // Flag to track whether the orange color has been found
bool track = false;
int Speed = 200; // Initial speed
long stoppedTime;
int moveCase = 1;
bool changeCondition = false;
long changeTime = 4000;
String conditionMove = "s";

void setup() {
    Serial.begin(115200);
    myservo.attach(servoPin);  
    myservo2.attach(servoPin2);
    motorLeft.setSpeed(Speed);
    motorRight.setSpeed(Speed);
    motorC.setSpeed(255);
 
    // Initialize ultrasonic sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
 
    Wire.begin();
    while (!huskylens.begin(Wire)) {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1. Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings >> Protocol Type >> I2C)"));
        Serial.println(F("2. Please recheck the connection."));
        delay(100);
    }
 
    // Switch to color recognition algorithm
    huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
 
  myservo.write(target_servo1);
  myservo2.write(target_servo2);


}
 
// Function to measure distance using the ultrasonic sensor
long getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
 
    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration * 0.034) / 2;
    return distance;
}
 
 
// speed
void moveMotorSpeed(int targetSpeed) {
    motorLeft.setSpeed(targetSpeed);
    motorRight.setSpeed(targetSpeed);
    //Serial.println("Speed");
}
// Motor control functions
void moveForward() {
    motorLeft.run(BACKWARD);
    motorRight.run(FORWARD);
    moveMotorSpeed(Speed);
    Serial.println("Moving forward");
}
 
void moveBackward() {
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
    moveMotorSpeed(Speed);
    Serial.println("Moving backward");
}
 
void stopMotors() {
    motorLeft.run(RELEASE);
    motorRight.run(RELEASE);
    Serial.println("Stopping motors");
}
 
void turnLeft() {
    motorLeft.run(BACKWARD);
    motorRight.run(BACKWARD);
    moveMotorSpeed(Speed);
    Serial.println("Turning left");
}
 
void turnRight() {
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
    moveMotorSpeed(Speed);
    Serial.println("Turning right");
}

void fRight() {
	motorRight.run(FORWARD);
}

void fLeft() {
  motorLeft.run(FORWARD);
}

void bRight() {
  motorRight.run(BACKWARD);
}

void bLeft() {
  motorLeft.run(BACKWARD);
}


bool isInside(int value, int min, int max) {
    return (value >= min && value <= max);
}
 
void kick() {
    //moveToTarget_servo2(target_servo2_Kick);
    myservo2.write(target_servo2_Kick);
    myservo2.write(target_servo1_min);
    motorC.run(FORWARD);
    Serial.println("kick");
}
void kick_reverse() {
    //moveToTarget_servo2(target_servo2_Kick);
    myservo2.write(target_servo2_Kick);
    myservo2.write(target_servo1_min);
    motorC.run(BACKWARD);
    Serial.println("kick");
}
 
void stop_Kick() {
    motorC.run(RELEASE);
    motorC.run(BRAKE);
}
 
 
 
void loop() {

  if (!colorFound && !track) {
      stoppedTime = millis(); 
  }
  if (millis() - changeTime > 5000) {
      changeCondition = true;
  }
  
  if(changeCondition){
      stopMotors();
      stop_Kick();
      if(moveCase == 1){
        kick();
        conditionMove = "f";
      }else if(moveCase == 3){
        conditionMove = "l";
      }else if(moveCase == 6){
        conditionMove = "r";
      }else if(moveCase == 4){
        kick_reverse();
        conditionMove = "b";
      }else if(moveCase == 2){
        conditionMove = "q";
      }else if(moveCase == 7){
        conditionMove = "w";
      }else if(moveCase == 5){
        conditionMove = "e";
      }else if(moveCase == 8){
        conditionMove = "t";
      }
      changeTime = millis();
      changeCondition = false;
      moveCase++;
      if(moveCase > 8){
        moveCase = 1;
      }
  }

  if (conditionMove == "f"){
    moveForward();
  }else if (conditionMove == "l"){
    turnLeft();
  }else if (conditionMove == "r"){
    turnRight();
  }else if (conditionMove == "b"){
    moveBackward();
  }else if (conditionMove == "q"){
    fRight();
  }else if (conditionMove == "w"){
    fRight();
  }else if (conditionMove == "e"){
    bRight();
  }else if (conditionMove == "t"){
    bLeft();
  }else{
    stopMotors();
  }

  if (!huskylens.request()) {
      Serial.println(F("Fail to request objects from HUSKYLENS!"));
  } else if (!huskylens.isLearned()) {
      Serial.println(F("Color not learned!"));
      stopMotors();  // Stop if color not learned
  } else if (!huskylens.available()) {
      Serial.println(F("Color disappeared!"));
      track = false;
        // Check if color has disappeared while moving forward
      if (colorFound && !track && !huskylens.available()) {
          Serial.println(F("Color disappeared while centered, starting search"));
          kick();
          int i = 0;
          while(i < 10){
            moveForward();
            delay(150);
            stopMotors();
            delay(50);
            i++;
          }
          stop_Kick();
          moveBackward();
          delay(100);
          turnLeft();
          delay(500);
          stopMotors();
          colorFound = false; // Reset colorFound flag
      }
  } else {
      HUSKYLENSResult result = huskylens.read();

      int xCenter = result.xCenter; // Get the xCenter of the detected color
      int xLeft = 160 - 50;         // Define left boundary for center region (a little narrower)
      int xRight = 160 + 50;        // Define right boundary for center region (a little narrower)

      // Orange color detected
      colorFound = true;
      track = true;

      if (xCenter < xLeft) {  // Color is on the left side, turn left
          turnLeft();
          Serial.println("Turning left to center the color");
      } else if (xCenter > xRight) {  // Color is on the right side, turn right
          turnRight();
          Serial.println("Turning right to center the color");
      } else {
          // Color is centered, move forward
          moveForward();
          Serial.println("Color is centered, moving forward");
      }
      printResult(result);  // Print the result for debugging
      delay(1);
  }  
}
 
// Function to print HUSKYLENS result for debugging
void printResult(HUSKYLENSResult result) {
    if (result.command == COMMAND_RETURN_BLOCK) {
        Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
    } else if (result.command == COMMAND_RETURN_ARROW) {
        Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
    } else {
        Serial.println("Object unknown!");
    }
}