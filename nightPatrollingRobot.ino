#include <Servo.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <GSM.h>

#define Echo A0
#define Trig A1
#define motor 10
#define Speed 170
#define spoint 103
#define SOUND_PIN A2

char value;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);
SoftwareSerial SIM900(7, 8); // RX, TX for GSM module
SoftwareSerial bluetooth(2, 3); // RX, TX for Bluetooth module

#define TRIGGER_PIN  9
#define ECHO_PIN     10
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
  pinMode(Echo, OUTPUT);
  pinMode(Trig, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
  SIM900.begin(9600);  
  bluetooth.begin(9600); 
}

void loop() {
  detectSound();
  Bluetoothcontrol();
  pathFollowing();
}

void detectSound() {
  int soundValue = analogRead(SOUND_PIN);
  if (soundValue > 500) { 
    sendSMS("Loud noise detected! Please check.");
  }
}

void Bluetoothcontrol() {
  if (bluetooth.available() > 0) {
    value = bluetooth.read();
    Serial.println(value);
    if (value == 'F') {
      forward();
    } else if (value == 'B') {
      backward();
    } else if (value == 'L') {
      left();
    } else if (value == 'R') {
      right();
    } else if (value == 'S') {
      Stop();
    }
  }
}

void sendSMS(String message) {
  Serial.println("Sending SMS...");
  SIM900.print("AT+CMGF=1\r"); 
  delay(1000);
  SIM900.println("AT + CMGS = \"+1234567890\""); 
  delay(1000);
  SIM900.println(message); 
  SIM900.println((char)26); 
  delay(1000);
  SIM900.println();
}

void pathFollowing() {
  int distance = sonar.ping_cm();
  if (distance <= 10) { 
    stopMotors();
    delay(500);
    reverse();
    delay(1000);
    int randomTurn = random(2); 
    if (randomTurn == 0) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(1000); 
    moveForward(); 
  } else { 
    moveForward();
  }
}

void moveForward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void right() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void left() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

void turnLeft() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
  delay(500); 
  stopMotors();
}

void turnRight() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
  delay(500); 
  stopMotors();
}
