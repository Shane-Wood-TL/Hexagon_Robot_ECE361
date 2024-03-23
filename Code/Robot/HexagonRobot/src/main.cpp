#include <Arduino.h>
#include <ESP32Servo.h>

Servo motorA;
Servo motorC;
Servo motorB;

void setup() {
  pinMode(6, OUTPUT); //A0
  pinMode(36, OUTPUT); //A1

  pinMode(41, OUTPUT); //A0
  pinMode(42, OUTPUT); //A1

  pinMode(1, OUTPUT); //A0
  pinMode(2, OUTPUT); //A1
  //pinMode(15, OUTPUT); //enA
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  motorA.attach(15);
  motorB.attach(16);
  motorC.attach(7);
  motorA.setPeriodHertz(500);
  motorB.setPeriodHertz(500);
  motorC.setPeriodHertz(500);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(6, LOW);
  digitalWrite(36,HIGH);
  motorA.writeMicroseconds(2000);

   digitalWrite(41, LOW);
  digitalWrite(42,HIGH);
  motorB.writeMicroseconds(2000);

   digitalWrite(1, LOW);
  digitalWrite(2,HIGH);
  motorC.writeMicroseconds(2000);
  //~790 lowest
  //2000 (100%) = max
  //speed barely changes
}
