#include <Arduino.h>

//radio libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//line following library
#include <PID_v1.h> 

//wall following library
#include <math.h>

//temp + display libraries
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal_I2C.h>

//class defintions and constants
#include <constants.h>

float motorFreq = 500; 
float minOnTime = 790;
float maxOnTime = 2000;

float intialSpeedOfSound = 0.0343; //in cm/mirco second 

float v1,v2,v3; //Motor speeds
float spinDir;

void getData();



struct PayloadStruct {
  uint8_t mode; //sw2
  uint8_t speedX;
  uint8_t speedY;
  uint8_t spin;
  bool eStop;
};




//radio
PayloadStruct payload;
//RF24 radio(ce, csn)
RF24 radio(CE,SCN);
bool newData = false;
int oldState = 0;
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};


//temperature sensor
Adafruit_BMP085 bmp;

//display
LiquidCrystal_I2C lcd(0x27,20,4);


//motor class instances
motor motorA(A0, A1,enA,motorFreq,minOnTime,maxOnTime); //Mtr1
motor motorB(B0, B1,enB,motorFreq,minOnTime,maxOnTime); //Mtr2
motor motorC(C0, C1,enC,motorFreq,minOnTime,maxOnTime); //Mtr3



//distance sensor instances
DistanceSensor Dis0(D0,intialSpeedOfSound);
DistanceSensor Dis1(D1,intialSpeedOfSound);
DistanceSensor Dis2(D2,intialSpeedOfSound);
DistanceSensor Dis3(D3,intialSpeedOfSound);
DistanceSensor Dis4(D4,intialSpeedOfSound);
DistanceSensor Dis5(D5,intialSpeedOfSound);

//distance array instance
distances distanceArray(&Dis0, &Dis1, &Dis2, &Dis3, &Dis4, &Dis5, &bmp);





void setup() {
  //PWM for servo library (would like to use analog write if possible)
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);


  //i2c setup
  Wire.begin(SDA,SCL); //SDA, SCL
  lcd.init();           
  lcd.backlight();
  lcd.clear();
  lcd.print("working");


  //begin SPI
  SPI.begin(SCK, MISO, MOSI);
  
 
  // begin radio
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    lcd.print("no radio");
    while (1) {}  // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
  Serial.println("Radio Active");

  //line following pins
  pinMode(L0, INPUT);
  pinMode(L1, INPUT);
}

void loop() {
  getData();
  if (payload.eStop == true){
    motorA.brake();
    motorB.brake();
    motorC.brake();
    
  }
  switch(payload.mode){
    case 0:{ //user control mode
      invKin(payload.speedX,payload.speedY,v1,v2,v3)
      motorA.setSpeed(v1); 
      motorB.setSpeed(v2);
      motorC.setSpeed(v3);

      break;
    }
    case 1:{ //wall following
      //find which sensor is closest to wall (B)
      //match the 2 (A,C)adjacent sensors in distance to have a edge parallel with the wall
      //drive forward keeping A and C equal distance and B fairly close (<30 cm)
      //if a wall is detected in front, switch that sensor to be A
      //if 2 walls are detected, pick a random one
      break;
    }
    case 2:{ //line following
      //use pid to increase speed and reduce the bouncing from one extreme to the other
      //has no ability to detect intersections (tmk) so it can only follow line paths
      int L0Value = digitalRead(L0);
      int L1Value = digitalRead(L1);
      break;
    }
    case3:{ //Spin
          float sX = payload.speedX;
          float sY = payload.speedY;
          
          float positiveSpin = 1;
          float negativeSpin = -1;

          if(sX >= sY){
            if(sX > JOYSTICK_CENTER){
              spinDir = positiveSpin;
            }
            else if(sX < JOYSTICK_CENTER){
              spinDir = negativeSpin;
            }
            else if(sX == JOYSTICK_CENTER)
            {
              spinDir = 0;
            }
          }
          else if(sX < sY){
            if(sY > JOYSTICK_CENTER){
              spinDir = positiveSpin;
            }
            else if(sY < JOYSTICK_CENTER){
              spinDir = negativeSpin;
            }
            else if(sY == JOYSTICK_CENTER)
            {
              spinDir = 0;
            }
          }
          motorA.setSpeed(spinDir); 
          motorB.setSpeed(spinDir);
          motorC.setSpeed(spinDir);
    }
    default:{
      motorA.brake();
      motorB.brake();
      motorC.brake();
    }
  }
}


//gets data from radio, checks if data was recieved
void getData(){
   if (radio.available()) {
    radio.read(&payload, sizeof(payload));
    newData = true;
  //  }else{
  //   payload.eStop ==true; 
  }
}

//sets the motor speeds and direction
void invKin(uint8_t speedX,uint8_t speedY,float v1, float v2, float v3)
{
  float sX = speedX;
  float sY = speedY;

  v1 = ((-sqrt(3)/2) * speedX) - (0.5 * speedY);
  v2 = ((sqrt(3)/2) * speedX) - (0.5 * speedY);
  v3 = speedY;
}