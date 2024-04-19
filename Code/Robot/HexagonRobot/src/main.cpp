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

//#define printing

float bmotorOffset = 0.98;

float motorFreq = 500; 
float minOnTime = 790;
float maxOnTime = 2000;

float intialSpeedOfSound = 0.0343; //in cm/mirco second 

float v1,v2,v3; //Motor speeds
float spinDir;

void getData();



struct PayloadStruct {
  uint8_t mode;   //simple mode, basic int
  float speed; //a int centered at 127
  float angle; //a int centered at 127
  uint8_t spin = 127;   //a int centered at 127
  uint8_t eStop;  // bascially a bool
  uint8_t PID;
  uint8_t disable;
};




// //radio
PayloadStruct payload;
RF24 radio(CE,SCN);
bool newData = false;
int oldState = 0;
const byte thisSlaveAddress[5] = {'T','r','i','n','E'};


// //temperature sensor
Adafruit_BMP085 bmp;

// //display
LiquidCrystal_I2C lcd(0x3F,16,2);


//motor class instances
motor motorA(A0_, A1_, enA_,0); //Mtr1
motor motorB(B0_, B1_, enB_,3); //Mtr2
motor motorC(C0_, C1_, enC_,7); //Mtr3



//distance sensor instances
// DistanceSensor Dis0(D0,intialSpeedOfSound);
// DistanceSensor Dis1(D1,intialSpeedOfSound);
// DistanceSensor Dis2(D2,intialSpeedOfSound);
// DistanceSensor Dis3(D3,intialSpeedOfSound);
// DistanceSensor Dis4(D4,intialSpeedOfSound);
// DistanceSensor Dis5(D5,intialSpeedOfSound);

//distance array instance
//Distances(int A, int B,int C,int D,int E,int F, float speedIntial){
Distances sonarArray(D0,19,D2,D3,D4,D5,intialSpeedOfSound);




void setup() {
  //PWM for servo library (would like to use analog write if possible)

  Serial.begin(115200);

  //i2c setup
  Wire.begin(SDA,SCL); //SDA, SCL
  lcd.init();           
  lcd.backlight();
  lcd.clear();
  lcd.print("working");
  bmp.begin();


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

  pinMode(enA_, OUTPUT);
  pinMode(A0_, OUTPUT);
  pinMode(A1_, OUTPUT);

  pinMode(enB_, OUTPUT);
  pinMode(B0_, OUTPUT);
  pinMode(B1_, OUTPUT);

  pinMode(enC_, OUTPUT);
  pinMode(C0_, OUTPUT);
  pinMode(C1_, OUTPUT);
  
  ledcSetup(0,1000,8);
  ledcSetup(3,1000,8);
  ledcSetup(7,1000,8);


  ledcAttachPin(enA_, 0);
  ledcAttachPin(enB_, 3);
  ledcAttachPin(enC_, 7);
  Serial.println("setupDone");

}

void loop() {
  getData();

  //only look at the modes if not stopped
  if (payload.eStop != 1){
    //handle nomove + nospin
    switch (payload.disable){
      case 1:{
        payload.spin = 127;
        break;
      }
      case 2:{
        payload.speed = 0;
        break;
      }
      default:
        payload.speed = payload.speed;
        payload.spin = payload.spin;
        break;
    }

    switch(payload.mode){
      case 0:{ //user control mode
        //Dynamic direction control
        invKin(payload.speed, payload.angle, payload.spin, &v1, &v2, &v3);
        motorA.setSpeed(v1);
        motorB.setSpeed(v2*bmotorOffset);
        motorC.setSpeed(v3);
        break;
      }
      case 1:{ //wall following
        //find which sensor is closest to wall (B)
        //match the 2 (A,C)adjacent sensors in distance to have a edge parallel with the wall
        //drive forward keeping A and C equal distance and B fairly close (<30 cm)
        //if a wall is detected in front, switch that sensor to be A
        //if 2 walls are detected, pick a random one
        moveValues wallFollow;
        wallFollow = sonarArray.wallFollow();
        invKin(wallFollow.speed, wallFollow.angle, wallFollow.spin, &v1, &v2, &v3);
        motorA.setSpeed(v1);
        motorB.setSpeed(v2*bmotorOffset);
        motorC.setSpeed(v3);
        if (wallFollow.spin == 255){
          delay(200);
          invKin(0,0,127, &v1, &v2, &v3);
          motorA.setSpeed(v1);
          motorB.setSpeed(v2*bmotorOffset);
          motorC.setSpeed(v3);
        }else{
          //set moving to a set distance (rather than being based on distance sensors)
          delay(400);
          //spin to account for robot not moving straight
          invKin(0,0,255, &v1, &v2, &v3);
          motorA.setSpeed(v1);
          motorB.setSpeed(v2*bmotorOffset);
          motorC.setSpeed(v3);
          delay(150);
          invKin(0,0,127, &v1, &v2, &v3);
          motorA.setSpeed(v1);
          motorB.setSpeed(v2*bmotorOffset);
          motorC.setSpeed(v3);
        }
        break;
      }
      case 2:{ //line following
        //use pid to increase speed and reduce the bouncing from one extreme to the other
        //has no ability to detect intersections (tmk) so it can only follow line paths
        int L0Value = digitalRead(L0); //Right
        int L1Value = digitalRead(L1); //Left
        moveValues LineSensor;
        LineSensor = lineFollowing(L1Value, L0Value);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print((int)L1Value);

      
        lcd.setCursor(10,0);
        lcd.print((int)L0Value);


        //move with line follower
        invKin(LineSensor.speed,LineSensor.angle,LineSensor.spin,&v1,&v2,&v3);
        motorA.setSpeed(v1);
        motorB.setSpeed(v2*bmotorOffset);
        motorC.setSpeed(v3);

        //set timing for motors being on
        if(LineSensor.speed == 0){
          delay(150);
          invKin(0,0,127, &v1, &v2, &v3);
          motorA.setSpeed(v1);
          motorB.setSpeed(v2*bmotorOffset);
          motorC.setSpeed(v3);
        }else{
          delay(15);
          invKin(0,0,127, &v1, &v2, &v3);
          motorA.setSpeed(v1);
          motorB.setSpeed(v2*bmotorOffset);
          motorC.setSpeed(v3);
        }
        break;
      }
    }
  }else{
    motorA.brake();
    motorB.brake();
    motorC.brake();
    // switch (payload.mode){
    // case(0):{
    //   lcd.clear();
    //   lcd.setCursor(0,0);
    //   lcd.print("User");
    //   break;
    // }
    // case(1):{
    //   moveValues wallFollow;
    //   wallFollow = sonarArray.wallFollow();
    //   break;
    // }
    // case(2):{
    //  int L0Value = digitalRead(L0); //Right
    //   int L1Value = digitalRead(L1); //Left 
    //   lcd.clear();
    //   lcd.setCursor(0,0);
    //   lcd.print((int)L1Value);
     
    //   lcd.setCursor(10,0);
    //   lcd.print((int)L0Value);
    //   break;
    // }
    // }
  }

}


//gets data from radio, checks if data was recieved
void getData(){
   if (radio.available()) {
    radio.read(&payload, sizeof(payload));
    newData = true;
  }
}

//sets the motor speeds and direction
void invKin(float speed, float angle, int spin, float* v1, float* v2, float* v3)
{
  float spinMod = map(spin, 0,255, -127,127);
  float v1T = speed*sin(decrad(330-angle));
  float v2T = speed*sin(decrad(210-angle));
  float v3T = speed*sin(decrad(90-angle));
  *v1 = v1T+spinMod;
  *v2 = v2T+spinMod;
  *v3 = v3T+spinMod;
}







































