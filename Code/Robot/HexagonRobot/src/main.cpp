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


//#define Printing



float motorFreq = 500; 
float minOnTime = 790;
float maxOnTime = 2000;

float intialSpeedOfSound = 0.0343; //in cm/mirco second 

float v1,v2,v3; //Motor speeds
float spinDir;

void getData();



struct PayloadStruct {
  uint8_t mode;   //simple mode, basic int
  float speed; 
  float angle;
  uint8_t spin;   //a int centered at 127
  bool eStop;  // bascially a bool
  bool PID;
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
motor motorA(A0_, A1_, enA_, 2); //Mtr1
motor motorB(B0_, B1_, enB_, 1); //Mtr2
motor motorC(C0_, C1_, enC_, 0); //Mtr3



//distance sensor instances
DistanceSensor Dis0(D0,intialSpeedOfSound);
DistanceSensor Dis1(D1,intialSpeedOfSound);
DistanceSensor Dis2(D2,intialSpeedOfSound);
DistanceSensor Dis3(D3,intialSpeedOfSound);
DistanceSensor Dis4(D4,intialSpeedOfSound);
DistanceSensor Dis5(D5,intialSpeedOfSound);

//distance array instance
//distances distanceArray(&Dis0, &Dis1, &Dis2, &Dis3, &Dis4, &Dis5, &bmp);




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


  // ledcSetup(motorA.channel, 1000, 8); // channel 0, 5000 Hz, 8-bit resolution
  // ledcSetup(motorB.channel, 1000, 8); // channel 0, 5000 Hz, 8-bit resolution
  // ledcSetup(motorC.channel, 1000, 8); // channel 0, 5000 Hz, 8-bit resolution
  
  // Attach the PWM channel to the LED pin
  // ledcAttachPin(motorA.enP, motorA.channel);
  // ledcAttachPin(motorB.enP, motorB.channel);
  // ledcAttachPin(motorC.enP, motorC.channel);
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
  //code to test sensors
  #ifdef printing
  Serial.print(digitalRead(L0));
  Serial.print(" ");
  Serial.print(digitalRead(L1));
  Serial.print(" ");
  Serial.print(Dis0.getDistance());
  Serial.print(" ");
  Serial.print(Dis1.getDistance());
  Serial.print(" ");
  Serial.print(Dis2.getDistance());
  Serial.print(" ");
  Serial.print(Dis3.getDistance());
  Serial.print(" ");
  Serial.print(Dis4.getDistance());
  Serial.print(" ");
  Serial.print(Dis5.getDistance());
  Serial.print(" ");
  Serial.print(bmp.readTemperature());
  Serial.print(" ");
  Serial.print(payload.mode);
  Serial.print(" ");
  Serial.print(payload.speed);
  Serial.print(" ");
  Serial.print(payload.angle);
  Serial.print(" ");
  Serial.print(payload.spin);
  // Serial.println(payload.eStop);
  // float v1=0, v2=0, v3=0;
  Serial.println();
  #endif



  //Dynamic direction control
  invKin(payload.speed, payload.angle, payload.spin, &v1, &v2, &v3);
  ledcWrite(0,abs(v1)); //A
  ledcWrite(3,abs(v2)); //B
  ledcWrite(7,abs(v3)); //C 


  if(v1 >= 0) //A
    {
      digitalWrite(A0_, HIGH);
      digitalWrite(A1_, LOW);
    }
    else if(v1 < 0)
    {
      digitalWrite(A0_, LOW);
      digitalWrite(A1_, HIGH);
    }

    if(v2 >= 0) //B
    {
      digitalWrite(B0_, HIGH);
      digitalWrite(B1_, LOW);
    }
    else if(v2 < 0)
    {
      digitalWrite(B0_, LOW);
      digitalWrite(B1_, HIGH);
    }

    if(v3 >= 0) //C
    {
      digitalWrite(C0_, HIGH);
      digitalWrite(C1_, LOW);
    }
    else if(v3 < 0)
    {
      digitalWrite(C0_, LOW);
      digitalWrite(C1_, HIGH);
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



void motor::setSpeed(float speed){
      if(speed == 0){
        brake();
      }else if (speed > 0){
        //move forward
        pinMode(P0, OUTPUT);
        pinMode(P1, OUTPUT);
        digitalWrite(P0, HIGH);
        digitalWrite(P1, LOW);
      }else if (speed < 0){
        //move forward
        pinMode(P0, OUTPUT);
        pinMode(P1, OUTPUT);
        digitalWrite(P0, LOW);
        digitalWrite(P1, HIGH);
      }
    }











































/*


  if (payload.eStop == true){
    motorA.brake();
    //motorB.brake();
    //motorC.brake();
    
  }
  switch(payload.mode){
    case 0:{ //user control mode
      float *v1,*v2,*v3;
      invKin(payload.speedX,payload.speedY,v1,v2,v3);
      motorA.setSpeed(*v1); 
      //motorB.setSpeed(*v2);
      //motorC.setSpeed(*v3);

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
          //motorB.setSpeed(spinDir);
          //motorC.setSpeed(spinDir);
    }
    default:{
      motorA.brake();
      //motorB.brake();
      //motorC.brake();
    }
  }
  */