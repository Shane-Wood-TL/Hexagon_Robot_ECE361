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




// //radio
// PayloadStruct payload;
// RF24 radio(CE,SCN);
// bool newData = false;
// int oldState = 0;
// const byte thisSlaveAddress[5] = {'R','x','A','A','A'};


// //temperature sensor
// Adafruit_BMP085 bmp;

// //display
// LiquidCrystal_I2C lcd(0x27,20,4);


//motor class instances
motor motorA(A0_, A1_, enA_, 2); //Mtr1
motor motorB(B0_, B1_, enB_, 1); //Mtr2
motor motorC(C0_, C1_, enC_, 0); //Mtr3


/*
//distance sensor instances
DistanceSensor Dis0(D0,intialSpeedOfSound);
DistanceSensor Dis1(D1,intialSpeedOfSound);
DistanceSensor Dis2(D2,intialSpeedOfSound);
DistanceSensor Dis3(D3,intialSpeedOfSound);
DistanceSensor Dis4(D4,intialSpeedOfSound);
DistanceSensor Dis5(D5,intialSpeedOfSound);

//distance array instance
distances distanceArray(&Dis0, &Dis1, &Dis2, &Dis3, &Dis4, &Dis5, &bmp);

*/



void setup() {
  //PWM for servo library (would like to use analog write if possible)

  Serial.begin(115200);

  //i2c setup
  //Wire.begin(SDA,SCL); //SDA, SCL
  // lcd.init();           
  // lcd.backlight();
  // lcd.clear();
  // lcd.print("working");


  //begin SPI
  //SPI.begin(SCK, MISO, MOSI);
  
 
  // begin radio
  /*
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
  */


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
  Serial.println("in loop");
  // digitalWrite(A0_, LOW);
  // digitalWrite(A1_, HIGH);
  // digitalWrite(B0_, HIGH);
  // digitalWrite(B1_, LOW);
  // digitalWrite(C0_, LOW);
  // digitalWrite(C1_, HIGH);
  // digitalWrite(enA_, HIGH);
  // digitalWrite(enB_, HIGH);
  // digitalWrite(enC_, HIGH);

  //void invKin(uint8_t speedX, uint8_t speedY, float* v1, float* v2, float* v3)
  float v1=0, v2=0, v3=0;
  
  //working f/w
  // invKin(254,0,&v1,&v2,&v3);

  // // Serial.print(abs(v1));
  // // Serial.print(" ");
  // // Serial.print(abs(v2));
  // // Serial.print(" ");
  // // Serial.println(abs(v3));

  // ledcWrite(0, abs(v1));
  // ledcWrite(3, abs(v2));
  // ledcWrite(7, abs(v3));
  // motorA.setSpeed(v1);
  // motorA.setSpeed(v2);
  // motorA.setSpeed(v3);

  // Serial.print(v1);
  // Serial.print(" ");
  // Serial.print(v2);
  // Serial.print(" ");
  // Serial.println(v3);

  // digitalWrite(A0_, LOW);
  // digitalWrite(A1_, HIGH);
  // digitalWrite(B0_, HIGH);
  // digitalWrite(B1_, LOW);
  // digitalWrite(C0_, LOW);
  // digitalWrite(C1_, HIGH);
  // delay(2000);

// invKin(-254,0,&v1,&v2,&v3);

//   // Serial.print(abs(v1));
//   // Serial.print(" ");
//   // Serial.print(abs(v2));
//   // Serial.print(" ");
//   // Serial.println(abs(v3));

//   ledcWrite(0, abs(v1));
//   ledcWrite(3, abs(v2));
//   ledcWrite(7, abs(v3));
//   motorA.setSpeed(v1);
//   motorA.setSpeed(v2);
//   motorA.setSpeed(v3);

//   Serial.print(v1);
//   Serial.print(" ");
//   Serial.print(v2);
//   Serial.print(" ");
//   Serial.println(v3);

//   digitalWrite(A0_, HIGH);
//   digitalWrite(A1_, LOW);
//   digitalWrite(B0_, LOW);
//   digitalWrite(B1_, HIGH);
//   digitalWrite(C0_, HIGH);
//   digitalWrite(C1_, LOW);
//   delay(2000);










  invKin(0,255,0,&v1,&v2,&v3);
  Serial.print(v1);
  Serial.print(" ");
  Serial.print(v2);
  Serial.print(" ");
  Serial.println(v3);
  ledcWrite(0, abs(255));
  ledcWrite(3, abs(255));
  ledcWrite(7, abs(255));
  delay(5);
  
  float f1, f2, f3;
  //f1 = map(abs(v1), 0, 255, 150, 255);
  //f2 = map(abs(v2), 0, 255, 150, 255);
  //f3 = map(abs(v3), 0, 255, 150, 255);

  // ledcWrite(0, 150);
  // ledcWrite(3, 210);
  // ledcWrite(7, 255);

  //Previous values with smaller wheels
  // ledcWrite(0,abs(v1)+35); //A
  // ledcWrite(3,abs(v2)+35); //B
  // ledcWrite(7,abs(v3)); //C

  ledcWrite(0,abs(v1)); //A
  ledcWrite(3,abs(v2)); //B
  ledcWrite(7,abs(v3)); //C 

  Serial.print(f1);
  Serial.print(" ");
  Serial.print(f2);
  Serial.print(" ");
  Serial.println(f3);
  
  //Static direction control
  // digitalWrite(A0_, LOW);
  // digitalWrite(A1_, HIGH);
  // digitalWrite(B0_, LOW); 
  // digitalWrite(B1_, HIGH);
  // digitalWrite(C0_, HIGH);
  // digitalWrite(C1_, LOW);

  //Dynamic direction control
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

  delay(2000);


  

  // invKin(0,-254,&v1,&v2,&v3);
  // ledcWrite(0, abs(v1));
  // ledcWrite(3, abs(v2));
  // ledcWrite(7, abs(v3));
  // digitalWrite(A0_, LOW);
  // digitalWrite(A1_, HIGH);
  // digitalWrite(B0_, HIGH);
  // digitalWrite(B1_, LOW);
  // digitalWrite(C0_, LOW);
  // digitalWrite(C1_, HIGH);

  // delay(2000);
  // for (int dutyCycle = 150; dutyCycle <= 255; dutyCycle++) {
  //   // Set PWM duty cycle
  //   // motorA.setSpeed(dutyCycle);
  //   // motorB.setSpeed(dutyCycle);
  //   // motorC.setSpeed(dutyCycle);
  //   ledcWrite(0, dutyCycle);
  //   ledcWrite(3, dutyCycle);
  //   ledcWrite(7, dutyCycle);
  //   delay(100); // Wait for a short duration for gradual change
  // }
  
  // // Decrease brightness gradually
  // for (int dutyCycle = 255; dutyCycle >= 150; dutyCycle--) {
  //   // Set PWM duty cycle
  //   // motorA.setSpeed(dutyCycle);
  //   // motorB.setSpeed(dutyCycle);
  //   // motorC.setSpeed(dutyCycle);
  //   ledcWrite(0, dutyCycle);
  //   ledcWrite(3, dutyCycle);
  //   ledcWrite(7, dutyCycle);
  //   delay(100); // Wait for a short duration for gradual change
  // }




    //getData();
  // motorA.setSpeed(100);
  // for (int i = 0; i <= 255; i++) {
  //   // Print the current value of i
  //   motorA.setSpeed(i);
  //   delay(100); // Delay for readability, adjust as needed
  // }

  /*
  payload.mode = 0;
  payload.speedX = 100;
  payload.speedY = 100;
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
}


//gets data from radio, checks if data was recieved
void getData(){
  //  if (radio.available()) {
  //   radio.read(&payload, sizeof(payload));
  //   newData = true;
  // //  }else{
  // //   payload.eStop ==true; 
  // }
}

//sets the motor speeds and direction
void invKin(int speedX, int speedY, int spin, float* v1, float* v2, float* v3)
{

  //Shane new math
    float PHI = atan2(speedY,speedX);

    float thetaC = (3.14/2) - PHI;
    float thetaB = (7*3.14/6) - PHI;
    float thetaA = (11*3.14/6) - PHI;

    float FullSpeed = sqrt(((speedX * speedX) + (speedY * speedY)));

    *v3 = FullSpeed * sin(thetaC);
    *v2 = FullSpeed * sin(thetaB);
    *v1 = FullSpeed * sin(thetaA);

    //Somewhat working code

    //Version with exact value.
    /* *v1 = ((-sqrt(3)/2) * speedX) - (0.5 * speedY);
     *v2 = ((sqrt(3)/2) * speedX) - (0.5 * speedY);
     *v3 = speedY;*/
    
    //Version with rounded value.
    /*
    *v1 = (-0.8660254 * speedX) - (0.5 * speedY); //A
     *v2 = (0.8660254 * speedX) - (0.5 * speedY);//B
     *v3 = speedY; //C*/

    

   

  

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