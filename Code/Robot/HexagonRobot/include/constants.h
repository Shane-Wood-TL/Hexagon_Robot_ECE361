#include <Arduino.h>

//distance sensor library
#include <NewPing.h>

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


//all constant pin declarations

//I2C pins
#define SDA 4
#define SCL 5


//motor output pins
#define A0_ 6
#define A1_ 36
#define enA_ 15

#define B0_ 42
#define B1_ 41
#define enB_ 16

#define C0_ 1
#define C1_ 2
#define enC_ 7


//internal pwm channel numbers
#define aChannel 0
#define bChannel 3
#define cChannel 6


//pwm values
#define pwmHz 1000
#define pwmBit 8

//distance sensor input output pins
#define D0 37
#define D1 19
#define D2 9
#define D3 35
#define D4 39
#define D5 40


//spi pins
#define MISO 10
#define MOSI 11
#define SCK 12
#define SCN 13
#define CE 14


//line following pins
#define L0 18
#define L1 8


//joystick settings
#define JOYSTICK_CENTER 128
#define PWM_FREQ 1000

//max distance measured by hc-sr04
#define maxDistance 70


//base increase to pwm signal
#define pwmGain 20

#define bit8Max 255


#define noSpin 127 //value where spin is zeroed

#define lineFollowingSpeed 230 //max speed when line following
#define wallFollowStalling 0.6 //wall following needs a motor to stall partially to work


//display defined in main.cpp
extern LiquidCrystal_I2C lcd;

//radio from main
extern RF24 radio;


//struct to send movement values to main
struct moveValues{
  float speed;
  float angle;
  float spin;
};


//struct from the controller
struct PayloadStruct {
  uint8_t mode;   //simple mode, basic int
  float speed; //a int centered at 127
  float angle; //a int centered at 127
  uint8_t spin = 127;   //a int centered at 127
  uint8_t eStop;  // bascially a bool
  uint8_t disable;
};

extern PayloadStruct payload; //payload struct from main

//motor class, handles direction given a +- value and speed
class motor{
  private:
    int P0;
    int P1;
    int enP;
    int channel;
  public:
    motor(int P0V, int P1V, int enPV, int channelV){
      P0 = P0V;
      P1 = P1V;
      enP = enPV;
      channel = channelV;
    }
    void setSpeed(float speed){
      if(speed == 0){
        brake();
      }else if (speed > 0){
        //move forwards
        digitalWrite(P0, HIGH);
        digitalWrite(P1, LOW);
        //bumping up the min signal by gain
        if(abs(speed) >= 235){
          speed = 235;
        }
        ledcWrite(channel, constrain(abs(speed)+pwmGain, 0, bit8Max));
      }else if (speed < 0){
        //move backwards
        digitalWrite(P0, LOW);
        digitalWrite(P1, HIGH);
        //bumping up the min signal gain
        ledcWrite(channel,constrain(abs(speed)+pwmGain, 0, bit8Max));
      }
    }
    void brake(){
      //stop motor
      digitalWrite(P0, HIGH);
      digitalWrite(P1, HIGH);
    }
};




//class that handles the distance sensor code
class DistanceSensor{
  private:
    NewPing sonar;
    float speed;
    float distance;
  public:
    DistanceSensor(int pinV, float speedV) : sonar(pinV, pinV, maxDistance), speed(speedV) {
    }

    float getDistance(){
      //run sensor once
      float duration = sonar.ping_median(1);
      //instant hit (does not work correctlly / impossible or out of range)
      if (duration == 0){
        return 0;
      }
      //only get one path (duration has sending and receiving time)
      //mutiply by current speed of sound to get cm
      distance = (duration / 2) * speed;
      return distance;
    }
    void setSpeed(float speedV){
      //set the speed of sound
      speed = speedV;
    }
};

//sonar array class
class Distances{
  private:
    DistanceSensor *sonar[6];
    float distances[6];
    float speedOfSound;
    float intialSpeed;
    moveValues move;
    int b;

  public:
    Distances(int A, int B,int C,int D,int E,int F, float speedIntial){
      // set up distance sensors 
        intialSpeed = speedIntial;
        sonar[0] = new DistanceSensor(A,intialSpeed);
        sonar[1] = new DistanceSensor(B,intialSpeed);
        sonar[2] = new DistanceSensor(C,intialSpeed);
        sonar[3] = new DistanceSensor(D,intialSpeed);
        sonar[4] = new DistanceSensor(E,intialSpeed);
        sonar[5] = new DistanceSensor(F,intialSpeed);
        //get initial distances
        updateDistances();
        //get closest sensor
        getClosest(&b);
    }
    void updateDistances(){
      //update all 6 sensors
      for(int i = 0; i < 6; i++){
        distances[i] = sonar[i]->getDistance();
      }
      //display values to screen
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print((int)distances[0]);

      lcd.setCursor(5,0);
      lcd.print((int)distances[1]);

      lcd.setCursor(10,0);
      lcd.print((int)distances[2]);

      lcd.setCursor(0,1);
      lcd.print((int)distances[3]);

      lcd.setCursor(5,1);
      lcd.print((int)distances[4]);

       lcd.setCursor(10,1);
      lcd.print((int)distances[5]);
    }

    void getClosest(int *b){
      //find the sensor with the lowest non-zero distance
      int nearSensor = -1;
      float closeDistance = 99999; //value that all sensors should be less than
      for(int i = 0; i< 6; i++){
        if (distances[i] != 0){
          if(distances[i]< closeDistance){
            nearSensor = i;
            closeDistance = distances[i]; //update the nearest value and sensor value
          }
        }
      }
      *b = nearSensor;
    }

    void updateTemp(float temp){
      //update the temperature given a temp in C
      float soundsp = 331.4 + (0.606 * temp);
      float soundcm = soundsp / 10000.0;
      //update all of the sensors
      for(int i = 0; i< 6; i++){
        sonar[i]->setSpeed(soundcm);
      }
    }


moveValues wallFollow() {
  const int tooClose = 33; // distance to move away from wall
  const int tooFar = 38; // distance to move towards wall
  move.spin = 127; // Default spin value
  move.speed = 255; //max speed if going away / forward
  float closerSpeed = 200; //speed of moving towards wall

  updateDistances(); //update sensors
  getClosest(&b);

  move.angle = 0; //default direction


  switch (b){
        case 0:{
          if(distances[b] <= tooClose){
            move.angle = 150;
            move.speed = closerSpeed;
            //move away from wall
          }else if(distances[b] >= tooFar){
            move.angle = 90;
            //move towards wall
          }else{
            move.angle = 120;
            //move forward parrallel with wall
          }
          break;
        }
        case 1:{
          if(distances[b] <= tooClose){
            move.angle = 210;
            move.speed = closerSpeed;
          }else if(distances[b] >= tooFar){
            move.angle = 150;
          }else{
            move.angle = 180;
          }
          break;
        }
        case 2:{
          if(distances[b] <= tooClose){
            move.angle = 270;
            move.speed = closerSpeed;
          }else if(distances[b] >= tooFar){
            move.angle = 210;
          }else{
            move.angle = 240;
          }
          break;
        }
        case 3:{
          if(distances[b] <= tooClose){
            move.angle = 330;
            move.speed = closerSpeed;
          }else if(distances[b] >= tooFar){
            move.angle = 270;
          }else{
            move.angle = 300;
          }
          break;
        }
        case 4:{
          if(distances[b] <= tooClose){
            move.angle = 30;
            move.speed = closerSpeed;
          }else if(distances[b] >= tooFar){
            move.angle = 330;
          }else{
            move.angle = 0;
          }
          break;
        }
        case 5:{
          if(distances[b] <= tooClose){
            move.angle = 90;
            move.speed = closerSpeed;
          }else if(distances[b] >= tooFar){
            move.angle = 30;
          }else{
            move.angle = 60;
          }
          break;
        }
      }
  return move; //send values back to main
}
};




//converts radians to degrees
float raddec(float rad)
{
  rad = rad * (180 / PI);
  return rad;
}


//converts degrees to radians
float decrad(float deg)
{
  deg = deg * (PI / 180);
  return deg;
}


//sets the motor speeds and direction
void invKin(float speed, float angle, int spin, float* v1, float* v2, float* v3)
{
  float spinMod = map(spin, 0,255, -127,127); //spinning
  float v1T = speed*sin(decrad((330-angle)));
  float v2T = speed*sin(decrad((210-angle)));
  float v3T = speed*sin(decrad((90-angle)));
  *v1 = v1T+spinMod; //add spinning
  *v2 = v2T+spinMod;
  *v3 = v3T+spinMod;
}






//line following function
moveValues lineFollowing(int Left, int Right)
{
  moveValues follow; //value to move to on return

  int CW = 0; //clockwise turn
  int CCW = 255; //ccw turn

  //not detected, move forward
  if(Left == 0 and Right == 0)
  {
    follow.speed = lineFollowingSpeed;
    follow.angle = 90;
    follow.spin = noSpin;
  }
  //detected, turn right / cw
  else if(Left == 0 and Right == 1)
  {
    follow.speed = 0;
    follow.angle = 0;
    follow.spin = CW;
  }
  //detected, turn left / ccw
  else if(Left == 1 and Right == 0)
  {
    follow.speed = 0;
    follow.angle = 0;
    follow.spin = CCW;
  }else{ //default going forward
    follow.speed = lineFollowingSpeed;
    follow.angle = 90;
    follow.spin = noSpin;
  }

  return follow;
  
}




//gets data from radio, checks if data was recieved
void getData(){
   if (radio.available()) {
    radio.read(&payload, sizeof(payload));
  }
}

