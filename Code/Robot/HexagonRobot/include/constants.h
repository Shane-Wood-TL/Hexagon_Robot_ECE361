#include <Arduino.h>
#include <NewPing.h>

#define SDA 4
#define SCL 5

#define A0_ 6
#define A1_ 36
#define enA_ 15

#define B0_ 42
#define B1_ 41
#define enB_ 16

#define C0_ 1
#define C1_ 2
#define enC_ 7

#define D0 37
#define D1 17
#define D2 9
#define D3 35
#define D4 39
#define D5 40

#define MISO 10
#define MOSI 11
#define SCK 12
#define SCN 13
#define CE 14

#define L0 18
#define L1 8

#define JOYSTICK_CENTER 128
#define PWM_FREQ 1000

extern LiquidCrystal_I2C lcd;

struct moveValues{
  float speed;
  float angle;
  float spin;
  int goOutV;
};

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
        //move forward
        digitalWrite(P0, HIGH);
        digitalWrite(P1, LOW);
        ledcWrite(channel, constrain(abs(speed)+20, 0, 255));
      }else if (speed < 0){
        //move forward
        digitalWrite(P0, LOW);
        digitalWrite(P1, HIGH);
        ledcWrite(channel,constrain(abs(speed)+20, 0, 255));
      }
    }
    void brake(){
      digitalWrite(P0, HIGH);
      digitalWrite(P1, HIGH);
    }
};





class DistanceSensor{
  private:
    NewPing sonar;
    float speed;
    float distance;
  public:
    DistanceSensor(int pinV, float speedV) : sonar(pinV, pinV, 70), speed(speedV) {
    }

    float getDistance(){
      float duration = sonar.ping_median(1);
      if (duration == 0){
        return 0;
      }
      distance = (duration / 2) * speed;
      return distance;
    }
    void setSpeed(float speedV){
      speed = speedV;
    }
};


class Distances{
  private:
    DistanceSensor *sonar[6];
    float distances[6];
    float speedOfSound;
    float intialSpeed;
    moveValues move;
    int oldB;
    int b;
    float oldAngle;
    int  sensor_angles[6] = {30,90,150,210,270,330};

  public:
    bool startUp = true;
    Distances(int A, int B,int C,int D,int E,int F, float speedIntial){
        intialSpeed = speedIntial;
        sonar[0] = new DistanceSensor(A,intialSpeed);
        sonar[1] = new DistanceSensor(B,intialSpeed);
        sonar[2] = new DistanceSensor(C,intialSpeed);
        sonar[3] = new DistanceSensor(D,intialSpeed);
        sonar[4] = new DistanceSensor(E,intialSpeed);
        sonar[5] = new DistanceSensor(F,intialSpeed);
        updateDistances();
        getClosest(&b);
    }
    void updateDistances(){
      for(int i = 0; i < 6; i++){
        distances[i] = sonar[i]->getDistance();
      }
      
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
      int nearSensor = -1;
      float closeDistance = 99999;
      for(int i = 0; i< 6; i++){
        if (distances[i] != 0){
          if(distances[i]< closeDistance){
            nearSensor = i;
            closeDistance = distances[i];
          }
        }
      }
      *b = nearSensor;
    }

    void updateTemp(float temp){
      float soundsp = 331.4 + (0.606 * temp);
      float soundcm = soundsp / 10000.0;
      for(int i = 0; i< 6; i++){
        sonar[i]->setSpeed(soundcm);
      }
    }


moveValues wallFollow() {
  const int tooClose = 30; // goal distance

  move.spin = 127; // Default spin value
  move.speed = 255;
  float closerSpeed = 200;
  float spinClose = 0;
  float spinFar = 0;
  float tooFar = 35;
  float angleSpread = 30;
  updateDistances();
  getClosest(&b);


  switch (b){
        case 0:{
          move.angle = 0;
          if(distances[b] <= tooClose){
            move.angle = 150;
            move.speed = closerSpeed;
            move.spin += spinClose;
          }else if(distances[b] >= tooFar){
            move.angle = 90;
            move.spin += spinFar;
          }else{
            move.angle = 120;
          }
          break;
        }
        case 1:{
          move.angle = 0;
          if(distances[b] <= tooClose){
            move.angle = 210;
            move.speed = closerSpeed;
            move.spin += spinClose;
          }else if(distances[b] >= tooFar){
            move.angle = 150;
            move.spin += spinFar;
          }else{
            move.angle = 180;
          }
          break;
        }
        case 2:{
          move.angle = 0;
          if(distances[b] <= tooClose){
            move.angle = 270;
            move.speed = closerSpeed;
            move.spin += spinClose;
          }else if(distances[b] >= tooFar){
            move.angle = 210;
            move.spin += spinFar;
          }else{
            move.angle = 240;
          }
          break;
        }
        case 3:{
          if(distances[b] <= tooClose){
            move.angle = 330;
            move.speed = closerSpeed;
            move.spin += spinClose;
          }else if(distances[b] >= tooFar){
            move.angle = 270;
            move.spin += spinFar;
          }else{
            move.angle = 300;
          }
          break;
        }
        case 4:{
          if(distances[b] <= tooClose){
            move.angle = 30;
            move.speed = closerSpeed;
            move.spin += spinClose;
          }else if(distances[b] >= tooFar){
            move.angle = 330;
            move.spin += spinFar;
          }else{
            move.angle = 0;
          }
          break;
        }
        case 5:{
          if(distances[b] <= tooClose){
            move.angle = 90;
            move.speed = closerSpeed;
            move.spin += spinClose;
          }else if(distances[b] >= tooFar){
            move.angle = 30;
            move.spin += spinFar;
          }else{
            move.angle = 60;
          }
          break;
        }
      }
  oldB = b;
  return move;
}
};

void invKin(float speed, float angle, int spin, float* v1, float* v2, float* v3);


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

moveValues lineFollowing(int Left, int Right)
{
  moveValues follow;

  int noSpin = 127;
  int CW = 0;
  int CCW = 255;
  int spin;

  if(Left == 0 and Right == 0)
  {
    follow.speed = 255;
    follow.angle = 90;
    follow.spin = noSpin;
  }

  else if(Left == 0 and Right == 1)
  {
    follow.speed = 0;
    follow.angle = 0;
    follow.spin = CW;
  }

  else if(Left == 1 and Right == 0)
  {
    follow.speed = 0;
    follow.angle = 0;
    follow.spin = CCW;
  }else{
    follow.speed = 255;
    follow.angle = 90;
    follow.spin = noSpin;
  }

  return follow;
  
}
