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
        ledcWrite(channel,abs(speed));
      }else if (speed < 0){
        //move forward
        digitalWrite(P0, LOW);
        digitalWrite(P1, HIGH);
        ledcWrite(channel,abs(speed));
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
    DistanceSensor(int pinV, float speedV) : sonar(pinV, pinV, 100), speed(speedV) {
    }

    float getDistance(){
      float duration = sonar.ping_median(5);
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
  public:
    Distances(int A, int B,int C,int D,int E,int F, float speedIntial){
        intialSpeed = speedIntial;
        sonar[0] = new DistanceSensor(A,intialSpeed);
        sonar[1] = new DistanceSensor(B,intialSpeed);
        sonar[2] = new DistanceSensor(C,intialSpeed);
        sonar[3] = new DistanceSensor(D,intialSpeed);
        sonar[4] = new DistanceSensor(E,intialSpeed);
        sonar[5] = new DistanceSensor(F,intialSpeed);
        updateDistances();
    }
    void updateDistances(){
      for(int i = 0; i < 6; i++){
        distances[i] = sonar[i]->getDistance();
      }
      // lcd.clear();
      // lcd.setCursor(0,0);
      // lcd.print((int)distances[0]);

      // lcd.setCursor(5,0);
      // lcd.print((int)distances[1]);

      // lcd.setCursor(10,0);
      // lcd.print((int)distances[2]);

      // lcd.setCursor(0,1);
      // lcd.print((int)distances[3]);

      // lcd.setCursor(5,1);
      // lcd.print((int)distances[4]);

      //  lcd.setCursor(10,1);
      // lcd.print((int)distances[5]);
    }

    int getClosest(){
      int sensor = 0;
      float distance = 99999;
      for(int i = 0; i< 6; i++){
        if (distances[i] != 0){
          if(distances[i]< distance){
            sensor = i;
            distance = distances[i];
          }
        }
      }
      return sensor;
    }

    void updateTemp(float temp){
      float soundsp = 331.4 + (0.606 * temp);
      float soundcm = soundsp / 10000;
      for(int i = 0; i< 6; i++){
        sonar[i]->setSpeed(soundcm);
      }
    }
    
    moveValues wallFollow(){
      moveValues move;
      updateDistances();
      int b = getClosest();
      move.spin =127;
      //get the 2 nearest sensors
      int a1 = b+1;
      int c1 = b-1;
      int neighbor = -1;
      if (c1 > 5){
        c1 = 0;
      }
      if (a1 < 0){
        a1 = 5;
      }
      if (distances[a1] > distances[c1]){
        neighbor = c1;
      }else{
        neighbor = a1;
      }
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print((int) distances[0]);
      lcd.setCursor(5,0);
      lcd.print((int)distances[1]);
      lcd.setCursor(10,0);
      lcd.print((int)distances[2]);
      lcd.setCursor(0,1);
      lcd.print((int) distances[3]);
      lcd.setCursor(5,1);
      lcd.print((int)distances[4]);
      lcd.setCursor(10,1);
      lcd.print((int)distances[5]);

      if (b == 1 or b==4){
        move.speed = 255;
        move.angle = 0;
        if(distances[b] > distances[neighbor]){
          move.spin-= abs(distances[neighbor]-distances[b]);
        }else{
          move.spin+= abs(distances[neighbor]-distances[b]);
        }
      }else if(b == 2 or b==5){
        move.speed = 255;
        move.angle = 60;
        if(distances[neighbor] > distances[b]){
          move.spin-= abs(distances[neighbor] - distances[b]);
        }else{
          move.spin+= abs(distances[neighbor] - distances[b]);
        }
      }else if(b == 0 or b==3){
        move.speed = 255;
        move.angle = 120;
        if(distances[neighbor] > distances[b]){
          move.spin-= abs(distances[neighbor] - distances[b]);
        }else{
          move.spin+= abs(distances[neighbor] - distances[b]);
        }
      }
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

