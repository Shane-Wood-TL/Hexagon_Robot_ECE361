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

class motor{
  private:
    int P0;
    int P1;
  public:
    int channel;
    int enP;
    motor(int P0V, int P1V, int enPV, int channelV){
      P0 = P0V;
      P1 = P1V;
      enP = enPV;
      // pinMode(P0, OUTPUT);
      // pinMode(P1, OUTPUT);
      // pinMode(enP, OUTPUT);
      channel = channelV;
    }
    void setSpeed(float speed);
    void brake(){
      digitalWrite(P0, HIGH);
      digitalWrite(P1, HIGH);
    }
};





class DistanceSensor{
  private:
    NewPing sonar;
    float speedOfSound;
  public:
    DistanceSensor(int pinV, float speed) : sonar(pinV, pinV, 400), speedOfSound(speed) {
    }

    
    float getDistance(){
      float soundsp = 331.4 + (0.606 * 22);
      float soundcm = soundsp / 10000;
      float duration = sonar.ping_median(5);
      float distance = (duration / 2) * soundcm;

      return distance;
    }


    void setSpeed(float speed){
      speedOfSound = speed;
    }
};





class distances{
  private: 
    DistanceSensor *D[6];
    float DistanceB;
    int sensor = 0;
    int sensorA;
    int sensorC;
    Adafruit_BMP085 bmp;

    float perpendicularDistance(float distance){
      return distance  * cos(0.785398); //45 deg to rads
    }


  public:
    distances(DistanceSensor *A, DistanceSensor *B, DistanceSensor *C, DistanceSensor *D, DistanceSensor *E, const DistanceSensor *F, Adafruit_BMP085 *bmpV){
      D[0] = *A;
      D[1] = *B;
      D[2] = *C;
      D[3] = *D;
      D[4] = *E;
      D[5] = *F;
      bmp = *bmpV;
    }


    float getClosest(){
      DistanceB = 1000;
      for(int i = 0; i <= 5; i++){
        float distance = D[i]->getDistance();
        if(distance < DistanceB){
          DistanceB = distance;
          sensor = i;
        }
      }
      return DistanceB;
    }


    void getNearSensor(){
      int sensorA = sensor++;
      if (sensorA > 5){
        sensorA = 0;
      }

      int sensorB = sensor--;
      if (sensorB < 0){
        sensorB = 5;
      }
    }


    float distanceA(){
      return perpendicularDistance(D[sensorA]->getDistance());
    }


    float distanceB(){
      return perpendicularDistance(D[sensorC]->getDistance());
    }


    void updateSpeed(){
      float adjustedSpeed = ((331.4 + (0.606 * bmp.readTemperature()))/pow(10, 3)); //convert from m/s to cm/us 10^-3 / 10^-6 = 10^-3
      for(int i = 0; i <= 5; i++){
        D[i]->setSpeed(adjustedSpeed);
      }
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