#include <Arduino.h>
#include <ESP32Servo.h>


#define SDA 4
#define SCL 5

#define A0 6
#define A1 36
#define enA 15

#define B0 42
#define B1 41
#define enB 16

#define C0 1
#define C1 2
#define enC 7

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



class motor{
  private:
    int P0;
    int P1;
    float freq;
    float minFreq;
    float maxFreq;
    Servo motorX;
  public:
    motor(int P0V, int P1V, int enPV, float freqV, float minFreqV, float maxFreqV){
      P0 = P0V;
      P1 = P1V;
      pinMode(P0, OUTPUT);
      pinMode(P1, OUTPUT);
      minFreq = minFreqV; 
      maxFreq = maxFreqV;
      motorX.attach(enPV);
      motorX.setPeriodHertz(freqV);
    }
    float speed;
    void setSpeed(float speed){
      if(speed == 0){
        brake();
      }else if (speed > 0){
        //move forward
        digitalWrite(P0, HIGH);
        digitalWrite(P1, LOW);
        motorX.writeMicroseconds(map(speed, 0, 255, minFreq, maxFreq));
      }else if (speed < 0){
        //move forward
        digitalWrite(P0, LOW);
        digitalWrite(P1, HIGH);
        motorX.writeMicroseconds(map(speed, 0, 255, minFreq, maxFreq));
      }
    }
    void brake(){
      digitalWrite(P0, HIGH);
      digitalWrite(P1, HIGH);
      motorX.writeMicroseconds(0);
    }
};






class DistanceSensor{
  private:
    int Dpin;
    float speedOfSound;
  public:
    DistanceSensor(int pinV, float speed){
      Dpin = pinV;
      speedOfSound = speed;
    }

    
    float getDistance(){
      //send a pulse out
      pinMode(Dpin, OUTPUT);
      digitalWrite(Dpin, LOW);
      delayMicroseconds(2);
      digitalWrite(Dpin, HIGH);
      delayMicroseconds(10);
      digitalWrite(Dpin, LOW);

      //check for pulse input (switch pin modes)
      pinMode(Dpin, INPUT);
      float lenght = pulseIn(Dpin, HIGH);

      return lenght * speedOfSound; 
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
