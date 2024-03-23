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
