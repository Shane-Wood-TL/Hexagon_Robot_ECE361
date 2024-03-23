#include <Arduino.h>

//radio libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//line following library
#include <PID_v1.h> 

//temp + display libraries
#include <Wire.h>
#include <Adafruit_BMP085.h>


//class defintions and constants
#include <constants.h>



struct PayloadStruct {
  uint8_t mode; //sw2
  uint8_t speedX;
  uint8_t speedY;
  uint8_t spin;
  uint8_t eStop;
};

//radio
PayloadStruct payload;
//RF24 radio(ce, csn)
RF24 radio(CE,SCN);
bool newData = false;
int oldState = 0;
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};



Adafruit_BMP085 bmp;




class DistanceSensor{
  private:
    int Dpin;
    Adafruit_BMP085 bmp;
  public:
    DistanceSensor(int pinV, Adafruit_BMP085 bmpV){
      Dpin = pinV;
      bmp = bmpV;
    }
    float getDistance(){
      //need to find the speed of sound given the current temp
      float adjustedSpeed = 331.4 + (0.606 * bmp.readTemperature());
      //add pulse + recieve here
    }
};



void setup() {
  motor motorA(A0, A1,enA,500,790,2000);
  motor motorB(B0, B1,enB,500,790,2000);
  motor motorC(C0, C1,enC,500,790,2000);


  //pin setups

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
}

void loop() {

}
