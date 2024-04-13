#include <Arduino.h>

//radio libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//display libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define MISO 10
#define MOSI 11
#define SCK 12
#define SCN 13
#define CE 14

#define SDA 4
#define SCL 5

#define S0 1
#define S1 2
#define S2 42
#define S3 41
#define S4 40
#define S5 39
#define S6 38
#define S7 37
#define S8 36
#define S9 35

#define Y0 15
#define X0 16
#define J0 17

#define Y1 18
#define X1 8
#define J1 9

#define maxStates 3





struct PayloadStruct {
  uint8_t mode;   //simple mode, basic int
  float speed; //a int centered at 127
  float angle; //a int centered at 127
  uint8_t spin;   //a int centered at 127
  uint8_t eStop;  // bascially a bool
  uint8_t PID;
  uint8_t disable;
};

struct inputValues{
  //switches
  int S0V;
  int S1V;
  int S2V;
  int S3V;
  int S4V;
  int S5V;
  int S6V;
  int S7V;
  int S8V;
  int S9V;
  //joystick values
  float J0XV;
  float J0YV;
  int J0JV;
  float J1XV;
  float J1YV;
  int J1JV;
};




void updateMenu(int state, PayloadStruct payload);
int decState(int state);
int incState(int state);
void updateAll(inputValues &values);