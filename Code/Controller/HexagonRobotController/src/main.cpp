#include <Arduino.h>


//radio libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//display libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

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
const byte slaveAddress[5] = {'R','x','A','A','A'};


//display
LiquidCrystal_I2C lcd(0x27,20,4);



void setup() {
  Wire.begin(SDA,SCL); //SDA, SCL
  lcd.init();           
  lcd.backlight();
  lcd.clear();
  lcd.print("working");

  radio.begin();
  // begin radio
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    lcd.print("no radio");
    while (1) {}  // hold in infinite loop
  }
  radio.setDataRate( RF24_250KBPS );
   radio.setRetries(3,5); // delay, count
   radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(slaveAddress);
}

void loop() {
  bool sent;
  sent = radio.write(&payload, sizeof(PayloadStruct));
}
