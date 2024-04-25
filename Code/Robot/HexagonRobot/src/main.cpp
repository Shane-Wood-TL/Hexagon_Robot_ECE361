//class defintions, structures, functions and constants
#include <constants.h>

#define debugging

//motor rebalancing
float bmotorOffset = 0.9;
float aMotorOffset = 1.07;

float intialSpeedOfSound = 0.0343; //in cm/mirco second 



float v1,v2,v3; //Motor speeds


// //radio
PayloadStruct payload;
RF24 radio(CE,SCN);
bool newData = false;
int oldState = 0;
const byte address[5] = {'T','r','i','n','E'};


// //temperature sensor
Adafruit_BMP085 bmp;

// //display
LiquidCrystal_I2C lcd(0x3F,16,2);


//motor class instances
motor motorA(A0_, A1_, enA_,aChannel); //Mtr1
motor motorB(B0_, B1_, enB_,bChannel); //Mtr2
motor motorC(C0_, C1_, enC_,cChannel); //Mtr3


//distance array instance
Distances sonarArray(D0,D1,D2,D3,D4,D5,intialSpeedOfSound);


void setup() {
  #ifdef debugging
  Serial.begin(115200); //only use serial if needed
  #endif

  //i2c setup
  Wire.begin(SDA,SCL); //SDA, SCL
  lcd.init();           
  lcd.backlight();
  lcd.clear();
  lcd.print("working");
  bmp.begin();


  //begin SPI
  SPI.begin(SCK, MISO, MOSI);
  
 
  // begin radio
  if (!radio.begin()) {
    #ifdef debugging
    Serial.println(F("radio hardware is not responding!!"));
    #endif
    lcd.print("no radio");
    while (1) {}  // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_MAX); //signal amplification
  radio.setDataRate(RF24_250KBPS); //transfer rate
  radio.openReadingPipe(1, address); //wireless address 
  radio.startListening(); //set receiving

  #ifdef debugging
  Serial.println("Radio Active"); 
  #endif

  //set pin directions
  pinMode(L0, INPUT);
  pinMode(L1, INPUT);

  pinMode(enA_, OUTPUT);
  pinMode(A0_, OUTPUT);
  pinMode(A1_, OUTPUT);

  pinMode(enB_, OUTPUT);
  pinMode(B0_, OUTPUT);
  pinMode(B1_, OUTPUT);

  pinMode(enC_, OUTPUT);
  pinMode(C0_, OUTPUT);
  pinMode(C1_, OUTPUT);
  
  //pwm setup
  ledcSetup(aChannel,pwmHz,pwmBit);
  ledcSetup(bChannel,pwmHz,pwmBit);
  ledcSetup(cChannel,pwmHz,pwmBit);


  ledcAttachPin(enA_, aChannel);
  ledcAttachPin(enB_, bChannel);
  ledcAttachPin(enC_, cChannel);
  #ifdef debugging
  Serial.println("setupDone");
  #endif
}

void loop() {
  //update the radio
  getData();


  //only look at the modes if not stopped
  if (payload.eStop != 1){
    //handle nomove + nospin
    switch (payload.disable){
      case 1:{
        payload.spin = noSpin;
        break;
      }
      case 2:{
        payload.speed = 0;
        break;
      }
      default:
        payload.speed = payload.speed;
        payload.spin = payload.spin;
        break;
    }

    switch(payload.mode){
      case 0:{ //user control mode
        //Dynamic direction control
        invKin(payload.speed, payload.angle, payload.spin, &v1, &v2, &v3); //map controller values to speed, direction and spin
        motorA.setSpeed(v1); //set motor speeds + direction
        motorB.setSpeed(v2*.9);
        motorC.setSpeed(v3);
        break;
      }
      case 1:{ //wall following
        moveValues wallFollow;
        wallFollow = sonarArray.wallFollow(); //get what move to make
        invKin(wallFollow.speed, wallFollow.angle, wallFollow.spin, &v1, &v2, &v3); //move
        motorA.setSpeed(v1*aMotorOffset*wallFollowStalling); //set motor speeds + direction
        motorB.setSpeed(v2*bmotorOffset);
        motorC.setSpeed(v3);
        break;
      }
      case 2:{ //line following
        int L0Value = digitalRead(L0); //Right
        int L1Value = digitalRead(L1); //Left
        moveValues LineSensor;
        LineSensor = lineFollowing(L1Value, L0Value);  //get what move to make

        //update 2 sensor values on display
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print((int)L1Value);

      
        lcd.setCursor(10,0);
        lcd.print((int)L0Value);


        //move with line follower
        invKin(LineSensor.speed,LineSensor.angle,LineSensor.spin,&v1,&v2,&v3); //move
        motorA.setSpeed(v1*aMotorOffset);//set motor speeds + direction
        motorB.setSpeed(v2);
        motorC.setSpeed(v3);

        //set timing for motors being on
        if(LineSensor.speed == 0){
          delay(150);
          //turn for 150 ms
          invKin(0,0,127, &v1, &v2, &v3);
        }else{
          delay(15);
          //drive for 15ms
          invKin(0,0,127, &v1, &v2, &v3);
        }
        motorA.setSpeed(v1*aMotorOffset);
        motorB.setSpeed(v2);
        motorC.setSpeed(v3);
        break;
      }
    }
  }else{
    //stop motors on stop
    motorA.brake();
    motorB.brake();
    motorC.brake();

    //only update temperature when stopped
    sonarArray.updateTemp(bmp.readTemperature());
  }

}