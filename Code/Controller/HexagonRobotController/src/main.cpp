#include <constants.h>


//radio variables
PayloadStruct payload;
RF24 radio(CE,SCN);
bool newData = false;
const byte slaveAddress[5] = {'R','x','A','A','A'};


//display object
LiquidCrystal_I2C lcd(0x3F,16,2);

//all inputs in a struct
inputValues readingValues;

//old and present state (used to prevent extra display refreshes)
int oldState = 0;
int state = 0;

void setup() {
  //Start serial
  Serial.begin(115200);

  //start i2c
  Wire.begin(SDA,SCL);

  //setup display
  lcd.init();           
  lcd.backlight();
  lcd.clear();
  lcd.print("working ");


  //start spi
  SPI.begin(SCK, MISO, MOSI);
  radio.begin();
  
  // begin radio
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    lcd.print("no radio");
    while (1) {}  // hold in infinite loop
  }
  //radio.printPrettyDetails();
  
  //set radio speed, and wireless address
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3,5); // delay, count
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(slaveAddress);

  //display that radio working
  lcd.clear();
  lcd.print("radio working");

  //setup all input pins
  pinMode(S0, INPUT_PULLUP);
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  pinMode(S5, INPUT_PULLUP);
  pinMode(S6, INPUT_PULLUP);
  pinMode(S7, INPUT_PULLUP);
  pinMode(S8, INPUT_PULLUP);
  pinMode(S9, INPUT_PULLUP);

  pinMode(X0, INPUT);
  pinMode(Y0, INPUT);
  pinMode(J0, INPUT);
  pinMode(X1, INPUT);
  pinMode(Y1, INPUT);
  pinMode(J1, INPUT);
}

void loop() {
  bool sent;
  updateAll(readingValues);

  //some basic fixed inputs
  //confirmed additions
  //SW0(0,1) Estop / nothing / nothing
  //SW1(2,3) Mode switcher  / nothing / nothing

  //maybe additions
  //SW2(4,5) PID (for line following mode) / nothing / PID (for wall following mode)
  //SW3(6,7) disable spin / nothing / disable move
  //SW4(8,9) nothing / nothing / nothing

  //switches are active low eg flipped = 0


  //code to handle estop switch + prevent extra display refreshes
  //only go to display when value has changed
  if(readingValues.S0V != 1){
    if(payload.eStop != 1){
      payload.eStop = 1;
      updateMenu(state, payload);
    }
  }else{
    if(payload.eStop != 0){
      payload.eStop = 0;
      updateMenu(state, payload);
    }
  }

  //once mode switching is done switch modes
  if (state != oldState && readingValues.S2V == 1){
    oldState = state;
    payload.mode = oldState;    
  }

  //menu interface to switch modes
  if(readingValues.S2V != 1){
    float menuJ = map(readingValues.J0YV, 0,4096,-80,80);
    int temp_state = state;
      if(menuJ >=75){
        state = decState(state);
        delay(100);
      }else if(menuJ <= -75){
        state = incState(state);
        delay(100);
      }
      if(temp_state!=state){
        updateMenu(state, payload);
      }
  }



  //the rest of the values to be sent through payload can be set here like
  payload.speedX = int(map(readingValues.J0XV, 0, 4096, 0,255));
  payload.speedY = int(map(readingValues.J0YV, 0, 4096, 0,255));
  payload.spin = int(map(readingValues.J1YV, 0, 4096, 0,255));


  sent = radio.write(&payload, sizeof(PayloadStruct)); //actually send values
}



//function to update all inputs on the controller
void updateAll(inputValues &values){
  values.S0V = digitalRead(S0);
  values.S1V = digitalRead(S1);
  values.S2V = digitalRead(S2);
  values.S3V = digitalRead(S3);
  values.S4V = digitalRead(S4);
  values.S5V = digitalRead(S5);
  values.S6V = digitalRead(S6);
  values.S7V = digitalRead(S7);
  values.S8V = digitalRead(S8);
  values.S9V = digitalRead(S9);

  values.J0JV = digitalRead(J0);
  values.J1JV = digitalRead(J1);

  values.J0XV = analogRead(X0);
  values.J0YV = analogRead(Y0);
  values.J1XV = analogRead(X1);
  values.J1YV = analogRead(Y1);
}