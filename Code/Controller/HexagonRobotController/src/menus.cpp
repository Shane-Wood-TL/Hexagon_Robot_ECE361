#include <constants.h>

extern LiquidCrystal_I2C lcd;


//code to loop increase state (when reach max state go back to 0)
int incState(int state){
  state++;
  if (state >= maxStates){
    state = 0;
  }
  return state;
}

int decState(int state){
  state--;
  if (state < 0){
    state = maxStates-1;
  }
  return state;
}

//code that updates the display
void updateMenu(int state, PayloadStruct payload){
  //clear display
  lcd.clear();
  //second row text
  if(payload.eStop == true){
    lcd.setCursor(1,1);
      lcd.print("STOP");
  }else{
      lcd.setCursor(1,1);
      lcd.print("   ");
  }
  //top row text
  lcd.setCursor(0,0);
  lcd.print("Mode ");
  switch (state)
  {
  case 0:{
    lcd.print("User");
    break;
  }
  case 1:{
     lcd.print("Wall Follow");
     break;
  }
  case 2:{
    lcd.print("Line Follow");
    break;
  }
  default:{
    break;
  }
  } 
}