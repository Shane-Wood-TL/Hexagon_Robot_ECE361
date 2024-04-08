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
    lcd.setCursor(0,1);
      lcd.print("STOP");
  }else{
      lcd.setCursor(0,1);
      lcd.print("   ");
  }

  if(payload.PID == true){
    lcd.setCursor(5,1);
      lcd.print("PID");
  }else{
      lcd.setCursor(5,1);
      lcd.print("   ");
  }

  if(payload.disable == 0){
    lcd.setCursor(9,1);
      lcd.print("     ");
  }else if (payload.disable == 1){
      lcd.setCursor(9,1);
      lcd.print("nospin");
  }else if (payload.disable == 2){
      lcd.setCursor(9,1);
      lcd.print("nomove");
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