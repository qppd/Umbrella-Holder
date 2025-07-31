#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27, 16, 2);
LiquidCrystal_I2C lcd(0x27, 20, 4);



//-----------------------------------------------------------------
//FUNCTION FOR CLEARNING MESSAGE FROM I2C LCD----------------------
//-----------------------------------------------------------------
void initLCD(){
  lcd.init();
  lcd.backlight();
}


//-----------------------------------------------------------------
//FUNCTION FOR CLEARNING MESSAGE FROM I2C LCD----------------------
//-----------------------------------------------------------------
void clearLCD(){
  lcd.clear();
}



//-----------------------------------------------------------------
//FUNCTION FOR SETTING MESSAGE I2C LCD-----------------------------
//-----------------------------------------------------------------
void setLCDText(String text, int x, int y) {
  lcd.setCursor(x, y);
  lcd.print(text);
}
void setLCDText(double value, int x, int y) {
  lcd.setCursor(x, y);
  lcd.print(value);
}
void setLCDText(float value, int x, int y) {
  lcd.setCursor(x, y);
  lcd.print(value);
}
void setLCDText(int value, int x, int y) {
  lcd.setCursor(x, y);
  lcd.print(value);
}
void setLCDText(char text, int x, int y) {
  lcd.setCursor(x, y);
  lcd.print(text);
}