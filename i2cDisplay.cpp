#include "I2cDisplay.h"

I2cDisplay::I2cDisplay(uint8_t address, uint8_t columns, uint8_t rows)
    : lcd(address, columns, rows) {}

void I2cDisplay::init() {
    lcd.init();
    lcd.backlight();
}

void I2cDisplay::clear() {
    lcd.clear();
}

void I2cDisplay::setText(const String& text, int x, int y) {
    lcd.setCursor(x, y);
    lcd.print(text);
}
void I2cDisplay::setText(double value, int x, int y) {
    lcd.setCursor(x, y);
    lcd.print(value);
}
void I2cDisplay::setText(float value, int x, int y) {
    lcd.setCursor(x, y);
    lcd.print(value);
}
void I2cDisplay::setText(int value, int x, int y) {
    lcd.setCursor(x, y);
    lcd.print(value);
}
void I2cDisplay::setText(char text, int x, int y) {
    lcd.setCursor(x, y);
    lcd.print(text);
}
