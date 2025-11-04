#include "I2cDisplay.h"

I2cDisplay::I2cDisplay(uint8_t address, uint8_t columns, uint8_t rows)
    : lcd(address, columns, rows), cols(columns), rows(rows) {}

void I2cDisplay::init() {
    lcd.init();
    lcd.backlight();
}

void I2cDisplay::clear() {
    lcd.clear();
}

void I2cDisplay::clearLine(int y) {
    lcd.setCursor(0, y);
    for (int i = 0; i < cols; i++) {
        lcd.print(' ');
    }
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

void I2cDisplay::setTextPadded(const String& text, int x, int y, int width) {
    lcd.setCursor(x, y);
    String padded = text;
    while (padded.length() < width) {
        padded += ' ';
    }
    if (padded.length() > width) {
        padded = padded.substring(0, width);
    }
    lcd.print(padded);
}

void I2cDisplay::setTextPadded(float value, int x, int y, int width, int decimals) {
    lcd.setCursor(x, y);
    char buffer[20];
    dtostrf(value, width, decimals, buffer);
    lcd.print(buffer);
}

void I2cDisplay::setTextPadded(int value, int x, int y, int width) {
    lcd.setCursor(x, y);
    char buffer[20];
    sprintf(buffer, "%*d", width, value);
    lcd.print(buffer);
}
