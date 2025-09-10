#ifndef I2C_DISPLAY_H
#define I2C_DISPLAY_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class I2cDisplay {
public:
    I2cDisplay(uint8_t address = 0x27, uint8_t columns = 20, uint8_t rows = 4);
    void init();
    void clear();
    void setText(const String& text, int x, int y);
    void setText(double value, int x, int y);
    void setText(float value, int x, int y);
    void setText(int value, int x, int y);
    void setText(char text, int x, int y);
private:
    LiquidCrystal_I2C lcd;
};

#endif // I2C_DISPLAY_H
