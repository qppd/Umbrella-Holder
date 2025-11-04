#ifndef I2C_DISPLAY_H
#define I2C_DISPLAY_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class I2cDisplay {
public:
    I2cDisplay(uint8_t address = 0x27, uint8_t columns = 20, uint8_t rows = 4);
    void init();
    void clear();
    void clearLine(int y); // Clear specific line
    void setText(const String& text, int x, int y);
    void setText(double value, int x, int y);
    void setText(float value, int x, int y);
    void setText(int value, int x, int y);
    void setText(char text, int x, int y);
    void setTextPadded(const String& text, int x, int y, int width); // Pad with spaces to prevent artifacts
    void setTextPadded(float value, int x, int y, int width, int decimals = 1);
    void setTextPadded(int value, int x, int y, int width);
private:
    LiquidCrystal_I2C lcd;
    uint8_t cols;
    uint8_t rows;
};

#endif // I2C_DISPLAY_H
