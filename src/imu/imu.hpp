#pragma once
#include <Arduino.h>
#include <Wire.h>

class BNO055 {
public:
    explicit BNO055(uint8_t addr = 0x28);

    bool begin();
    bool readQuaternion(float &w, float &x, float &y, float &z);

private:
    uint8_t _addr;

    void write8(uint8_t reg, uint8_t value);
    void readLen(uint8_t reg, uint8_t *buffer, uint8_t len);
};