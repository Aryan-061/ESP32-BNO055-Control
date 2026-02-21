#include "imu/imu.hpp"

/* ================= BNO055 Registers ================= */
static constexpr uint8_t REG_OPR_MODE        = 0x3D;
static constexpr uint8_t REG_PWR_MODE        = 0x3E;
static constexpr uint8_t REG_SYS_TRIGGER     = 0x3F;

/* Quaternion data starts here (8 bytes total) */
static constexpr uint8_t REG_QUA_DATA_W_LSB  = 0x20;

/* ================= Modes ================= */
static constexpr uint8_t MODE_CONFIG         = 0x00;
static constexpr uint8_t MODE_NDOF_FMC_OFF   = 0x0B;

/* ================= Power ================= */
static constexpr uint8_t POWER_NORMAL        = 0x00;

/* =================================================== */

BNO055::BNO055(uint8_t addr) : _addr(addr) {}

bool BNO055::begin() {
    Wire.begin();

    /* Enter CONFIG mode (MANDATORY) */
    write8(REG_OPR_MODE, MODE_CONFIG);
    delay(25);

    /* Reset internal fusion MCU */
    write8(REG_SYS_TRIGGER, 0x20);
    delay(650);

    /* Set normal power mode */
    write8(REG_PWR_MODE, POWER_NORMAL);
    delay(10);

    /* Start fusion (NDOF, no fast mag cal) */
    write8(REG_OPR_MODE, MODE_NDOF_FMC_OFF);
    delay(25);

    return true;
}

bool BNO055::readQuaternion(float &w, float &x, float &y, float &z) {
    uint8_t buf[8];

    readLen(REG_QUA_DATA_W_LSB, buf, 8);

    int16_t qw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t qx = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t qy = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t qz = (int16_t)((buf[7] << 8) | buf[6]);

    /* Datasheet scale: 1 LSB = 1 / 16384 */
    w = qw / 16384.0f;
    x = qx / 16384.0f;
    y = qy / 16384.0f;
    z = qz / 16384.0f;

    return true;
}

/* ================= Low-level I2C ================= */

void BNO055::write8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void BNO055::readLen(uint8_t reg, uint8_t *buffer, uint8_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);   // repeated start

    Wire.requestFrom(_addr, len);
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = Wire.read();
    }
}