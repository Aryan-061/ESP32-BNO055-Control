#include <Arduino.h>
#include "imu/imu.hpp"

// math / quaternion utilities
#include "math/quaternion.hpp"

// control interface
#include "control/control.hpp"
int motorL = 0;
int motorR = 0;
int motorB = 0;
// -------------------- Globals --------------------
BNO055 imu;

// Desired attitude (identity = level, no rotation)
static Quaternion q_ref = { 1.0f, 0.0f, 0.0f, 0.0f };

void setup() {
    Serial.begin(115200);
    delay(1000);

    if (!imu.begin()) {
        Serial.println("BNO055 init failed");
        while (1);
    }

    Serial.println("BNO055 OK, starting control loop");

    // Initialize controller if required
    //Control::init();   // <-- remove ONLY if your control has no init()
}

void loop() {
    float w, x, y, z;

    if (!imu.readQuaternion(w, x, y, z)) {
        return;
    }

    // -------------------- Measured orientation --------------------
    Quaternion q_meas = { w, x, y, z };

    // Optional safety normalization
    quatNormalize(q_meas);

    // -------------------- Quaternion error --------------------
    // q_err = q_ref ⊗ conj(q_meas)
    Quaternion q_err = quatMultiply(q_ref, quatConjugate(q_meas));

    // -------------------- Small-angle attitude error --------------------
    float ex = 2.0f * q_err.x;   // roll error
    float ey = 2.0f * q_err.y;   // pitch error
    float ez = 2.0f * q_err.z;   // yaw error

    // -------------------- Control update --------------------
    Control::update();//ex, ey, ez;
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // THIS is where PID / LQR consumes quaternion-based errors

    // -------------------- Debug (temporary) --------------------
    Serial.print("err: ");
    Serial.print(ex, 4); Serial.print(" ");
    Serial.print(ey, 4); Serial.print(" ");
    Serial.println(ez, 4);

    delay(5); // ~200 Hz control loop
}