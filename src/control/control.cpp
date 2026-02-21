#include "control/control.hpp"
#include "state/state.hpp"
#include "math/quaternion.hpp"
#include <algorithm>
#include <cmath>

extern State state;
extern int motorL, motorR, motorB;

static constexpr float dt = 0.01f;

// Outer loop gains (attitude)
static constexpr float Kp_att = 6.0f;
static constexpr float Ki_att = 0.5f;

static float int_err_x = 0;
static float int_err_y = 0;

// Inner loop LQR 
static float K_lqr[3][2] = {
    { 1.5f, -1.0f },
    { -1.5f, -1.0f },
    { 0.0f,  2.0f }
};

static constexpr float U_MAX = 1.0f;
static float u_smooth[3] = {0,0,0};
static constexpr float beta = 0.2f;

static inline float constrain(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

int clampDSHOT(float value) {
    if (value >= 0)
        return constrain((int)(value + 48), 0, 1000);
    else
        return constrain((int)(-value + 1049), 1001, 2000);
}

void Control::update() {

    // Desired attitude = level
    Quaternion q_ref = { 1, 0, 0, 0 };
   

    // Attitude error quaternion
    Quaternion q_err = quatMultiply(q_ref, quatConjugate(state.q));
    quatNormalize(q_err);

    // Small-angle approx: vector part ≈ attitude error
    float ex = q_err.x;
    float ey = q_err.y;

    // Integrator
    int_err_x += ex * dt;
    int_err_y += ey * dt;

    int_err_x = constrain(int_err_x, -0.1f, 0.1f);
    int_err_y = constrain(int_err_y, -0.1f, 0.1f);

    // Desired angular rates
    float wx_ref = Kp_att * ex + Ki_att * int_err_x;
    float wy_ref = Kp_att * ey + Ki_att * int_err_y;

    // Angular rate error
    float omega_err[2] = {
        state.wx - wx_ref,
        state.wy - wy_ref
    };

    // LQR control
    float u[3];
    for (int i = 0; i < 3; i++) {
        u[i] = -(K_lqr[i][0] * omega_err[0] +
                 K_lqr[i][1] * omega_err[1]);
    }

    float umax = std::max({fabs(u[0]), fabs(u[1]), fabs(u[2])});
    if (umax > U_MAX) {
        for (int i = 0; i < 3; i++)
            u[i] *= U_MAX / umax;
    }

    for (int i = 0; i < 3; i++)
        u_smooth[i] = beta * u[i] + (1 - beta) * u_smooth[i];

    motorL = clampDSHOT(u_smooth[0] * 150);
    motorR = clampDSHOT(u_smooth[1] * 150);
    motorB = clampDSHOT(u_smooth[2] * 150);
}