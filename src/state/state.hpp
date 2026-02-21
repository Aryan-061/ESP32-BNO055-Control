#pragma once
#include <cmath>

// Quaternion struct
struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

// Global state shared across modules
struct State {
    // --- Attitude ---
    Quaternion q;        // orientation (body → world)

    // --- Angular rates (body frame) ---
    float wx;             // roll rate  (rad/s)
    float wy;             // pitch rate (rad/s)
    float wz;             // yaw rate   (rad/s)

    // --- Optional: Euler angles (derived, not used for control) ---
    float roll;
    float pitch;
    float yaw;
};