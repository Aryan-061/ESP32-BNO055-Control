First Test – IMU Validation & Control Readiness
Objective - The goal of this first test was to validate the BNO055 IMU orientation data. No motors or thrusters were involved at this stage.

What Was Tested: //CONFIG_MODE (was included)//
i) BNO055 running in NDOF_FMC_OFF fusion mode. (w/o yaw)
ii) Quaternion-based orientation output. (can be later used to get calculated roll, pitch and ~yaw~ for reading ease)
iii) Validity of custom library using hard i2c reads(for IMU), instead of using Adafruit or 7Semi library.
iv) Controls on ESP32 with IMU
System behavior during:

a) Small tilts

b) Large tilts (approaching ±90°)

Key Observations:
1. Stable Orientation Estimates -
 |->Quaternion data changed smoothly with physical motion.
  |->No sudden jumps, freezes, or discontinuities.
   |->No instability near large angles.
*This confirms that sensor fusion and IMU configuration are working correctly.*

2. Large-Angle Robustness -
  |->Orientation data remained valid even near extreme tilts
   |->No gimbal-lock–like behavior in quaternion output
    |->Any odd behavior observed when converting to Euler angles is due to representation limits, not sensor failure.
*(Meaning no real sensor or math issues , but due to faulty representation in Euler coordinate system)*

3. Quaternion Error is Reliable -
The attitude error is computed as:
   |->q_err = q_ref × conjugate(q_current)
    |->This quaternion represents the rotation required to go from the current orientation to the desired (level) orientation.

A) For small tilts:
//{
q_err.x ≈ roll error (radians)
q_err.y ≈ pitch error (radians)
//}
*This approximation is valid because the controller operates close to level.*
*Yaw (z) is ignored since it does not affect balance or stability.*

Why This Data Is Good -
 This test confirms that:
  |->The IMU provides continuous, predictable and reliable orientation data
   |->The attitude error signal is smooth and well-behaved
    |->The system is safe to use as feedback for control loops and for the application of filters for fusion.
*This is critical — unstable or noisy orientation data would make closed-loop control unsafe once thrusters are introduced.*

B) What This Enables Next:
With sensing validated, the project can now safely move to control integration:

  a) Outer loop (PID-like)
    ->Converts attitude error → desired angular rates
  b) Inner loop (LQR)
    ->Converts angular rate error → thruster commands
  c) Actuation testing
  d) Dry thruster tests
  e) Then in-water tests

Structure:

  IMU Driver
       ↓
Quaternion Math
       ↓
Control Logic (PID + LQR)
       ↓
Motor / Thruster Mapping

Because of this separation:
  |->The control logic is hardware-independent
   |->Porting from ESP32 to RP2040 (Pico) will only require driver and PWM changes

