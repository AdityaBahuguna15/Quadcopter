# 🚁 DIY Quadcopter — from scratch

Teensy 4.0 microcontroller, MPU-6050 IMU, BMP-280 barometer, Flysky FS-i6 RC system.
Three flight modes implemented in C++.

![build status](https://img.shields.io/badge/flight_modes-3-brightgreen)
![weight](https://img.shields.io/badge/weight-%3C800g-blue)
![flight time](https://img.shields.io/badge/flight_time-~10min-blue)

---

## What I applied

| Feature | Details |
|---|---|
| Gyroscope + I²C comms | Raw roll/pitch/yaw rates from MPU-6050 at 400 kHz. 10 Hz low-pass filter for motor vibration. |
| Gyro calibration | Startup offset averaging. LEDs blink during 4-second calibration. |
| ESC + motor control | PWM at 250 Hz to four ESCs. Motor arming with battery voltage pre-check (≥7.5 V). |
| RC receiver decoding | Flysky FS-i6 channels mapped to desired roll/pitch/yaw rates and throttle. |
| Battery management | Continuous voltage monitoring, motor output correction for sag, low-battery LED warning. |
| Rate mode PID | Three independent PID loops (roll, pitch, yaw) with integrator anti-windup. |
| 1-D Kalman filter | Fuses gyro integration + accelerometer tilt for stable roll and pitch angles. |
| Altitude (barometer) | BMP-280 → altitude via Bosch compensation. Referenced to startup altitude. |
| 2-D Kalman filter | Matrix Kalman fusing BMP-280 altitude + Z-axis accel for vertical velocity estimate. |
| Stabilize mode (cascade PID) | Outer angle loop feeds desired rate into inner rate loop. Enables self-leveling. |
| Vertical velocity control | PID on Kalman velocity. Throttle stick = ±150 cm/s. Hover hold without constant input. |
| Quadcopter dynamics | Transfer functions for roll/pitch/yaw/velocity derived from inertia + motor specs. |

---

## Stack

`Teensy 4.0` `MPU-6050` `BMP-280` `Flysky FS-i6` `T-Motor Velox V2207 1950kV` `45A ESC` `6S LiPo` `C++ / Arduino`

---

## Media

### Photos
<!-- Insert photos below -->
| | |
|---|---|
| ![CAD](pictures/CAD.png) | SolidWorks prototyping |
| ![circuit](pictures/Circuit.png) | Electronics prototyping |
| ![build](pictures/Assembled.jpg) | Assembled frame + electronics |
| ![serial](pictures/Kalman.png) | Serial monitor — Kalman output |

### Videos
![Test_Flight](pictures/DroneVideo.mp4)

Test objective: Validating the rate mode PID controller's response to rapid, intentional disturbances — erratic inputs along a circular path to stress-test how quickly the controller corrects roll and pitch rates back to zero. Flight times were kept deliberately short because, well... this was my room. Do not try this at home. Or do, just don't blame me when you see the drone flying at you at mach 10 or better, your drone goes kaboom and you set off the fire alarm. 🚁💥

Built and programmed from the ground up using the Carbon Aeronautics manual.
