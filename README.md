# Line-Following Robot with PID Control

This project implements a **line-following robot** using **PID (Proportional–Integral–Derivative) control** on an Arduino-based system.
The robot reads analog values from infrared (IR) sensors to detect the line and adjusts the speed of its motors to stay centered.

Demo: https://youtu.be/5oXEWmr-w1s

---

## Features

* **PID control loop** to correct deviation from the line.
* **Three IR sensors** (left, center, right) for line detection.
* **PWM motor control** for smooth movement.
* Configurable parameters for **base speed**, **minimum speed**, and **maximum speed**.
* Tunable **PID constants** for fine adjustments.

---

## How It Works

1. The IR sensors measure reflectivity:

   * **Black line = lower value**
   * **White surface = higher value**
2. The error is calculated as the difference between the left and right sensors.
3. PID control is applied:

   * **P** → proportional correction based on error.
   * **I** → accumulates past error.
   * **D** → predicts future error by measuring rate of change.
4. The robot adjusts left and right motor speeds accordingly.

---

## Pinout

### Motors

* `AIA = 3`
* `AIB = 11`
* `BIA = 10`
* `BIB = 6`

### Sensors

* IR sensors (digital control):

  * Left: `9`
  * Center: `7`
  * Right: `8`
* Analog inputs (reading sensor values):

  * Right: `A0`
  * Center: `A1`
  * Left: `A2`

### LED

* `ledTest = 13` (status indicator)

---

## PID Tuning Parameters

You can adjust the following constants to fine-tune performance:

```cpp
float Kp2 = 0.0008;   // Proportional gain factor
float Kd2 = 0.1;      // Derivative gain factor
float Ki2 = 0.0001;   // Integral gain factor
```

They are dynamically scaled depending on the sensor readings.

---

## Adjustable Speed Settings

```cpp
int baseSpeed = 200;   // Default motor speed
int maxSpeed  = 255;   // Maximum PWM speed
int minSpeed  = 75;    // Minimum PWM speed
```

---

## Installation

1. Connect your Arduino and motors according to the **pinout** above.
2. Upload the code using the Arduino IDE.
3. Place the robot on a track with a black line on a white surface.
4. Adjust PID constants if needed for stability.

---

## Usage

* When the center sensor detects the line (`> 450`), PID control is activated.
* The robot continuously reads sensors and applies corrections to follow the line.
* The built-in LED blinks at startup for a quick test.

---

## Future Improvements

* Implement **automatic calibration** for sensor thresholds.
* Add **serial output logging** for PID values and error debugging.
* Support for **different track shapes** and speeds.

---

Would you like me to also create a **schematic diagram (in Markdown with ASCII or using Fritzing-style description)** to visually show the wiring in the README?
