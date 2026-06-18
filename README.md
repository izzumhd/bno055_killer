# BNO055 Killer (9-DoF AHRS Fusion)

A high-performance Arduino/C++ library for 9-DoF (Degrees of Freedom) sensor fusion using the **Madgwick AHRS** algorithm. It is designed to work with your choice of Magnetometer (**QMC5883L** or **HMC5883L**) and three popular 6-axis IMUs:
* **LSM6DS3**
* **BMI160**
* **MPU6050**

As the name implies, this library is designed as an alternative (and potentially **outperforms**) the legendary **BNO055** smart sensor, offering much lower latency, lower noise, and independent filter tuning control. It is highly suitable for Drones, Balancing Robots, Camera Gimbals, or other high-precision telemetry systems.

---

## Key Features

- **Extremely High Update Rate:** Not limited to 100Hz like the BNO055. When using a 32-bit MCU (STM32, ESP32, Teensy), the fusion rate can run at >400Hz - 800Hz for incredibly smooth latency.
- **Modern Low-Noise Sensors:** The LSM6DS3 features superior noise density compared to the older sensor architecture inside the BNO055.
- **Transparent Filter (No Black-Box):** You have full control over the fusion algorithm's sensitivity by adjusting the `Beta` parameter in the Madgwick filter.
- **Auto-EEPROM Calibration:** Provides an interactive calibration function for Gyroscope (Bias) and Magnetometer (Hard-Iron & Soft-Iron), with results saved permanently to the microcontroller's EEPROM.
- **BNO055-like API:** The header-only interface is designed to be very beginner-friendly, just like using the BNO055: `imu.getYaw()`, `imu.getPitch()`, `imu.getRoll()`.
- **Multi-Sensor Support:** Out of the box, it includes drivers for **LSM6DS3**, **BMI160**, and **MPU6050** IMUs, as well as **QMC5883L** and **HMC5883L** magnetometers. The mathematical algorithms are completely separated from the hardware drivers, making it extremely easy to switch between sensors.

---

## Trade-offs

While this library can outperform hardware-fusion sensors in speed and noise, it comes with architectural trade-offs:
- **Host CPU Load:** The BNO055 has an internal processor to calculate quaternions, meaning zero math load for your Arduino. This library calculates the complex Madgwick floating-point math on your main microcontroller. Therefore, a **32-bit MCU (STM32, ESP32, Teensy)** with a hardware FPU is highly recommended. Running this on an 8-bit Arduino Uno/Nano is possible but will be slow and consume significant program memory.
- **Manual Initial Calibration:** The BNO055 continuously runs auto-calibration in the background. With this library, you must manually perform the "Figure-8" calibration routine once and save it to the EEPROM for accurate Yaw tracking. *(Note: While less plug-and-play, this static calibration actually prevents the random heading jumps that sometimes plague the BNO055's unpredictable background auto-calibration).*

---

## How to Switch Sensors (IMU & Magnetometer)

By default, the library uses the `LSM6DS3` and `QMC5883L`. To switch to a different sensor combination:
1. Open the file `src/imu_handler.h`.
2. Look for the `IMU selection` and `Magnetometer selection` blocks (around line 138).
3. **Uncomment** the sensors you want to use, and **Comment out** the others.

```cpp
    // IMU selection - Select the IMU to use by uncommenting the corresponding line
    LSM6DS3Driver  _imu; // uncomment when using LSM6DS3
    // BMI160Driver  _imu; // uncomment when using BMI160
    // MPU6050Driver  _imu; // uncomment when using MPU6050
    // =============================

    // =============================
    // Magnetometer selection - Select the Mag to use by uncommenting the corresponding line
    QMC5883LDriver _mag; // uncomment when using QMC5883L (GY-271)
    // HMC5883LDriver _mag; // uncomment when using HMC5883L
```
That's it! The entire library will automatically adapt to your new hardware combination.

---

## Hardware Requirements (Wiring & Pinout)

This library requires an **I2C** connection. Here is a standard wiring example (e.g., on STM32 / ESP32):

| Sensor Pin | MCU Connection | Additional Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Logic level must be 3.3V! |
| **GND** | GND | |
| **SDA** | I2C SDA | |
| **SCL** | I2C SCL | |
| **AD0 / SA0** | GND | Sets the I2C Address to the default expected by the drivers |
| **CS** | VCC / 3.3V | Forces SPI-capable sensors (like LSM6DS3/BMI160) into I2C mode |

> **Note:** The QMC5883L address is fixed at `0x0D`. Ensure you are actually using a **QMC5883L** module, not an HMC5883L, as their registers differ (blue GY-271 modules mostly contain QMC5883L chips nowadays).

---

## Installation

1. Download or Clone this repository.
2. If using the Arduino IDE, move the `bno055_killer` folder into your `Documents/Arduino/libraries/` folder.
3. *(For PlatformIO users, place it in your project's `lib/` folder).*
4. Run `Wire.begin()` and `Wire.setClock(400000)` in your `setup()` before initializing the library.

---

## Quick Start (Reading Yaw, Pitch, Roll)

A minimal code example to read 3D orientation angles.

```cpp
#include <Wire.h>
#include <bno055_killer.h>

IMUHandler imu;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);

    Wire.begin();
    Wire.setClock(400000UL); // Use I2C Fast-Mode (400kHz)

    if (!imu.begin()) {
        Serial.println("Failed to initialize LSM6DS3! Check wiring.");
        while (1);
    }
}

void loop() {
    // 1. Must be called in every loop iteration for fusion computation!
    imu.update();

    // 2. Display Output in Serial Plotter
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 20) {
        lastPrint = millis();
        Serial.print("Yaw:");   Serial.print(imu.getYaw());
        Serial.print(", Pitch:"); Serial.print(imu.getPitch());
        Serial.print(", Roll:");  Serial.println(imu.getRoll());
    }
}
```
*A comprehensive example with all features is available in `bno055_killer_demo.ino` inside the `/examples/` folder.*

---

## How to Calibrate

Unlike the BNO055 which quietly performs background calibration, this custom system requires a **Manual Calibration (Only Once)** for accurate compass (Yaw) results. Without calibration, the Yaw will drift unpredictably.

To perform calibration, use the `bno055_killer_demo.ino` example script, open the Serial Monitor, and follow these steps:
1. Type `g` in the Serial Monitor, and leave the sensor **perfectly still for 2 seconds** to calibrate the Gyroscope (Finding zero-rate bias).
2. Type `m` to start the compass calibration mode.
3. Pick up the sensor and **slowly rotate it in a Figure-8 pattern** in the air. Ensure the sensor covers all orientations (up, down, tilted, upside-down) for about 10-15 seconds.
4. Type `f` to finish the calibration calculation.
5. Type `s` to **Save the results to EEPROM**.
6. Done! From now on, every time the microcontroller boots up, the calibration data will be loaded automatically.

---

## Examples Included

The `examples/` directory contains several sketches to help you get started:
- **`read_ypr`**: The simplest example to read Yaw, Pitch, and Roll.
- **`calibration`**: Semi-automatic interactive calibration utility to save Gyro and Hard/Soft-Iron offsets to EEPROM.
- **`read_raw_data`**: For reading pure accelerometer, gyroscope, and magnetometer data without the AHRS fusion.
- **`heading_hold_pid`**: Demonstrates how to use the built-in wrap-around PID controller to maintain a specific heading (useful for robots/drones).
- **`quaternion_visualizer`**: Outputs data specially formatted for 3D visualization tools (like Processing) and includes an ASCII 3D cube and Attitude Indicator.
- **`bno055_killer_demo`**: The "kitchen sink" demo showing all features, serial commands, and various data output modes.

---

## Library Structure (For Developers)

This library is highly modular and separated by responsibility:
- `imu_handler.h/cpp` : Main class (High-level interface).
- `madgwick.h/cpp` : Sensor Fusion algorithm (Quaternion math).
- `kinematics.h/cpp` : Coordinate transformation and conversion (Quaternion to Euler degrees).
- `calibration.h/cpp` : Hard-iron/Soft-iron compensation calculation & EEPROM management.
- `lsm6ds3_driver.h/cpp` : Low-level read/write registers specifically for the LSM6DS3 chip.
- `qmc5883l_driver.h/cpp` : Low-level read/write registers specifically for the QMC5883L chip.
- `pid.h` : Bonus PID Controller utility with wrap-around support (useful for robotic heading control).

---

## License
Following standard Open Source projects, this code is distributed under the MIT License. You are free to use, modify, and include it in commercial projects.
