# Autonomous Snowplow Robot (SYSC 4805)
- **Role:** Hardware Wiring, System Integration & RTOS Designer
- **Hardware:** Arduino Due, Cytron Driver, 3 Ultrasonic Sensors, 2 IR Line Sensors
- **Firmware:** C++ (FreeRTOS & Sequential Loop versions)

![Robot Hardware Integration](robot_chassis.jpg)

## 1. Project Overview
This project involved the design and implementation of an autonomous robot tasked with clearing "snow" (small wooden blocks) from a defined arena.
* **Objective:** Clear a 2.5m x 2.5m enclosed space of 100 wooden cubes within 5 minutes.
* **Constraints:** The robot must remain within black tape boundaries and avoid larger static obstacles placed around the perimeter.

## 2. My Technical Contributions
* **Hardware Integration:** Wired and integrated the Ultrasonic sensors, Line Detectors, and Motor Drivers to the Arduino Due.
* **Program Logic Implementation:** Integrated all sensor and motor code into a single file and designed the sequential control loop to avoid obstacles and stay within the perimeter.
* **RTOS Implementation:** Migrated the system from a sequential loop to **FreeRTOS** to ensure deterministic timing. This allowed the robot to poll sensors and control motors concurrently without blocking execution.
* **Safety Mechanisms:** Designed and implemented a **Watchdog Timer** to detect system hangs and automatically reset the microcontroller if a critical failure occurred.

## 3. Demonstration Performance Results
Both demonstrations were conducted using the RTOS implementation of the firmware.
| Metric | Attempt 1 | Attempt 2 |
| :--- | :--- | :--- |
| **Blocks Cleared** | **32 / 100** | **20 / 100** |
| **Obstacle Collisions** | 2 | 5 |
| **Configuration** | Higher Ultrasonic detection distance. | Reduced Ultrasonic detection distance. |

**Discussion:** 
* **Line Sensor Variance:** While functional, the line sensors suffered from false positives due to the low contrast of the arena's grey floor (vs. white test surfaces). Additionally, despite identical mounting heights, the sensors showed significant variance. We prioritized boundary safety by setting a low threshold, which caused the right sensor (which consistently read higher) to trigger the turning algorithm more frequently.
* **Ultrasonic Signal Noise:** Ultrasonic instability was the primary cause of the performance drop in Attempt 2. Reducing the detection distance to clear blocks closer to obstacles made the sensors susceptible to background noise. This frequently trapped the robot in the turning mode loop when no obstacle was present, it also at points did not detect the obstacles despite the lower threshold leading to collisions.
* **Pathing Constraints:** The motor control logic was simplified to execute unidirectional turns (always turning right) upon detection. While simple to test, this limited pathing efficiency, often caused the robot to take longer to navigate out of corners.

## 4. System Architecture
The final version utilizes **FreeRTOS** to manage four concurrent tasks:
1.  **SensorTask (High Priority):** Polls Ultrasonic and Line sensors at 50Hz and updates a shared data structure protected by a Mutex to prevent race conditions.
2.  **MotorTask (Medium Priority):** Consumes shared sensor data to determine appropriate pathing and control the motor driver states.
3.  **DebugTask (Low Priority):** Prints the shared sensor data structure to the serial monitor for real-time analysis.
4.  **WatchdogTask (Highest Priority):** A supervisor task that monitors system health. It "kicks" the hardware watchdog only if all other tasks report their active status bitmasks, ensuring the system auto-resets if a task hangs.
