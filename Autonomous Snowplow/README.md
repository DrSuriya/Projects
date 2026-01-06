# Autonomous Snowplow Robot (SYSC 4805)
**Role:** Hardware Wiring, System Integration & RTOS Designer
**Group:** 18 (Team Maroon)
**Hardware:** Arduino Due, Cytron Driver, 3 Ultrasonic Sensors, 2 IR Line Sensors
**Firmware:** C++ (FreeRTOS & Sequential versions)

![Robot Hardware Integration](robot_chassis.jpg)

## 1. Project Overview
This project involved the design and implementation of an autonomous robot tasked with clearing "snow" (small wooden blocks) from a defined arena.
* **Objective:** Clear a 100cm x 100cm enclosed space of 100 wooden cubes within 5 minutes.
* **Constraints:** The robot must remain within black tape boundaries and avoid larger static obstacles placed around the perimeter.

## 2. My Technical Contributions
My primary role was **System Integration** and **Firmware Architecture**. I was responsible for connecting the individual sensor/hardware modules into a cohesive system and implementing the Real-Time Operating System (RTOS).

### Key Responsibilities
* **Hardware Integration:** Wired and integrated the Ultrasonic sensors, Line Detectors, and Motor Drivers to the Arduino Due, managing power distribution and signal integrity.
* **Program Logic Implementation:** Integrated all sensor and motor code into a single file and designed the sequential control loop to avoid obstacles and stay within the perimeter.
* **RTOS Implementation:** Migrated the system from a sequential loop to **FreeRTOS** to ensure deterministic timing. This allowed the robot to poll sensors and control motors concurrently without blocking execution.

## 3. Demonstration Performance Results
Both demonstrations were conducted using the RTOS implementation of the firmware.

| Metric | Attempt 1 | Attempt 2 |
| :--- | :--- | :--- |
| **Blocks Cleared** | **32 / 100** | **41 / 100** |
| **Obstacle Collisions** | 2 | 5 |
| **Configuration** | High sensitivity for obstacle detection (Longer detection distance). | Reduced detection distance (Lower interrupt threshold). |
| **Analysis** | **High Safety / Low Efficiency:** The robot triggered avoidance frequently (false positives), which reduced clearing time. It successfully avoided obstacles but missed blocks near the perimeter. | **Higher Efficiency / Lower Safety:** Reducing sensitivity allowed the robot to clear blocks closer to the obstacles. However, at higher speeds, the sensors occasionally failed to register the obstacle in time, leading to collisions. |

## 4. System Architecture
The final version utilizes **FreeRTOS** to manage three concurrent tasks:
1.  **SensorTask (High Priority):** Polls Ultrasonic and Line sensors at 50Hz; updates a shared data structure protected by a Mutex to prevent race conditions.
2.  **MotorTask (Medium Priority):** Consumes sensor data to drive the H-Bridge; executes avoidance maneuvers when safety thresholds are breached.
3.  **WatchdogTask (Highest Priority):** A supervisor task that monitors system health. It "kicks" the hardware watchdog only if all other tasks report their active status bitmasks, ensuring the system auto-resets if a task hangs.




