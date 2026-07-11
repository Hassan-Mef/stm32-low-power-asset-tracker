# 🚀 Low-Power Asset Tracker with Geofence Buzzer Alert

> **Embedded Systems Project**  
> **Platform:** STM32L475 IoT Discovery Board (B-L475E-IOT01A1)  
> **Framework:** STM32 HAL + FreeRTOS (CMSIS-RTOS v2)

---

# 📖 Overview

This project implements a **Low-Power Asset Tracking System** capable of monitoring physical movement, acquiring GPS coordinates, evaluating geofence boundaries, and generating audible alerts whenever an asset leaves a predefined safe zone.

Unlike traditional GPS trackers that continuously consume power, this implementation focuses on **energy-efficient operation** by keeping the microcontroller in **low-power sleep mode** for most of its lifetime. The system only wakes when motion is detected by the onboard accelerometer, performs GPS acquisition and geofence evaluation, then returns to sleep when the asset becomes stationary.

---

# 🎯 Project Objectives

- Detect asset movement using the onboard accelerometer.
- Wake the MCU only when movement occurs.
- Acquire GPS coordinates.
- Calculate the distance from a predefined geofence.
- Trigger a buzzer when the asset leaves the allowed region.
- Return to low-power operation whenever stationary.
- Minimize energy consumption using FreeRTOS Tickless Idle.

---

# 🛠 Hardware Used

| Component | Description |
|-----------|-------------|
| STM32L475 IoT Discovery Board | Main MCU |
| LSM6DSL Accelerometer | Motion Detection |
| External GPS Module | Location Tracking |
| Passive Buzzer | Geofence Alert |
| ST-Link VCP | UART Debugging |

---

# 📂 System Architecture

(/images/system_architecture.png)


---

# 🧠 Software Architecture

> **Placeholder:** Replace with exported PlantUML diagram.

```text
/images/software_architecture.png
```

---

# ⚙️ FreeRTOS Architecture

## Tasks

| Task | Priority | Responsibility |
|------|----------|----------------|
| Motion Manager | High | Wake-up handling |
| GPS Manager | Medium | GPS acquisition & geofence |
| Buzzer Task | Low | Alarm generation |

## Synchronization

- Binary Semaphore (ISR → Motion Task)
- Event Groups
- Queue
- Software Timers

> **Placeholder:** FreeRTOS Task Diagram

```text
/images/freertos_architecture.png
```

---

# Motion Detection Flow

> **Placeholder**

```text
/images/motion_flow.png
```

---

# GPS Processing Flow

> **Placeholder**

```text
/images/gps_flow.png
```

---

# Geofence Evaluation

The firmware uses the **Haversine Formula** to calculate the distance between the current GPS position and the predefined geofence center.

Three consecutive out-of-bound GPS fixes are required before confirming a breach.

> **Placeholder**

```text
/images/geofence_flow.png
```

---

# Low-Power Design

The firmware spends most of its lifetime in Stop Mode.

Power optimization techniques include:

- FreeRTOS Tickless Idle
- Stop Mode
- GPS Power Switching
- Motion-triggered Wake-up
- Peripheral Shutdown

> **Placeholder**

```text
/images/power_flow.png
```

---

# Current Consumption

| Mode | Current |
|------|---------|
| Active | ~16 mA |
| Sleep | ~2 mA |

## Active Mode

```text
/images/active_current.jpg
```

## Sleep Mode

```text
/images/sleep_current.jpg
```

---

# UART Output

```text
Motion Detected
GPS Power ON
Latitude : xx.xxxxxx
Longitude: xx.xxxxxx
Distance : xx.xx m
INSIDE GEOFENCE
No Motion
GPS Power OFF
Entering Stop Mode
```

---

# Repository Structure

```text
.
├── Core/
├── Drivers/
├── Middlewares/
├── GPS/
├── LSM6DSL/
├── FreeRTOS/
├── Images/
└── README.md
```

---

# Features

- Motion-triggered wake-up
- GPS tracking
- Geofence monitoring
- PWM buzzer alerts
- FreeRTOS multitasking
- Tickless Idle
- Low-power firmware
- UART debugging

---

# Future Improvements

- LoRaWAN
- LTE/GSM
- BLE Mobile App
- SD Card Logging
- OTA Updates
- Cloud Dashboard

---

# Learning Outcomes

- FreeRTOS
- STM32 HAL
- Low-Power Embedded Systems
- GPS Integration
- Event-driven Firmware
- Embedded Power Optimization
