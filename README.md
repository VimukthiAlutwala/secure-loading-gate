# Secure Industrial Loading Gate

ESP32-based IoT loading gate built for **COMP50069 – Hardware, Microcontrollers and Sensors** (APIIT Sri Lanka / University of Staffordshire), Scenario 1.

The gate opens automatically when a vehicle is detected, closes when the path is clear, and stops if anything is in the way while closing. After two automatic recoveries, a repeated obstruction locks the gate until a worker presses Reset.

**Wokwi simulation:** [paste your Wokwi link here]

## Features

- Three modes: Autonomous, Manual, Safety
- PIR vehicle detection using a hardware interrupt
- Ultrasonic obstruction detection (20 cm threshold)
- Potentiometer sets hold-open time (autonomous) or gate position (manual)
- Retry-then-latch safety recovery with a physical Reset button
- Non-blocking timing (hardware timer and `millis()`, no `delay()`)
- Live status on an I2C OLED and the serial monitor

## Hardware and Wiring

| Component | ESP32 pin |
|---|---|
| PIR sensor (OUT) | GPIO 14 |
| Ultrasonic TRIG / ECHO | GPIO 5 / GPIO 18 |
| Potentiometer | GPIO 34 |
| Servo signal | GPIO 13 |
| Buzzer | GPIO 12 |
| Reset button | GPIO 4 |
| OLED SDA / SCL | GPIO 21 / GPIO 22 |

## How to Run

**Simulation:** open the Wokwi link above and press play.

**Hardware:**
1. Install the Arduino IDE with the ESP32 board package (version 3.x).
2. Install the libraries: Adafruit SSD1306, Adafruit GFX, ESP32Servo.
3. Open `firmware/secure_loading_gate_v2/secure_loading_gate_v2.ino` and upload it to the ESP32.
4. Open the Serial Monitor at 115200 baud.

## Serial Commands

| Command | Action |
|---|---|
| `a` | Autonomous mode |
| `m` | Manual mode |
| `r` | Clear a locked fault |

Mode commands are ignored while a safety fault is active.

## Repository Structure

```
firmware/     ESP32 source code
simulation/   Wokwi wiring (diagram.json)
docs/         State diagram
```

## Author

Vimukthi Alutwala – CB015497
