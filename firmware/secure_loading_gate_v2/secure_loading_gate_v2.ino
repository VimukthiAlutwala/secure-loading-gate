/**
 * @file    secure_loading_gate_v2.ino
 * @brief   Scenario 1 - Secure Industrial Loading Gate (ESP32 firmware, v2).
 *
 * COMP50069 - Hardware, Microcontrollers and Sensors.
 *
 * Modes:  AUTONOMOUS  |  MANUAL  |  SAFETY
 * Timing: millis() for scheduling + one ESP32 hardware timer ISR + one PIR
 *         interrupt. There is deliberately NO delay() in the main loop.
 *
 * Design challenge: hybrid retry-then-latch recovery. After an obstruction
 * while closing, the gate retreats and idles (up to MAX_AUTO_RETRIES times);
 * on the next trip it latches until a worker performs a manual reset.
 *
 * v2 changes:
 *   - Mode commands ('a' / 'm') are refused while in SAFETY mode, so a latched
 *     fault can only be cleared by the Error Reset (button or 'r').
 *   - Returning to AUTONOMOUS from MANUAL now resumes the cycle from the gate's
 *     actual position, so a part-open gate closes itself once the path is clear.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

/* ------------------------------------------------------------------ */
/*  Pin map (ESP32 DevKit-C v4). See wiring guide for the reasoning.   */
/* ------------------------------------------------------------------ */
#define PIN_PIR        14   ///< PIR motion OUT  (digital in, interrupt)
#define PIN_TRIG        5   ///< Ultrasonic TRIG (digital out)
#define PIN_ECHO       18   ///< Ultrasonic ECHO (digital in)
#define PIN_POT        34   ///< Potentiometer wiper (ADC1_CH6, INPUT-ONLY pin)
#define PIN_SERVO      13   ///< Servo signal (PWM via LEDC)
#define PIN_BUZZER     12   ///< Buzzer (digital out)
#define PIN_RESET_BTN   4   ///< Error Reset push-button (digital in, pullup)
/* OLED runs on the ESP32 default I2C bus: SDA = 21, SCL = 22 */

/* ------------------------------------------------------------------ */
/*  Peripherals                                                        */
/* ------------------------------------------------------------------ */
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

Servo gateServo;
#define GATE_CLOSED_ANGLE  0    ///< servo angle when fully closed
#define GATE_OPEN_ANGLE   90    ///< servo angle when fully open

#define SAFE_DISTANCE_CM  20    ///< obstruction threshold while closing

/* ---- Hybrid safety recovery tuning (design challenge) ---- */
#define MAX_AUTO_RETRIES  2      ///< auto-retreat this many times, then latch
#define SAFETY_IDLE_MS    12000  ///< hold-open window after a retreat (~12 s for demo; minutes in production)

/* ------------------------------------------------------------------ */
/*  State machine                                                      */
/* ------------------------------------------------------------------ */
enum SystemMode { MODE_AUTONOMOUS, MODE_MANUAL, MODE_SAFETY };
volatile SystemMode currentMode = MODE_AUTONOMOUS;

/** Sub-states used while the system is autonomously running the gate. */
enum GateState { GATE_CLOSED_IDLE, GATE_OPENING, GATE_OPEN_HOLD, GATE_CLOSING };
GateState gateState = GATE_CLOSED_IDLE;

/** Phases the system moves through once a safety halt is triggered. */
enum SafetyPhase { SAFE_RETREATING, SAFE_IDLE_HOLD, SAFE_LATCHED };
SafetyPhase safetyPhase = SAFE_RETREATING;

int  tripCount     = 0;   ///< safety trips since the last clean close cycle
unsigned long safetyIdleStartMs = 0;  ///< when the retreat idle window began

/* ------------------------------------------------------------------ */
/*  Non-blocking timing state                                         */
/* ------------------------------------------------------------------ */
int  currentServoAngle   = GATE_CLOSED_ANGLE;
unsigned long lastServoStepMs = 0;
const unsigned long SERVO_STEP_INTERVAL_MS = 15;  ///< 1 degree every 15 ms

unsigned long holdStartMs   = 0;
unsigned long holdOpenDelayMs = 3000;  ///< tuned live by the potentiometer

/* --- ESP32 hardware timer: raises a flag every 100 ms for sampling --- */
volatile bool sampleFlag = false;
hw_timer_t *sampleTimer = NULL;
void IRAM_ATTR onSampleTimer() { sampleFlag = true; }

/* --- PIR interrupt: sets a flag the loop consumes --- */
volatile bool motionFlag = false;
void IRAM_ATTR onMotion() { motionFlag = true; }

/* ================================================================== */
/*  SETUP                                                             */
/* ================================================================== */
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
  // PIN_POT (GPIO34) needs no pinMode; it is analogue input-only.

  // OLED init (I2C address 0x3C is standard for the 128x64 SSD1306)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init failed"));
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();

  // Servo on LEDC PWM
  gateServo.attach(PIN_SERVO, 500, 2400);   // min/max pulse for SG90
  gateServo.write(GATE_CLOSED_ANGLE);

  // PIR motion interrupt (rising edge = motion started)
  attachInterrupt(digitalPinToInterrupt(PIN_PIR), onMotion, RISING);

  // Hardware timer (ESP32 Arduino core 3.x API).
  // Run the timer at 1 MHz so 1 tick = 1 us, then fire every 100000 us = 100 ms.
  sampleTimer = timerBegin(1000000);                 // frequency in Hz
  timerAttachInterrupt(sampleTimer, &onSampleTimer); // 2-arg form in core 3.x
  timerAlarm(sampleTimer, 100000, true, 0);          // period, auto-reload, count

  Serial.println(F("Gate ready. Serial cmds: a=Auto  m=Manual  r=Reset"));
}

/* ================================================================== */
/*  MAIN LOOP  (must stay non-blocking - no delay() here)             */
/* ================================================================== */
void loop() {
  handleSerial();          // UART configuration / mode input

  if (sampleFlag) {        // runs ~every 100 ms, driven by the HW timer
    sampleFlag = false;
    readPotentiometer();   // ADC -> holdOpenDelayMs
    updateDisplay();       // OLED status for the current mode
  }

  switch (currentMode) {
    case MODE_AUTONOMOUS: handleAutonomous(); break;
    case MODE_MANUAL:     handleManual();     break;
    case MODE_SAFETY:     handleSafety();     break;
  }
}

/* ================================================================== */
/*  MODE HANDLERS                                                     */
/* ================================================================== */

/**
 * @brief Autonomous gate cycle driven by PIR + ultrasonic + timers.
 */
void handleAutonomous() {
  switch (gateState) {

    case GATE_CLOSED_IDLE:
      if (motionFlag) {                 // vehicle arrived (PIR ISR)
        motionFlag = false;
        gateState = GATE_OPENING;
      }
      break;

    case GATE_OPENING:
      if (stepServoTowards(GATE_OPEN_ANGLE)) {  // returns true when reached
        holdStartMs = millis();
        gateState = GATE_OPEN_HOLD;
      }
      break;

    case GATE_OPEN_HOLD:
      // Hold open for the potentiometer-tuned delay, then close only if the
      // path is clear. If blocked, this check simply repeats every loop.
      if (millis() - holdStartMs >= holdOpenDelayMs) {
        if (measureDistanceCm() > SAFE_DISTANCE_CM) {
          gateState = GATE_CLOSING;
        }
      }
      break;

    case GATE_CLOSING:
      // SAFETY: obstruction detected while physically closing.
      if (measureDistanceCm() < SAFE_DISTANCE_CM) {
        enterSafety();
        break;
      }
      if (stepServoTowards(GATE_CLOSED_ANGLE)) {
        tripCount = 0;            // clean close -> forgive earlier trips
        gateState = GATE_CLOSED_IDLE;
      }
      break;
  }
}

/**
 * @brief Manual mode - supervisor takes direct control of gate position.
 *
 * The potentiometer is re-interpreted in this mode: instead of setting the
 * hold-open delay, it directly commands the gate angle. The PIR and the
 * automatic open/close cycle are ignored.
 *
 * IMPORTANT: automation is suspended here, but SAFETY IS NOT. If an
 * obstruction appears while the gate is moving in the closing direction, the
 * same safety halt applies. A safety function should not be defeatable by
 * selecting a mode.
 */
void handleManual() {
  // Pot position -> demanded gate angle (0% = closed, 100% = open).
  int target = map(analogRead(PIN_POT), 0, 4095,
                   GATE_CLOSED_ANGLE, GATE_OPEN_ANGLE);

  // Are we being asked to move toward the closed position?
  bool movingToClose = (target < currentServoAngle);

  // Safety still applies whenever the gate is closing, even under manual control.
  if (movingToClose && measureDistanceCm() < SAFE_DISTANCE_CM) {
    enterSafety();
    return;
  }

  stepServoTowards(target);
}

/**
 * @brief Safety halt behaviour = design challenge (hybrid retry-then-latch).
 *
 *   - Trips 1..MAX_AUTO_RETRIES : RETREAT to fully open, hold open for
 *     SAFETY_IDLE_MS so the obstruction can be cleared, then resume the normal
 *     cycle. Closing will not begin until the path is clear anyway.
 *   - Trip MAX_AUTO_RETRIES+1   : LATCH. Freeze and refuse to move until a
 *     worker presses the Error Reset button (or sends 'r' over serial).
 * The trip counter is cleared by a clean close (see handleAutonomous), so
 * unrelated one-off events over time do not accumulate into a latch.
 */
void handleSafety() {
  chirpBuzzer();                        // non-blocking intermittent warning

  switch (safetyPhase) {

    case SAFE_RETREATING:
      // Drive the gate back to fully open (non-blocking).
      if (stepServoTowards(GATE_OPEN_ANGLE)) {
        safetyIdleStartMs = millis();
        safetyPhase = SAFE_IDLE_HOLD;
      }
      break;

    case SAFE_IDLE_HOLD:
      // Hold open and give staff time to clear the obstruction.
      if (millis() - safetyIdleStartMs >= SAFETY_IDLE_MS) {
        digitalWrite(PIN_BUZZER, LOW);
        currentMode = MODE_AUTONOMOUS;  // resume; won't close until path clear
        gateState   = GATE_OPEN_HOLD;
        holdStartMs = millis();
      }
      break;

    case SAFE_LATCHED:
      // Frozen. Only a manual reset (button or serial 'r') clears this.
      if (digitalRead(PIN_RESET_BTN) == LOW) {
        clearLatchAndRecover();
      }
      break;
  }
}

/**
 * @brief Enter the safety state after an obstruction while closing.
 *        Increments the trip counter and decides retreat vs latch.
 */
void enterSafety() {
  currentMode = MODE_SAFETY;
  tripCount++;
  Serial.print(F("!! SAFETY HALT - obstruction while closing. Trip "));
  Serial.println(tripCount);

  if (tripCount > MAX_AUTO_RETRIES) {
    safetyPhase = SAFE_LATCHED;         // too many trips -> demand a human
    Serial.println(F("   Repeated fault -> LATCHED. Manual reset required."));
  } else {
    safetyPhase = SAFE_RETREATING;      // gate reopens, then idles
  }
}

/** @brief Clear a latched fault and return to normal running. */
void clearLatchAndRecover() {
  digitalWrite(PIN_BUZZER, LOW);
  tripCount   = 0;
  currentMode = MODE_AUTONOMOUS;
  gateState   = GATE_OPENING;
  Serial.println(F("-> Manual reset accepted. Recovering."));
}

/**
 * @brief Return from MANUAL to AUTONOMOUS, resuming from the gate's real position.
 *
 * In manual mode the gate can be left anywhere. If it is fully closed, the
 * cycle resumes in CLOSED_IDLE. Otherwise it resumes in OPEN_HOLD with a fresh
 * hold timer, so the gate closes itself once the hold time has passed and the
 * path is clear, instead of being stranded part-open.
 * Any PIR event recorded while in manual mode is discarded.
 */
void resumeAutonomousFromManual() {
  motionFlag  = false;
  currentMode = MODE_AUTONOMOUS;
  if (currentServoAngle == GATE_CLOSED_ANGLE) {
    gateState = GATE_CLOSED_IDLE;
  } else {
    gateState   = GATE_OPEN_HOLD;
    holdStartMs = millis();
  }
  Serial.println(F("-> AUTONOMOUS (resuming from current gate position)"));
}

/* ================================================================== */
/*  SENSOR / ACTUATOR HELPERS                                         */
/* ================================================================== */

/**
 * @brief  Move the servo one step toward a target angle (non-blocking).
 * @param  target  desired final angle in degrees.
 * @return true when the servo has reached the target, false while moving.
 */
bool stepServoTowards(int target) {
  if (currentServoAngle == target) return true;
  if (millis() - lastServoStepMs < SERVO_STEP_INTERVAL_MS) return false;
  lastServoStepMs = millis();
  currentServoAngle += (target > currentServoAngle) ? 1 : -1;
  gateServo.write(currentServoAngle);
  return (currentServoAngle == target);
}

/**
 * @brief  Read the HC-SR04 ultrasonic sensor.
 * @return distance to nearest object in centimetres (999 if no echo).
 */
long measureDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long dur = pulseIn(PIN_ECHO, HIGH, 30000);  // 30 ms timeout (~5 m)
  if (dur == 0) return 999;                    // no echo
  return dur * 0.0343 / 2;
}

/**
 * @brief Read the potentiometer (ADC) and map it to the hold-open delay.
 *        Demonstrates the ADC -> behaviour requirement.
 */
void readPotentiometer() {
  int raw = analogRead(PIN_POT);              // 0..4095 on ESP32
  holdOpenDelayMs = map(raw, 0, 4095, 1000, 8000);  // 1 s .. 8 s
}

/** @brief Non-blocking intermittent buzzer chirp (toggles every 300 ms via millis()). */
void chirpBuzzer() {
  digitalWrite(PIN_BUZZER, (millis() / 300) % 2);  // ~1.6 Hz on/off
}

/** @brief Draw a mode-appropriate status screen on the OLED. */
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  switch (currentMode) {
    case MODE_AUTONOMOUS:
      display.println(F("MODE: AUTONOMOUS"));
      display.print(F("Hold: ")); display.print(holdOpenDelayMs / 1000.0, 1); display.println(F("s"));
      display.print(F("Dist: ")); display.print(measureDistanceCm()); display.println(F("cm"));
      break;
    case MODE_MANUAL:
      display.println(F("MODE: MANUAL"));
      display.println(F("AUTO CYCLE OFF"));
      display.print(F("Gate: "));
      display.print(map(currentServoAngle, GATE_CLOSED_ANGLE, GATE_OPEN_ANGLE, 0, 100));
      display.println(F("%"));
      display.print(F("Dist: ")); display.print(measureDistanceCm()); display.println(F("cm"));
      break;
    case MODE_SAFETY:
      display.setTextSize(2);
      display.println(F("SAFETY"));
      display.setTextSize(1);
      display.print(F("Trip ")); display.print(tripCount);
      display.print(F("/")); display.println(MAX_AUTO_RETRIES);
      if (safetyPhase == SAFE_LATCHED) {
        display.println(F("LATCHED"));
        display.println(F("Press RESET to clear"));
      } else {
        display.println(F("Retreating - clear path"));
      }
      break;
  }
  display.display();
}

/* ================================================================== */
/*  UART CONFIG INPUT                                                 */
/* ================================================================== */
/**
 * @brief Handle single-character serial commands for mode/config.
 *        a = autonomous, m = manual, r = reset a latched fault.
 *
 * Mode changes are refused while in SAFETY mode. The only ways out of SAFETY
 * are the defined recovery paths (idle window ending, or a manual reset), so a
 * latched fault cannot be escaped by simply switching modes.
 */
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();

  if ((c == 'a' || c == 'm') && currentMode == MODE_SAFETY) {
    Serial.println(F("-> Refused: safety fault active. Clear it first."));
    return;
  }

  switch (c) {
    case 'a':
      if (currentMode == MODE_MANUAL) {
        resumeAutonomousFromManual();
      }
      break;

    case 'm':
      if (currentMode != MODE_MANUAL) {
        currentMode = MODE_MANUAL;
        Serial.println(F("-> MANUAL"));
      }
      break;

    case 'r':
      // Manual reset only means something when the fault has LATCHED.
      if (currentMode == MODE_SAFETY && safetyPhase == SAFE_LATCHED) {
        clearLatchAndRecover();
      }
      break;
  }
}
