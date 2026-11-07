/*
 * ESC Test Sketch — Interactive Serial CLI
 * -----------------------------------------
 * Board : ESP32 Dev Module
 * ESCs  :
 *   ESC1 (Front-Left,  CCW) -> GPIO25
 *   ESC2 (Front-Right, CW)  -> GPIO26
 *   ESC3 (Rear-Right,  CCW) -> GPIO27
 *   ESC4 (Rear-Left,   CW)  -> GPIO33
 *
 * PWM: 50 Hz, 1000–2000 µs (servo-style)
 *
 * Serial Commands (115200 baud):
 *   arm        — Send min throttle (1000µs) for 3s to arm all ESCs
 *   stop       — Set all ESCs to 1000µs (min throttle)
 *   all <val>  — Set all ESCs to <val> (1000–2000)
 *   1 <val>    — Set ESC1 to <val>
 *   2 <val>    — Set ESC2 to <val>
 *   3 <val>    — Set ESC3 to <val>
 *   4 <val>    — Set ESC4 to <val>
 *   sweep      — Slowly ramp all ESCs from 1000 to 1300 and back
 *   test       — Spin each motor individually at low throttle for 2s
 *   help       — Show commands
 *
 * ⚠ REMOVE PROPELLERS BEFORE TESTING ⚠
 */

#include <ESP32Servo.h>

// ── Pin definitions ──
#define ESC1_PIN 25   // Front-Left   CCW
#define ESC2_PIN 26   // Front-Right  CW
#define ESC3_PIN 27   // Rear-Right   CCW
#define ESC4_PIN 33   // Rear-Left    CW

#define NUM_ESCS 4

// ── ESC pulse range ──
#define PWM_MIN 1000
#define PWM_MAX 2000

// ── Servo objects ──
Servo esc[NUM_ESCS];
const uint8_t escPins[NUM_ESCS] = { ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN };
const char* escNames[NUM_ESCS] = {
  "ESC1 (FL-CCW)",
  "ESC2 (FR-CW)",
  "ESC3 (RR-CCW)",
  "ESC4 (RL-CW)"
};

int currentThrottle[NUM_ESCS] = { PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN };

void setESC(int index, int value) {
  value = constrain(value, PWM_MIN, PWM_MAX);
  currentThrottle[index] = value;
  esc[index].writeMicroseconds(value);
}

void setAllESC(int value) {
  for (int i = 0; i < NUM_ESCS; i++) {
    setESC(i, value);
  }
}

void printStatus() {
  Serial.println(F("\n--- Current Throttle ---"));
  for (int i = 0; i < NUM_ESCS; i++) {
    Serial.print("  ");
    Serial.print(escNames[i]);
    Serial.print(" = ");
    Serial.print(currentThrottle[i]);
    Serial.println(" µs");
  }
  Serial.println();
}

void printHelp() {
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║        ESC TEST — COMMANDS             ║"));
  Serial.println(F("╠════════════════════════════════════════╣"));
  Serial.println(F("║  arm       Arm all ESCs (3s min throt) ║"));
  Serial.println(F("║  stop      All ESCs to min (1000)      ║"));
  Serial.println(F("║  all 1100  Set all ESCs to 1100µs      ║"));
  Serial.println(F("║  1 1100    Set ESC1 to 1100µs          ║"));
  Serial.println(F("║  2 1200    Set ESC2 to 1200µs          ║"));
  Serial.println(F("║  3 1100    Set ESC3 to 1100µs          ║"));
  Serial.println(F("║  4 1200    Set ESC4 to 1200µs          ║"));
  Serial.println(F("║  sweep     Ramp all 1000→1300→1000     ║"));
  Serial.println(F("║  test      Spin each motor 2s          ║"));
  Serial.println(F("║  status    Show current throttle       ║"));
  Serial.println(F("║  help      Show this menu              ║"));
  Serial.println(F("╚════════════════════════════════════════╝"));
  Serial.println(F("\n⚠  REMOVE PROPELLERS BEFORE TESTING  ⚠\n"));
}

void armESCs() {
  Serial.println(F("\n[ARM] Sending 1000µs to all ESCs for 3 seconds..."));
  setAllESC(PWM_MIN);
  for (int i = 3; i > 0; i--) {
    Serial.print("  ");
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }
  Serial.println(F("[ARM] Done — ESCs should be armed now."));
  Serial.println(F("[ARM] You can now send throttle values (1000-2000).\n"));
}

void sweepTest() {
  Serial.println(F("\n[SWEEP] Ramping 1000 → 1300 → 1000 (all ESCs)"));
  Serial.println(F("[SWEEP] Send 'stop' at any time to abort.\n"));

  // Ramp up
  for (int val = PWM_MIN; val <= 1300; val += 10) {
    setAllESC(val);
    Serial.print("  Throttle: ");
    Serial.println(val);
    delay(200);

    // Check for abort
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "stop") {
        setAllESC(PWM_MIN);
        Serial.println(F("[SWEEP] Aborted!"));
        return;
      }
    }
  }

  delay(1000);

  // Ramp down
  for (int val = 1300; val >= PWM_MIN; val -= 10) {
    setAllESC(val);
    Serial.print("  Throttle: ");
    Serial.println(val);
    delay(200);

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "stop") {
        setAllESC(PWM_MIN);
        Serial.println(F("[SWEEP] Aborted!"));
        return;
      }
    }
  }

  setAllESC(PWM_MIN);
  Serial.println(F("[SWEEP] Complete.\n"));
}

void individualTest() {
  Serial.println(F("\n[TEST] Testing each motor individually at 1150µs for 2 seconds."));
  Serial.println(F("⚠  REMOVE PROPELLERS  ⚠\n"));
  delay(1000);

  for (int i = 0; i < NUM_ESCS; i++) {
    Serial.print("[TEST] Spinning ");
    Serial.print(escNames[i]);
    Serial.println("...");

    setESC(i, 1150);
    delay(2000);
    setESC(i, PWM_MIN);
    delay(500);

    Serial.print("[TEST] ");
    Serial.print(escNames[i]);
    Serial.println(" — stopped.\n");
    delay(500);
  }

  Serial.println(F("[TEST] All motors tested.\n"));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Attach ESCs
  for (int i = 0; i < NUM_ESCS; i++) {
    esc[i].setPeriodHertz(50);
    esc[i].attach(escPins[i], PWM_MIN, PWM_MAX);
    esc[i].writeMicroseconds(PWM_MIN);
  }

  Serial.println(F("\n"));
  Serial.println(F("========================================"));
  Serial.println(F("   ESP32 Quadcopter ESC Tester"));
  Serial.println(F("========================================"));
  Serial.println(F("  ESC1  GPIO25  Front-Left   CCW"));
  Serial.println(F("  ESC2  GPIO26  Front-Right  CW"));
  Serial.println(F("  ESC3  GPIO27  Rear-Right   CCW"));
  Serial.println(F("  ESC4  GPIO33  Rear-Left    CW"));
  Serial.println(F("========================================"));

  printHelp();
  Serial.println(F("Type 'arm' to begin.\n"));
  Serial.print(F("> "));
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      Serial.print(F("> "));
      return;
    }

    Serial.print(F(">> "));
    Serial.println(input);

    // Parse command
    if (input == "arm") {
      armESCs();
    }
    else if (input == "stop") {
      setAllESC(PWM_MIN);
      Serial.println(F("[STOP] All ESCs set to 1000µs.\n"));
    }
    else if (input == "sweep") {
      sweepTest();
    }
    else if (input == "test") {
      individualTest();
    }
    else if (input == "status") {
      printStatus();
    }
    else if (input == "help") {
      printHelp();
    }
    else if (input.startsWith("all ")) {
      int val = input.substring(4).toInt();
      if (val >= PWM_MIN && val <= PWM_MAX) {
        setAllESC(val);
        Serial.print(F("[ALL] Set to "));
        Serial.print(val);
        Serial.println(F("µs\n"));
      } else {
        Serial.println(F("[ERROR] Value must be 1000-2000\n"));
      }
    }
    else if (input.charAt(0) >= '1' && input.charAt(0) <= '4' && input.charAt(1) == ' ') {
      int escNum = input.charAt(0) - '1';  // 0-indexed
      int val = input.substring(2).toInt();
      if (val >= PWM_MIN && val <= PWM_MAX) {
        setESC(escNum, val);
        Serial.print("[");
        Serial.print(escNames[escNum]);
        Serial.print("] Set to ");
        Serial.print(val);
        Serial.println("µs\n");
      } else {
        Serial.println(F("[ERROR] Value must be 1000-2000\n"));
      }
    }
    else {
      Serial.println(F("[ERROR] Unknown command. Type 'help'.\n"));
    }

    Serial.print(F("> "));
  }
}
