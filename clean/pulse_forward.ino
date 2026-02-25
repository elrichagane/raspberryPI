/*
  gpio_timestamp_bridge.ino (Arduino Mega 2560)

  Input: TTL pulse from Raspberry Pi GPIO (3.3V) into an interrupt pin
  Output: Timestamp for each pulse over USB serial

  Output format (CSV):
    PULSE,<seq>,<t_us_32>,<t_us_64>

  Notes:
  - Connect Pi GND to Arduino GND.
  - Use Arduino Mega interrupt-capable pins: 2, 3, 18, 19, 20, 21.
  - 3.3V from Pi is OK into Mega input (do NOT send 5V into Pi GPIO).
*/

#include <Arduino.h>

// ------------ User configuration ------------
const uint8_t PULSE_PIN = 2;          // Mega interrupt pin (recommended D2)
const unsigned long BAUD = 115200;

// Ignore triggers closer than this (helps reject double edges/noise).
// Set to 0 to disable.
const uint32_t MIN_INTERVAL_US = 200;
// -------------------------------------------

// ISR-updated state
volatile uint32_t isr_last_micros = 0;
volatile uint32_t isr_micros = 0;
volatile bool     isr_flag = false;

// 64-bit time extension state (main loop)
uint32_t prev_micros_main = 0;
uint64_t micros64_base = 0;  // increments by 2^32 when micros() wraps

// Extend 32-bit micros() to 64-bit monotonically increasing microseconds
uint64_t micros64_now(uint32_t now32) {
  if (now32 < prev_micros_main) {
    micros64_base += (1ULL << 32);
  }
  prev_micros_main = now32;
  return micros64_base + (uint64_t)now32;
}

// Print uint64_t on AVR by splitting into two 32-bit halves
void printU64(uint64_t v) {
  uint32_t hi = (uint32_t)(v >> 32);
  uint32_t lo = (uint32_t)(v & 0xFFFFFFFFULL);

  if (hi == 0) {
    Serial.print(lo);
    return;
  }

  // Print high part, then low part zero-padded to 10 digits (since 2^32-1 is 10 digits)
  Serial.print(hi);
  char buf[11];
  // zero-pad low to 10 digits
  snprintf(buf, sizeof(buf), "%010lu", (unsigned long)lo);
  Serial.print(buf);
}

// Interrupt handler: timestamps rising edge
void onPulseRise() {
  uint32_t t = micros();

  if (MIN_INTERVAL_US > 0) {
    uint32_t dt = t - isr_last_micros;  // unsigned handles wrap
    if (dt < MIN_INTERVAL_US) return;
    isr_last_micros = t;
  }

  isr_micros = t;
  isr_flag = true;
}

uint32_t seq = 0;

void setup() {
  Serial.begin(BAUD);
  while (!Serial) { ; } // helps on native USB boards; harmless on Mega

  pinMode(PULSE_PIN, INPUT);  // Pi drives the line

  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), onPulseRise, RISING);

  Serial.println("READY,gpio_timestamp_bridge");
}

void loop() {
  if (isr_flag) {
    // Copy ISR data atomically
    noInterrupts();
    uint32_t t32 = isr_micros;
    isr_flag = false;
    interrupts();

    uint64_t t64 = micros64_now(t32);
    seq++;

    Serial.print("PULSE,");
    Serial.print(seq);
    Serial.print(",");
    Serial.print(t32);
    Serial.print(",");
    printU64(t64);
    Serial.println();
  }
}