/*
  gpio_timestamp_bridge.ino

  Arduino connected to Raspberry Pi:
  - Input: TTL pulse from Pi GPIO (3.3V)
  - Output: Timestamp of each pulse over USB serial

  Output line format (CSV):
    PULSE,<seq>,<t_us_32>,<t_us_64>

  Where:
    <seq>      = incrementing event counter
    <t_us_32>  = raw micros() (32-bit, wraps ~70 min)
    <t_us_64>  = extended microseconds (no wrap, derived in firmware)

  Notes on wiring:
  - Pi GPIO is 3.3V logic. Arduino Uno/Nano are 5V logic.
  - Most 5V Arduinos *usually* read 3.3V HIGH fine, but it’s not guaranteed.
    Best practice: use a level shifter or a transistor buffer.
  - Always connect grounds: Pi GND <-> Arduino GND.
*/

#include <Arduino.h>

// ------------ User configuration ------------
const uint8_t PULSE_PIN = 2;      // Use D2 for external interrupt on Uno/Nano
const unsigned long BAUD = 115200;

// Optional: ignore pulses closer than this (debounce / double-trigger protection)
const uint32_t MIN_INTERVAL_US = 200;  // adjust or set to 0 to disable
// -------------------------------------------

// Volatile data set inside ISR
volatile uint32_t isr_last_micros = 0;
volatile uint32_t isr_micros = 0;
volatile bool     isr_flag = false;

// 64-bit time extension state (handled outside ISR)
uint32_t prev_micros_main = 0;
uint64_t micros64_base = 0;   // increments by 2^32 when micros() wraps

uint64_t micros64_now(uint32_t now32) {
  // Detect wrap: micros() decreased compared to previous
  if (now32 < prev_micros_main) {
    micros64_base += (1ULL << 32);
  }
  prev_micros_main = now32;
  return micros64_base + (uint64_t)now32;
}

void IRAM_ATTR onPulseRise() {
  // Timestamp quickly
  uint32_t t = micros();

  // Simple minimum interval gate to reduce false double triggers
  if (MIN_INTERVAL_US > 0) {
    uint32_t dt = t - isr_last_micros; // unsigned arithmetic handles wrap
    if (dt < MIN_INTERVAL_US) return;
    isr_last_micros = t;
  }

  isr_micros = t;
  isr_flag = true;
}

uint32_t seq = 0;

void setup() {
  Serial.begin(BAUD);
  while (!Serial) {
    ; // For boards with native USB (Leonardo/Micro). Safe on Uno too.
  }

  pinMode(PULSE_PIN, INPUT);   // or INPUT_PULLDOWN if your board supports it
  // If your pulses are open-drain or you want a defined idle state, consider:
  // pinMode(PULSE_PIN, INPUT_PULLUP); and invert your pulse logic accordingly.

  // Attach interrupt on rising edge
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), onPulseRise, RISING);

  // Print a header / ready line for the Pi to sync on
  Serial.println("READY,gpio_timestamp_bridge");
}

void loop() {
  // Copy ISR values atomically
  if (isr_flag) {
    noInterrupts();
    uint32_t t32 = isr_micros;
    isr_flag = false;
    interrupts();

    // Extend to 64-bit time base
    uint64_t t64 = micros64_now(t32);

    seq++;
    // Emit one line per pulse
    Serial.print("PULSE,");
    Serial.print(seq);
    Serial.print(",");
    Serial.print(t32);
    Serial.print(",");
    Serial.println((unsigned long long)t64);
  }

  // (Optional) small sleep to reduce CPU usage
  // delay(0); // not necessary on most boards
}