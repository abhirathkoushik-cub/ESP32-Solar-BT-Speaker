/*
Author: Abhirath Koushik and Bharath Varma
Brief: Connects to Phone through Bluetooth, play music through audio amplifier. GPIO Buttons on the speaker control play/pause, vol+ etc
Rev-1 11/25/2025: GPIO button control completed
Rev-2 11/26/2025: (Bharath) Added I2C Communication with BQ for Battery Metrics
                  (Abhirath) Added Solar voltage and modularized code
*/

#include <Arduino.h>
#include "bq_i2c_commands.h"
#include "max_audio.h"
#include "power_mngt.h"

const unsigned long doubleClickSpeed = 400; // Time in ms to wait for a second click

// --- AUDIO OBJECTS ---
I2SStream out;
BluetoothA2DPSink a2dp_sink(out);

void setup() {
  Serial.begin(115200);

  // Check if waking from deep sleep
  esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
  if (wakeup == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woken from deep sleep!");
    pm_exit_deep_sleep();
    delay(100);
  }

  // Initialize power management
  pm_init();
  
  Wire.begin(18, 19); // Using SDA Pin (IO18) and SCL Pin (IO19)
  disableBQWatchdog(); // clearing watchdog before ADC engine enable
  enableBQADC(); // Enable ADC engine in the charger

  // Configure Internal Pullups for GPIO Pins
  pinMode(PIN_PLAY_PAUSE, INPUT_PULLUP);
  pinMode(PIN_NEXT_VOL, INPUT_PULLUP);
  pinMode(PIN_PREV_VOL, INPUT_PULLUP);

  // Configure Audio I2S Pins
  auto cfg = out.defaultConfig();
  cfg.pin_bck  = 26;
  cfg.pin_ws   = 25;
  cfg.pin_data = 22;
  out.begin(cfg);

  // Register Callback to keep states in sync
  a2dp_sink.set_avrc_rn_playstatus_callback(avrc_rn_playstatus_callback);

  // Start Bluetooth
  a2dp_sink.start("LPEDT_Pulse_Speaker");
  a2dp_sink.set_volume((uint8_t)volume_value);

  uint8_t regValue = readBQ25798Register(BQ25798_REG00_MINIMAL_SYSTEM_VOLTAGE);
  Serial.printf("BQ25798 REG00 value: 0x%02X\n", regValue);
  
  Serial.println("Starting Bluetooth Now!");
}


void loop() {

  // Read Play/Pause button
  int reading_17 = digitalRead(PIN_PLAY_PAUSE);
  if (lastState_17 == HIGH && reading_17 == LOW) {
    // Simple debounce with 50ms
    if (millis() - lastDebounce_17 > 50) {
      
      // Toggle Action between Play and Pause
      if (localPlayingState) {
        a2dp_sink.pause();
        Serial.println("Button Cmd: PAUSE");
        localPlayingState = false;
        pm_record_pause(); // Recording Pause Time
      } else {
        a2dp_sink.play();
        Serial.println("Button Cmd: PLAY");
        localPlayingState = true;
        pm_record_activity(); // Check timer and power state
      }
      lastDebounce_17 = millis();
    }
  }
  lastState_17 = reading_17;

  // Read Vol+/Next Button 
  int reading_23 = digitalRead(PIN_NEXT_VOL);
  if (lastState_23 == HIGH && reading_23 == LOW) {
    if (millis() - lastPressTime_23 > 50) {
      clickCount_23++;
      lastPressTime_23 = millis(); 
      pm_record_activity();
    }
  }
  lastState_23 = reading_23;

  // Check Timer for buttons clicks
  if (clickCount_23 > 0 && (millis() - lastPressTime_23 > doubleClickSpeed)) {
    if (clickCount_23 == 1) {
      // --- SINGLE CLICK ACTION ---
      Serial.println("Btn2: Vol Up");
      changeVolume(volumeStep);
    } else {
      // --- DOUBLE CLICK ACTION ---
      Serial.println("Btn2: Next Track");
      a2dp_sink.next();
    }
    // Reset counter
    clickCount_23 = 0;
    pm_record_activity();
  }

  // Read Vol-/Prev Button 
  int reading_27 = digitalRead(PIN_PREV_VOL);
  if (lastState_27 == HIGH && reading_27 == LOW) {
    if (millis() - lastPressTime_27 > 50) {
      clickCount_27++;
      lastPressTime_27 = millis(); // Reset timer
      pm_record_activity();
    }
  }
  lastState_27 = reading_27;

  // Check Timer when button clicked
  if (clickCount_27 > 0 && (millis() - lastPressTime_27 > doubleClickSpeed)) {
    if (clickCount_27 == 1) {
      // --- SINGLE CLICK ACTION ---
      Serial.println("Btn3: Vol Down");
      changeVolume(-volumeStep);
    } else {
      // --- DOUBLE CLICK ACTION ---
      Serial.println("Btn3: Prev Track");
      a2dp_sink.previous();
    }
    // Reset counter
    clickCount_27 = 0;
    pm_record_activity();
  }

  // --- Every 30 s: print battery & charging info ---
  if (millis() - lastBQUpdate >= BQ_UPDATE_INTERVAL_MS) {
    printBQStatus();
    lastBQUpdate = millis();
  }

  // --- Power State Update ---
  pm_update();  // Check if transition to DEEP_SLEEP is required
  
  delay(10);
}
