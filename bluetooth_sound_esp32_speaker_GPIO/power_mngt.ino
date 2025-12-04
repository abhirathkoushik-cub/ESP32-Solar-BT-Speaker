#include "power_mngt.h"
#include "max_audio.h"
#include "driver/rtc_io.h"

// --- Global Power Variables ---
PowerState powerState = STREAMING;
unsigned long pauseStartTime = 0;
bool isPaused = true;  // Track pause state

// Timeout (in milliseconds)
const unsigned long DEEP_SLEEP_TIMEOUT = 0.5 * 60 * 1000;  // 2 min

// --- Power Management Functions ---

void pm_init() {
  pinMode(MAX_SD_PIN, OUTPUT);
  digitalWrite(MAX_SD_PIN, HIGH);
  pauseStartTime = millis(); 
  Serial.println("[PM] Initialized, timer running");
}

void pm_record_pause() {
  if (!isPaused) {
    isPaused = true;
    pauseStartTime = millis();
    Serial.printf("[PM] Pause recorded at %lu ms\n", pauseStartTime);
  }
}

void pm_enter_deep_sleep() {
  if (powerState == DEEP_SLEEP) return;
  Serial.println("[PM] Entering DEEP_SLEEP");
  
  digitalWrite(MAX_SD_PIN, LOW);
  delay(100);
  Serial.println("[PM] MAX disabled (SD_MODE = LOW)");

  gpio_num_t wakeupPin = (gpio_num_t)PIN_PREV_VOL;

  rtc_gpio_pullup_en(wakeupPin);
  rtc_gpio_pulldown_dis(wakeupPin);

  esp_sleep_enable_ext0_wakeup(wakeupPin, LOW);

  Serial.println("[PM] Deep sleep starting... press Vol-/Prev button to wake");
  Serial.flush();
  delay(100);

  powerState = DEEP_SLEEP; 
  
  esp_deep_sleep_start(); // ESP reboots on GPIO interrupt
}

void pm_exit_deep_sleep() {
  Serial.println("[PM] Woken from deep sleep!");
  digitalWrite(MAX_SD_PIN, HIGH);
  delay(100);
  Serial.println("[PM] MAX enabled (SD_MODE = HIGH)");
  powerState = STREAMING;
  isPaused = false;
  pauseStartTime = millis();
}

void pm_record_activity() {
  if (powerState != STREAMING || isPaused) {
     isPaused = false;
     Serial.println("[PM] Activity: Timer Reset");
  }
  pauseStartTime = millis();
}

void pm_update() {
  unsigned long now = millis();
  
  if (localPlayingState) {
    if (isPaused) {
      isPaused = false;
      Serial.println("[PM] Music started, timer cleared");
    }
    // Reset timer while playing music
    pauseStartTime = now;
  } 
  else {
    // Music is NOT playing. Record Pause
    if (!isPaused) {
      pm_record_pause();
    }
    
    // Check if Deep Sleep Timeout has passed
    if (now - pauseStartTime > DEEP_SLEEP_TIMEOUT) {
      Serial.printf("[PM] Idle for %lu ms. Sleeping.\n", now - pauseStartTime);
      pm_enter_deep_sleep();
    }
  }
}
