/*
Author: Abhirath Koushik
Brief: Connects to Phone through Bluetooth, play music through audio amplifier. GPIO Buttons on the speaker control play/pause, vol+ etc
Rev-1 11/25/2025: GPIO button control completed
*/

#include <Arduino.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

// --- PIN DEFINITIONS ---
#define PIN_PLAY_PAUSE  17
#define PIN_NEXT_VOL    23
#define PIN_PREV_VOL    27

// --- CONFIGURATION ---
const int volumeStep = 5;
const unsigned long doubleClickSpeed = 400; // Time in ms to wait for a second click

// --- AUDIO OBJECTS ---
I2SStream out;
BluetoothA2DPSink a2dp_sink(out);
int volume_value = 80; // Volume values can range from 0 to 127

// Button 1 (Play/Pause)
int lastState_17 = HIGH;
unsigned long lastDebounce_17 = 0;
bool localPlayingState = false; 

// Button 2 (Vol+ and Next)
int lastState_23 = HIGH;
int clickCount_23 = 0;
unsigned long lastPressTime_23 = 0;

// Button 3 (Vol- and Prev)
int lastState_27 = HIGH;
int clickCount_27 = 0;
unsigned long lastPressTime_27 = 0;


// --- Update Volume Function ---
void changeVolume(int amount) {
  volume_value += amount;
  // Clamp volume between 0 and 127
  if (volume_value > 127) volume_value = 127;
  if (volume_value < 0) volume_value = 0;
  
  a2dp_sink.set_volume((uint8_t)volume_value);
  Serial.printf("Volume: %d\n", volume_value);
}

// --- Sync Local State with Real Bluetooth State ---
void avrc_rn_playstatus_callback(esp_avrc_playback_stat_t playback) {
  switch (playback) {
    case ESP_AVRC_PLAYBACK_PLAYING:
      Serial.println("Callback: Playing");
      localPlayingState = true;
      break;
    case ESP_AVRC_PLAYBACK_PAUSED:
    case ESP_AVRC_PLAYBACK_STOPPED:
      Serial.println("Callback: Paused/Stopped");
      localPlayingState = false;
      break;
    default:
      break;
  }
}


void setup() {
  Serial.begin(115200);

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
  a2dp_sink.start("AudioKit");
  a2dp_sink.set_volume((uint8_t)volume_value);

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
      } else {
        a2dp_sink.play();
        Serial.println("Button Cmd: PLAY");
        localPlayingState = true;
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
      lastPressTime_23 = millis(); // Reset timer on every click
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
  }

  // Read Vol-/Prev Button 
  int reading_27 = digitalRead(PIN_PREV_VOL);
  if (lastState_27 == HIGH && reading_27 == LOW) {
    if (millis() - lastPressTime_27 > 50) {
      clickCount_27++;
      lastPressTime_27 = millis(); // Reset timer
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
  }
  
  delay(10);
}
