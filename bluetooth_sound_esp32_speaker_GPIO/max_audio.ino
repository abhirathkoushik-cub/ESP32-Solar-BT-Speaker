#include "max_audio.h"

const int volumeStep = 5;
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
