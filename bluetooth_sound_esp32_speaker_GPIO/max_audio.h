#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

// --- PIN DEFINITIONS ---
#define PIN_PLAY_PAUSE  17
#define PIN_NEXT_VOL    23
#define PIN_PREV_VOL    27

// --- CONFIGURATION ---
extern const int volumeStep;
extern int volume_value;

// Button 1 (Play/Pause)
extern int lastState_17;
extern unsigned long lastDebounce_17;
extern bool localPlayingState; 

// Button 2 (Vol+ and Next)
extern int lastState_23;
extern int clickCount_23;
extern unsigned long lastPressTime_23;

// Button 3 (Vol- and Prev)
extern int lastState_27;
extern int clickCount_27;
extern unsigned long lastPressTime_27;

// Function Declarations
void changeVolume(int amount);
void avrc_rn_playstatus_callback(esp_avrc_playback_stat_t playback);
