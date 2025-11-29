#ifndef POWER_MNGT_H
#define POWER_MNGT_H

#include <Arduino.h>

// --- Power State Variables ---
#define MAX_SD_PIN 21

enum PowerState { STREAMING = 0, DEEP_SLEEP = 1 };

// --- Global Power Variables ---
extern PowerState powerState;
extern unsigned long pauseStartTime;
extern bool isPaused;

// Timeouts (in milliseconds)
extern const unsigned long DEEP_SLEEP_TIMEOUT;

// --- Function Declarations ---
void pm_init();
void pm_enter_deep_sleep();
void pm_exit_deep_sleep();
void pm_record_activity();
void pm_update();
void pm_record_pause();

#endif