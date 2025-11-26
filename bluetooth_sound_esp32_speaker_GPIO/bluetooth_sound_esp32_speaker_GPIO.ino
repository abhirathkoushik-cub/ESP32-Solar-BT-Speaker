/*
Author: Abhirath Koushik
Brief: Connects to Phone through Bluetooth, play music through audio amplifier. GPIO Buttons on the speaker control play/pause, vol+ etc
Rev-1 11/25/2025: GPIO button control completed
*/

#include <Arduino.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <Wire.h>

#define BQ25798_ADDR 0x6B // 7-bit I2C address

// BQ25798 register offsets
#define BQ25798_REG00_MINIMAL_SYSTEM_VOLTAGE      0x00
#define BQ25798_REG01_CHARGE_VOLTAGE_LIMIT        0x01
#define BQ25798_REG03_CHARGE_CURRENT_LIMIT        0x03
#define BQ25798_REG05_INPUT_VOLTAGE_LIMIT         0x05
#define BQ25798_REG06_INPUT_CURRENT_LIMIT         0x06
#define BQ25798_REG08_PRECHARGE_CONTROL           0x08
#define BQ25798_REG09_TERMINATION_CONTROL         0x09
#define BQ25798_REG0A_RECHARGE_CONTROL            0x0A
#define BQ25798_REG0B_VOTG_REGULATION             0x0B
#define BQ25798_REG0D_IOTG_REGULATION             0x0D
#define BQ25798_REG0E_TIMER_CONTROL               0x0E
#define BQ25798_REG0F_CHARGER_CONTROL_0           0x0F
#define BQ25798_REG10_CHARGER_CONTROL_1           0x10
#define BQ25798_REG11_CHARGER_CONTROL_2           0x11
#define BQ25798_REG12_CHARGER_CONTROL_3           0x12
#define BQ25798_REG13_CHARGER_CONTROL_4           0x13
#define BQ25798_REG14_CHARGER_CONTROL_5           0x14
#define BQ25798_REG15_MPPT_CONTROL                0x15
#define BQ25798_REG16_TEMPERATURE_CONTROL         0x16
#define BQ25798_REG17_NTC_CONTROL_0               0x17
#define BQ25798_REG18_NTC_CONTROL_1               0x18
#define BQ25798_REG19_ICO_CURRENT_LIMIT           0x19
#define BQ25798_REG1B_CHARGER_STATUS_0            0x1B
#define BQ25798_REG1C_CHARGER_STATUS_1            0x1C
#define BQ25798_REG1D_CHARGER_STATUS_2            0x1D
#define BQ25798_REG1E_CHARGER_STATUS_3            0x1E
#define BQ25798_REG1F_CHARGER_STATUS_4            0x1F
#define BQ25798_REG20_FAULT_STATUS_0              0x20
#define BQ25798_REG21_FAULT_STATUS_1              0x21
#define BQ25798_REG22_CHARGER_FLAG_0              0x22
#define BQ25798_REG23_CHARGER_FLAG_1              0x23
#define BQ25798_REG24_CHARGER_FLAG_2              0x24
#define BQ25798_REG25_CHARGER_FLAG_3              0x25
#define BQ25798_REG26_FAULT_FLAG_0                0x26
#define BQ25798_REG27_FAULT_FLAG_1                0x27
#define BQ25798_REG28_CHARGER_MASK_0              0x28
#define BQ25798_REG29_CHARGER_MASK_1              0x29
#define BQ25798_REG2A_CHARGER_MASK_2              0x2A
#define BQ25798_REG2B_CHARGER_MASK_3              0x2B
#define BQ25798_REG2C_FAULT_MASK_0                0x2C
#define BQ25798_REG2D_FAULT_MASK_1                0x2D
#define BQ25798_REG2E_ADC_CONTROL                 0x2E
#define BQ25798_REG2F_ADC_FUNCTION_DISABLE_0      0x2F
#define BQ25798_REG30_ADC_FUNCTION_DISABLE_1      0x30
#define BQ25798_REG31_IBUS_ADC                     0x31
#define BQ25798_REG33_IBAT_ADC                     0x33
#define BQ25798_REG35_VBUS_ADC                     0x35
#define BQ25798_REG37_VAC1_ADC                     0x37
#define BQ25798_REG39_VAC2_ADC                     0x39
#define BQ25798_REG3B_VBAT_ADC                     0x3B    //Battery percentage in ADC value. each bit indicates a step size of 1mV. outputs 2 bytes of data when read
#define BQ25798_REG3D_VSYS_ADC                     0x3D
#define BQ25798_REG3F_TS_ADC                       0x3F
#define BQ25798_REG41_TDIE_ADC                     0x41
#define BQ25798_REG43_DPLUS_ADC                    0x43
#define BQ25798_REG45_DMINUS_ADC                   0x45
#define BQ25798_REG47_DPDM_DRIVER                  0x47
#define BQ25798_REG48_PART_INFORMATION             0x48




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


// Read 16-bit register (MSB first) from BQ25798
uint16_t readBQ25798Register16(uint8_t reg) {
  Wire.beginTransmission(BQ25798_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);              // repeated start

  Wire.requestFrom(BQ25798_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 8) | lsb;        // combine MSB:LSB
}

// Write 16-bit register (MSB first) to BQ25798
void writeBQ25798Register16(uint8_t reg, uint16_t value) {
  uint8_t msb = (value >> 8) & 0xFF;
  uint8_t lsb = value & 0xFF;

  Wire.beginTransmission(BQ25798_ADDR);
  Wire.write(reg);
  Wire.write(msb);
  Wire.write(lsb);
  Wire.endTransmission();
}


// I2C Read 1 byte from a register
uint8_t readBQ25798Register(uint8_t reg) {
  Wire.beginTransmission(BQ25798_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Restart for read
  Wire.requestFrom(BQ25798_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// I2C Write 1 byte to a register
void writeBQ25798Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BQ25798_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

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
  Wire.begin();
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

  uint8_t regValue = readBQ25798Register(0x00);
  Serial.printf("BQ25798 REG00 value: 0x%02X\n", regValue);  //should read value 0x4. indicates the min sys voltage of 3.5V
  
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

