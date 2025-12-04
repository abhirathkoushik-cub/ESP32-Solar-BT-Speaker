
#include "bq_i2c_commands.h"

const char* inputStateStr[] = {
"No input",
"USB SDP",
"USB CDP",
"USB DCP",
"HVDCP",
"Unknown adapter",
"Non-standard",
"OTG",
"Not qualified",
"Reserved",
"Reserved",
"Direct VBUS",
"Backup mode",
"Reserved",
"Reserved",
"Reserved"
};

const char* chargeStateStr[] = {
"Not charging",
"Trickle",
"Pre-charge",
"Fast charge (CC)",
"Taper (CV)",
"Reserved",
"Top-off",
"Charge done"
};

unsigned long lastBQUpdate = 0;
const unsigned long BQ_UPDATE_INTERVAL_MS = 10000UL; // Prints on serial terminal every 30sec

// I2C Write 1 byte to a register
void writeBQ25798Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BQ25798_ADDR);
  Wire.write(reg);
  Wire.write(value);
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

// ---------- Battery / charging monitor ----------

// Enable ADC in BQ25798
void enableBQADC() {
  uint8_t adcCtrl = readBQ25798Register(BQ25798_REG2E_ADC_CONTROL);
  adcCtrl |= 0x80;                 // Set ADCEN bit
  writeBQ25798Register(BQ25798_REG2E_ADC_CONTROL, adcCtrl);
}

// Li-ion voltage-to-precentage approximation for 1‑cell
float estimateBatteryPercent(float vbat) {
  float pct = 0.0f;

  if (vbat >= 4.20f) pct = 100.0f;
  else if (vbat >= 4.00f) pct = 80.0f + (vbat - 4.00f) * 100.0f;     // 4.0–4.2 V
  else if (vbat >= 3.80f) pct = 50.0f + (vbat - 3.80f) * 150.0f;     // 3.8–4.0 V
  else if (vbat >= 3.60f) pct = 20.0f + (vbat - 3.60f) * 150.0f;     // 3.6–3.8 V
  else if (vbat >= 3.30f) pct = (vbat - 3.30f) * 66.7f;              // 3.3–3.6 V
  else pct = 0.0f;

  if (pct > 100.0f) pct = 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  return pct;
}

// Disable BQ Watchdog before enabling ADC for reading registers
void disableBQWatchdog() {
  uint8_t reg10 = readBQ25798Register(BQ25798_REG10_CHARGER_CONTROL_1);
  reg10 &= ~0x07;          // Clear WATCHDOG[2:0] -> 000 = disabled
  writeBQ25798Register(BQ25798_REG10_CHARGER_CONTROL_1, reg10);
}

// Read and print charging state along with battery metrics
void printBQStatus() {

  uint8_t status1 = readBQ25798Register(BQ25798_REG1C_CHARGER_STATUS_1);
  uint8_t chargeStatus = (status1 >> 5) & 0x07;   // bits 7:5
  uint8_t vbusStatus   = (status1 >> 1) & 0x0F;   // bits 4:1

  // ADC readings: 1 mV/LSB for voltages, 1 mA/LSB for currents
  uint16_t vbat_raw = readBQ25798Register16(BQ25798_REG3B_VBAT_ADC);
  uint16_t vsys_raw = readBQ25798Register16(BQ25798_REG3D_VSYS_ADC);
  uint16_t vbus_raw = readBQ25798Register16(BQ25798_REG35_VBUS_ADC);
  uint16_t ibat_raw = readBQ25798Register16(BQ25798_REG33_IBAT_ADC);
  uint16_t vac2_raw = readBQ25798Register16(BQ25798_REG39_VAC2_ADC); // Solar Voltage in VAC2

  float vbat = vbat_raw / 1000.0f; 
  float vsys = vsys_raw / 1000.0f;
  float vbus = vbus_raw / 1000.0f;
  float vsolar = vac2_raw / 1000.0f;

  int16_t ibat_signed = (int16_t)ibat_raw;
  float ibat = (float)ibat_signed;

  float pct = estimateBatteryPercent(vbat);

  Serial.println("\n=== BQ25798 Battery / Charge Status ===");
  Serial.printf("Charge state : %s\n", chargeStateStr[chargeStatus]);
  Serial.printf("Input source : %s\n", inputStateStr[vbusStatus]);

  Serial.println("\nVoltages:");
  Serial.printf("  VBAT : %.3f V (%.0f%%)\n", vbat, pct);
  Serial.printf("  VSYS : %.3f V\n", vsys);
  Serial.printf("  VBUS : %.3f V\n", vbus);
  Serial.printf("  VSOLAR: %.3f V (VAC2)\n", vsolar);

  Serial.println("Current:");
  if (ibat_signed > 0) {
    Serial.printf("  Charging  : %.0f mA\n", ibat);
  } else if (ibat_signed < 0) {
    Serial.printf("  Discharge : %.0f mA\n", -ibat);
  } else {
    Serial.println("  No battery current");
  }

  Serial.println("=======================================\n");
}
