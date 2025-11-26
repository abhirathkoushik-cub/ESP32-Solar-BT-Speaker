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

extern const char* inputStateStr[];

extern const char* chargeStateStr[];

// --- BQ monitor config ---
extern const unsigned long BQ_UPDATE_INTERVAL_MS;
extern unsigned long lastBQUpdate;

// Function Declarations
void writeBQ25798Register(uint8_t reg, uint8_t value);
uint8_t readBQ25798Register(uint8_t reg);
void writeBQ25798Register16(uint8_t reg, uint16_t value);
uint16_t readBQ25798Register16(uint8_t reg);
void enableBQADC();
float estimateBatteryPercent(float vbat);
void disableBQWatchdog();
void printBQStatus();
