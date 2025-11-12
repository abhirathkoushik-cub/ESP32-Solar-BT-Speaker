/*
 * Simple ESP32 I2S WAV Player
 *
 * This example plays a WAV file that has been converted into a C header file
 * and compiled into the program.
 *
 * HARDWARE:
 * - ESP32
 * - MAX98357A I2S Amplifier
 * - 8 Ohm Speaker
 *
 * WIRING (Same as your original):
 * - ESP32 5V   -> MAX98357A Vin
 * - ESP32 GND  -> MAX98357A GND
 * - ESP32 25   -> MAX98357A LRC (I2S_LRC)
 * - ESP32 26   -> MAX98357A BCLK (I2S_BCLK)
 * - ESP32 22   -> MAX98357A DIN (I2S_DIN)
 *
 * SPEAKER:
 * - Connect to the output terminals of the MAX98357A
 *
 * HOW TO USE:
 * 1. Get a WAV file. It MUST be:
 * - MONO
 * - 16-bit
 * - 8000Hz Sample Rate (or change const sampleRate to match)
 * 2. Convert the WAV file to a .h file.
 * - You can use the `xxd` command-line tool:
 * `xxd -i your_sound.wav > sound_wav.h`
 * - Or use a web-based converter (search "wav to c header").
 * 3. Copy the contents of the generated .h file into this sketch
 * (or add as a new tab in the Arduino IDE).
 * - It will look like:
 * unsigned char your_sound_wav[] = { 0x52, 0x49, ... };
 * unsigned int your_sound_wav_len = 123456;
 * 4. Rename the array and length variables to `sound_data` and `sound_data_len`
 * OR change the code below to use your file's variable names.
 * 5. Upload and listen!
 */

#include <ESP_I2S.h>
#include "my_sound.h"

// The GPIO pins from your working example
#define I2S_LRC 25
#define I2S_BCLK 26
#define I2S_DIN 22

// --- I2S Configuration ---
// IMPORTANT: These MUST match your WAV file properties
const int sampleRate = 8000;
i2s_data_bit_width_t bps = I2S_DATA_BIT_WIDTH_16BIT;

// These are from your original code
i2s_mode_t mode = I2S_MODE_STD;
i2s_slot_mode_t slot = I2S_SLOT_MODE_STEREO;
// --- End I2S Configuration ---

I2SClass i2s;

// We start playing the audio data after the 44-byte WAV header
unsigned int wav_data_index = 44;

void setup() {
  Serial.begin(115200);
  Serial.println("I2S simple WAV player");

  i2s.setPins(I2S_BCLK, I2S_LRC, I2S_DIN);

  // Start I2S
  if (!i2s.begin(mode, sampleRate, bps, slot)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
}

void loop() {
  // Check if we've reached the end of the sound data
  if (wav_data_index >= ej_actual_song_audacity_wav_len) {
    wav_data_index = 44; // Loop back to the start (after the header)
  }

  // Read one 16-bit sample (which is 2 bytes) from the array
  // WAV files are little-endian, so low byte comes first
  uint8_t low_byte = ej_actual_song_audacity_wav[wav_data_index];
  uint8_t high_byte = ej_actual_song_audacity_wav[wav_data_index + 1];

  // Your original code was set to STEREO, and your MAX98357A is mono.
  // This works fine. We just need to send the same MONO data to
  // both the Left and Right I2S channels.

  // Write to Left Channel (Low byte, then High byte)
  i2s.write(low_byte);
  i2s.write(high_byte);

  // Write to Right Channel (same data)
  i2s.write(low_byte);
  i2s.write(high_byte);

  // Move to the next 16-bit sample (advance by 2 bytes)
  wav_data_index += 2;

  // The i2s.write() calls will block automatically when the I2S
  // buffer is full, which naturally controls the playback speed
  // to match the sampleRate. No delay() is needed.
}