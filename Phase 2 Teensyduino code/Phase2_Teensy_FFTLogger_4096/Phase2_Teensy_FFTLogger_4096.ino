#include <Wire.h>
#include "OpenAudio_ArduinoLibrary.h"
#include "AudioStream_F32.h"
#include <SD.h>
#include <EEPROM.h>

const int myInput = AUDIO_INPUT_LINEIN;   // Use SGTL5000 line-in as the audio source

//Recording duration
const unsigned long RECORD_MS = 1800UL * 1000;  // time (default = 30 minutes)
unsigned long startMillis;                      // Timestamp when recording starts

#define SDCARD_CS_PIN  BUILTIN_SDCARD  //  Use Teensy 4.1’s onboard SD slot

AudioInputI2S_F32           audioIn;      // audio shield line‐in
AudioAnalyzeFFT4096_IQ_F32  FFT4096iq1;   // 4096‐point, 50%‐overlap FFT
AudioControlSGTL5000        audioShield;  // SGTL5000 codec control

File specFile; // Output file for raw power-spectrogram frames

AudioConnection_F32 patchCord1(audioIn, 0, FFT4096iq1, 0); // feeds mono waveform into the 'I' port of the FFT
AudioConnection_F32 patchCord2(audioIn, 1, FFT4096iq1, 1); // feeds nothing (zero) into the 'Q' port of the FFT

// EEPROM layout: at address 0, store a uint16_t run-counter (LSB first)
const int EEPROM_ADDR = 0;


void setup() {
  Serial.begin(115200);
  AudioMemory_F32(100);   // Allocate 100 audio blocks for F32 processing (enough for 4096-pt FFT)

  // Read the last run number from EEPROM, increment it, and store it back.
  // Each boot creates a new sequential file name (REC0001.bin, REC0002.bin, ...).
  uint16_t runCount = 0;  
  EEPROM.get(EEPROM_ADDR, runCount);
  runCount++; 
  EEPROM.put(EEPROM_ADDR, runCount);
  

  // Format a zero-padded 4-digit filename (e.g., REC0007.bin)
  char filename[16];
  snprintf(filename, sizeof(filename), "REC%04u.bin", runCount);
  Serial.print("Starting run #"); 
  Serial.print(runCount);
  Serial.print(" → opening file "); 
  Serial.println(filename);


  // Configure SGTL5000 audio codec
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.lineInLevel(15);

  // Initialise SD card interface
  Serial.print("SD init… ");
  if (!SD.begin(SDCARD_CS_PIN)) {    // Initialise the built-in SD slot
    Serial.println("FAIL");
    while (1) { delay(10); }         // Halt if SD card is not detected
  }
  Serial.println("OK");

  // Open binary output file on SD
  specFile = SD.open(filename, FILE_WRITE);
  if (!specFile) {               // Check file opened successfully
    Serial.println("FAIL");
    while (1) { delay(10); }     // Halt if file creation fails
  }
  Serial.println("OK");

  // Configure FFT analysis object
  FFT4096iq1.setOutputType(FFT_POWER);                // choose FFT_RMS, FFT_POWER, or FFT_DBFS
  FFT4096iq1.windowFunction(AudioWindowHanning4096);  // Apply Hanning window
  FFT4096iq1.setXAxis(0x01);                          // Map output bins to frequency domain
  FFT4096iq1.setNAverage(1);                          // No frame averaging 

  // Now that setup is done, start timing
  startMillis = millis();     // Mark start time of recording session
  Serial.println("→ setup complete, recording starts now");
}

void loop() {
  static uint16_t flushCounter = 0;     // Counts FFT frames since last SD flush

  // Stop recording after the defined duration
  if (millis() - startMillis >= RECORD_MS) {
    specFile.close();
    Serial.println("Time elapsed—recording stopped, file closed.");
    while (1) { delay(1000); }
  }

  // Process FFT frames as they become available
  if (FFT4096iq1.available()) {
    float* pPwr = FFT4096iq1.getData();     // Pointer to current FFT power data buffer

    // Write the 2048 float values (8192 bytes) to the binary .bin file
    for (int i = 0; i < 2048; i++) {
      specFile.write((uint8_t*)&pPwr[i], sizeof(float));    
    }

    // Periodic SD flush for data integrity 
    if ((++flushCounter & 0x3F) == 0) {    // Flush every 64 frames (~3 seconds)
      specFile.flush();
      Serial.print("Flushed ");
      Serial.print(flushCounter);
      Serial.println(" frames to SD.");
    }
  }
}
