#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32.h>

BLEMIDI_CREATE_INSTANCE("ESP32-MIDI", MIDI);

#define MIDI_SERIAL_RX 20  // GPIO20 for UART RX
#define MIDI_BAUD_RATE 31250

HardwareSerial MIDISerial(0); // Use UART0

bool isConnected = false;

void setup() {
  // Start serial for debugging
  Serial.begin(115200);
  
  // Initialize MIDI serial input with INVERTED logic for PC817
  MIDISerial.begin(MIDI_BAUD_RATE, SERIAL_8N1, MIDI_SERIAL_RX, -1, true);
  // The 'true' parameter enables inverted RX signal
  
  // Initialize BLE MIDI
  MIDI.begin();
  
  BLEMIDI.setHandleConnected([]() {
    isConnected = true;
    Serial.println("BLE MIDI Connected");
  });
  
  BLEMIDI.setHandleDisconnected([]() {
    isConnected = false;
    Serial.println("BLE MIDI Disconnected");
  });
  
  Serial.println("ESP32 MIDI DIN to BLE adapter ready (PC817 version)");
}

void loop() {
  // Read MIDI from hardware serial
  while (MIDISerial.available() > 0) {
    uint8_t midiByte = MIDISerial.read();
    
    // Forward to BLE MIDI if connected
    if (isConnected) {
      // Parse and send MIDI messages
      static uint8_t midiBuffer[3];
      static uint8_t bufferIndex = 0;
      static bool expectingData = false;
      
      // Status byte (starts with 1)
      if (midiByte >= 0x80) {
        bufferIndex = 0;
        midiBuffer[bufferIndex++] = midiByte;
        
        // Determine message length
        uint8_t messageType = midiByte & 0xF0;
        if (messageType == 0xC0 || messageType == 0xD0) {
          expectingData = true; // 2-byte message
        } else if (messageType >= 0x80 && messageType <= 0xE0) {
          expectingData = false; // 3-byte message
        }
      }
      // Data bytes (start with 0)
      else if (bufferIndex > 0 && bufferIndex < 3) {
        midiBuffer[bufferIndex++] = midiByte;
        
        // Send complete message
        uint8_t messageType = midiBuffer[0] & 0xF0;
        
        if (messageType == 0xC0 || messageType == 0xD0) {
          // 2-byte message (Program Change, Channel Pressure)
          if (bufferIndex >= 2) {
            MIDI.send((midi::MidiType)midiBuffer[0], midiBuffer[1], 0, (midiBuffer[0] & 0x0F) + 1);
            bufferIndex = 0;
          }
        } else if (bufferIndex >= 3) {
          // 3-byte message (Note On/Off, CC, Pitch Bend, etc)
          MIDI.send((midi::MidiType)midiBuffer[0], midiBuffer[1], midiBuffer[2], (midiBuffer[0] & 0x0F) + 1);
          bufferIndex = 0;
        }
      }
    }
  }
  
  // Process BLE MIDI
  MIDI.read();
}