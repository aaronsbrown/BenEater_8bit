#include <avr/pgmspace.h>  // Required for PROGMEM usage

#define SHIFT_DATA 2
#define SHIFT_CLOCK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define EEPROM_WRITE_EN 13

#define EEPROM_SIZE 2048  // Defined EEPROM size for clarity
#define EEPROM_WRITE_DELAY 5  // Adjust based on EEPROM specs

#define HLT   0b1000000000000000
#define MI    0b0100000000000000
#define RI    0b0010000000000000
#define RO    0b0001000000000000
#define II    0b0000100000000000
#define IO    0b0000010000000000
#define AI    0b0000001000000000
#define AO    0b0000000100000000
#define EO    0b0000000010000000
#define SU    0b0000000001000000
#define BI    0b0000000000100000
#define OI    0b0000000000010000
#define CE    0b0000000000001000
#define CO    0b0000000000000100
#define J     0b0000000000000010
#define FI    0b0000000000000001

const uint16_t DATA[] PROGMEM {
  MI|CO,  RO|II|CE,   0,      0,      0,      0, 0, 0,  // 0000 NOP
  MI|CO,  RO|II|CE,   IO|MI,  RO|AI,  0,      0, 0, 0,  // 0001 LDA
  MI|CO,  RO|II|CE,   IO|MI,  RO|BI,  EO|AI,  0, 0, 0,  // 0010 ADD
  MI|CO,  RO|II|CE,   0,      0,      0,      0, 0, 0,  // 0011 NOP
  MI|CO,  RO|II|CE,   0,      0,      0,      0, 0, 0,  // 0100 NOP 
  MI|CO,  RO|II|CE,   AO|OI,  0,      0,      0, 0, 0,  // 1110 OUT
  MI|CO,  RO|II|CE,   HLT,    0,      0,      0, 0, 0,  // 1111 HLT 
};

uint16_t getControlWord(byte index) {
  return pgm_read_word(&DATA[index]);
}

// Function to Check Free SRAM
extern int __heap_start, *__brkval;
int freeMemory() {
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// Set EEPROM Data Pins as INPUT or OUTPUT
void setEEPROMDataMode(bool isWrite) {
    uint8_t mode = isWrite ? OUTPUT : INPUT;
    for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin++) {
        pinMode(pin, mode);
    }
}

// Set EEPROM Address using Shift Registers
void setAddress(int address, bool outputEnable) {
    shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, highByte(address) | (outputEnable ? 0x00 : 0x80));
    shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, lowByte(address));

    digitalWrite(SHIFT_LATCH, LOW);
    delayMicroseconds(5);  // Small delay for stability
    digitalWrite(SHIFT_LATCH, HIGH);
    delayMicroseconds(5);
    digitalWrite(SHIFT_LATCH, LOW);
}

// Read Byte from EEPROM
byte readEEPROM(int address) {
    setEEPROMDataMode(false);  // Set pins to input
    setAddress(address, true);

    byte data = 0;
    for (int pin = EEPROM_D7; pin >= EEPROM_D0; pin--) {
        data = (data << 1) | digitalRead(pin);
    }
    return data;
}

// Write Byte to EEPROM
void writeEEPROM(int address, byte data) {
    if (readEEPROM(address) == data) return;  // Skip redundant writes

    setEEPROMDataMode(true);  // Set pins to output
    setAddress(address, false);

    for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin++) {
        digitalWrite(pin, data & 1);
        data >>= 1;
    }

    digitalWrite(EEPROM_WRITE_EN, LOW);
    delayMicroseconds(1);
    digitalWrite(EEPROM_WRITE_EN, HIGH);
    delay(EEPROM_WRITE_DELAY);
}

// Clear Entire EEPROM
void clearEEPROM() {
    for (int address = 0; address < EEPROM_SIZE; address++) {
        if (readEEPROM(address) != 0xFF) {  // Skip writing if already erased
            writeEEPROM(address, 0xFF);
        }
    }
}

// Print EEPROM Contents with Improved Formatting
void printContents(int length) {
    Serial.print(F("Free SRAM before printing: "));
    Serial.println(freeMemory());

    for (int base = 0; base <= length; base += 16) {
        byte data[16];
        for (int offset = 0; offset < 16; offset++) {
            data[offset] = readEEPROM(base + offset);
        }

        char buf[80];
        snprintf_P(buf, sizeof(buf), PSTR("%03x: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x"),
                   base, data[0], data[1], data[2], data[3], data[4], 
                   data[5], data[6], data[7], data[8], data[9], 
                   data[10], data[11], data[12], data[13], data[14], data[15]);

        Serial.write(buf, strlen(buf));
        Serial.write('\n');
    }

    Serial.print(F("Free SRAM after printing: "));
    Serial.println(freeMemory());
}

// Setup Function
void setup() {
    Serial.begin(57600);
    
    pinMode(SHIFT_DATA, OUTPUT);
    pinMode(SHIFT_CLOCK, OUTPUT);
    pinMode(SHIFT_LATCH, OUTPUT);
    digitalWrite(EEPROM_WRITE_EN, HIGH);
    pinMode(EEPROM_WRITE_EN, OUTPUT);

    Serial.print(F("Free SRAM at startup: "));
    Serial.println(freeMemory());

    Serial.println(F("Erasing EEPROM..."));
    clearEEPROM();

    Serial.println(getControlWord(1));

    Serial.println(F("Programming EEPROM with microcode data..."));
    for (int address = 0; address < sizeof(DATA)/sizeof(getControlWord(0)); address++) {
      // EEPROM 1
      writeEEPROM(address, getControlWord(address) >> 8);

      // EEPROM 2
      //writeEEPROM(address, getControlWord(address));
    }

    Serial.print(F("Free SRAM after writing microcode data: "));
    Serial.println(freeMemory());

    printContents(EEPROM_SIZE - 1);

    Serial.print(F("Final Free SRAM: "));
    Serial.println(freeMemory());
}

void loop() {
    // Empty loop
}

