#include <avr/pgmspace.h>  // Required for PROGMEM usage

// Uncomment only ONE of the following lines at a time
// #define PROGRAM_EEPROM1  // Comment this out when programming EEPROM 2
#define PROGRAM_EEPROM2  // Uncomment this when programming EEPROM 2

#define SHIFT_DATA 2
#define SHIFT_CLOCK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define EEPROM_WRITE_EN 13

#define EEPROM_SIZE 2048  // Defined EEPROM size for clarity
#define EEPROM_WRITE_DELAY 5  // Adjust based on EEPROM specs

#define HLT   0b1000000000000000  // HALT
#define MI    0b0100000000000000  // MEM ADDRESS IN
#define RI    0b0010000000000000  // RAM IN
#define RO    0b0001000000000000  // RAM OUT
#define II    0b0000100000000000  // INSTRUCTION IN
#define IO    0b0000010000000000  // INSTRUCTION OUT (Lower byte)
#define AI    0b0000001000000000  // REG A IN
#define AO    0b0000000100000000  // REG A OUT
#define EO    0b0000000010000000  // ALU OUT
#define SU    0b0000000001000000  // SUBTRACT BIT IN
#define BI    0b0000000000100000  // REG B IN
#define OI    0b0000000000010000  // OUTPUT REG IN
#define CE    0b0000000000001000  // PC ENABLE
#define CO    0b0000000000000100  // PC OUT
#define J     0b0000000000000010  // JUMP
#define FI    0b0000000000000001  // FLAG REG IN

const uint16_t DATA[] PROGMEM {
  CO|MI,  RO|II|CE,   0,      0,      0,        0, 0, 0,  // 0000 NOP
  CO|MI,  RO|II|CE,   IO|MI,  RO|AI,  0,        0, 0, 0,  // 0001 LDA *indirect
  CO|MI,  RO|II|CE,   IO|MI,  RO|BI,  EO|AI,    0, 0, 0,  // 0010 ADD
  CO|MI,  RO|II|CE,   IO|MI,  RO|BI,  EO|AI|SU, 0, 0, 0,  // 0011 SUB
  CO|MI,  RO|II|CE,   IO|MI,  AO|RI,  0,        0, 0, 0,  // 0100 STA
  CO|MI,  RO|II|CE,   IO|AI,  0,      0,        0, 0, 0,  // 0101 LDI *direct 
  CO|MI,  RO|II|CE,   IO|J,   0,      0,        0, 0, 0,  // 0110 JMP 
  CO|MI,  RO|II|CE,   0,      0,      0,        0, 0, 0,  // 0111 NOP 
  CO|MI,  RO|II|CE,   0,      0,      0,        0, 0, 0,  // 1000 NOP 
  CO|MI,  RO|II|CE,   0,      0,      0,        0, 0, 0,  // 1001 NOP 
  CO|MI,  RO|II|CE,   0,      0,      0,        0, 0, 0,  // 1010 NOP 
  CO|MI,  RO|II|CE,   0,      0,      0,        0, 0, 0,  // 1011 NOP 
  CO|MI,  RO|II|CE,   0,      0,      0,        0, 0, 0,  // 1100 NOP  
  CO|MI,  RO|II|CE,   IO|MI,  RO|OI,  0,        0, 0, 0,  // 1101 OUTM 
  CO|MI,  RO|II|CE,   AO|OI,  0,      0,        0, 0, 0,  // 1110 OUTA
  CO|MI,  RO|II|CE,   HLT,    0,      0,        0, 0, 0,  // 1111 HLT 
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
    setEEPROMDataMode(false);  // Set data pins to input
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

    setEEPROMDataMode(true);  // Set data pins to output
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


    Serial.println(F("Programming EEPROM "));
    for (int address = 0; address < sizeof(DATA) / sizeof(getControlWord(0)); address++) {
        writeEEPROM(address, getControlWord(address) >> 8); 
      }
    // Serial.println(F("EEPROM 1 Programming Complete. Swap to EEPROM 2."));
  
    for (int address = 0; address < sizeof(DATA) / sizeof(getControlWord(0)); address++) {
        writeEEPROM(address + 128, getControlWord(address)); 
    }
    // Serial.println(F("EEPROM 2 Programming Complete. Swap to system."));
    
   printContents(EEPROM_SIZE - 1);

    Serial.print(F("Final Free SRAM: "));
    Serial.println(freeMemory());
}

// Empty Loop
void loop() {}