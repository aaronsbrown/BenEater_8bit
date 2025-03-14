#include <Arduino.h>
#include <avr/pgmspace.h>  // Required for PROGMEM usage

// ======================================================================
// Configuration Options
// ======================================================================
// Uncomment only ONE of the following lines at a time
// #define PROGRAM_EEPROM1  // Comment this out when programming EEPROM 2
#define PROGRAM_EEPROM2    // Uncomment this when programming EEPROM 2

// ======================================================================
// Pin Definitions
// ======================================================================
constexpr uint8_t SHIFT_DATA      = 2;
constexpr uint8_t SHIFT_CLOCK     = 3;
constexpr uint8_t SHIFT_LATCH     = 4;
constexpr uint8_t EEPROM_D0       = 5;   // Data pin D0 (lowest)
constexpr uint8_t EEPROM_D7       = 12;  // Data pin D7 (highest)
constexpr uint8_t EEPROM_WRITE_EN = 13;  // Write enable for EEPROM

// ======================================================================
// EEPROM and Timing Parameters
// ======================================================================
constexpr uint16_t EEPROM_SIZE        = 2048; // EEPROM size for clarity
constexpr uint16_t EEPROM_WRITE_DELAY = 5;    // Write delay in ms (adjust per EEPROM spec)

// ======================================================================
// Microinstruction Bit Definitions
// ======================================================================
constexpr uint16_t HLT = 0b1000000000000000;  // HALT
constexpr uint16_t MI  = 0b0100000000000000;  // MEM ADDRESS IN
constexpr uint16_t RI  = 0b0010000000000000;  // RAM IN
constexpr uint16_t RO  = 0b0001000000000000;  // RAM OUT
constexpr uint16_t II  = 0b0000100000000000;  // INSTRUCTION IN
constexpr uint16_t IO  = 0b0000010000000000;  // INSTRUCTION OUT (Lower byte)
constexpr uint16_t AI  = 0b0000001000000000;  // REG A IN
constexpr uint16_t AO  = 0b0000000100000000;  // REG A OUT
constexpr uint16_t EO  = 0b0000000010000000;  // ALU OUT
constexpr uint16_t SU  = 0b0000000001000000;  // SUBTRACT BIT IN
constexpr uint16_t BI  = 0b0000000000100000;  // REG B IN
constexpr uint16_t OI  = 0b0000000000010000;  // OUTPUT REG IN
constexpr uint16_t CE  = 0b0000000000001000;  // PC ENABLE
constexpr uint16_t CO  = 0b0000000000000100;  // PC OUT
constexpr uint16_t J   = 0b0000000000000010;  // JUMP
constexpr uint16_t FI  = 0b0000000000000001;  // FLAG REG IN

// ======================================================================
// Instruction/Flag Definitions
// ======================================================================
// Flags are stored in two bits: bit0 for Carry (CF), bit1 for Zero (ZF)
// For dynamic patching in the microcode generation:
//   - JC_INSTR (instruction 7) should be patched if CF is set.
//   - JZ_INSTR (instruction 8) should be patched if ZF is set.
constexpr uint8_t JC_INSTR = 7;  // Instruction index for JC
constexpr uint8_t JZ_INSTR = 8;  // Instruction index for JZ

// ======================================================================
// UCODE Template Stored in Flash (PROGMEM)
// ======================================================================
// Each row corresponds to a microinstruction step for a given instruction.
const PROGMEM uint16_t UCODE_TEMPLATE[16][8]  = {
  { CO|MI,  RO|II|CE,   0,            0, 0, 0, 0, 0 },  // 0000 NOP
  { CO|MI,  RO|II|CE,   IO|MI,        RO|AI, 0, 0, 0, 0 },  // 0001 LDA *indirect
  { CO|MI,  RO|II|CE,   IO|MI,        RO|BI, EO|AI|FI, 0, 0, 0 },  // 0010 ADD
  { CO|MI,  RO|II|CE,   IO|MI,        RO|BI, EO|AI|SU|FI, 0, 0, 0 },  // 0011 SUB
  { CO|MI,  RO|II|CE,   IO|MI,        AO|RI, 0, 0, 0, 0 },  // 0100 STA
  { CO|MI,  RO|II|CE,   IO|AI,        0, 0, 0, 0, 0 },  // 0101 LDI *direct 
  { CO|MI,  RO|II|CE,   IO|J,         0, 0, 0, 0, 0 },  // 0110 JMP 
  { CO|MI,  RO|II|CE,   0,            IO|J, 0, 0, 0, 0 },  // 0111 JMP2 
  { CO|MI,  RO|II|CE,   0,            0, 0, 0, 0, 0 },  // 1000 JZ 
  { CO|MI,  RO|II|CE,   0,            0, 0, 0, 0, 0 },  // 1001 NOP 
  { CO|MI,  RO|II|CE,   0,            0, 0, 0, 0, 0 },  // 1010 NOP 
  { CO|MI,  RO|II|CE,   0,            0, 0, 0, 0, 0 },  // 1011 NOP 
  { CO|MI,  RO|II|CE,   0,            0, 0, 0, 0, 0 },  // 1100 NOP  
  { CO|MI,  RO|II|CE,   IO|MI,        RO|OI, 0, 0, 0, 0 },  // 1101 OUTM 
  { CO|MI,  RO|II|CE,   AO|OI,        0, 0, 0, 0, 0 },  // 1110 OUTA
  { CO|MI,  RO|II|CE,   HLT,          0, 0, 0, 0, 0 }   // 1111 HLT 
};

// ======================================================================
// Inline Helper Functions for Field Extraction
// ======================================================================
// The address is structured as follows (bit positions):
//   A9-A8: Flags (Carry and Zero)
//   A7   : Byte select (high or low byte of microinstruction)
//   A6-A3: Instruction index (0-15)
//   A2-A0: Microinstruction step (0-7)
inline uint8_t extractFlags(uint16_t address) {
  return (address >> 8) & 0x03;  // bits 9-8: 2-bit flags
}

inline uint8_t extractByteSelect(uint16_t address) {
  return (address >> 7) & 0x01;  // bit 7: byte selector (0 = high, 1 = low)
}

inline uint8_t extractInstruction(uint16_t address) {
  return (address >> 3) & 0x0F;  // bits 6-3: instruction index (0-15)
}

inline uint8_t extractStep(uint16_t address) {
  return address & 0x07;         // bits 2-0: microinstruction step (0-7)
}

// ======================================================================
// Dynamic Control Word Generation
// ======================================================================
// Reads the base control word from PROGMEM and patches it dynamically if
// the instruction is a conditional jump and the corresponding flag is set.
uint16_t getUCodeByte(uint8_t flags, uint8_t instruction, uint8_t step) {
  uint16_t controlWord = pgm_read_word(&UCODE_TEMPLATE[instruction][step]);

  // Only at step 2 are JC and JZ instructions patched.
  if (step == 2) {
    // If processing the JC instruction and the Carry flag (bit 0) is set:
    if ((instruction == JC_INSTR) && (flags & 0x01)) {
      controlWord = IO | J;
    }
    // If processing the JZ instruction and the Zero flag (bit 1) is set:
    else if ((instruction == JZ_INSTR) && (flags & 0x02)) {
      controlWord = IO | J;
    }
  }
  return controlWord;
}

// ======================================================================
// Free SRAM Check
// ======================================================================
// This function estimates free SRAM. Note that in some cases the estimation
// might not be exact.
extern int __heap_start, *__brkval;
int freeMemory() {
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// ======================================================================
// EEPROM Utility Functions
// ======================================================================

// Sets EEPROM data pins to either INPUT (read) or OUTPUT (write)
void setEEPROMDataMode(bool isWrite) {
    uint8_t mode = isWrite ? OUTPUT : INPUT;
    for (uint8_t pin = EEPROM_D0; pin <= EEPROM_D7; pin++) {
        pinMode(pin, mode);
    }
}

// Sets the EEPROM address via shift registers. The outputEnable flag controls
// a specific bit in the high address byte (see hardware design).
void setAddress(uint16_t address, bool outputEnable) {
    // Combine high address byte with OE flag (0x80 = disable output)
    shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, highByte(address) | (outputEnable ? 0x00 : 0x80));
    shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, lowByte(address));

    // Toggle the latch to transfer the address to the output
    digitalWrite(SHIFT_LATCH, LOW);
    delayMicroseconds(5);  // Small delay for stability
    digitalWrite(SHIFT_LATCH, HIGH);
    delayMicroseconds(5);
    digitalWrite(SHIFT_LATCH, LOW);
}

// Reads one byte from EEPROM at the given address
byte readEEPROM(uint16_t address) {
    setEEPROMDataMode(false);  // Set data pins to input for reading
    setAddress(address, true);

    byte data = 0;
    // Read bits from EEPROM_D7 (MSB) to EEPROM_D0 (LSB)
    for (int8_t pin = EEPROM_D7; pin >= EEPROM_D0; pin--) {
        data = (data << 1) | digitalRead(pin);
    }
    return data;
}

// Writes one byte to EEPROM at the given address.
// This function first checks if a write is necessary to reduce wear.
void writeEEPROM(uint16_t address, byte data) {
    if (readEEPROM(address) == data) return;  // Skip write if data is unchanged

    setEEPROMDataMode(true);  // Set data pins to output for writing
    setAddress(address, false);

    // Write data bit by bit (LSB first)
    for (uint8_t pin = EEPROM_D0; pin <= EEPROM_D7; pin++) {
        digitalWrite(pin, data & 1);
        data >>= 1;
    }

    // Pulse the write enable signal
    digitalWrite(EEPROM_WRITE_EN, LOW);
    delayMicroseconds(1);
    digitalWrite(EEPROM_WRITE_EN, HIGH);
    delay(EEPROM_WRITE_DELAY);

    // Optional: Add verification read-back here if needed.
}

// Clears the entire EEPROM by writing 0xFF to each address,
// skipping addresses that already contain 0xFF.
void clearEEPROM() {
    for (uint16_t address = 0; address < EEPROM_SIZE; address++) {
        if (readEEPROM(address) != 0xFF) {
            writeEEPROM(address, 0xFF);
        }
    }
}

// Prints EEPROM contents in a formatted manner.
void printContents(uint16_t length) {
    Serial.print(F("Free SRAM before printing: "));
    Serial.println(freeMemory());

    for (uint16_t base = 0; base <= length; base += 16) {
        byte data[16];
        for (uint8_t offset = 0; offset < 16; offset++) {
            data[offset] = readEEPROM(base + offset);
        }

        char buf[80];
        snprintf_P(buf, sizeof(buf),
                   PSTR("%03x: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x"),
                   base, data[0], data[1], data[2], data[3], data[4],
                   data[5], data[6], data[7], data[8], data[9],
                   data[10], data[11], data[12], data[13], data[14], data[15]);
        Serial.write(buf, strlen(buf));
        Serial.write('\n');
    }

    Serial.print(F("Free SRAM after printing: "));
    Serial.println(freeMemory());
}

// ======================================================================
// Main Setup and Loop Functions
// ======================================================================
void setup() {
    // Initialize shift register pins and write enable pin
    pinMode(SHIFT_DATA, OUTPUT);
    pinMode(SHIFT_CLOCK, OUTPUT);
    pinMode(SHIFT_LATCH, OUTPUT);
    digitalWrite(EEPROM_WRITE_EN, HIGH);
    pinMode(EEPROM_WRITE_EN, OUTPUT);

    Serial.begin(57600);
    Serial.print(F("Free SRAM at startup: "));
    Serial.println(freeMemory());

    Serial.println(F("Erasing EEPROM..."));
    clearEEPROM();

    Serial.println(F("Programming EEPROM "));
    // Program EEPROM. The address structure is:
    //   - Bits 9-8: Flags (Carry and Zero)
    //   - Bit 7   : Byte select (high or low)
    //   - Bits 6-3: Instruction index (0-15)
    //   - Bits 2-0: Microinstruction step (0-7)
    for (uint16_t address = 0; address < EEPROM_SIZE/2; address++) {
        uint8_t flags       = extractFlags(address);
        uint8_t byteSelect  = extractByteSelect(address);
        uint8_t instruction = extractInstruction(address);
        uint8_t step        = extractStep(address);

        // Generate the control word dynamically based on current flags
        uint16_t controlWord = getUCodeByte(flags, instruction, step);

        // Write high or low byte based on the byteSelect flag:
        //   - If byteSelect is 0, write the high byte.
        //   - If byteSelect is 1, write the low byte.
        if (byteSelect) { 
            writeEEPROM(address, controlWord & 0xFF);  // Write Low Byte
        } else {
            writeEEPROM(address, controlWord >> 8);      // Write High Byte
        }
     
        if (address % 64 == 0) {
            Serial.print(".");
        }
    }
    
    printContents(EEPROM_SIZE/2 - 1);
    Serial.print(F("Final Free SRAM: "));
    Serial.println(freeMemory());
}

void loop() {
    // Empty loop.
}