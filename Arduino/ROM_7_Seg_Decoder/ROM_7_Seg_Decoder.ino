#define SHIFT_DATA 2
#define SHIFT_CLOCK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define EEPROM_WRITE_EN 13

void setAddress(int address, bool outputEnable) {
  
   shiftOut(SHIFT_DATA, SHIFT_CLOCK,  MSBFIRST, highByte(address) | (outputEnable ? 0x00 : 0x80  ) ) ;
   shiftOut(SHIFT_DATA, SHIFT_CLOCK,  MSBFIRST, lowByte(address) );
 

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}

void clear() {
  for(int address = 0; address <= 2047; address += 1) {
    writeEEPROM(address, 0xff);
  }
}

byte readEEPROM(int address) {

  for(int pin = EEPROM_D0; pin <= EEPROM_D7; pin +=1) {
    pinMode(pin, INPUT);
  }

  setAddress(address, /*outputEnabled*/ true);
  byte data = 0;
  for( int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }
  return data;  
}

void writeEEPROM(int address, byte data) {
  
  for(int pin = EEPROM_D0; pin <= EEPROM_D7; pin +=1) {
    pinMode(pin, OUTPUT);
  }
    
  
  setAddress(address, false);
  
  for( int pin = EEPROM_D0; pin <= EEPROM_D7; pin +=1 ) {
    digitalWrite(pin, data & 1); // get least significant bit
    data = data >> 1;   
  }

  digitalWrite(EEPROM_WRITE_EN, LOW);
  delayMicroseconds(1);
  digitalWrite(EEPROM_WRITE_EN, HIGH);
  delay(10);
}

void printContents(int length) {
  for ( int base = 0; base <= length; base +=16 ) {
    byte data[16];
    for( int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%03x: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x",
      base, data[0], data[1], data[2], data[3], data[4], 
      data[5], data[6], data[7], data[8], data[9], 
      data[10], data[11], data[12], data[13], data[14], data[15], data[16]);
    Serial.println(buf);
  }
}

// DIGITS
byte SEVEN_SEG_TRUTH_TABLE[] = {
  0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b 
};

void setup() {
  
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLOCK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  
  digitalWrite(EEPROM_WRITE_EN, HIGH); // sets pullup resistor
  pinMode(EEPROM_WRITE_EN, OUTPUT); // now enable output pin 
  

  Serial.begin(57600);

  // ===== UNSIGNED =======================================

  // // ONES
  // Serial.println(("UNSIGNED: Programming ones place"));
  // for(int value = 0; value <= 255; value +=1) {
  //   writeEEPROM(value, SEVEN_SEG_TRUTH_TABLE[ value % 10] );
  // }
 
  // // TENS
  // Serial.println(("UNSIGNED: Programming tens place"));
  // for(int value = 0; value <= 255; value +=1) {
  //   writeEEPROM(value + 256, SEVEN_SEG_TRUTH_TABLE[ (value / 10) % 10] );
  // }
  
  // // HUNDREDS
  // Serial.println(("UNSIGNED: Programming hundreds place"));
  // for(int value = 0; value <= 255; value +=1) {
  //   writeEEPROM(value + 512, SEVEN_SEG_TRUTH_TABLE[ (value / 100) % 10] );
  // }

  // // unused
  // Serial.println(("UNSIGNED: Programming unused"));
  // for(int value = 0; value <= 255; value +=1) {
  //   writeEEPROM(value + 768, 0x00);
  // }

  // // ===== SIGNED =======================================

  // // ONES
  // Serial.println(("SIGNED: Programming ones place"));
  // for(int value = -128; value <= 127; value +=1) {
  //   // cast 'value' to unsigned byte => 8bit byte in 2s complement (i.e. a positive number)
  //   writeEEPROM( (byte)value + 1024, SEVEN_SEG_TRUTH_TABLE[ abs(value) % 10] ); 
  // }
  
  // Serial.println(("SIGNED: Programming tens place"));
  // for(int value = -128; value <= 127; value +=1) {
  //   // cast 'value' to unsigned byte => 8bit byte in 2s complement (i.e. a positive number)
  //   writeEEPROM( (byte)value + 1280, SEVEN_SEG_TRUTH_TABLE[ abs(value / 10 ) % 10] ); 
  // }
  
  // Serial.println(("SIGNED: Programming hundreds place"));
  // for(int value = -128; value <= 127; value +=1) {
  //   // cast 'value' to unsigned byte => 8bit byte in 2s complement (i.e. a positive number)
  //   writeEEPROM( (byte)value + 1536, SEVEN_SEG_TRUTH_TABLE[ abs(value / 100 ) % 10] ); 
  // }

  // Serial.println(("SIGNED: Programming minus"));
  // for(int value = -128; value <= 127; value +=1) {
  //   if(value < 0) {
  //     writeEEPROM( (byte)value + 1792, 0x01); 
  //   } else {
  //     writeEEPROM( (byte)value + 1792, 0); 
  //   }
    
  // }


  printContents(0);

  Serial.println("=================");
  byte foo = readEEPROM(1);
  
  char buf[3];
  sprintf(buf, "%02x", foo);
  Serial.println(buf);
}

void loop() {
  // put your main code here, to run repeatedly:

}
