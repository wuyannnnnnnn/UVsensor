#include <Wire.h>
#include "LTR390UV.h"

#ifdef __AVR__
 #include <avr/pgmspace.h>
 #define WIRE Wire
#else
//  #define PROGMEM
//  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
 #define WIRE Wire
#endif

// LTR390UV Register Map
#define LTR390UV_ADDRESS                0x53 // I2C Address
#define LTR390UV_MAIN_CTRL              0x00 // ALS/UVS operation mode control
#define LTR390UV_ALS_UVS_MEAS_RATE      0x04 // ALS/UVS measurement rate
#define LTR390UV_ALS_UVS_GAIN           0x05 // ALS/UVS analog gain range
#define LTR390UV_PART_ID                0x06 // part number ID
#define LTR390UV_MAIN_STATUS            0x07 // power-on/interrupt/data status
#define LTR390UV_ALS_DATA_0             0x0D // ALS ADC measurement data (LSB)
#define LTR390UV_ALS_DATA_1             0x0E // ALS ADC measurement data
#define LTR390UV_ALS_DATA_2             0x0F // ALS ADC measurement data (MSB)
#define LTR390UV_UVS_DATA_0             0x10 // UVS ADC measurement data (LSB)
#define LTR390UV_UVS_DATA_1             0x11 // UVS ADC measurement data
#define LTR390UV_UVS_DATA_2             0x12 // UVS ADC measurement data (MSB)
#define LTR390UV_INT_CFG                0x19 // interrupt configuration
#define LTR390UV_INT_PST                0x1A // interrupt persist setting
#define LTR390UV_ALS_UVS_THRES_UP_0     0x21 // ALS/UVS interrupt upper threshold (LSB)
#define LTR390UV_ALS_UVS_THRES_UP_1     0x22 // ALS/UVS interrupt upper threshold 
#define LTR390UV_ALS_UVS_THRES_UP_2     0x23 // ALS/UVS interrupt upper threshold (MSB)
#define LTR390UV_ALS_UVS_THRES_LOW_0    0x24 // ALS/UVS interrupt lower threshold (LSB)
#define LTR390UV_ALS_UVS_THRES_LOW_1    0x25 // ALS/UVS interrupt lower threshold
#define LTR390UV_ALS_UVS_THRES_LOW_2    0x26 // ALS/UVS interrupt lower threshold (MSB)

#if (ARDUINO >= 100)
 #include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
 // Macro to deal with the difference in I2C write functions from old and new Arduino versions.
 #define _I2C_WRITE write
 #define _I2C_READ  read
#else
 #include <WProgram.h>
 #define _I2C_WRITE send
 #define _I2C_READ  receive
#endif


// instantiates a new LTR390 class
LTR390UV::LTR390UV(void) {};

bool LTR390UV::begin(void) {
  int currentByte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_PART_ID);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  // check part ID!
  if ((currentByte) != 0xB2) {
    return false;
    Serial.print("the wrong part ID is");
    Serial.println(currentByte);
  }

  return true;
}


/*!
 *  @brief  Perform a soft reset with 10ms delay.
 *  @returns True on success (reset bit was cleared post-write)
 */
bool LTR390UV::reset(void) {
  int currentByte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(uint8_t(LTR390UV_MAIN_CTRL));
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  if (bitRead(currentByte,4)){
    delay(10);
    begin();
    return false; // reset
  }
  
  return true; // reset is not triggered (default)
}


/*!
 *  @brief  Enable or disable the light sensor
 *  @param  en True to enable, False to disable
 */
void LTR390UV::writeEnable(bool en) {
  int currentByte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(uint8_t(LTR390UV_MAIN_CTRL));
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  if (en)
    bitWrite(currentByte,1,1);
  else
    bitWrite(currentByte,1,0);

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(uint8_t(LTR390UV_MAIN_CTRL));
  WIRE._I2C_WRITE(currentByte);
  WIRE.endTransmission();
}

/*!
 *  @brief  Read the enabled-bit from the sensor
 *  @returns True if enabled
 */
bool LTR390UV::readEnable(void) {
  int currentByte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(uint8_t(LTR390UV_MAIN_CTRL));
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  return bitRead(currentByte,1);
}


/*!
 *  @brief  Set the sensor mode to EITHER ambient (LTR390_MODE_ALS) or UV
 * (LTR390_MODE_UVS)
 *  @param  mode The desired mode - LTR390_MODE_UVS or LTR390_MODE_ALS
 */
void LTR390UV::writeMode(LTR390UV_MODE mode) {
  int currentByte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(uint8_t(LTR390UV_MAIN_CTRL));
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();
  
  if (mode == LTR390UV_MODE_ALS){ // ALS
    bitWrite(currentByte,3,0);
  }else{ // UVS
    bitWrite(currentByte,3,1);
  }

  // Serial.print("write Mode");
  // Serial.println(currentByte,HEX);

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(uint8_t(LTR390UV_MAIN_CTRL));
  WIRE._I2C_WRITE(currentByte);
  WIRE.endTransmission();
}

/*!
 *  @brief  get the sensor's mode
 *  @returns The current mode - LTR390_MODE_UVS or LTR390_MODE_ALS
 */
LTR390UV_MODE LTR390UV::readMode(){
  int mode;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(uint8_t(LTR390UV_MAIN_CTRL));
  WIRE.endTransmission();
  
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  mode = WIRE._I2C_READ() >> 3;
  return static_cast<LTR390UV_MODE>(mode);
}


/*!
 *  @brief  Set the sensor resolution. Higher resolutions take longer to read!
 *  @param  res The desired resolution: LTR390_RESOLUTION_13BIT,
 *  LTR390_RESOLUTION_16BIT, LTR390_RESOLUTION_17BIT, LTR390_RESOLUTION_18BIT,
 *  LTR390_RESOLUTION_19BIT or LTR390_RESOLUTION_20BIT
 */
void LTR390UV::writeResolution(LTR390UV_RESOLUTION res){
  int currentByte;
  
  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_MEAS_RATE);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  switch(res){
    case LTR390UV_RESOLUTION_20BIT: // 20 bits
        bitWrite(currentByte,4,0);
        bitWrite(currentByte,5,0);
        bitWrite(currentByte,6,0); break;
    case LTR390UV_RESOLUTION_19BIT: // 19 bits
        bitWrite(currentByte,4,1);
        bitWrite(currentByte,5,0);
        bitWrite(currentByte,6,0); break;
    case LTR390UV_RESOLUTION_18BIT: // 18 bits
        bitWrite(currentByte,4,0);
        bitWrite(currentByte,5,1);
        bitWrite(currentByte,6,0); break;
    case LTR390UV_RESOLUTION_17BIT: // 17 bits
        bitWrite(currentByte,4,1);
        bitWrite(currentByte,5,1);
        bitWrite(currentByte,6,0); break;
    case LTR390UV_RESOLUTION_16BIT: // 16 bits
        bitWrite(currentByte,4,0);
        bitWrite(currentByte,5,0);
        bitWrite(currentByte,6,1); break;
    case LTR390UV_RESOLUTION_13BIT: // 13 bits
        bitWrite(currentByte,4,1);
        bitWrite(currentByte,5,0);
        bitWrite(currentByte,6,1); break;
    default: // 18 bits
        bitWrite(currentByte,4,0);
        bitWrite(currentByte,5,1);
        bitWrite(currentByte,6,0); break;
  }

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_MEAS_RATE);
  WIRE._I2C_WRITE(currentByte);
  WIRE.endTransmission();
}

/*!
 *  @brief  Get the sensor's resolution
 *  @returns The current resolution: LTR390_RESOLUTION_13BIT,
 *  LTR390_RESOLUTION_16BIT, LTR390_RESOLUTION_17BIT, LTR390_RESOLUTION_18BIT,
 *  LTR390_RESOLUTION_19BIT or LTR390_RESOLUTION_20BIT
 */
int LTR390UV::readResolution(void){
  int currentByte, gain = 0, retVal = 0;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_MEAS_RATE);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  bitWrite(gain,0,bitRead(currentByte,4));
  bitWrite(gain,1,bitRead(currentByte,5));
  bitWrite(gain,2,bitRead(currentByte,6));
 
  switch(gain) {
    case 0: retVal = 20; break; // 20 bits
    case 1: retVal = 19; break; // 19 bits
    case 2: retVal = 18; break; // 18 bits
    case 3: retVal = 17; break; // 17 bits
    case 4: retVal = 16; break; // 16 bits
    case 5: retVal = 13; break; // 13 bits
    default: retVal = 0; break; // Reserved
  }

  return retVal;
}


/*!
 *  @brief  Set the sensor gain
 *  @param  gain The desired gain: LTR390_GAIN_1, LTR390_GAIN_3, LTR390_GAIN_6
 *  LTR390_GAIN_9 or LTR390_GAIN_18
 */
void LTR390UV::writeGain(LTR390UV_GAIN gain){
  int currentByte;
  
  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_GAIN);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  switch(gain){
    case LTR390UV_GAIN_1: // gain range:1
        bitWrite(currentByte,0,0);
        bitWrite(currentByte,1,0);
        bitWrite(currentByte,2,0); break;
    case LTR390UV_GAIN_3: // gain range:3
        bitWrite(currentByte,0,1);
        bitWrite(currentByte,1,0);
        bitWrite(currentByte,2,0); break;
    case LTR390UV_GAIN_6: // gain range:3
        bitWrite(currentByte,0,0);
        bitWrite(currentByte,1,1);
        bitWrite(currentByte,2,0); break;
    case LTR390UV_GAIN_9: // gain range:3
        bitWrite(currentByte,0,1);
        bitWrite(currentByte,1,1);
        bitWrite(currentByte,2,0); break;
    case LTR390UV_GAIN_18: // gain range:3
        bitWrite(currentByte,0,0);
        bitWrite(currentByte,1,0);
        bitWrite(currentByte,2,1); break;
    default: // gain range:3
        bitWrite(currentByte,0,0);
        bitWrite(currentByte,1,0);
        bitWrite(currentByte,2,0); break;
  }

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_GAIN);
  WIRE._I2C_WRITE(currentByte);
  WIRE.endTransmission();
}

/*!
 *  @brief  Get the sensor's gain
 *  @returns gain The current gain: LTR390_GAIN_1, LTR390_GAIN_3, LTR390_GAIN_6
 *  LTR390_GAIN_9 or LTR390_GAIN_18
 */
int LTR390UV::readGain(void){
  int currentByte, gain = 0, retVal = 0;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_GAIN);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();

  bitWrite(gain,0,bitRead(currentByte,0));
  bitWrite(gain,1,bitRead(currentByte,1));
  bitWrite(gain,2,bitRead(currentByte,2));
 
  switch(gain) {
    case 0: retVal = 1; break; // gain range:1
    case 1: retVal = 3; break; // gain range:3
    case 2: retVal = 6; break; // gain range:6
    case 3: retVal = 9; break; // gain range:9
    case 4: retVal = 18; break; // gain range:18
    default: retVal = 0; break; // ERROR
  }
}


int LTR390UV::readPower(void){
  //int currentByte, power = 0;
  int currentByte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_MAIN_STATUS);
  WIRE.endTransmission();
  
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();
  //power = bitRead(currentByte,5);

  if (bitRead(currentByte,5) == 1){
    return 1;  // power on
  }else{
    return 0;
  }
}

int LTR390UV::readInterrupt(void){
  //int currentByte, interrupt = 0;
  int currentByte;
  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_MAIN_STATUS);
  WIRE.endTransmission();
  
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();
  // interrupt = bitRead(currentByte,4);
  
  // return interrupt;

  if (bitRead(currentByte,4) == 0){
    return 1;  // interrupt is not trriggered (default)
  }else{
    return 0;  // interrupt is triggered 
  }
}

int LTR390UV::readDataStatus(void){
  //int currentByte, data = 0;
  int currentByte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_MAIN_STATUS);
  WIRE.endTransmission();
  
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte = WIRE._I2C_READ();
  // data = bitRead(currentByte,3);
  
  // return data;

  if (bitRead(currentByte,3) == 1){
    return 1;  // new data (hasn't been read)
  }else{
    return 0;  // old data
  }
}

uint32_t LTR390UV::readALS(void){
  int byte1, byte2, byte3,byte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_DATA_0);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 3);
  byte1 = WIRE._I2C_READ();
  byte2 = WIRE._I2C_READ();
  byte3 = WIRE._I2C_READ();

//  byte = (byte3 << 16)|(byte2 << 8)|(byte1);
//   Serial.println(byte,HEX);
  return ((byte3 << 16)|(byte2 << 8)|(byte1));
}

uint32_t LTR390UV::readUVI(void){
  int byte1, byte2, byte3,byte;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_UVS_DATA_0);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 3);
  byte1 = WIRE._I2C_READ();
  byte2 = WIRE._I2C_READ();
  byte3 = WIRE._I2C_READ();
  byte = (byte3 << 16)|(byte2 << 8)|(byte1);
  Serial.println(byte,HEX);
  return ((byte3 << 16)|(byte2 << 8)|(byte1));
}

/*!
 *  @brief  Set the interrupt output threshold range for lower and upper.
 *  When the sensor is below the lower, or above upper, interrupt will fire
 *  @param  lower The lower value to compare against the data register.
 *  @param  higher The higher value to compare against the data register.
 */
void LTR390UV::setThresholds(uint32_t low, uint32_t up) {
  int low_byte1, low_byte2, low_byte3;
  int up_byte1, up_byte2, up_byte3;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_THRES_LOW_0);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 3);
  low_byte1 = WIRE._I2C_READ();
  low_byte2 = WIRE._I2C_READ();
  low_byte3 = WIRE._I2C_READ();

  // WIRE.beginTransmission(LTR390UV_ADDRESS);
  // WIRE._I2C_WRITE(LTR390UV_ALS_UVS_THRES_LOW_0);
  // WIRE.endTransmission();
  // WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  // low_byte1 = WIRE._I2C_READ();

  // WIRE.beginTransmission(LTR390UV_ADDRESS);
  // WIRE._I2C_WRITE(LTR390UV_ALS_UVS_THRES_LOW_1);
  // WIRE.endTransmission();
  // WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  // low_byte2 = WIRE._I2C_READ();

  // WIRE.beginTransmission(LTR390UV_ADDRESS);
  // WIRE._I2C_WRITE(LTR390UV_ALS_UVS_THRES_LOW_2);
  // WIRE.endTransmission();
  // WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  // low_byte3 = WIRE._I2C_READ();

  low = ((low_byte3 << 16)|(low_byte2 << 8)|(low_byte1));

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_THRES_UP_0);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  up_byte1 = WIRE._I2C_READ();

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_THRES_UP_1);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  up_byte2 = WIRE._I2C_READ();

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_ALS_UVS_THRES_UP_2);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  up_byte3 = WIRE._I2C_READ();

  up = ((up_byte3 << 16)|(up_byte2 << 8)|(up_byte1));
}


/*!
 *  @brief  Configure the interrupt based on the thresholds in setThresholds()
 *  When the sensor is below the lower, or above upper thresh, interrupt will
 * fire
 *  @param  enable Whether the interrupt output is enabled
 *  @param  source Whether to use the ALS or UVS data register to compare
 *  @param  persistance The number of consecutive out-of-range readings before
 *          we fire the IRQ. Default is 0 (each reading will fire)
 */
void LTR390UV::configInterrupt(bool enable, LTR390UV_MODE mode, uint8_t persistance) {
  int currentByte1, currentByte2;

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_INT_CFG);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte1 = WIRE._I2C_READ();
  bitWrite(currentByte1,2,enable);
  
  if (mode == LTR390UV_MODE_ALS){
    bitWrite(currentByte1,5,0);
    bitWrite(currentByte1,4,1);
  }
  if (mode == LTR390UV_MODE_UVI){
    bitWrite(currentByte1,5,1);
    bitWrite(currentByte1,4,1);
  }

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_INT_CFG);
  WIRE._I2C_WRITE(currentByte1);
  WIRE.endTransmission();


  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_INT_PST);
  WIRE.endTransmission();
  WIRE.requestFrom(LTR390UV_ADDRESS, 1);
  currentByte2 = WIRE._I2C_READ();

  bitWrite(currentByte2,7,bitRead(persistance,7));
  bitWrite(currentByte2,6,bitRead(persistance,6));
  bitWrite(currentByte2,5,bitRead(persistance,5));
  bitWrite(currentByte2,4,bitRead(persistance,4));

  WIRE.beginTransmission(LTR390UV_ADDRESS);
  WIRE._I2C_WRITE(LTR390UV_INT_PST);
  WIRE._I2C_WRITE(currentByte2);
  WIRE.endTransmission();
}

