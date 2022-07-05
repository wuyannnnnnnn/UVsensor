#ifndef _LTR390UV_H_
#define _LTR390UV_H_

#include <Arduino.h>

// measurement for ambient or UV light 
enum LTR390UV_MODE { LTR390UV_MODE_ALS = 0x00, LTR390UV_MODE_UVI = 0x01};


// sensor gain
enum LTR390UV_GAIN { LTR390UV_GAIN_1 = 0x00, LTR390UV_GAIN_3 = 0x01, LTR390UV_GAIN_6 = 0x02, 
LTR390UV_GAIN_9 = 0x03, LTR390UV_GAIN_18 = 0x04};


// resolution
enum LTR390UV_RESOLUTION {LTR390UV_RESOLUTION_20BIT = 0x00, LTR390UV_RESOLUTION_19BIT = 0x01, 
LTR390UV_RESOLUTION_18BIT = 0x02, LTR390UV_RESOLUTION_17BIT = 0x03, 
LTR390UV_RESOLUTION_16BIT = 0x04, LTR390UV_RESOLUTION_13BIT = 0x05};


class LTR390UV {
public:
  LTR390UV();
  bool begin(void);
  bool reset(void);
  void writeEnable(bool en);
  bool readEnable(void);

  static void writeMode(LTR390UV_MODE mode);
  static LTR390UV_MODE readMode();

  void writeResolution(LTR390UV_RESOLUTION res);
  int readResolution(void);

  void writeGain(LTR390UV_GAIN gain);
  int readGain(void);

  int readPower(void);
  int readInterrupt(void);
  int readDataStatus(void);

  uint32_t readALS(void);
  uint32_t readUVI(void);

  void setThresholds(uint32_t low, uint32_t up);

  void configInterrupt(bool enable, LTR390UV_MODE mode, uint8_t persistance = 0);

private:

};


#endif // _LTR390UV_H_