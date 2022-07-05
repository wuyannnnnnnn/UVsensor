#ifndef BARCODE_UTILS_H
#define BARCODE_UTILS_H

#include <Arduino.h>

enum Recommendation
{
  Recommendation_Good = 1,
  Recommendation_Bad = 0,
  Recommendation_NotSet = -1,
  Recommendation_BadSlotNumber = 10
};

typedef struct __attribute__((__packed__)) DataFlag_t
{
  uint8_t mLifeStyle:4;
  uint8_t mReserve:4;
  uint8_t mAllergy:8;
  DataFlag_t() : mLifeStyle(0xF), mReserve(0xF), mAllergy(0xFF){};
} DataFlag;

class Barcode
{
  private:
    uint64_t barcode:48;
    uint64_t recommendation:8;  // if 1 good else bad
    uint64_t setMask:8;         // if 1 recommendation valid
    DataFlag dataFlag;

    uint64_t parseBarcode(const char *barcode);
  public:
    Barcode();
    Barcode(String barcode) : Barcode(barcode.c_str()) {
    }
    Barcode(const char *barcode);
    Barcode(const char *barcode, uint8_t recommendation, uint8_t setMask, DataFlag dataFlag);
    Barcode(uint64_t data);
    Barcode(uint64_t barcode, uint8_t recommendation, uint8_t setMask, DataFlag dataFlag);
    
    String toString();
    uint64_t getCode() {
      return barcode;
    }

    void setCode(uint64_t barcodeValue) {
      barcode = barcodeValue;
    }

    uint8_t getAllergy(void) {
      return this->dataFlag.mAllergy;
    }
    
    void setDataFlag(DataFlag  dataFlag) {
      this->dataFlag = dataFlag;
    }

    uint8_t getLifeStyle(void) {
      return this->dataFlag.mLifeStyle;
    }

    Recommendation getRecommendation();
    void setRecommendation(Recommendation rec);
    void setRecommendation(Barcode a) {
      this->recommendation = a.recommendation;
      this->setMask = a.setMask;
    }
    uint8_t returnRecommendation()
    {
      return (uint8_t) this->recommendation;
    }
    uint8_t returnSetMask()
    {
      return (uint8_t)this->setMask;
    }
    inline bool operator==(Barcode a) {
      return barcode == a.barcode;
    }

    inline operator bool() {
      return 0 != barcode;
    }
  };

#endif // !BARCODE_UTILS_H
