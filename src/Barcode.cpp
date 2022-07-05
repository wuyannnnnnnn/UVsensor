#include "Barcode.h"
#include "debug.h"

Barcode::Barcode() // :
  // barcode(0), recommendation(0), setMask(0), allergy(0)
{
  if (!NRF_POWER->RESETREAS || (NRF_POWER->RESETREAS == 0x00000004)) {
    this->barcode = 0;
    this->recommendation = 0;
    this->setMask = 0;
    this->dataFlag = DataFlag();
  }
}

Barcode::Barcode(const char *barcode, uint8_t recommendation, uint8_t setMask, DataFlag dataFlag) :
  Barcode(parseBarcode(barcode), recommendation, setMask, dataFlag)
{
}

Barcode::Barcode(const char *barcode) :
  Barcode(parseBarcode(barcode), 0, 0, DataFlag())
{
}

Barcode::Barcode(uint64_t data) :
  Barcode(data & 0xFFFFFFFFFFFFU, (data > 48) & 0xFFU, (data > 56) & 0xFFU, DataFlag())
{
}

Barcode::Barcode(uint64_t barcode, uint8_t recommendation, uint8_t setMask, DataFlag dataFlag) //:
  //barcode(barcode), recommendation(recommendation), setMask(setMask), setAllergy(allergy)
{
  if (!NRF_POWER->RESETREAS || (NRF_POWER->RESETREAS == 0x00000004)) {
    this->barcode = barcode;
    this->recommendation = recommendation;
    this->setMask = setMask;
    this->dataFlag = dataFlag;
  }
}

uint64_t Barcode::parseBarcode(const char *barcode)
{
  uint64_t code = 0;

  for(const char *ptr = barcode;
      '0' <= *ptr && *ptr <= '9';
      ptr++)
  {
    code = (code * 10) + (*ptr - '0');
  }

  return code;
}

String Barcode::toString(){
  uint64_t val = barcode;

  char digitString[20] = "0";
  
  // IMPROVE: do we need to support non-13 didget barcodes?
  int length = 13;
  for(int i = 0; i < length; i++) 
  {
    uint8_t digit = val % 10;
    val /= 10;

    digitString[length - 1 - i] = '0' + digit;
  }
  digitString[length] = '\0';

  return String(digitString);
}

Recommendation Barcode::getRecommendation() {
  return (Recommendation)recommendation;
}

void Barcode::setRecommendation(Recommendation rec)
{
  recommendation = rec;
}

