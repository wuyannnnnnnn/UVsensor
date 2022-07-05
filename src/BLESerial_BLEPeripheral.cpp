
#ifndef ADAFRUIT_NRF52_CORE

#ifndef ENABLE_DEBUG_BLE_SERIAL
#undef BLE_SERIAL_DEBUG 
#endif

#include "debug.h"

#include "BLESerial.h"

BLESerial* BLESerial::_instance = NULL;

BLESerial::BLESerial(unsigned char req, unsigned char rdy, unsigned char rst) :
  BLEPeripheral(req, rdy, rst), _onReceive([this](const uint8_t* /* data */, size_t /* size */) {} )
{
  BLESerial::_instance = this;

  addAttribute(this->_uartService);
  addAttribute(this->_uartNameDescriptor);
  setAdvertisedServiceUuid(this->_uartService.uuid());
  addAttribute(this->_rxCharacteristic);
  addAttribute(this->_rxNameDescriptor);
  this->_rxCharacteristic.setEventHandler(BLEWritten, BLESerial::_received);
  addAttribute(this->_txCharacteristic);
  addAttribute(this->_txNameDescriptor);


}

void BLESerial::begin(...) {
  //setDeviceName("NUDGEband");
  //setLocalName("NUDGEband");
  setAdvertisingInterval(1000);

  BLEPeripheral::begin();
  #ifdef BLE_SERIAL_DEBUG
    DBUGLN(F("BLESerial::begin()"));
  #endif

  // nRF51822: -40, -30, -20, -16, -12, -8, -4, 0, 4
  setTxPower(0);
}

void BLESerial::poll() {
  BLEPeripheral::poll();
}

void BLESerial::end() {
  this->_rxCharacteristic.setEventHandler(BLEWritten, NULL);
  BLEPeripheral::disconnect();
  BLEPeripheral::end();
}

void BLESerial::write(const uint8_t* data, size_t size) {
  this->_txCharacteristic.setValue(data, size);
  BLEPeripheral::poll();
  #ifdef BLE_SERIAL_DEBUG
    DBUGLN(F("BLESerial::write()"));
  #endif
}

BLESerial::operator bool() {
  bool retval = BLEPeripheral::connected();
  #ifdef BLE_SERIAL_DEBUG
    DBUG(F("BLESerial::operator bool() = "));
    DBUGLN(retval);
  #endif
  return retval;
}

void BLESerial::_received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic) {
  BLESerial::_instance->_onReceive(rxCharacteristic.value(), rxCharacteristic.valueLength());
}

#endif