#ifndef _BLE_SERIAL_H_
#define _BLE_SERIAL_H_

#include <Arduino.h>

#undef min
#undef max
#include <functional>

typedef std::function<void(const uint8_t* data, size_t size)> ReceiveHandlerFunction;

#ifndef BLE_ATTRIBUTE_MAX_VALUE_LENGTH
#define BLE_ATTRIBUTE_MAX_VALUE_LENGTH  244// 251-4-3
#endif

#define MAX_MSG_LEN (BLE_ATTRIBUTE_MAX_VALUE_LENGTH + 1)

#ifndef ADAFRUIT_NRF52_CORE

#include <BLEPeripheral.h>

class BLESerial : public BLEPeripheral
{
  public:
    BLESerial(unsigned char req = BLE_DEFAULT_REQ, unsigned char rdy = BLE_DEFAULT_RDY, unsigned char rst = BLE_DEFAULT_RST);

    void begin(...);
    void poll();
    void end();
    virtual operator bool();

    void onReceive(ReceiveHandlerFunction receive) {
      _onReceive = receive;
    }

    void write(const uint8_t* data, size_t size);

  private:
    unsigned long _flushed;
    static BLESerial* _instance;
    ReceiveHandlerFunction _onReceive;

    BLEService _uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    BLEDescriptor _uartNameDescriptor = BLEDescriptor("2901", "UART");
    BLECharacteristic _rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
    BLEDescriptor _rxNameDescriptor = BLEDescriptor("2901", "RX - Receive Data (Write)");
    BLECharacteristic _txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
    BLEDescriptor _txNameDescriptor = BLEDescriptor("2901", "TX - Transfer Data (Notify)");

    static void _received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic);
};

#else

#include <bluefruit.h>

//! Band Device - Mode Type
#ifdef BLE_BAND_MATCHING
#define BLE_DUAL_CENTRAL    0x01
#define BLE_DUAL_PERIPHERAL 0x02

#define DOUBLE_TAP_DETECTED     0x01
#define DOUBLE_TAP_NOT_DETECTED 0x00

//! Threshold values for RSSI and Scan Timeout
#ifndef BLE_DUAL_RSSI_THRESHOLD
#define BLE_DUAL_RSSI_THRESHOLD -85
#endif

#define BLE_DUAL_SCAN_TIMEOUT   0   // 0 - Start Scanning Continously , 120 - Start scanning and Stop after 120 seconds(2 minutes)
#endif //BLE_BAND_MATCHING

#ifdef BLE_BAND_MATCHING
//! Connect callback in central mode
typedef std::function<void()> central_connect_callback_t;
//! DisConnect callback in central mode
typedef std::function<void()> central_disconnect_callback_t;
//! Connect callback in Peripheral mode
typedef std::function<void()> Peripheral_connect_callback_t;
//! DisConnect callback in Peripheral mode
typedef std::function<void()> Peripheral_disconnect_callback_t;
//! Recive callback in central mode
typedef std::function<void(const uint8_t *buf, size_t size)> central_Receive_callback_t;
#endif //BLE_BAND_MATCHING

class BLESerial
{
  public:
    BLESerial();

	#ifdef BLE_BAND_MATCHING
    void SetCentralConnectCallback   ( central_connect_callback_t  fp);
    void SetCentralDisConnectCallback ( central_disconnect_callback_t  fp);
    void SetPeripheralConnectCallback   ( Peripheral_connect_callback_t  fp);
    void SetPeripheralDisConnectCallback ( Peripheral_disconnect_callback_t  fp);
    void SetCentralReceiveCallback   ( central_Receive_callback_t  fp);
    void startScanning(void);
    void stopScanning(void);
    void DisconnectPeripheral(void);
    void begin(char *devName);
    void central_write(const uint8_t* data, size_t size);
    void CheckForScannedPacket(void);
    void setRSSIThrehold(int8_t rssiThresholdValue);
	#endif //BLE_BAND_MATCHING

	void begin(...);
	
    void poll();
    void end();
    virtual operator bool();

    void onReceive(ReceiveHandlerFunction receive) {
      _onReceive = receive;
    }

    void write(const uint8_t* data, size_t size);

    void setDeviceName(const char *deviceName);

    void setLocalName(const char *localName) {
      setDeviceName(localName);
    }

    uint16_t getMTU();

  private:
    static BLESerial* _instance;
    uint16_t _connhandle = 255;
    struct manufacturerData_s {
      uint16_t companyID;
      uint8_t doubleTapStatus;
      uint8_t housingType;  // 0-Plastic, 1-Metal
    } __attribute__((packed));


    manufacturerData_s manufacturerData;
    ReceiveHandlerFunction _onReceive;

	#ifdef BLE_BAND_MATCHING
    //! Connect callback in central mode
    central_connect_callback_t    _central_connect_cb;
    //! DisConnect callback in central mode
    central_disconnect_callback_t    _central_disconnect_cb;
    //! Connect callback in Peripheral mode
    Peripheral_connect_callback_t    _Peripheral_connect_cb;
    //! DisConnect callback in Peripheral mode
    Peripheral_disconnect_callback_t    _Peripheral_disconnect_cb;
    //! Recive callback in central mode
    central_Receive_callback_t    _receive_cb;
	#endif //BLE_BAND_MATCHING
	
    // Peripheral uart service
    BLEUart bleuart;
    // BLE Service
    BLEDis  bledis;

    #ifdef BLE_BAND_MATCHING	
    //! Central uart client
    BLEClientUart clientUart;

    uint16_t central_conn_handle;

    #endif //BLE_BAND_MATCHING

    BLEBas  blebas;

    void startAdv(void);

    void onConnect(uint16_t conn_handle);
    void onDisconnect(uint16_t conn_handle, uint8_t reason);

	#ifdef BLE_BAND_MATCHING
    //! Central callback functions 
    void scan_callback(ble_gap_evt_adv_report_t* report);
    void ParseScanParamsAndConnect(ble_gap_evt_adv_report_t* report);
    void cent_connect_callback(uint16_t conn_handle);
    void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason);
    void cent_bleuart_rx_callback(BLEClientUart& cent_uart);
	#endif //BLE_BAND_MATCHING
};

#endif

#endif
