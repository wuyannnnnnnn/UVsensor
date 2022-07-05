
#ifndef ENABLE_DEBUG_BLE_SERIAL
#undef ENABLE_DEBUG
#endif

#ifdef ADAFRUIT_NRF52_CORE

#include "debug.h"
#include "config.h"
#include "BLESerial.h"
#include "FlashManagerTask.h"
#include "Version.h"

#ifdef BLE_BAND_MATCHING
#include "BleManagerTask.h"
#endif //BLE_BAND_MATCHING

BLESerial* BLESerial::_instance = NULL;

//! To Use Custom UUID for Band matching
//! Enable USE_CUSTOM_UUID in plartform.ini file for this custom UUID feature
#ifdef USE_CUSTOM_UUID
//! Custom UUID for Band Matching Profile
// 9B0F556B-1250-4907-8477-AACB566DE158
const uint8_t BAND_MATCHING_UUID[] =
{
    0x58, 0xE1, 0x6D, 0x56, 0xCB, 0xAA, 0x77, 0x84,
    0x07, 0x49, 0x50, 0x12, 0x6B, 0x55, 0x0F, 0x9B
};

BLEUuid bandMatchUuid = BLEUuid(BAND_MATCHING_UUID);
#endif //USE_CUSTOM_UUID

BLESerial::BLESerial() :
  _onReceive([this](const uint8_t* /* data */, size_t /* size */) {}),
  #ifdef BLE_BAND_MATCHING
  _central_connect_cb([this]() {}),
  _central_disconnect_cb([this]() {}),
  _Peripheral_connect_cb([this]() {}),
  _Peripheral_disconnect_cb([this]() {}),
  _receive_cb([this](const uint8_t* /* data */, size_t /* size */) {}),
  #endif
  bleuart(), 
  bledis(),
  #ifdef BLE_BAND_MATCHING
  clientUart(), 
  #endif
  blebas()
{
  BLESerial::_instance = this;
  #ifdef BLE_BAND_MATCHING
  //! Cleaar Connection Handle for Central(mode) device
  central_conn_handle = 255;
  #endif //BLE_BAND_MATCHING
}

void BLESerial::begin(...) 
{
  // Setup company identifier for correct adv packets
  manufacturerData.companyID = DNANUDGE_BLE_COMPANY_ID;

  // Disable BLE from managing the LED
  Bluefruit.autoConnLed(false);

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
  Bluefruit.setConnInterval(6,12);  // 1.25ms unit. 1.25*12 = 15ms minimum allowed after iOS10

  #ifdef BLE_BAND_MATCHING
  //! Initialize Bluefruit with max concurrent connections as Peripheral = 1, Central = 1
  //! SRAM usage required by SoftDevice will increase with number of connections
  // Bluefruit.configPrphConn(185, 6, 3, 1); // 185 is the max MTU used after iOS10
  Bluefruit.begin(1, 1);
  #else
  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.begin();
  #endif //BLE_BAND_MATCHING

  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  //Bluefruit.setName("NudgeBand2");
  
  // Callbacks for Peripheral
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback([](uint16_t conn_handle) { _instance->onConnect(conn_handle); });
  Bluefruit.setDisconnectCallback([](uint16_t conn_handle, uint8_t reason) { _instance->onDisconnect(conn_handle, reason); });
  bleuart.setRxCallback([]() { _instance->poll(); });

  #ifdef BLE_BAND_MATCHING
  //! Callbacks for Central Device
  Bluefruit.Central.setConnectCallback([](uint16_t conn_handle) { _instance->cent_connect_callback(conn_handle); });
  Bluefruit.Central.setDisconnectCallback([](uint16_t conn_handle, uint8_t reason) { _instance->cent_disconnect_callback(conn_handle, reason); });
  #endif //BLE_BAND_MATCHING

  // Configure and Start Device Information Service
  bledis.setManufacturer("DNAnudge");
  bledis.setModel(MODEL_NUMBER);
  char version[10]; 
  sprintf(version, "%d.%d.%d(%d)", FW_MAJOR_VERSION, FW_MINOR_VERSION, FW_SUB_VERSION, FW_BUILD_VERSION);
  bledis.setSoftwareRev(version);
  bledis.setHardwareRev("V4.1");
  //bledis.setModel("NUDGEband");
  bledis.begin();

  // Configure and Start BLE Peripheral Uart Service
  bleuart.begin();

  #ifdef BLE_BAND_MATCHING

  //! Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback([](BLEClientUart& cent_uart) { _instance->cent_bleuart_rx_callback(cent_uart); });

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback([](ble_gap_evt_adv_report_t* report) { _instance->scan_callback(report); });
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(80, 40); // in unit of 0.625 ms

  #ifdef USE_CUSTOM_UUID
  //! Filter only Custom UUID generated for Band Matching
  Bluefruit.Scanner.filterUuid(bandMatchUuid);
  #ifdef ENABLE_RSSI_FILTER
  Bluefruit.Scanner.filterRssi(BLE_DUAL_RSSI_THRESHOLD);
  #endif //ENABLE_RSSI_FILTER
  #else
  //! Filter only BLE UART Servie UUID
  //Bluefruit.Scanner.filterUuid(bleuart.uuid);
  Bluefruit.Scanner.filterService(clientUart);
  #ifdef ENABLE_RSSI_FILTER
  Bluefruit.Scanner.filterRssi(BLE_DUAL_RSSI_THRESHOLD);
  #endif //ENABLE_RSSI_FILTER
  #endif //USE_CUSTOM_UUID

  Bluefruit.Scanner.useActiveScan(true);
  //Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning else stop scanning after n seconds
  #endif //BLE_BAND_MATCHING

  // Start BLE Battery Service
  // blebas.begin();
  // blebas.write(100);

  // Set up and start advertising
  //startAdv();
}

void BLESerial::startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  #ifdef USE_CUSTOM_UUID
  // Include custom 128-bit uuid
  Bluefruit.Advertising.addUuid(bandMatchUuid);
  #else
  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);
  #endif //USE_CUSTOM_UUID·
  // Bluefruit.Advertising.addName();
  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  Bluefruit.ScanResponse.addService(bledis);
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(60);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void BLESerial::end()
{

}

uint16_t BLESerial::getMTU()
{
  return Bluefruit.Gap.getMTU(_connhandle);;
}

void BLESerial::onConnect(uint16_t conn_handle)
{
  if (255 == central_conn_handle) {
    char central_name[32] = { 0 };
    Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

    DBUG("Peripheral Connected to ");
    DBUGLN(central_name);

    _connhandle = conn_handle;
    #ifdef BLE_BAND_MATCHING
    //! Connect the callback to BLE Manager
      _Peripheral_connect_cb();
    #endif //BLE_BAND_MATCHING
  }
  else {
    Bluefruit.disconnect();
  }
}

void BLESerial::onDisconnect(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  DBUGLN("Peripheral Disconnected");
  _connhandle = 255;

  #ifdef BLE_BAND_MATCHING
  //! Connect the callback to BLE Manager
  _Peripheral_disconnect_cb();
  #endif //BLE_BAND_MATCHING
}

#ifdef BLE_BAND_MATCHING
//! Set the RSSI Threshold
void BLESerial::setRSSIThrehold(int8_t rssiThresholdValue)
{
  DBUG("RSSI Threshold : ");
  DBUGLN(rssiThresholdValue);

  #ifdef ENABLE_RSSI_FILTER
  Bluefruit.Scanner.filterRssi(rssiThresholdValue);
  #endif //ENABLE_RSSI_FILTER
}
#endif //BLE_BAND_MATCHING

//! Check for Data in Peripheral Mode
void BLESerial::poll() {
  uint8_t buffer[MAX_MSG_LEN];
  int size = 0;
  int i = 0;  

  // if(bleuart.available())
  // {
    size = bleuart.read(buffer, sizeof(buffer));

    DBUG("RX Size = ");
    DBUGLN(size);

    DBUG("Peri Recvd :");
    for( i = 0; i < size; i++)
    {
      DBUG(buffer[i]);
      DBUG(" ");
    }
    DBUGLN();

    _onReceive(buffer, size);
  // }
}

//! Write Data in Peripheral Mode
void BLESerial::write(const uint8_t* data, size_t size) 
{
  int i = 0;
  DBUG("Peri Sent :");
  for( i = 0; i < size; i++)
  {
    DBUG(data[i]);
    DBUG(" ");
  }
  DBUGLN();
  bleuart.write(data, size);
}

BLESerial::operator bool() {
  return Bluefruit.connected();
}


void BLESerial::setDeviceName(const char *deviceName)
{
  char tempDeviceName[DEVICE_NAME_SIZE+1];
  memset(tempDeviceName, 0, DEVICE_NAME_SIZE+1);

  //! Can Set Device name upto Maximum Length
  //! If it is greater than Max then trim it to Max
  strncpy(tempDeviceName, deviceName, DEVICE_NAME_SIZE);
  Bluefruit.setName(tempDeviceName);

  // Default value of BLE BAND MATCHING. DO NOT MOVE THIS TO startAdv()
  // which is called at other places.
  #ifdef BLE_BAND_MATCHING
  Bluefruit.Advertising.stop(); 
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  uint8_t doubleTapDetectState = DOUBLE_TAP_NOT_DETECTED;
  manufacturerData.doubleTapStatus = doubleTapDetectState;
  manufacturerData.housingType = HOUSING_TYPE;
  //! Update Double Tap Detected State as Manufacturing Data
  Bluefruit.Advertising.addManufacturerData(&manufacturerData, sizeof(manufacturerData));

  #endif //BLE_BAND_MATCHING

  startAdv();
}

#ifdef BLE_BAND_MATCHING

/*------------------------------------------------------------------*/
/* Central
 *------------------------------------------------------------------*/

//! Write Data in Central Mode
void BLESerial::central_write(const uint8_t* data, size_t size) 
{
  int i = 0;
  DBUG("Central Sent :");
  for( i = 0; i < size; i++)
  {
    DBUG(data[i]);
    DBUG(" ");
  }
  DBUGLN();
  clientUart.write(data, size);
}

//! Used to Set Recive callback from BLE Manager
void BLESerial::SetCentralReceiveCallback(central_Receive_callback_t  fp)
{
  _receive_cb = fp;
}

//! Used to Set Connect callback from BLE Manager
void BLESerial::SetCentralConnectCallback ( central_connect_callback_t  fp)
{
    _central_connect_cb = fp;
}

//! Used to Set DisConnect callback from BLE Manager
void BLESerial::SetCentralDisConnectCallback ( central_disconnect_callback_t  fp)
{
    _central_disconnect_cb = fp;
}

//! Used to Set Connect callback from BLE Manager
void BLESerial::SetPeripheralConnectCallback ( Peripheral_connect_callback_t  fp)
{
    _Peripheral_connect_cb = fp;
}

//! Used to Set DisConnect callback from BLE Manager
void BLESerial::SetPeripheralDisConnectCallback ( Peripheral_disconnect_callback_t  fp)
{
    _Peripheral_disconnect_cb = fp;
}

//! Start Central Scanning
void BLESerial::startScanning(void)
{
  Bluefruit.Scanner.start(BLE_DUAL_SCAN_TIMEOUT);    // (n) = Start scanning and Stop after n seconds

  //! Stop Adv
  Bluefruit.Advertising.stop(); 
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();
  uint8_t doubleTapDetectState = DOUBLE_TAP_DETECTED;
  manufacturerData.doubleTapStatus = doubleTapDetectState;
  manufacturerData.housingType = HOUSING_TYPE;
  //! Update Double Tap Detected State as Manufacturing Data
  Bluefruit.Advertising.addManufacturerData(&manufacturerData, sizeof(manufacturerData));
  
  //! Restart with new params
  startAdv();               
}

//! Stop Central Scanning
void BLESerial::stopScanning(void)
{
  Bluefruit.Scanner.stop();  

  //! Stop Adv
  Bluefruit.Advertising.stop(); 
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();
  uint8_t doubleTapDetectState = DOUBLE_TAP_NOT_DETECTED;
  manufacturerData.doubleTapStatus = doubleTapDetectState;
  manufacturerData.housingType = HOUSING_TYPE;
  //! Update Double Tap Detected State as Manufacturing Data
  Bluefruit.Advertising.addManufacturerData(&manufacturerData, sizeof(manufacturerData));

  //! Restart with new params
  startAdv();
}

//! Disconnect Peripheral from Central
void BLESerial::DisconnectPeripheral(void)
{
  Bluefruit.Central.disconnect(BLESerial::central_conn_handle);
}

//! Scan Call back
void BLESerial::scan_callback(ble_gap_evt_adv_report_t* report)
{
  ParseScanParamsAndConnect(report);
}

//! Parse Scan Paramaters and Connect with other band
void BLESerial::ParseScanParamsAndConnect(ble_gap_evt_adv_report_t* report)
{
  uint8_t rxManufactureData[3];
  uint8_t doubleTapDetect = DOUBLE_TAP_NOT_DETECTED;
  uint8_t housingType;
  DBUG("Advertising Report .rssi Value ");
  DBUGLN(report->rssi);

  #ifdef USE_CUSTOM_UUID
  // Check if advertising contain Custom Bandmatching UUID
  if ( Bluefruit.Scanner.checkReportForUuid(report, bandMatchUuid) )
  #else
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  #endif //USE_CUSTOM_UUID
  {
    //! Check for the Valid RSSI
    if (0 != report->rssi)
    {
      //! Device should be in Bandmatching State
      if( BM_STATE_WAIT_FOR_CONNECTION == BleManagerTask::getBandmatchingState() )
      {
        //! Check for Double Tap Detection Other Band
        Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, rxManufactureData, 4);
        doubleTapDetect = rxManufactureData[2];
        housingType = rxManufactureData[3];

        // doubleTapDetect = rxManufactureData[2];
        //! Allow Connection only if both bands double tapped
        if(DOUBLE_TAP_DETECTED == doubleTapDetect)
        {
          DBUGLN("Both BLE Bands are Double Tapped");
          switch (HOUSING_TYPE) {
            case 0x0: // Plastic Housing
              // No special treatment
              DBUGLN("Near to Each Other. Connecting ...");
              // Connect to device with bleuart service in advertising
              Bluefruit.Central.connect(report);
            break;
            case 0x1: // Metal Housing
              if ( (0x0 == housingType && report->rssi >= BLE_DUAL_RSSI_THRESHOLD + 30) || (0x1 == housingType) ) {
                // Metal band found plastic band and quite near
                DBUGLN("Near to Each Other. Connecting ...");
                Bluefruit.Central.connect(report);
              }
            break;
          }          
        }
      }
    }
    else
    {
      DBUGLN("Invalid Rssi Value 0.");
    }
  }
}

//! Connection callback in Central Mode
void BLESerial::cent_connect_callback(uint16_t conn_handle)
{
  if ( 255 == _connhandle ) {
    char peer_name[32] = { 0 };

    //! Store Central Connection Hanlde
    central_conn_handle = conn_handle;

    Bluefruit.Gap.getPeerName(conn_handle, peer_name, sizeof(peer_name));

    DBUG("Central Connected to ");
    DBUGLN(peer_name);;

    if ( clientUart.discover(conn_handle) )
    {
      DBUGLN("Request MTU to be 247");
      sd_ble_gattc_exchange_mtu_request(conn_handle, 247);

      // Enable TXD's notify
      clientUart.enableTXD();

      //! Connect the callback to BLE Manager
      _central_connect_cb();
    }else
    {
      // Disconnect since we couldn't find bleuart service
      DBUGLN("Disconnect since we couldn't find bleuart service");
      Bluefruit.Central.disconnect(BLESerial::central_conn_handle);
    }
  }
  else {
    Bluefruit.Central.disconnect(conn_handle);
  }
}

//! Dis Connect callback in Central Mode
void BLESerial::cent_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  //! Clear Central Connection Hanlde
  central_conn_handle = 255;
  
  DBUGLN("Central Disconnected");

  //! Call the DisConnect callback from BLE Manager
    _central_disconnect_cb();
}


//! Receive callback in Central Mode
void BLESerial::cent_bleuart_rx_callback(BLEClientUart& clientUart)
{
  int i = 0;
  size_t size = 0;
  uint8_t buffer[MAX_MSG_LEN] = {0};

  if(clientUart.available())
  {
    size = clientUart.read(buffer, sizeof(buffer));

    DBUG("RX Size = ");
    DBUGLN(size);

    DBUG("Central Recvd :");
    for( i = 0; i < size; i++)
    {
      DBUG(buffer[i]);
      DBUG(" ");
    }
    DBUGLN();

    //! Call Central Receive Callback function
    _receive_cb(buffer, size);
  }
}

#endif //BLE_BAND_MATCHING
#endif
