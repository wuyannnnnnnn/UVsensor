#ifndef ENABLE_DEBUG_BLE
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>
#include <ArduinoLowPower.h>
#include <nrf_nvic.h>

#include "BleManagerTask.h"
#include "debug.h"
#include "config.h"
#include "Version.h"

#define DEFAULT_MTU_SIZE    23

#define XSTR(x) #x
#define STR(x) XSTR(x)

#ifdef BLE_BAND_MATCHING
//! To Maintain Band matching state
TEBandMatchingState BleManagerTask::eBandMatchingState = BM_STATE_READY;
TSBandMatchFlashTable BleManagerTask::strBandMatchFlashTable;
#endif //BLE_BAND_MATCHING

#ifdef BLE_BAND_MATCHING
BleManagerTask::BleManagerTask(DataStore *dataStore, LedManagerTask *led,
                               ScannerTask *scanner, MpuManagerTask *mpu,
                               BarcodeManagerTask *barcodeManager,
                               DeviceManager *deviceManager,
                               TimerHandle_t *sleepAlarm,
                               BatteryMonitor *batteryMonitor,
                               FlashManagerTask *flashManager,
                               LifeStyle *lifeStyle)
    : bleSerial(), dataStore(dataStore), led(led),
      scanner(scanner), mpu(mpu), barcodeManager(barcodeManager),
      deviceManager(deviceManager), sleepAlarm(sleepAlarm), batteryMonitor(batteryMonitor), 
      flashManager(flashManager), lifeStyle(lifeStyle), preCount(0), preAllergyCount(0), bPeripheralConnected(false)
{
  //! Initialize the device name and flags
  memset(DeviceName, 0, DEVICE_NAME_SIZE + 1);
  memset(dnaMatchScore, 0, BAND_DNA_TRAITS_MAX_GROUP_COUNT);

  resetSystem = false;

  //! Initialize the MTU size
  mtu_size = 0;

  //! Initialize the timers and counters
  bandMatchStartTime = millis();
  bandMatchDNADataStartTime = millis();
  bandMatchDNADataRetry = 0;
  bandMatchLedAckStartTime = millis();
  bandMatchLedAckRetry = 0;

  //! clear the Bandmatch History Flash Table
  memset(&strBandMatchFlashTable, 0, sizeof(TSBandMatchFlashTable));

}
#else
BleManagerTask::BleManagerTask(DataStore *dataStore, LedManagerTask *led,
                               ScannerTask *scanner, MpuManagerTask *mpu,
                               BarcodeManagerTask *barcodeManager,
                               DeviceManager *deviceManager,
                               TimerHandle_t *sleepAlarm,
                               BatteryMonitor *batteryMonitor)
    : bleSerial(), dataStore(dataStore), led(led),
      scanner(scanner), mpu(mpu), barcodeManager(barcodeManager),
      sleepAlarm(sleepAlarm), deviceManager(deviceManager),
      batteryMonitor(batteryMonitor), preCount(0), preAllergyCount(0), 
      enable(true), bPeripheralConnected(false)
{
  //! Initialize the device name and flags
  memset(DeviceName, 0, DEVICE_NAME_SIZE + 1);
  resetSystem = false;
  batteryMonitor->bleManager = this;
}
#endif //BLE_BAND_MATCHING

BleManagerTask::~BleManagerTask()
{
}

void BleManagerTask::setup()
{
  allergySlotIndex = deviceManager->getAllergySlotIndex();
  barcodeManager->updateAllergySlotIndex();

  scanner->setScanningCallback([this]() { 
    if (BleManagerTask::eBandMatchingState != BM_STATE_READY) {
      DBUGLN("STOP BLE scanning...");
      DBUGLN(BleManagerTask::eBandMatchingState);
      BleManagerTask::eBandMatchingState = BM_STATE_READY;
      bleSerial.stopScanning();
    }
   });
  //! Receive
  bleSerial.onReceive(
      [this](const uint8_t *data, size_t size) { onReceive(data, size); });

#ifdef BLE_BAND_MATCHING
  //! Enable Central Connect Callback
  bleSerial.SetCentralConnectCallback([this]() { BleCentral_connect_callback(); });

  //! Enable Central DisConnect Callback
  bleSerial.SetCentralDisConnectCallback([this]() { BleCentral_disconnect_callback(); });

  //! Enable Peripheral Connect Callback
  bleSerial.SetPeripheralConnectCallback([this]() { BlePeripheral_connect_callback(); });

  //! Enable Peripheral DisConnect Callback
  bleSerial.SetPeripheralDisConnectCallback([this]() { BlePeripheral_disconnect_callback(); });

  //! Enable Central Receive Callback
  bleSerial.SetCentralReceiveCallback([this](const uint8_t *data, size_t size) { BleCentral_Receive_callback(data, size); });

  //! Enable Double Tap Detection Callback
  mpu->SetDoubleTapDetectedCallback([this]() { DoubleTapDetectedCallback(); });
#endif //BLE_BAND_MATCHING

  //! Initialize Central and Peripheral Uart Serial Mode
  bleSerial.begin();

#ifdef BLE_BAND_MATCHING
  //! Read and Restore BandMatching History and DNA Data from Flash
  RestoreBandMatchingHistoryAndDNADataFromFlash();

  //! Set Sleeping Timeout
  if((MINIMUM_AUTO_SLEEP_TIME > strBandMatchFlashTable.autoSleepTime) || (MAXIMUM_AUTO_SLEEP_TIME < strBandMatchFlashTable.autoSleepTime) ) //! Sleep timeout not set in Flash or Out of Range  
  {
    //! Update to default sleep Time
    strBandMatchFlashTable.autoSleepTime = DEFAULT_AUTO_SLEEP_TIME;
  }
  setAutoSleepTimemout(strBandMatchFlashTable.autoSleepTime);
  
  //! Set the RSSI Threshold
  if((0 == strBandMatchFlashTable.rssiThreshold) || (-1 == strBandMatchFlashTable.rssiThreshold))
  {
    //! Update to default RSSI Threshold
    strBandMatchFlashTable.rssiThreshold = BLE_DUAL_RSSI_THRESHOLD;
  }
  bleSerial.setRSSIThrehold(strBandMatchFlashTable.rssiThreshold);
#endif //BLE_BAND_MATCHING

  //! Set Band Device Name
  setDeviceName();
}

#ifdef BLE_BAND_MATCHING

uint8_t BleManagerTask::setAutoSleepTimemout(uint8_t autoSleepTimeout)
{
  uint32_t sleepTimeout = 0;
  uint8_t status = STATUS_FAILURE;

  DBUG("Sleep Timeout(mins) : ");
  DBUGLN(autoSleepTimeout);

  if( (MINIMUM_AUTO_SLEEP_TIME <= autoSleepTimeout) &&
              (MAXIMUM_AUTO_SLEEP_TIME >= autoSleepTimeout) )
  {
    sleepTimeout = (autoSleepTimeout * 60 * 1000);
    // sleepTimeout = 24*60*60*1000;
    //! Set Sleep Timeout in milli seconds
    if ( xTimerIsTimerActive( *sleepAlarm ) != pdFALSE ) {
      xTimerStop(*sleepAlarm, 0);
    }
    #ifndef DISABLE_DEVICE_SLEEP
    xTimerChangePeriod(*sleepAlarm, sleepTimeout * configTICK_RATE_HZ / 1000, 0);
    #endif

    if (true == batteryMonitor->getChargerStatus()) {
      // Stop the timer if it is on charger.
      // Do not check timer status here as 
      // it will still show stop since the timerQueue hasn't executed the change period
      xTimerStop(*sleepAlarm, 0);
    }
    status = STATUS_SUCCESS;
  }
  else
  {
    DBUG("Sleep Timeout Out Of Range");
  }

  return status;
}

//! Check For Bandmatching command timeouts
void BleManagerTask::CheckForBandmatchingCommandTimeout(void)
{
  //! Check the Bandmatching state
  switch(eBandMatchingState)
  {
    //! If system is in bandmatching mode then wait for 30 seconds(configurable)
    //! after the timeout move back to system ready state
    case BM_STATE_WAIT_FOR_CONNECTION:
    {
      //! System in Bandmatching state
      //! Waiting for another band to connect with it

      //! Check for time difference from the start time
      //! If the time difference reached the maximum timeout then exit bandmatch mode 
      //! To avoid timer overflow error
      if(millis() < bandMatchStartTime)
      {
        bandMatchStartTime = millis();
      }
      if(MAX_BAND_MATCHING_TIMEOUT <= (millis() - bandMatchStartTime) )
      {
        //! Timeout - Stop the Band matching mode
        DBUGLN("Exit Bandmatching Mode");

        //! Stop Central Scanning
        bleSerial.stopScanning();
        //! Move back to Ready State
        BleManagerTask::eBandMatchingState = BM_STATE_READY;
        led->setBM_Exited();
      }
    }
    break;
    case BM_STATE_CENTRAL_WAIT_FOR_DNA_DATA:
    {
      //! Waiting for DNA Data from the pheripheral Band
      //! Check for Timeout if timeout occurs resend the GetDNAData command Upto maximum Retries
      //! If the maximum retries reached disconnect the band, move to Wait For Connection State
      //! and rescan for another band
      if(millis() < bandMatchDNADataStartTime)
      {
        bandMatchDNADataStartTime = millis();
      }
      if(MAX_BAND_MATCH_DNA_DATA_TIMEOUT <= (millis() - bandMatchDNADataStartTime) )
      {
        //! Timeout - Retry GetDNAData Command
        bandMatchDNADataRetry++;
        if(MAX_BAND_MATCH_DNA_DATA_RETRY > bandMatchDNADataRetry)
        {
          DBUGLN("Get DNA Data Command Timeout.Retry");
          bandMatchDNADataStartTime = millis();
          //! Send Get DNA Data command to Peripheral band
          SendDNADataRequestToPeripheralDevice();
        }
        else
        {
          //! Reached Maximum Retries
          //! Disconnect and wait for any other band. Re-Scanning
          DBUGLN("Reached Maximum Retries. Disconnect and wait for any other band. Re-Scanning"); 
          //! Disconnect the Peripheral
          bleSerial.DisconnectPeripheral();
          //! Move to Wait For Connetion State
          BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
          //! Restore the Start Time
          bandMatchStartTime = millis();
          led->setBM_Entered();
        }
      }
    }
    break;
    case BM_STATE_CENTRAL_WAIT_FOR_UPDATE_LED_ACK:
    {
      //! Waiting for LED Ack from the pheripheral Band
      //! Check for Timeout if timeout occurs resend the Update LED Status command Upto maximum Retries
      //! If the maximum retries reached disconnect the band, move to Wait For Connection State
      //! and rescan for another band
      if(millis() < bandMatchLedAckStartTime)
      {
        bandMatchLedAckStartTime = millis();
      }
      if(MAX_BAND_MATCH_LED_ACK_TIMEOUT <= (millis() - bandMatchLedAckStartTime) )
      {
        //! Timeout - Retry Update LED Status Command
        bandMatchLedAckRetry++;
        if(MAX_BAND_MATCH_LED_ACK_RETRY > bandMatchLedAckRetry)
        {
          DBUGLN("Update LED Status Command Timeout.Retry");
          bandMatchLedAckStartTime = millis();
          //! Send Update LED Status command to Peripheral Device
          SendUpdateLEDStatusRequestToPeripheralDevice();
        }
        else
        {
          //! Reached Maximum Retries
          //! Disconnect and wait for any other band. Re-Scanning
          DBUGLN("Reached Maximum Retries. Disconnect and wait for any other band. Re-Scanning"); 
          //! Disconnect the Peripheral
          bleSerial.DisconnectPeripheral();
          //! Move to Wait For Connetion State
          BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
          //! Restore the Start Time
          bandMatchStartTime = millis();
          led->setBM_Entered();
        }
      }
    }
    break;
    default:
      //! Nothing to do
    break;
  }
}

//! Read and Restore BandMatching History and DNA Data from Flash
void BleManagerTask::RestoreBandMatchingHistoryAndDNADataFromFlash(void)
{
  //DBUGVAR(sizeof(TSBandMatchFlashTable));
  //strBandMatchFlashTable.dnaDataSetInFlash = BAND_DNA_DATA_NOT_SET_IN_FLASH;
  //memset(strBandMatchFlashTable.bandDnaData, 0, BAND_DNA_DATA_SIZE);
  //ClearBandmatchingHistory();

  //! Get Bandmatch History and DNA Data from Flash
  if (true == GetBandMatchingHistoryandDNADataFromFlash(&strBandMatchFlashTable))
  {
    //! Check DNA Data from Flash is valid or not
    if (BAND_DNA_DATA_SET_IN_FLASH == strBandMatchFlashTable.dnaDataSetInFlash)
    {
      //! Allow Bandmatching
      DBUGLN("DNA Data Set. Bandmatching Allowed");

      //! Check whether Bandmatching History is Valid or Not
      if (MAX_BANDMATCHING_RECORDS >= strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards)
      {
        DBUGLN("Bandmatch History from Flash is Valid");
        DBUG("Bandmatch History Count : ");
        DBUGLN(strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards);
      }
      else
      {
        DBUGLN("Bandmatch History from Flash is invalid");
        //! Clear the BM history datas and rewrite into flash
        ClearBandmatchingHistory();
      }
    }
    else
    {
      //! DNA Data not written into Flash
      //! Do not Allow Bandmatching
      DBUGLN("DNA Data Not Set. Bandmatching Not Allowed");

      //! Clear the Band Match Flash Table
      memset(&strBandMatchFlashTable, 0, sizeof(TSBandMatchFlashTable));
      strBandMatchFlashTable.dnaDataSetInFlash = BAND_DNA_DATA_NOT_SET_IN_FLASH;
      strBandMatchFlashTable.autoSleepTime = DEFAULT_AUTO_SLEEP_TIME;
      strBandMatchFlashTable.rssiThreshold = BLE_DUAL_RSSI_THRESHOLD;
    }
  }
  else
  {
    DBUGLN("Bandmatch History and DNA Data from Flash Failed");
    DBUGLN("Bandmatching Not Allowed");
    //! Clear the Band Match Flash Table
    memset(&strBandMatchFlashTable, 0, sizeof(TSBandMatchFlashTable));
    strBandMatchFlashTable.dnaDataSetInFlash = BAND_DNA_DATA_NOT_SET_IN_FLASH;
    strBandMatchFlashTable.autoSleepTime = DEFAULT_AUTO_SLEEP_TIME;
    strBandMatchFlashTable.rssiThreshold = BLE_DUAL_RSSI_THRESHOLD;
  }

  //ClearBandmatchingHistory();

#if 0
  //! Write Sample Data As of Now
  strBandMatchFlashTable.dnaDataSetInFlash = BAND_DNA_DATA_SET_IN_FLASH;
  strBandMatchFlashTable.bandDnaData[0] = 0x55;
  strBandMatchFlashTable.bandDnaData[1] = 0xAA;
  strBandMatchFlashTable.bandDnaData[2] = 0x55;
  strBandMatchFlashTable.bandDnaData[3] = 0xAA;
  strBandMatchFlashTable.bandDnaData[4] = 0x55;
  strBandMatchFlashTable.bandDnaData[5] = 0xAA;
  strBandMatchFlashTable.bandDnaData[6] = 0x55;
  strBandMatchFlashTable.bandDnaData[7] = 0xAA;
#endif //0
}

//! Get Bandmatching State
TEBandMatchingState BleManagerTask::getBandmatchingState(void)
{
  return BleManagerTask::eBandMatchingState;
}

//! Get Bandmatch History Flash Table
TSBandMatchFlashTable *BleManagerTask::getBandmatchHistoryDataFlashTable(void)
{
   return &strBandMatchFlashTable;
}

void BleManagerTask::StopBandMatching() {
  DBUGLN("Stop Bandmatching Mode");
  //! Move back to Ready State
  BleManagerTask::eBandMatchingState = BM_STATE_READY;
  led->setBM_Exited();
}

//! This callback Will be called once Double Tap Detected by MPU Task
void BleManagerTask::DoubleTapDetectedCallback(void)
{
  if (led->isFamilyScanEnabled()) {
    led->setBM_Exited();
    DBUGLN("Disable NudgeMatch and Family scanning");
    bleSerial.stopScanning();
    DBUGLN("Stop Central Scanning");
    BleManagerTask::eBandMatchingState = BM_STATE_READY;
    DBUGLN("Disconnect Peripheral");
    //! Disconnect Peripheral
    bleSerial.DisconnectPeripheral();
  }
  else {
    //! Based on the Bandmatching state take action
    switch (BleManagerTask::eBandMatchingState)
    {
    case BM_STATE_READY:
    {
      //! If the device is not connected with any other device 
      //! and the DNA Data is set in Flash then allow bandmatching
      //! Else deney it
      if ( (false == bPeripheralConnected) && (BAND_DNA_DATA_SET_IN_FLASH == strBandMatchFlashTable.dnaDataSetInFlash))
      {
        //! Allow Bandmatching
        DBUGLN("Enter Bandmatching Mode. Scanning");

        //! store the Bandmatching Start Time
        bandMatchStartTime = millis();

        //! Start Central Scanning
        bleSerial.startScanning();

        //! Wait for another band to connect
        BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
        led->setBM_Entered();
      }
      else
      {
        //! Band Matching Not Allowed
        if(true == bPeripheralConnected)
        {
          DBUGLN("Device is already connected with other device. Bandmatching not Allowed");
        }
        else
        {
          DBUGLN("DNA Data not Set. Bandmatching not Allowed");
        }
        led->setBM_MatchingNotAllowed();
      }
    }
    break;
    case BM_STATE_WAIT_FOR_CONNECTION:
    {
      //! User want to Stop the Band matching
      DBUGLN("Exit Bandmatching Mode");

      //! Stop Central Scanning
      bleSerial.stopScanning();
      //! Move back to Ready State
      BleManagerTask::eBandMatchingState = BM_STATE_READY;
      led->setBM_Exited();
    }
    break;
    case BM_STATE_CENTRAL_CONNECTED:
    case BM_STATE_CENTRAL_WAIT_FOR_DNA_DATA:
    case BM_STATE_CENTRAL_UPDATE_LED_STATUS:
    case BM_STATE_CENTRAL_WAIT_FOR_UPDATE_LED_ACK:
    {
      DBUGLN("Exit Bandmatching Mode");
      //! Device in Central Mode
      //! Move back to Ready State
      BleManagerTask::eBandMatchingState = BM_STATE_READY;
      led->setBM_Exited();

      DBUGLN("Disconnect Peripheral");
      //! Disconnect Central/Peripheral
      bleSerial.DisconnectPeripheral();
    }
    break;
    default:
    {
      DBUGLN("Exit Bandmatching Mode");
      //! Move back to Ready State
      BleManagerTask::eBandMatchingState = BM_STATE_READY;
      led->setBM_Exited();
    }
    break;
    }
  }
}

//! This callback Will be called if Data received in Central Mode
void BleManagerTask::BleCentral_Receive_callback(const uint8_t *buf, size_t size)
{
  onBandMatchReqRspReceive(buf, size);
}
//! This callback Will be called if another band is disconencted in Central mode
void BleManagerTask::BleCentral_disconnect_callback(void)
{
  //! Based on the Bandmatching state take action
  switch (BleManagerTask::eBandMatchingState)
  {
  case BM_STATE_READY:
  {
    DBUGLN("Central Disconnected from Peripheral: Bandmatch Ready State");
    //! Stop Central Scanning
    bleSerial.stopScanning();
  }
  break;
  case BM_STATE_PERIPHERAL_UPDATE_DNA_DATA:
  case BM_STATE_PERIPHERAL_PROCESS_LED_STATUS:
  {
    DBUGLN("Bandmatching Peripheral Mode : Central Disconnected from other band.");
    //! Stop Central Scanning
    bleSerial.stopScanning();
  }
  break;
  case BM_STATE_PERIPHERAL_CONNECTED:
  {
    DBUGLN("Bandmatching Peripheral Mode : Already Connected. Stop Central Scanning");
    //! Stop Central Scanning
    bleSerial.stopScanning();
  }
  break;
  default:
  {
    DBUGLN("Central Disconnected from Peripheral. Re-Scanning");
    //! Move System State to Wait For Connection State
    BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
    led->setBM_Entered();
  }
  break;
  }
}

//! This callback Will be called if another band connected in Central mode
void BleManagerTask::BleCentral_connect_callback(void)
{
  //! Based on the Bandmatching state take action
  switch (BleManagerTask::eBandMatchingState)
  {
  case BM_STATE_WAIT_FOR_CONNECTION:
  {
    DBUGLN("Central Connected");
    bleSerial.stopScanning();
    //! Move to Central Connected State
    BleManagerTask::eBandMatchingState = BM_STATE_CENTRAL_CONNECTED;
    //! Clear the Old DNA Match Status
    memset(dnaMatchScore, 0, BAND_DNA_TRAITS_MAX_GROUP_COUNT);

    //! Store Get DNA Data Start Time
    bandMatchDNADataStartTime = millis();
    bandMatchDNADataRetry = 0;

    //! Send Get DNA Data command to Peripheral band
    SendDNADataRequestToPeripheralDevice_v2(); // Send new format request first.
    //! Move to Wait for DNA data State
    BleManagerTask::eBandMatchingState = BM_STATE_CENTRAL_WAIT_FOR_DNA_DATA;
  }
  break;
  case BM_STATE_READY:
  {
    DBUGLN("Central Connected : Not in Bandmatching mode : Disconnect other band");
    //! Disconnect Peripheral
    bleSerial.DisconnectPeripheral();
  }
  break;
  case BM_STATE_PERIPHERAL_UPDATE_DNA_DATA:
  case BM_STATE_PERIPHERAL_PROCESS_LED_STATUS:
  {
    DBUGLN("Central Connected : But Device Already Connected as Periphral : Disconnect other band");
    //! Disconnect Peripheral
    bleSerial.DisconnectPeripheral();
  }
  break;
  default:
  {
    //! Invalid State
    DBUGLN("BleCentral_connect_callback():Invalid Band Matching State : Disconnect other band");
    DBUGVAR(BleManagerTask::eBandMatchingState);
    //! Disconnect Peripheral
    bleSerial.DisconnectPeripheral();
    StopBandMatching();
  }
  break;
  }
}

//! This callback Will be called if another band is disconencted in Peripheral mode
void BleManagerTask::BlePeripheral_disconnect_callback(void)
{
  DBUGLN("Peripheral Disconnected from Central");

  //! If it is connected with Central Band Device for Bandmatching
  //! Change the state to Wait for connection state
  switch (BleManagerTask::eBandMatchingState)
  {
    case BM_STATE_PERIPHERAL_CONNECTED:
    {
      DBUGLN("Bandmatching Connected State. Move to Wait For Connection");
      //! Move to Wait for Conenction State
      BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
      //! Restore the Start Time
      bandMatchStartTime = millis();
      led->setBM_Entered();
    }
    break;
    case BM_STATE_PERIPHERAL_UPDATE_DNA_DATA:
    case BM_STATE_PERIPHERAL_PROCESS_LED_STATUS:
    {
      DBUGLN("Bandmatching Connetced State. Move to Wait For Connection");
      //! Move to Wait for Conenction State
      BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
      //! Restore the Start Time
      bandMatchStartTime = millis();
      led->setBM_Entered();

      //! Start Central Scanning
      bleSerial.startScanning();
    }
    break;
    default:
    {
      //! Do Nothing
    }
    break;
  }

  //! Clear the flag as Peripheral Disconnected from other device(mobile App or another Band)
  bPeripheralConnected = false;

  //! Check for Reset Flag and reset the system if true
  if (true == resetSystem)
  {
//! 1 seconds Delay
//! Wait for Uart Write and print Debug Lines
#ifdef ADAFRUIT_NRF52_CORE
    delay(1000);
#else
    delay(1000);
#endif //ADAFRUIT_NRF52_CORE

    DBUGLN("Reboot");
    sd_nvic_SystemReset();
  }
}

//! This callback Will be called if another band connected in Peripheral mode
void BleManagerTask::BlePeripheral_connect_callback(void)
{
  //! Mark as Peripheral Connected with other device(mobile App or another Band)
  bPeripheralConnected = true;

  // reset the readyToReceive flag upon connection to allow capsule 
  // to process incoming messages that write into the flash
  barcodeManager->readyToReceive = true;
  // reset recommendation allergy transfer such that Hash address will
  // be calculated on the new transfer
  barcodeManager->newRecomAllergyBlock = true;
  //! Based on the bandmatching state take action
  switch (BleManagerTask::eBandMatchingState)
  {
  case BM_STATE_WAIT_FOR_CONNECTION:
  {
    DBUGLN("Peripheral Connected");
    bleSerial.stopScanning();
    //! Move to Peripheral Connected State
    BleManagerTask::eBandMatchingState = BM_STATE_PERIPHERAL_CONNECTED;
  }
  break;
  case BM_STATE_READY:
  {
    DBUGLN("Peripheral connected.");
  }
  break;
  default:
  {
    //! Invalid State
    DBUGLN("BlePeripheral_connect_callback():Invalid Band Matching State");
    DBUGVAR(BleManagerTask::eBandMatchingState);
    StopBandMatching();
  }
  break;
  }
}
//! Send Update LED Status Request to Peripheral Device
void BleManagerTask::SendUpdateLEDStatusRequestToPeripheralDevice(void)
{
  band_update_led_status_data cmd;

  cmd.status = dnaMatchScore[0];
  memcpy(cmd.deviceName, DeviceName, DEVICE_NAME_SIZE);

  DBUGLN("Send DNA Matching Status(LED Status) to Pheripheral");
  bleSerial.central_write((uint8_t *)&cmd, sizeof(cmd));
}

void BleManagerTask::SendUpdateLEDStatusRequestToPeripheralDevice_v2()
{
  band_update_led_status_data_v2 cmd(BAND_DNA_MATCH_STATUS_SIZE + DEVICE_NAME_SIZE + traitGroupCount);

  cmd.status = 0;
  memcpy(cmd.deviceName, DeviceName, DEVICE_NAME_SIZE);
  memcpy(cmd.matchScore, dnaMatchScore, sizeof(dnaMatchScore));

  DBUGLN("Send DNA Matching Status(LED Status) to Pheripheral (Extended Protocol)");
  DBUGVAR(sizeof(cmd));
  bleSerial.central_write((uint8_t *)&cmd, BAND_CMD_HDR_SIZE + BAND_CMD_CMDID_SIZE + BAND_DNA_MATCH_STATUS_SIZE + DEVICE_NAME_SIZE + traitGroupCount);
}
//! Send Get DNA Data Request to Peripheral Device
void BleManagerTask::SendDNADataRequestToPeripheralDevice()
{
  band_cmd_base cmd(BandCommand_BandGetDNADataREQ);
  DBUGLN("Send Get DNA Data Request to Peripheral Device");
  DBUGF("cmd length %d,sizeof cmd =%d\r", cmd.length, sizeof(cmd));
  bleSerial.central_write((uint8_t *)&cmd, sizeof(cmd));
}

void BleManagerTask::SendDNADataRequestToPeripheralDevice_v2()
{
  band_cmd_base cmd(BandCommand_BandGetDNAData_V2REQ);
  DBUGLN("Send Get DNA Data Request V2 to Peripheral Device");
  DBUGF("cmd length %d,sizeof cmd =%d\r", cmd.length, sizeof(cmd));
  bleSerial.central_write((uint8_t *)&cmd, sizeof(cmd));
}

//! Send DNA Data Response to Central Device
void BleManagerTask::SendDNADataResponseToCentralDevice(uint8_t cmdID, uint8_t getDNADataStatus)
{
  band_dna_data cmd(cmdID, getDNADataStatus);
  uint8_t tmpData;

  //! Update DNA Data
  if (GET_DNA_DATA_STATUS_SUCCESS == getDNADataStatus)
  {
    for (int idx=0; idx < sizeof(cmd.dnaData); idx++) {
      if (idx < (strBandMatchFlashTable.bandDnaData[1]&0x1F)) {
        tmpData = *(strBandMatchFlashTable.bandDnaData + 2 + idx);
        cmd.dnaData[sizeof(cmd.dnaData) - 1 - idx] = ((tmpData & 0x0F) << 4) | ((tmpData & 0xF0) >> 4);
      }
      else {
        cmd.dnaData[sizeof(cmd.dnaData) - 1 - idx] = 0;
      }
    }
  }

  //! Write data to Central device
  bleSerial.write((uint8_t *)&cmd, sizeof(cmd));
}

void BleManagerTask::SendDNADataResponseToCentralDevice_v2(uint8_t cmdID, uint8_t getDNADataStatus)
{
  band_dna_data_v2 cmd(cmdID, getDNADataStatus, strBandMatchFlashTable.bandDnaDataSize);
  DBUGVAR(strBandMatchFlashTable.bandDnaDataSize);
  //! Update DNA Data
  if (GET_DNA_DATA_STATUS_SUCCESS == getDNADataStatus)
  {
    memcpy(cmd.dnaData, strBandMatchFlashTable.bandDnaData, strBandMatchFlashTable.bandDnaDataSize);
    //! Write data to Central device
    bleSerial.write((uint8_t *)&cmd, sizeof(cmd) - BAND_MATCHING_DNA_DATA_REGION_SIZE + strBandMatchFlashTable.bandDnaDataSize);
  }
  else
  {
    //! Write data to Central device
    bleSerial.write((uint8_t *)&cmd, sizeof(cmd) - BAND_MATCHING_DNA_DATA_REGION_SIZE);
  }
}
//! Handle Band matching Request/Response message
bool BleManagerTask::onBandMatchReqRspReceive(const uint8_t *buf, size_t size)
{
  bool handled = false;
  uint8_t cmdID = 0, len = 0;
  uint8_t historyIndex = 0;
  uint8_t dnaStatusRsp[MAX_MSG_LEN] = {0};
  uint8_t status = STATUS_FAILURE;
  char dnaMatchDeviceName[DEVICE_NAME_SIZE + 1];

  //! Reset the sleep Timer
  if (xTimerIsTimerActive( *sleepAlarm ) != pdFALSE) {
    xTimerReset(*sleepAlarm, 0);
  }

  //! Variables for DNA compare
  uint8_t *srcDNA;
  uint8_t dstDNA[BAND_MATCHING_DNA_DATA_REGION_SIZE];
  uint8_t srcDNATrait = 0, dstDNATrait = 0;
  uint8_t srcDNATraitLevel = 0, dstDNATraitLevel = 0;
  bool bSrcDNATraitEnable = false, bDstDNATraitEnable = false;
  
  uint8_t autoSleepTime = 0;

  memset(dnaMatchDeviceName, 0, DEVICE_NAME_SIZE + 1);

  //! DNA Request and Response
  //! Packet Format
  //! SOF1(0xFE) SOF2(0xFE) Length cmdID Data0 .... DataN
  if ((BAND_CMD_SOF_1_VALUE == buf[BAND_CMD_SOF_1_POS]) &&
      (BAND_CMD_SOF_2_VALUE == buf[BAND_CMD_SOF_2_POS]))
  {
    handled = true;

    len = buf[BAND_CMD_LEN_POS];
    // DBUGF("len:%d, revLen:%d, cmd:0x%x\r", len, (size - BAND_CMD_HDR_SIZE), buf[BAND_CMD_CMD_POS]);
  
    if (len == (size - BAND_CMD_HDR_SIZE))
    {
      cmdID = buf[BAND_CMD_CMD_POS];
      // DBUGF("cmdId:0x%x\r", cmdID);
      // DBUGLN("");
      // DBUGF("size:%d\r", size);
      // DBUGLN("");
      switch (cmdID)
      {
      //! Band to Band commands
      case BandCommand_BandGetDNADataREQ:
      {
        DBUGLN("Band DNA Data Request V1");

        //! Check whether we have received required data bytes or not?
        if(BandCommand_BandGetDNADataREQ_SIZE == size )
        {
          //! If Band Matching Process is not Started
          //! then Send DNA Data Response to Central Device with NACK
          //! else Send DNA Data Response to Central Device with ACK
          if (BM_STATE_READY == BleManagerTask::eBandMatchingState)
          {
            DBUGLN("Not in Bandmatch mode.");
            DBUGLN("Send DNA Data Response to Central with NACK");
            //! Send DNA Data Response to Central Device
            SendDNADataResponseToCentralDevice(BandCommand_BandGetDNADataRSP, GET_DNA_DATA_STATUS_FAILURE);
          }
          else
          {
            //! Already Connected with a central Device for Band Matching
            //! Stop Central Scanning
            bleSerial.stopScanning();

            //! Move to Pheripheral Update DNA Data state
            BleManagerTask::eBandMatchingState = BM_STATE_PERIPHERAL_UPDATE_DNA_DATA;

            //! Clear the Old DNA Match Status
            dnaMatchScore[0] = 0;

            DBUGLN("Send DNA Data Response V1 to Central with ACK");
            //! Send DNA Data Response to Central Device
            SendDNADataResponseToCentralDevice(BandCommand_BandGetDNADataRSP, GET_DNA_DATA_STATUS_SUCCESS);
          }
        }
        else
        {
          DBUGLN("Data length not matched 0");
          //! Skip the packet
          //! 3 Retries available in central mode 
        }
      }
      break;
      case BandCommand_BandGetDNADataRSP:
      {
        DBUGLN("DNA Data Response V1");

        //! Check whether we have received required data bytes or not?
        if(BandCommand_BandGetDNADataRSP_SIZE == size )
        {
          if (BM_STATE_CENTRAL_WAIT_FOR_DNA_DATA == BleManagerTask::eBandMatchingState)
          {

            //! Check Whether Pheripheral is in Band matching mode or not
            //! If the cmd Status is Success, Peripheral is in Band match mode
            //! Else Disconnect the Peripheral Device

            if (GET_DNA_DATA_STATUS_SUCCESS == buf[BAND_CMD_DATA_POS])
            {
              //! Move to Central Update LED Status state
              BleManagerTask::eBandMatchingState = BM_STATE_CENTRAL_UPDATE_LED_STATUS;

              //! Get Own DNA Data and compare with the received DNA data

              //! Store DNA data into two 64 bit unsigned integer variables
              //! To simplify the shift operations
              //! Build Source DNA
              srcDNA = strBandMatchFlashTable.bandDnaData + 2;

              //! Build Destination DNA
              memcpy(dstDNA, (uint8_t *)&buf[BAND_CMD_DATA_POS + BAND_DNA_DATA_STATUS_SIZE], size - BandCommand_BandGetDNADataRSP_HDR_SIZE);

              // DBUGF("scrDNA=0x%x, dstDNA=0x%x\r", srcDNA, dstDNA);
              totalScore = 0;
              activeTraitCount = 0;
              //! Check individual traits match
              uint16_t traitCount = min(strBandMatchFlashTable.bandDnaData[1]&0x1F, ((int)size - BandCommand_BandGetDNADataRSP_HDR_SIZE)*2);

              for(int index = 0; index < traitCount; index++)
              {
                bSrcDNATraitEnable = false, bDstDNATraitEnable = false;

                //! Seperate individual Trait from Source DNA
                srcDNATrait = Get_Next_DNATrait(srcDNA, index);
                srcDNATraitLevel = srcDNATrait & 0x07;
                if(srcDNATrait & 0x08)
                {
                  bSrcDNATraitEnable = true;
                }
                // DBUGVAR(srcDNATrait);

                //! Seperate individual Trait from Destination DNA
                dstDNATrait = Get_Next_DNATrait(dstDNA, ((int)size - BandCommand_BandGetDNADataRSP_HDR_SIZE)*2 - 1 - index);
                dstDNATraitLevel = dstDNATrait & 0x07;
                if(dstDNATrait & 0x08)
                {
                  bDstDNATraitEnable = true;
                }
                // DBUGVAR(dstDNATrait);

                if( (true == bSrcDNATraitEnable) && (true == bDstDNATraitEnable))
                {
                  //! selected mismatch found
                  switch (abs(srcDNATraitLevel - dstDNATraitLevel)) {
                    case 0:
                      totalScore+=1;
                      break;
                    case 1:
                      totalScore+=0.5;
                      break;
                    case 2:
                      totalScore+=0.2;
                      break;
                    case 3:
                      totalScore+=0.05;
                      break;
                    default:
                      break;
                  }
                  activeTraitCount++;
                  DBUGVAR(srcDNATraitLevel);
                  DBUGVAR(dstDNATraitLevel);
                }
              }
              dnaMatchScore[0] = (totalScore / (double)(activeTraitCount)) * 100;
              DBUGVAR(activeTraitCount);
              DBUGVAR(totalScore);
              DBUGVAR(traitCount);
              DBUGVAR(dnaMatchScore[0]);
              if (dnaMatchScore[0] >= 100) {
                DBUGLN("Overall Match Found. Flash Green LED");
                led->setBM_OverallMatched();
              } else if (dnaMatchScore[0] > 70) {
                DBUGLN("DNA Band matching");
                led->setBM_Matched();
              } else if (dnaMatchScore[0] > 30) {
                DBUGLN("DNA Band AMBER matching");
                led->setBM_AmberMatched();
              } else {
                DBUGLN("DNA Band Not matching");
                led->setBM_NotMatched();
              }

              //! Move to Central Wait For Update LED Ack state
              BleManagerTask::eBandMatchingState = BM_STATE_CENTRAL_WAIT_FOR_UPDATE_LED_ACK;

              //! Store Update LED Status Start Time
              bandMatchLedAckStartTime = millis();
              bandMatchLedAckRetry = 0;
              
              //! Send Update LED Status Request to Peripheral Device
              SendUpdateLEDStatusRequestToPeripheralDevice();
            }
            else
            {
              //! Disconnect the Peripheral
              bleSerial.DisconnectPeripheral();

              DBUGLN("Previous Peripheral Not in Bandmatching Mode. Re-Scanning");
              //! Move to Wait For Connetion State
              BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
              led->setBM_Entered();
            }
          }
          else
          {
            DBUGLN("Invalid Band matching State");
          }
        }
        else
        {
          DBUGLN("Data length not matched 1");
          //! Skip the packet
          //! 3 Retries available in central mode 
        }
      }
      break;
      case BandCommand_BandGetDNAData_V2REQ:
      {
        DBUGLN("Band DNA Data Request V2");

        //! Check whether we have received required data bytes or not?
        if(BandCommand_BandGetDNADataREQ_SIZE == size )
        {
          //! If Band Matching Process is not Started
          //! then Send DNA Data Response to Central Device with NACK
          //! else Send DNA Data Response to Central Device with ACK
          if (BM_STATE_READY == BleManagerTask::eBandMatchingState)
          {
            DBUGLN("Not in Bandmatch mode.");
            DBUGLN("Send DNA Data Response V2 to Central with NACK");
            //! Send DNA Data Response to Central Device
            SendDNADataResponseToCentralDevice_v2(BandCommand_BandGetDNAData_V2RSP, GET_DNA_DATA_STATUS_FAILURE);
          }
          else
          {
            //! Already Connected with a central Device for Band Matching
            //! Stop Central Scanning
            bleSerial.stopScanning();

            //! Move to Pheripheral Update DNA Data state
            BleManagerTask::eBandMatchingState = BM_STATE_PERIPHERAL_UPDATE_DNA_DATA;

            //! Clear the Old DNA Match Status
            memset(dnaMatchScore, 0, BAND_DNA_TRAITS_MAX_GROUP_COUNT);

            DBUGLN("Send DNA Data Response V2 to Central with ACK");
            //! Send DNA Data Response to Central Device
            SendDNADataResponseToCentralDevice_v2(BandCommand_BandGetDNAData_V2RSP, GET_DNA_DATA_STATUS_SUCCESS);
          }
        }
        else
        {
          DBUGLN("Data length not matched 0");
          //! Skip the packet
          //! 3 Retries available in central mode 
        }
      }
      break;
      case BandCommand_BandGetDNAData_V2RSP:
      {
        DBUGLN("DNA Data Response V2");

        //! Check whether we have received required data bytes or not?
        if(BandCommand_BandGetDNADataRSP_HDR_SIZE < size )
        {
          if (BM_STATE_CENTRAL_WAIT_FOR_DNA_DATA == BleManagerTask::eBandMatchingState)
          {

            //! Check Whether Pheripheral is in Band matching mode or not
            //! If the cmd Status is Success, Peripheral is in Band match mode
            //! Else Disconnect the Peripheral Device

            if (GET_DNA_DATA_STATUS_SUCCESS == buf[BAND_CMD_DATA_POS])
            {
              //! Move to Central Update LED Status state
              BleManagerTask::eBandMatchingState = BM_STATE_CENTRAL_UPDATE_LED_STATUS;

              //! Get Own DNA Data and compare with the received DNA data

              srcDNA = strBandMatchFlashTable.bandDnaData;

              //! Build Destination DNA
              memcpy(dstDNA, (uint8_t *)&buf[BAND_CMD_DATA_POS+1], size - BandCommand_BandGetDNADataRSP_HDR_SIZE );
                            // DBUGF("scrDNA=0x%x, dstDNA=0x%x\r", srcDNA, dstDNA);
              totalScore = 0;
              //! Check individual traits match
              traitCount = 0;
              traitStartPos = 1;
              activeTraitCount = 0;
              traitGroupCount = min(srcDNA[0], dstDNA[0]);
              DBUGVAR(traitGroupCount);

              for (int grpIdx = 0; grpIdx < traitGroupCount; grpIdx++) {
                traitStartPos += 1 + (traitCount/2);
                traitCount = min(dstDNA[traitStartPos - 1] & TRAIT_COUNT_MASK, srcDNA[traitStartPos - 1] & TRAIT_COUNT_MASK);
                DBUGVAR(traitStartPos);
                DBUGVAR(traitCount);

                for(int index = 0; index < traitCount; index++)
                {
                  bSrcDNATraitEnable = false, bDstDNATraitEnable = false;

                  //! Seperate individual Trait from Source DNA
                  srcDNATrait = *(srcDNA + traitStartPos + (index >> 1)) >> (index & 0x1 ? 0:4);
                  srcDNATraitLevel = srcDNATrait & 0x07;
                  if(srcDNATrait & 0x08)
                  {
                    bSrcDNATraitEnable = true;
                  }
                  DBUGVAR(srcDNATraitLevel);

                  //! Seperate individual Trait from Destination DNA
                  dstDNATrait = *(dstDNA + traitStartPos + (index >> 1)) >> (index & 0x1 ? 0:4);
                  dstDNATraitLevel = dstDNATrait & 0x07;
                  if(dstDNATrait & 0x08)
                  {
                    bDstDNATraitEnable = true;
                  }
                  DBUGVAR(dstDNATraitLevel);

                  //! if both traits are Enabled then both should be matched
                  //! else if anyone of the two Enabled then consider as mismatch
                  //! else ignore it(both disabled condition)
                  if( (true == bSrcDNATraitEnable) && (true == bDstDNATraitEnable))
                  {
                    //! selected mismatch found
                    switch (abs(srcDNATraitLevel - dstDNATraitLevel)) {
                      case 0:
                        totalScore+=1;
                        break;
                      case 1:
                        totalScore+=0.5;
                        break;
                      case 2:
                        totalScore+=0.2;
                        break;
                      case 3:
                        totalScore+=0.05;
                        break;
                      default:
                        break;
                    }
                    activeTraitCount++;
                  }
                }

                if (0 == activeTraitCount) {
                  dnaMatchScore[grpIdx] = 255;  // Mark this one as not enabled
                }
                else {
                  dnaMatchScore[grpIdx] = (totalScore / (double)activeTraitCount) * 100;
                }
                DBUGVAR(activeTraitCount);
                DBUGVAR(totalScore);
                DBUGVAR(dnaMatchScore[grpIdx]);
                totalScore = 0;
                activeTraitCount = 0;
              }
              

              //! Send Update LED Status Request to Peripheral Device
              SendUpdateLEDStatusRequestToPeripheralDevice_v2();
              
              //! Use LED to indicate results.
              //It shows all the supported trait groups on this band.
              for (int grpIdx = 0; grpIdx < traitGroupCount; grpIdx++) {
                if (dnaMatchScore[grpIdx] > 100) {
                  DBUGLN("Select group of traits not enabled");
                  led->setBM_Unkonwn();
                } else if (dnaMatchScore[grpIdx] == 100) { 
                  DBUGLN("Overall Match Found. Flash Green LED");
                  led->setBM_OverallMatched();
                } else if (dnaMatchScore[grpIdx] > 70) {
                  DBUGLN("DNA Band matching");
                  led->setBM_Matched();
                } else if (dnaMatchScore[grpIdx] > 30) {
                  DBUGLN("DNA Band AMBER matching");
                  led->setBM_AmberMatched();
                } else if (dnaMatchScore[grpIdx] >= 0){
                  DBUGLN("DNA Band Not matching");
                  led->setBM_NotMatched();
                }

                delay(1000);
                while (false == led->isLEDIdle()) {
                  delay(1000);
                }
              }

              //! Move to Central Wait For Update LED Ack state
              BleManagerTask::eBandMatchingState = BM_STATE_CENTRAL_WAIT_FOR_UPDATE_LED_ACK;

              //! Store Update LED Status Start Time
              bandMatchLedAckStartTime = millis();
              bandMatchLedAckRetry = 0;
              
            }
            else
            {
              //! Disconnect the Peripheral
              bleSerial.DisconnectPeripheral();

              DBUGLN("Previous Peripheral Not in Bandmatching Mode. Re-Scanning");
              //! Move to Wait For Connetion State
              BleManagerTask::eBandMatchingState = BM_STATE_WAIT_FOR_CONNECTION;
              led->setBM_Entered();
            }
          }
          else
          {
            DBUGLN("Invalid Band matching State");
          }
        }
        else
        {
          DBUGLN("Data length not matched 1");
          //! Skip the packet
          //! 3 Retries available in central mode 
        }
      }
      break;
      case BandCommand_BandUpdateLedStatusREQ:
      {
        DBUGLN("DNA Match Status(LED Status) V1 from Central");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_BandUpdateLedStatusREQ_SIZE == size )
        {
          if (BM_STATE_PERIPHERAL_UPDATE_DNA_DATA == BleManagerTask::eBandMatchingState)
          {
            //! Move to Process LED Status State
            BleManagerTask::eBandMatchingState = BM_STATE_PERIPHERAL_PROCESS_LED_STATUS;
            dnaMatchScore[0] = buf[BAND_CMD_DATA_POS];
            //! Check the DNA match status and Flash the LED
            if (dnaMatchScore[0] >= 100)
            {
              //! Flash Green LED
              DBUGLN("Overall Match Found. Flash Green LED");
              led->setBM_OverallMatched();
            }
            else if (dnaMatchScore[0] > 70)
            {
              //! Flash Green LED
              DBUGLN("DNA Band matching");
              led->setBM_Matched();

            }
            else if (dnaMatchScore[0] > 30)
            {
              //! Flash Orange LED
              DBUGLN("DNA Band AMBER matching");
              led->setBM_AmberMatched();
            }
            else
            {
              //! Flash Red LED
              DBUGLN("DNA Band Not matching");
              led->setBM_NotMatched();
            }

            //! Get Peer Device Name
            memcpy(dnaMatchDeviceName, &buf[BAND_CMD_DATA_POS + BAND_DNA_DATA_STATUS_SIZE], DEVICE_NAME_SIZE);

            DBUG("Peer Central Band Name : ");
            DBUGLN(dnaMatchDeviceName);

            //! Update Bandmatch Status History
            UpdateBandmatchingHistory(dnaMatchDeviceName, dnaMatchScore, 1);

            //! Send Update LED Status Ack to Central Device
            dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
            dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

            len = BAND_CMD_CMDID_SIZE + DEVICE_NAME_SIZE;
            dnaStatusRsp[BAND_CMD_LEN_POS] = len;
            dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_BandUpdateLedStatusRSP;

            //! Update Device Name/BandID
            memcpy(&dnaStatusRsp[BAND_CMD_DATA_POS], DeviceName, DEVICE_NAME_SIZE);

            DBUGLN("Send DNA Matching Status(LED Status) V1 Ack to Central");
            bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_BandUpdateLedStatusRSP_SIZE);

            //! Band Matching. Peripheral Mode. Once Cycle is Over
            DBUGLN("Exit Bandmatching Mode");
            //! Move Back to Ready State
            BleManagerTask::eBandMatchingState = BM_STATE_READY;
          }
          else
          {
            DBUGLN("Invalid Band Match State");
          }
        }
        else
        {
          DBUGLN("Data length not matched 2");
          //! Skip the packet
          //! 3 Retries available in central mode 
        }
      }
      break;
      case BandCommand_BandUpdateLedStatus_V2REQ:
      {
        DBUGLN("DNA Match Status(LED Status) V2 from Central");
        //! Check whether we have received required data bytes or not?
        if(buf[BAND_CMD_LEN_POS] + BAND_CMD_HDR_SIZE == size )
        {
          DBUGVAR(BleManagerTask::eBandMatchingState);
          if (BM_STATE_PERIPHERAL_UPDATE_DNA_DATA == BleManagerTask::eBandMatchingState)
          {
            uint8_t traitCount = min(BAND_DNA_TRAITS_MAX_GROUP_COUNT, buf[BAND_CMD_LEN_POS] - BAND_CMD_CMDID_SIZE - BAND_CMD_STATUS_SIZE - DEVICE_NAME_SIZE);
            DBUGVAR(traitCount);
            for (uint8_t i = 0; i < traitCount; i++) {
              uint8_t _dnaMatchScore = buf[BandCommand_BandUpdateLedStatusREQ_SIZE+i];
              //! Check the DNA match status and Flash the LED
              if (_dnaMatchScore > 100)
              {
                DBUGLN("Select group of traits not enabled");
                led->setBM_Unkonwn();
              }
              else if (_dnaMatchScore == 100)
              {
                //! Flash Green LED
                DBUGLN("Overall Match Found. Flash Green LED");
                led->setBM_OverallMatched();
              }
              else if (_dnaMatchScore > 70)
              {
                //! Flash Green LED
                DBUGLN("DNA Band matching");
                led->setBM_Matched();

              }
              else if (_dnaMatchScore > 30)
              {
                //! Flash Orange LED
                DBUGLN("DNA Band AMBER matching");
                led->setBM_AmberMatched();
              }
              else if (_dnaMatchScore >= 0)
              {
                DBUGLN("DNA Band Not matching");
                led->setBM_NotMatched();
              }

              delay(1000);
              while (false == led->isLEDIdle()) {
                delay(1000);
              }
            }
            
            //! Get Peer Device Name
            memcpy(dnaMatchDeviceName, &buf[BAND_CMD_DATA_POS + BAND_DNA_DATA_STATUS_SIZE], DEVICE_NAME_SIZE);

            DBUG("Peer Central Band Name : ");
            DBUGLN(dnaMatchDeviceName);

            //! Update Bandmatch Status History
            if (traitCount <= 1) {
              UpdateBandmatchingHistory(dnaMatchDeviceName, &buf[BAND_CMD_DATA_POS], traitCount);
            }
            else {
              UpdateBandmatchingHistory(dnaMatchDeviceName, &buf[BandCommand_BandUpdateLedStatusREQ_SIZE], traitCount);
            }

            //! Send Update LED Status Ack to Central Device
            dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
            dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

            len = BAND_CMD_CMDID_SIZE + DEVICE_NAME_SIZE;
            dnaStatusRsp[BAND_CMD_LEN_POS] = len;
            dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_BandUpdateLedStatusRSP;

            //! Update Device Name/BandID
            memcpy(&dnaStatusRsp[BAND_CMD_DATA_POS], DeviceName, DEVICE_NAME_SIZE);

            DBUGLN("Send DNA Matching Status(LED Status) V2 Ack to Central");
            bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_BandUpdateLedStatusRSP_SIZE);

            //! Band Matching. Peripheral Mode. Once Cycle is Over
            DBUGLN("Exit Bandmatching Mode");
            //! Move Back to Ready State
            BleManagerTask::eBandMatchingState = BM_STATE_READY;
          }
          else
          {
            DBUGLN("Invalid Band Match State");
          }
        }
        else
        {
          DBUGLN("Data length not matched 2");
          //! Skip the packet
          //! 3 Retries available in central mode 
        }
      }
      break;
      case BandCommand_BandUpdateLedStatusRSP:
      {
        DBUGLN("DNA Match Status(LED Status) V1 Ack from Peripheral");
        
        //! Check whether we have received required data bytes or not?
        if(BandCommand_BandUpdateLedStatusRSP_SIZE == size )
        {
          if (BM_STATE_CENTRAL_WAIT_FOR_UPDATE_LED_ACK == BleManagerTask::eBandMatchingState)
          {
            //! Get Peer Device Name
            memcpy(dnaMatchDeviceName, &buf[BAND_CMD_DATA_POS], DEVICE_NAME_SIZE);

            DBUG("Peer Peripheral Band Name : ");
            DBUGLN(dnaMatchDeviceName);

            //! Update Bandmatch Status History
            UpdateBandmatchingHistory(dnaMatchDeviceName, dnaMatchScore, sizeof(dnaMatchScore));

            //! Band Matching. Central Mode. One Cycle is Over
            DBUGLN("Exit Bandmatching Mode");
            //! Move Back to Ready State
            BleManagerTask::eBandMatchingState = BM_STATE_READY;

            //! Disconnect Peripheral
            bleSerial.DisconnectPeripheral();
          }
          else
          {
            DBUGLN("Invalid Band Match State");
          }
        }
        else
        {
          DBUGLN("Data length not matched 3");
          //! Skip the packet
          //! 3 Retries available in central mode 
        }
      }
      break;
      //! Band to App commands
      case BandCommand_GetDataDNADataREQ:
      {
        DBUGLN("DNA Data Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        //! and then check whether the DNA Data is set in flash or not
        if ((BandCommand_GetDataDNADataREQ_SIZE == size ) && (BAND_DNA_DATA_SET_IN_FLASH == strBandMatchFlashTable.dnaDataSetInFlash))
        {
          DBUGLN("DNA Data Set in Flash. Get DNA Data Success");
          //! Send DNA Data Response to Central Device with Success
          SendDNADataResponseToCentralDevice_v2(BandCommand_GetDataDNAData_V2RSP, GET_DNA_DATA_STATUS_SUCCESS);
        }
        else
        {
          if(BandCommand_GetDataDNADataREQ_SIZE != size )
          {
            DBUGLN("Length Mismatch. Get DNA Data Failed");
          }
          else
          {
            DBUGLN("DNA Data not Set in Flash. Get DNA Data Failed");
          }
          //! Send DNA Data Response to Central Device with Failure
          SendDNADataResponseToCentralDevice_v2(BandCommand_GetDataDNAData_V2RSP, GET_DNA_DATA_STATUS_FAILURE);
        }
      }
      break;
      case BandCommand_SetDataDNADataREQ:
      {
        DBUGLN("Set DNA Data Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_SetDataDNADataREQ_HDR_SIZE < size )
        {
          //! Parse and Store the DNA Data into Flash
          if (true == UpdateDNADataIntoFlash((uint8_t *)&buf[BAND_CMD_DATA_POS], buf[BAND_CMD_LEN_POS] - BAND_CMD_STATUS_SIZE))
          {
            DBUGLN("Write DNA Data in Flash Success");
            status = STATUS_SUCCESS;
          }
          else
          {
            DBUGLN("Write DNA Data in Flash Failed");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched 4");
          status = STATUS_FAILURE;
        }
        //! Send Set DNA Data Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_DNA_MATCH_STATUS_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_SetDataDNADataRSP;

        //! Update Set DNA data status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        DBUGLN("Send Set DNA Data Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_SetDataDNADataRSP_SIZE);
      }
      break;
      case BandCommand_SetDataDNAData_V2REQ:
      {
        DBUGLN("Set DNA Data Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_SetDataDNADataREQ_HDR_SIZE < size )
        {
          //! Parse and Store the DNA Data into Flash
          if (true == UpdateDNADataIntoFlash((uint8_t *)&buf[BAND_CMD_DATA_POS], buf[BAND_CMD_LEN_POS] - BAND_CMD_STATUS_SIZE))
          {
            DBUGLN("Write DNA Data in Flash Success");
            status = STATUS_SUCCESS;
          }
          else
          {
            DBUGLN("Write DNA Data in Flash Failed");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched 4");
          status = STATUS_FAILURE;
        }
        //! Send Set DNA Data Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_DNA_MATCH_STATUS_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_SetDataDNADataRSP;

        //! Update Set DNA data status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        DBUGLN("Send Set DNA Data Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_SetDataDNADataRSP_SIZE);
      }
      break;
      case BandCommand_GetBMHistoryCountREQ:
      {
        DBUGLN("Get BM History Count Request from Mobile App");

        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetBMHistoryCountREQ_SIZE == size )
        {
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Data length not matched 5");
          status = STATUS_FAILURE;
        }

        //! Send Get History Count Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_DNA_DATA_STATUS_SIZE + BAND_MATCHING_HISTORY_COUNT_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_GetBMHistoryCountRSP;

        //! Update command status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        if(STATUS_SUCCESS == status)
        {
          DBUG("Bandmatch History Count : ");
          DBUGLN(strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards);

          //! Update Number of BM History Records
          dnaStatusRsp[BAND_CMD_DATA_POS + BAND_DNA_DATA_STATUS_SIZE] = strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards;
        }
        DBUGLN("Send Get BM History Count Response to Central Device");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_GetBMHistoryCountRSP_SIZE);
      }
      break;
      case BandCommand_GetBMHistoryByIndexREQ:
      {
        DBUGLN("Get BM History By Index Request from Mobile App");
        
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetBMHistoryByIndexREQ_SIZE == size )
        {
          band_matching_history_rsp rsp;

          //! Parse Index from command
          historyIndex = buf[BAND_CMD_DATA_POS];

          DBUG("Requested Index : ");
          DBUGLN(historyIndex);
          uint8_t numOfRecords = strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards;
          DBUGVAR(numOfRecords);
          if ((historyIndex >= 0) && (historyIndex < numOfRecords))
          {
            DBUGLN("STATUS_SUCCESS in");
            status = STATUS_SUCCESS;

            //! Update Device Name
            memcpy(rsp.deviceName,
                  strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[historyIndex].peerBandName, DEVICE_NAME_SIZE);
            //! Update DNA Match Status
            memcpy(rsp.matchScore, strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[historyIndex].matchStatus, sizeof(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[historyIndex].matchStatus));
            //! Update Timestamp
            memcpy(&rsp.timestamp,
                  &strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[historyIndex].time, BAND_MATCHING_HISTORY_TIMESTAMP_SIZE);

            DBUGLN("Send Get BM History By Index Success Response to Central Device ");
            bleSerial.write((uint8_t *)&rsp, sizeof(rsp));
          }
          else
          {
            DBUGLN("Index Out of Range.");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched 6");
          status = STATUS_FAILURE;
        }

        //! if the Given Index is Wrong or length mismatch
        //! then Send Failure Response
        if(STATUS_FAILURE == status)
        {
          len = BAND_CMD_CMDID_SIZE + BAND_DNA_DATA_STATUS_SIZE;
          dnaStatusRsp[BAND_CMD_LEN_POS] = len;
          dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_GetBMHistoryByIndexRSP;
          //! Update command status
          dnaStatusRsp[BAND_CMD_DATA_POS] = STATUS_FAILURE;

          DBUGLN("Send Get BM History By Index Failure Response to Central Device");
          bleSerial.write((uint8_t *)dnaStatusRsp, APP_CMD_GET_BM_HISTORY_BY_INDEX_FAILURE_RSP_SIZE);
        }
      }
      break;
      case BandCommand_ClearBMHistoryREQ:
      {
        DBUGLN("Clear BM History Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_ClearBMHistoryREQ_SIZE == size )
        {
          //! Clear the BM History and Write in Flash
          if (true == ClearBandmatchingHistory())
          {
            DBUGLN("Clear the BM History and Write in Flash Success");
            status = STATUS_SUCCESS;
          }
          else
          {
            DBUGLN("Clear the BM History and Write in Flash Failed");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched 7");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_DNA_MATCH_STATUS_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_ClearBMHistoryRSP;

        //! Update status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        DBUGLN("Send Clear BM History Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_ClearBMHistoryRSP_SIZE);
      }
      break;

      case BandCommand_GetSleepTimeoutREQ:
      {
        DBUGLN("Get Sleep Timeout Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetSleepTimeoutREQ_SIZE == size )
        {
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Data length not matched 8");
          status = STATUS_FAILURE;
        }

        //! Send Get Sleep Timeout Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_CMD_STATUS_SIZE + SLEEP_TIMEOUT_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_GetSleepTimeoutRSP;

        //! Update Get Sleep Timeout status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;
        
        if(STATUS_SUCCESS == status)
        {
          DBUG("Sleep Timeout : ");
          DBUGLN(strBandMatchFlashTable.autoSleepTime);
          //! Get the Sleep Timeout Data
          dnaStatusRsp[BAND_CMD_DATA_POS+1] = strBandMatchFlashTable.autoSleepTime;
        }

        DBUGLN("Send Get Sleep Timeout Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_GetDataDNADataRSP_SIZE);
      }
      break;
      case BandCommand_SetSleepTimeoutREQ:
      {
        DBUGLN("Set Sleep Timeout Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_SetSleepTimeoutREQ_SIZE == size )
        {
          if( (MINIMUM_AUTO_SLEEP_TIME <= buf[BAND_CMD_DATA_POS]) &&
              (MAXIMUM_AUTO_SLEEP_TIME >= buf[BAND_CMD_DATA_POS]) )
          {
            //! Store previous Sleep Time for backup
            autoSleepTime = strBandMatchFlashTable.autoSleepTime;

            //! Set the Auto Sleep Time
            status = setAutoSleepTimemout(buf[BAND_CMD_DATA_POS]);
            
            if(STATUS_SUCCESS == status)
            {
              //! Parse and Store the Sleep Timeout into Flash
              if (true == UpdateSleepTimeoutIntoFlash(buf[BAND_CMD_DATA_POS]))
              {
                DBUGLN("Write Sleep Timeout in Flash Success");
                status = STATUS_SUCCESS;
              }
              else
              {
                DBUGLN("Write Sleep Timeout in Flash Failed");
                status = STATUS_FAILURE;

                //! Restore the Old Sleep Time value
                strBandMatchFlashTable.autoSleepTime = autoSleepTime;
                setAutoSleepTimemout(strBandMatchFlashTable.autoSleepTime);
              }

              DBUG("Updated Sleep Timeout : ");
              DBUGLN(strBandMatchFlashTable.autoSleepTime);
            }
            else
            {
              DBUGLN("Set Auto Sleep Time Failed");
              status = STATUS_FAILURE;
            }
          }
          else
          {
            DBUGLN("Sleep Timeout Out Of Range.");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched 9");
          status = STATUS_FAILURE;
        }
        //! Send Set Sleep Timeout Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_CMD_STATUS_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_SetSleepTimeoutRSP;

        //! Update Set sleep timeout status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        DBUGLN("Send Set Sleep Timeout Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_SetDataDNADataRSP_SIZE);

      }
      break;

      case BandCommand_GetRssiThresholdREQ:
      {
        DBUGLN("Get RSSI Threshold Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetRssiThresholdREQ_SIZE == size )
        {
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Data length not matched 10");
          status = STATUS_FAILURE;
        }
        //! Send Get RSSI Threshold Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_CMD_STATUS_SIZE + SLEEP_TIMEOUT_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_GetRssiThresholdRSP;

        //! Update Get RSSI Threshold Status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;
        
        if(STATUS_SUCCESS == status)
        {
          DBUG("RSSI Threshold : ");
          DBUGLN(strBandMatchFlashTable.rssiThreshold);

          //! Get the RSSI Threshold Data
          dnaStatusRsp[BAND_CMD_DATA_POS+1] = strBandMatchFlashTable.rssiThreshold;
        }

        DBUGLN("Send Get RSSI Threshold Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_GetDataDNADataRSP_SIZE);
      }
      break;
      case BandCommand_SetRssiThresholdREQ:
      {
        DBUGLN("Set RSSI Threshold Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_SetRssiThresholdREQ_SIZE == size )
        {
          int8_t rssiValue = 0;

          rssiValue = buf[BAND_CMD_DATA_POS];
  
          if(0  > rssiValue)
          {
            //! Parse and Store the RSSI Threshold into Flash
            if ( true == UpdateRSSIThresholdIntoFlash(rssiValue))
            {
              DBUGLN("Write RSSI Threshold in Flash Success");
              bleSerial.setRSSIThrehold(strBandMatchFlashTable.rssiThreshold);
              status = STATUS_SUCCESS;
            }
            else
            {
              DBUGLN("Write RSSI Threshold in Flash Failed");
              status = STATUS_FAILURE;
            }
            DBUG("Updated RSSI Threshold : ");
            DBUGLN(strBandMatchFlashTable.rssiThreshold);
          }
          else
          {
            DBUGLN("Invalid RSSI Threshold");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched 11");
          status = STATUS_FAILURE;
        }

        //! Send Set RSSI Threshold Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_CMD_STATUS_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_SetRssiThresholdRSP;

        //! Update Set RSSI Threshold status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        DBUGLN("Send Set RSSI Threshold Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_SetDataDNADataRSP_SIZE);
      }
      break;
      case BandCommand_GetVersionNoREQ:
      {
        DBUGLN("Get Version Number Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetVersionNoREQ_SIZE == size )
        {
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Data length not matched 12");
          status = STATUS_FAILURE;
        }
        
        if(STATUS_SUCCESS == status)
        {
          DBUG("Version : ");
          DBUG(FW_MAJOR_VERSION);
          DBUG(".");
          DBUG(FW_MINOR_VERSION);
          DBUG(".");
          DBUG(FW_SUB_VERSION);
          DBUG(" B");
          DBUGLN(FW_BUILD_VERSION);
        }
        version_rsp_v2 rsp(status, FW_MAJOR_VERSION, FW_MINOR_VERSION, FW_SUB_VERSION, FW_BUILD_VERSION);

        DBUGLN("Send Get Version Number Response to Central Device ");
        bleSerial.write((uint8_t *)&rsp, sizeof(rsp));
      }
      break;
      case BandCommand_GetSampleIdREQ: {
        DBUGLN("Get Sample Id Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        uint64_t sampleId = UINT64_MAX;
        if(sizeof(band_get_sample_id_data) == size)
        {
          band_get_sample_id_data req;
          memcpy(&req, buf, size);
          if((sampleId = deviceManager->getUserSampleId(req.index)) == UINT64_MAX) {
            status = STATUS_USER_NOT_EXIST;
            DBUGLN("STATUS_USER_NOT_EXIST");
          } else {
            status = STATUS_SUCCESS;
          }
        }
        else
        {
          DBUGLN("Data length not matched 12");
          status = STATUS_FAILURE;
        }
        get_sample_id_rsp_data cmd(BandCommand_GetSampleIdRSP, status, sampleId);
        
        DBUGLN("Send Get Sample Id Response to Central Device ");
        DBUGF("sampleId:%s\r", (uint8_t *)&sampleId);
        DBUGF("status:%lx\r", status);
        bleSerial.write((uint8_t *)&cmd, sizeof(get_sample_id_rsp_data));
      } break;
      case BandCommand_SetSampleIdREQ:
      {
        DBUGLN("Set Sample Id Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(sizeof(band_set_sample_id_data) == size)
        {
          ErrorCode errorCode = ErrorCode_Failed;
          band_set_sample_id_data req;
          memcpy(&req, buf, size);
          if((errorCode = deviceManager->setUserSampleId(req.sampleId)) != ErrorCode_Success) {
            if (errorCode == ErrorCode_UserAlreadyExist) {
              status = STATUS_USER_EXIST;
              DBUGLN("STATUS_USER_EXIST");
            } else if (errorCode == ErrorCode_UserNotExist) {
              status = STATUS_USER_NOT_EXIST;
              DBUGLN("STATUS_USER_NOT_EXIST");
            } else if (errorCode == ErrorCode_UserSlotFull) {
              status = STATUS_USER_FULL;
              DBUGLN("STATUS_USER_FULL");
            }
          } else {
            status = STATUS_SUCCESS;
          }
        }
        else
        {
          DBUGLN("Data length not matched 12");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp cmd(BandCommand_SetSampleIdRSP, status);
        
        DBUGLN("Send Set Sample Id Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(band_cmd_base_rsp));
      } break;
      case BandCommand_GetEnabledUserSlotREQ:
      {
        band_req_get_enable_user_recomm_data req;
        DBUGLN("Get enable user slot");
        //! Check whether we have received required data bytes or not?
        if(sizeof(band_req_get_enable_user_recomm_data) == size )
        {
          memcpy(&req, buf, size);
          bool enabled = deviceManager->getEnabledUserSlotConfig(req.sampleId);
          band_cmd_base_rsp cmd(BandCommand_GetEnabledUserSlotRSP, enabled ? 0x1 : 0x0);
          DBUGF("userSlot raw api: 0x%x\r", cmd.status);
          bleSerial.write((uint8_t *)&cmd, sizeof(band_cmd_base_rsp));
        }
        else
        {
          DBUGLN("Data length not matched 13");
        }
        
      }
      break;
      case BandCommand_SetEnabledUserSlotREQ:
      {
        DBUGLN("Set Enabled Status for a User Slot Request from Mobile App");
        band_req_set_enable_user_recomm_data req;

        //! Check whether we have received required data bytes or not?
        if(sizeof(band_req_set_enable_user_recomm_data) == size)
        {
          memcpy(&req, buf, size);
          DBUGF("enableUserSlot[%s] : 0x%x\r", (uint8_t *)&req.sampleId, req.enable);
          if (deviceManager->setEnabledUserSlotConfig(req.sampleId, req.enable) != ErrorCode_Success) {
            DBUGLN("Set Enabled Status for a User Slot failed");
            status = STATUS_FAILURE;
          }
          else 
          {
            status = STATUS_SUCCESS;
          }
        }
        else
        {
          DBUGLN("Data length not matched 15");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp cmd(BandCommand_SetEnabledUserSlotRSP, status);

        DBUGLN("Send Set Enable User Slot Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_ClearUserSlotsREQ:
      {
        DBUGLN("Clear User Slot Request from Mobile App");
        band_cmd_base req;

        //! Check whether we have received required data bytes or not?
        if(sizeof(band_cmd_base) == size)
        {
          if (deviceManager->disableAllUserSlotConfig() != ErrorCode_Success) {
            DBUGLN("Clear User Slot failed");
            status = STATUS_FAILURE;
          }
          else
          {
            status = STATUS_SUCCESS;  
          }
        }
        else
        {
          DBUGLN("Data length not matched 15");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp cmd(BandCommand_ClearUserSlotsRSP, status);

        DBUGLN("Send Clear User Slot Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_GetDbVersionNoREQ:
      {
        DBUGLN("Get DB Version Request from Mobile App");
        band_cmd_base req;
        uint32_t version = 0;
        //! Check whether we have received required data bytes or not?
        if(sizeof(band_cmd_base) == size)
        {
          version = deviceManager->getDatabaseVersion();
          if (version == 0) {
            DBUGLN("Get DB Version Slot failed");
            status = STATUS_FAILURE;
          } else {
            status = STATUS_SUCCESS;
          }
        }
        else
        {
          DBUGLN("Data length not matched 15");
          status = STATUS_FAILURE;
        }
        // Version = [Year, Month, Major]
        get_db_version_rsp cmd(status, (version >> 16) & 0xFF, (version >> 8) & 0xFF, version & 0xFF);
        DBUGVAR((version >> 16) & 0xFF);
        DBUGVAR((version >> 8) & 0xFF);
        DBUGVAR(version & 0xFF);

        DBUGLN("Send Get DB Version Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(get_db_version_rsp));
      }
      break;
      case BandCommand_SetDbVersionNoREQ:
      {
        DBUGLN("Set DB Version Request from Mobile App");
        set_db_version_req req;
        //! Check whether we have received required data bytes or not?
        if(sizeof(set_db_version_req) == size)
        {
          memcpy(&req, buf, size);
          if (deviceManager->setDatabaseVersion(req.year << 16 | req.month <<8 | req.major) == ErrorCode_Success) {
            status = STATUS_SUCCESS;
          }
          else
          {
            DBUGLN("Set DB Version failed");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp cmd(BandCommand_SetDbVersionNoRSP, status);

        DBUGLN("Send Set DB Version Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(band_cmd_base_rsp));
      }
      break;
      // case BandCommand_GetLifeStyleHistCountREQ:
      // {
      //   DBUGLN("Get lifestyle History Count Request from Mobile App");

      //   //! Check whether we have received required data bytes or not?
      //   if(BandCommand_GetLifeStyleHistCountREQ_SIZE == size )
      //   {
      //     status = STATUS_SUCCESS;
      //   }
      //   else
      //   {
      //     DBUGLN("Data length not matched 5");
      //     status = STATUS_FAILURE;
      //   }

      //   //! Send Get History Count Response to Central Device
      //   uint32_t count = 0; //MIN(lifeStyle->getLifeStyleHistoryCount(), lifeStyle->getMaxLSHistoryCount());
      //   DBUGVAR(count);
      //   get_lifestyle_history_count_rsp rsp(count, status);
      //   DBUGLN("Send Get lifestyle History Count Response to Central Device");
      //   bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_history_count_rsp));
      // }
      // break;
      // case BandCommand_CleanPenaltyHisREQ:
      // {
      //   DBUGLN("Clear life style History Request from Mobile App");
      //   //! Check whether we have received required data bytes or not?
      //   if(BandCommand_CleanPenaltyHisREQ_SIZE == size )
      //   {
      //     //! Clear the BM History and Write in Flash
      //       lifeStyle->eraseLifeStyleHistory();
      //       DBUGLN("Clear the lifestyle History");
      //       status = STATUS_SUCCESS;
      //   }
      //   else
      //   {
      //     DBUGLN("Data length not matched 7");
      //     status = STATUS_FAILURE;
      //   }        
      //   band_cmd_base_rsp rsp(BC(CleanPenaltyHis, RSP), status);

      //   DBUGLN("Send Clear life style History Response to Central Device ");
      //   bleSerial.write((uint8_t *)&rsp, sizeof(band_cmd_base_rsp));
      // }
      // break;
      // case BandCommand_GetPenaltyHisByIndexREQ:
      // {
      //   DBUGLN("Get life style PENALTY History By Index Request from Mobile App");
        
      //   //! Check whether we have received required data bytes or not?
      //   uint8_t penalty_point = 0;
      //   uint32_t stepCount = 0;
      //   uint32_t timestamp = 0;
      //   status = STATUS_FAILURE;
      //   if(BandCommand_GetPenaltyHisByIndexREQ_SIZE  == size )
      //   {          
      //     //! Parse Index from command
      //     historyIndex = buf[BAND_CMD_DATA_POS];

      //     DBUG("Requested Index : ");
      //     DBUGLN(historyIndex);
      //     uint32_t numOfRecords;
      //     if(lifeStyle->getLifeStyleHistoryCount() >= lifeStyle->getMaxLSHistoryCount())
      //     {
      //       numOfRecords = lifeStyle->getMaxLSHistoryCount();
      //     }
      //     else
      //     {
      //       numOfRecords = lifeStyle->getLifeStyleHistoryCount();
      //     }
      //     DBUGVAR(numOfRecords);
      //     if ((historyIndex >= 0) && (historyIndex < numOfRecords))
      //     {
      //       DBUGLN("STATUS_SUCCESS in");
      //       status = STATUS_SUCCESS;
      //       lifeStyle->getLifeStyleHistory(historyIndex, &timestamp, &penalty_point, &stepCount);
      //       DBUGVAR(penalty_point);
      //       DBUGVAR(timestamp);
      //     }
      //     else
      //     {
      //       DBUGLN("Index Out of Range.");
      //     }
      //   }
      //   else
      //   {
      //     DBUGLN("Data length not matched 6");
      //   }
      //   get_lifestyle_history_rsp rsp(penalty_point, stepCount, timestamp, status);
      //   DBUGVAR(rsp.penaltyPoint);
      //   DBUGVAR(rsp.stepCount);
      //   bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_history_rsp));
      // }
      // break;
      case BandCommand_SetLifestyleSleepTimeREQ:
      {
        DBUGLN("Set life style sleeptime Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_SetLifestyleSleepTimeREQ_SIZE == size )
        {
          //! Setting lifestyle sleep time
          set_lifestyle_sleeping_time_req req;
          memcpy(&req, buf, sizeof(set_lifestyle_sleeping_time_req));
          status = lifeStyle->setSleepingTime(req.start_time, req.end_time)?STATUS_SUCCESS:STATUS_FAILURE;
          DBUGLN("set sleep time");
          DBUGVAR(req.start_time);
          DBUGVAR(req.end_time);
        }
        else
        {
          DBUGLN("Set sleep time faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        len = BAND_CMD_CMDID_SIZE + BAND_DNA_MATCH_STATUS_SIZE;
        dnaStatusRsp[BAND_CMD_LEN_POS] = len;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_SetLifestyleSleepTimeRSP;

        //! Update status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        DBUGLN("Send set sleep time Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_SetLifestyleSleepTimeRSP_SIZE);
      }
      break;
      case BandCommand_GetLifestyleSleepTimeREQ:
      {
        uint32_t getsleepstart = 0, getsleepstop = 0;  
        DBUGLN("Get life style sleeptime Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetLifestyleSleepTimeREQ_SIZE == size )
        {
          //! Setting lifestyle sleep time
                  
          lifeStyle->getSleepingTime(&getsleepstart, &getsleepstop);
          DBUGLN("get sleep time");
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get sleep time faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_lifestyle_sleeping_time_rsp rsp(getsleepstart, getsleepstop, status);
          
        DBUGLN("Send Get sleep time Response to Central Device ");
        DBUGVAR(getsleepstart);
        DBUGVAR(getsleepstop);
        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_sleeping_time_rsp));
      }
      break;
      case BandCommand_SetLifestyleSleepTimeEnabledREQ:
      {
        DBUGLN("Set life style sleep time enable status Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(sizeof(set_lifestyle_enable_req) == size )
        {
          //! Setting lifestyle sleep time
          set_lifestyle_enable_req req;
          memcpy(&req, buf, size);
          DBUGVAR(req.enabled);
          lifeStyle->enableSleepTime(req.enabled == 1);
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Set Sleep Time Enable Status faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        band_cmd_base_rsp rsp(BandCommand_SetLifestyleSleepTimeEnabledRSP, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_GetLifestyleSleepTimeEnabledREQ:
      {
        DBUGLN("Get life style sleep time enable status Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        bool enabled = false;
        if(sizeof(band_cmd_base) == size )
        {
          //! Setting lifestyle sleep time
          band_cmd_base req;
          memcpy(&req, buf, size);
          enabled = lifeStyle->getSleepTimeEnabled();
          DBUGLN("Get Sleep Time Enable Status");
          DBUGVAR(enabled);
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get Sleep Time Enable Status faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_lifestyle_enable_rsp rsp(BC(GetLifestyleSleepTimeEnabled, RSP), enabled, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_enable_rsp));
      }
      break;
      case BandCommand_SetLifestyleEnabledREQ:
      {
        DBUGLN("Set life style enable status Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(sizeof(set_lifestyle_enable_req) == size )
        {
          //! Setting lifestyle sleep time
          set_lifestyle_enable_req req;
          memcpy(&req, buf, size);
          lifeStyle->setLifeStyleEnableStatus(req.enabled == 1);
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Set Enable Status faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        band_cmd_base_rsp rsp(BandCommand_SetLifestyleEnabledRSP, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_GetLifestyleEnabledREQ:
      {
        DBUGLN("Get life style enable status Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        bool enabled = false;
        if(sizeof(band_cmd_base) == size )
        {
          //! Setting lifestyle sleep time
          enabled = lifeStyle->getLifeStyleEnableStatus();
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get Enable Status faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_lifestyle_enable_rsp rsp(BandCommand_GetLifestyleEnabledRSP, enabled, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_enable_rsp));
      }
      break;
      case BandCommand_SetLifestylePenaltyLimitsREQ:
      {
        DBUGLN("Set life style penalty points limits Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_SetLifestylePenaltyLimitsREQ_SIZE == size )
        {
          //! Setting lifestyle sleep time
          set_lifestyle_penalty_limits_req req;
          memcpy(&req, buf, sizeof(set_lifestyle_penalty_limits_req));
          status = lifeStyle->setPenaltyLimits(req.maxPenaltyPoints)?STATUS_SUCCESS:STATUS_FAILURE;
          DBUGLN("Set max penalty points");
          DBUGVAR(req.maxPenaltyPoints);
        }
        else
        {
          DBUGLN("Set max penalty points faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        dnaStatusRsp[BAND_CMD_SOF_1_POS] = BAND_CMD_SOF_1_VALUE;
        dnaStatusRsp[BAND_CMD_SOF_2_POS] = BAND_CMD_SOF_2_VALUE;

        dnaStatusRsp[BAND_CMD_LEN_POS] = BandCommand_SetLifestylePenaltyLimitsRSP_SIZE - BAND_CMD_HDR_SIZE;
        dnaStatusRsp[BAND_CMD_CMD_POS] = BandCommand_SetLifestylePenaltyLimitsRSP;

        //! Update status
        dnaStatusRsp[BAND_CMD_DATA_POS] = status;

        DBUGLN("Send set max penalty points Response to Central Device ");
        bleSerial.write((uint8_t *)dnaStatusRsp, BandCommand_SetLifestylePenaltyLimitsRSP_SIZE);
      }
      break;
      case BandCommand_GetLifestylePenaltyLimitsREQ:
      {
        uint8_t _maxPenaltyPoints = 0;  
        DBUGLN("Get life style max penalty points Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetLifestylePenaltyLimitsREQ_SIZE == size )
        {
          //! Setting lifestyle sleep time                  
          _maxPenaltyPoints = lifeStyle->getPenaltyLimits();
          status = STATUS_SUCCESS;
          DBUGLN("Get max penalty points");
        }
        else
        {
          DBUGLN("Get max penalty points faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_lifestyle_penalty_limits_rsp rsp(_maxPenaltyPoints, status);
          
        DBUGLN("Send get max penalty points Response to Central Device ");
        DBUGVAR(_maxPenaltyPoints);
        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_penalty_limits_rsp));
      }
      break;
      case BandCommand_SetLifestyleSyncStatusREQ:
      { 
        DBUGLN("Set life style sync status Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(sizeof(set_lifestyle_enable_req) == size )
        {
          //! Setting lifestyle sleep time
          set_lifestyle_enable_req req;
          memcpy(&req, buf, size);
          status = (deviceManager->setLifeStyleSyncStatus(req.enabled == 1) == ErrorCode_Success) ? STATUS_SUCCESS : STATUS_FAILURE;
        }
        else
        {
          DBUGLN("Set life style sync status faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        band_cmd_base_rsp rsp(BandCommand_SetLifestyleSyncStatusRSP, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_GetLifestyleSyncStatusREQ:
      {
        DBUGLN("Get life style sync status Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        bool enabled = false;
        if(sizeof(band_cmd_base) == size )
        {
          //! Setting lifestyle sleep time
          enabled = deviceManager->isLifeStyleSynced();
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get life style sync Status faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_lifestyle_enable_rsp rsp(BandCommand_GetLifestyleSyncStatusRSP, enabled, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_enable_rsp));
      }
      break;
      case BandCommand_GetLifestyleRunningStatusREQ:
      {
        DBUGLN("Get life style running status Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        bool isRunning = false;
        if(sizeof(band_cmd_base) == size )
        {
          //! Setting lifestyle sleep time
          isRunning = lifeStyle->getRunningStatus();
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get life style running status faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_lifestyle_enable_rsp rsp(BandCommand_GetLifestyleRunningStatusRSP, isRunning, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_enable_rsp));
      }
      break;
      case BandCommand_GetLifestylePenEndTimeREQ:
      {
        DBUGLN("Get life style penalty endtime Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        uint32_t _endTime = 0;
        if(sizeof(band_cmd_base) == size )
        {
          //! Setting lifestyle sleep time
          band_cmd_base req;
          memcpy(&req, buf, size);
          _endTime = lifeStyle->getPenaltyEndTime();
          DBUGLN("Get life style penalty endtime");
          DBUGVAR(_endTime);
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get life style penalty endtime failed");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_lifestyle_penalty_endtime_rsp rsp(_endTime, status);
          
        DBUGLN("Send Get life style penalty endtime Response to Central Device ");
        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_penalty_endtime_rsp));
      }
      break;
      case BandCommand_GetSittingStepCountREQ:
      {
        DBUGLN("Get sitting time, step count Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(sizeof(band_cmd_base) == size )
        {
          DBUGLN("Get sitting time, step count");
          DBUGVAR(deviceManager->lifeStyleConfig.penaltyPoints);
          DBUGVAR(deviceManager->lifeStyleConfig.penaltyLimits);
          DBUGVAR(deviceManager->lifeStyleConfig.stepCount);
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get sitting time, step count failed");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
#ifdef ENABLE_DEBUG_LIFESTYLE_UNIT_TEST
        static int testPoint = 0;
        static int testStepCount = 0;
        get_sitting_step_count_rsp rsp(testPoint++, lifeStyle->getPersonalGoal(), testStepCount, status);
        testStepCount+=300;
        if (testPoint > 48) {
          testPoint = 0;
        }
#else 
        get_sitting_step_count_rsp rsp(deviceManager->lifeStyleConfig.penaltyPoints, lifeStyle->getPersonalGoal(), deviceManager->lifeStyleConfig.stepCount, status);
#endif
          
        DBUGLN("Send Get sitting time, step count Response to Central Device ");
        bleSerial.write((uint8_t *)&rsp, sizeof(get_sitting_step_count_rsp));
      }
      break;
      case BandCommand_SetEquivalentStepsREQ:
      {
        DBUGLN("Set equivalent steps per penalty point Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(sizeof(set_equivalent_step_count_req) == size )
        {
          //! Setting lifestyle sleep time
          set_equivalent_step_count_req req;
          memcpy(&req, buf, size);
          status = ( lifeStyle->setEquivalentSteps(req.equivalentSteps)== ErrorCode_Success) ? STATUS_SUCCESS : STATUS_FAILURE;
        }
        else
        {
          DBUGLN("Set equivalent steps per penalty point faild");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp rsp(BandCommand_SetEquivalentStepsRSP, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_GetEquivalentStepsREQ:
      {
        uint16_t _equivalentSteps = 0;  
        DBUGLN("Get equivalent steps per penalty point Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetEquvalentStepsREQ_SIZE == size )
        {
          //! Setting lifestyle sleep time                  
          _equivalentSteps = lifeStyle->getEquivalentSteps();
          status = STATUS_SUCCESS;
          DBUGLN("Get equivalent steps per penalty point");
        }
        else
        {
          DBUGLN("Get equivalent steps per penalty point faild");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_equivalent_step_count_rsp rsp(_equivalentSteps, status);
          
        DBUGLN("Send equivalent steps per penalty point Response to Central Device ");
        DBUGVAR(_equivalentSteps);
        bleSerial.write((uint8_t *)&rsp, sizeof(get_equivalent_step_count_rsp));
      }
      break;
      case BandCommand_GetMacAddressREQ:
      {
        DBUGLN("Get MacAddress Request from Mobile App");
        uint8_t macAddress[MAC_ADDRESS_SIZE] = {0};
        //! Check whether we have received required data bytes or not?
        if(sizeof(band_cmd_base) == size )
        {
          Bluefruit.Gap.getAddr(macAddress);
          DBUGLN("Get MacAddress success");
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Get MacAddress failed / Data length not matched");
          status = STATUS_FAILURE;
        }
        
        //! Send Get MacAddress Response to Central Device
        get_mac_address_rsp rsp(macAddress, status);
        DBUGLN("Send Get MacAddress Response to Central Device ");
        bleSerial.write((uint8_t *)&rsp, sizeof(get_mac_address_rsp));
      }
      break;    
      case BandCommand_GetTestIdREQ:
      {
        DBUGLN("Get TestId Request from Mobile App");
        uint8_t testId[TEST_ID_SIZE] = {0};
        //! Check whether we have received required data bytes or not?
        if(sizeof(band_cmd_base) == size )
        {
#ifdef ENABLE_DEBUG_LIFESTYLE_UNIT_TEST
          memcpy(testId, "ec2771f1a7d8", 12);
          DBUGLN("Get TestId success");
          status = STATUS_SUCCESS;
#else
          if(deviceManager->getTestId(testId)) {
            DBUGLN("Get TestId success");
            status = STATUS_SUCCESS;
          } else {
            DBUGLN("Get TestId failed");
            status = STATUS_FAILURE;
          }
#endif
        }
        else
        {
          DBUGLN("Data length not matched");
          status = STATUS_FAILURE;
        }
        get_test_id_rsp rsp(testId, status);
        DBUGLN("Send Get TestId Response to Central Device ");
        bleSerial.write((uint8_t *)&rsp, sizeof(get_test_id_rsp));
      }
      break;
      case BandCommand_GetAllergySlotIndexREQ:
      {
        uint8_t _allergySlotIndex = 0;  
        DBUGLN("Get equivalent steps per penalty point Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetAllergySlotIndexREQ_SIZE == size )
        {
          //! Setting lifestyle sleep time                  
          _allergySlotIndex = allergySlotIndex;
          status = STATUS_SUCCESS;
          DBUGLN("Get current allergy slot index");
        }
        else
        {
          DBUGLN("Get allergy slot index failed");
          status = STATUS_FAILURE;
        }
        //! Send Clear BM History Response to Central Device
        get_allergy_slot_index_rsp rsp(_allergySlotIndex, status);
          
        DBUGLN("Send allergy slot index Response to Central Device ");
        DBUGVAR(_allergySlotIndex);
        bleSerial.write((uint8_t *)&rsp, sizeof(get_allergy_slot_index_rsp));
      }
      break;
      case BandCommand_SetAllergySlotIndexREQ:
      {
        DBUGLN("Set allergy slot index Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(sizeof(set_allergy_slot_index_req) == size )
        {
          //! Setting lifestyle sleep time
          set_allergy_slot_index_req req;
          memcpy(&req, buf, size);
          status = ( deviceManager->setAllergySlotIndex(req.index) == ErrorCode_Success) ? STATUS_SUCCESS : STATUS_FAILURE;
          if (status == STATUS_SUCCESS) {
            allergySlotIndex = req.index;
            barcodeManager->updateAllergySlotIndex();
            scanner->updateAllergySlotIndex();
          }
        }
        else
        {
          DBUGLN("Set eallergy slot index failed");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp rsp(BandCommand_SetAllergySlotIndexRSP, status);          
        bleSerial.write((uint8_t *)&rsp, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_GetUserRecVersionREQ:
      {
        DBUGLN("Get User Recommendation Version Request from Mobile App");
        get_rec_version_req req;
        uint32_t version = 0;
        //! Check whether we have received required data bytes or not?
        if(sizeof(get_rec_version_req) == size)
        {
          memcpy(&req, buf, sizeof(get_rec_version_req)); 
          version = deviceManager->getUserRecVersion(req.sampleID);
          if (version == 0) {
            DBUGF("Get User Recommendation Version Request failed for user %s", (const char*)&req.sampleID);
            status = STATUS_FAILURE;
          } else {
            status = STATUS_SUCCESS;
          }
        }
        else
        {
          DBUGLN("Data length not matched");
          status = STATUS_FAILURE;
        }
        // Version = [Year, Month, Major]
        get_rec_version_rsp cmd(status, (version >> 24) & 0xFF, (version >> 16) & 0xFF, (version >> 8) & 0xFF, (version >> 4) & 0xF, version & 0xF, req.sampleID);
        DBUGVAR((version >> 24) & 0xFF);
        DBUGVAR((version >> 16) & 0xFF);
        DBUGVAR((version >> 8) & 0xFF);
        DBUGVAR((version >> 4) & 0xF);
        DBUGVAR(version & 0xF);
        DBUGLN("Send Get User Recommendation Version Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(get_rec_version_rsp));
      }
      break;
      case BandCommand_SetUserRecVersionREQ:
      {
        DBUGLN("Set User Recommendation Version Request from Mobile App");
        set_rec_version_req req;
        //! Check whether we have received required data bytes or not?
        if(sizeof(set_rec_version_req) == size)
        {
          memcpy(&req, buf, size);
          if (deviceManager->setUserRecVersion(req.year << 24 | req.month << 16 | req.major << 8 | req.ageGroup << 4 | req.version, req.sampleID) == ErrorCode_Success) {
            DBUGVAR(req.year);
            DBUGVAR(req.month);
            DBUGVAR(req.major);
            DBUGVAR(req.ageGroup);
            DBUGVAR(req.version);
            status = STATUS_SUCCESS;
          }
          else
          {
            DBUGLN("Set User Recommendation Version failed");
            status = STATUS_FAILURE;
          }
        }
        else
        {
          DBUGLN("Data length not matched");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp cmd(BandCommand_SetUserRecVersionRSP, status);

        DBUGLN("Send Set User Recommendation Version Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_VirtualNudgeMatchREQ:
      {
        DBUGLN("Virtual NudgeMatch Request");

        //! Check whether we have received a request with DnaData
        if(BAND_CMD_HDR_SIZE + buf[BAND_CMD_LEN_POS] == size )
        {
          //! Check Whether Pheripheral is in Band matching mode or not
          //! If the cmd Status is Success, Peripheral is in Band match mode
          //! Else Disconnect the Peripheral Device

          //! Get Own DNA Data and compare with the received DNA data

          //! Build Source DNA
          srcDNA = strBandMatchFlashTable.bandDnaData;

          //! Build Destination DNA
          memcpy(dstDNA, (uint8_t *)&buf[BAND_CMD_DATA_POS], buf[BAND_CMD_LEN_POS] - BAND_CMD_STATUS_SIZE );

          totalScore = 0;
          //! Check individual traits match
          traitCount = 0;
          traitStartPos = 1;
          activeTraitCount = 0;
          traitGroupCount = min(srcDNA[0], dstDNA[0]);
          DBUGVAR(traitGroupCount);

          for (int grpIdx = 0; grpIdx < traitGroupCount; grpIdx++) {
            traitStartPos += 1 + (traitCount/2);
            traitCount = min(dstDNA[traitStartPos - 1] & TRAIT_COUNT_MASK, srcDNA[traitStartPos - 1] & TRAIT_COUNT_MASK);
            DBUGVAR(traitStartPos);
            DBUGVAR(traitCount);

            for(int index = 0; index < traitCount; index++)
            {
              bSrcDNATraitEnable = false, bDstDNATraitEnable = false;

              //! Seperate individual Trait from Source DNA
              srcDNATrait = *(srcDNA + traitStartPos + (index >> 1)) >> (index & 0x1 ? 0:4);
              srcDNATraitLevel = srcDNATrait & 0x07;
              if(srcDNATrait & 0x08)
              {
                bSrcDNATraitEnable = true;
              }
              DBUGVAR(srcDNATraitLevel);

              //! Seperate individual Trait from Destination DNA
              dstDNATrait = *(dstDNA + traitStartPos + (index >> 1)) >> (index & 0x1 ? 0:4);
              dstDNATraitLevel = dstDNATrait & 0x07;
              if(dstDNATrait & 0x08)
              {
                bDstDNATraitEnable = true;
              }
              DBUGVAR(dstDNATraitLevel);

              //! if both traits are Enabled then both should be matched
              //! else if anyone of the two Enabled then consider as mismatch
              //! else ignore it(both disabled condition)
              if( (true == bSrcDNATraitEnable) && (true == bDstDNATraitEnable))
              {
                //! selected mismatch found
                switch (abs(srcDNATraitLevel - dstDNATraitLevel)) {
                  case 0:
                    totalScore+=1;
                    break;
                  case 1:
                    totalScore+=0.5;
                    break;
                  case 2:
                    totalScore+=0.2;
                    break;
                  case 3:
                    totalScore+=0.05;
                    break;
                  default:
                    break;
                }
                activeTraitCount++;
              }
            }

            if (0 == activeTraitCount) {
              dnaMatchScore[grpIdx] = 255;  // Mark this one as not enabled
            }
            else {
              dnaMatchScore[grpIdx] = (totalScore / (double)activeTraitCount) * 100;
            }
            DBUGVAR(activeTraitCount);
            DBUGVAR(totalScore);
            DBUGVAR(dnaMatchScore[grpIdx]);
            totalScore = 0;
            activeTraitCount = 0;
          }

          //! Use LED to indicate results.
          //It shows all the supported trait groups on this band.
          for (int grpIdx = 0; grpIdx < traitGroupCount; grpIdx++) {
            if (dnaMatchScore[grpIdx] > 100) {
              DBUGLN("Select group of traits not enabled");
              led->setUnknown();
            } else if (dnaMatchScore[grpIdx] == 100) { 
              DBUGLN("Overall Match Found. Flash Green LED");
              led->setBM_OverallMatched();
            } else if (dnaMatchScore[grpIdx] > 70) {
              DBUGLN("DNA Band matching");
              led->setBM_Matched();
            } else if (dnaMatchScore[grpIdx] > 30) {
              DBUGLN("DNA Band AMBER matching");
              led->setBM_AmberMatched();
            } else if (dnaMatchScore[grpIdx] >= 0){
              DBUGLN("DNA Band Not matching");
              led->setBM_NotMatched();
            }

            delay(1000);
            while (false == led->isLEDIdle()) {
              delay(1000);
            }
          }
          status = STATUS_SUCCESS;
        }
        else {
          DBUGLN("Virtual NudgeMatch request doesn't have DNA data");
          status = STATUS_FAILURE;
        }

        virtual_nudgeMatch_rsp rsp(BAND_DNA_MATCH_STATUS_SIZE + traitGroupCount, status);
        memcpy(rsp.matchScore, dnaMatchScore, sizeof(dnaMatchScore));
        DBUGLN("Send Virtual NudgeMatch Response to App ");
        bleSerial.write((uint8_t *)&rsp, BAND_CMD_HDR_SIZE + BAND_CMD_CMDID_SIZE + BAND_DNA_MATCH_STATUS_SIZE + traitGroupCount);
      }
      break;
      case BandCommand_CleanDNADataNudgeMatchREQ:
      {
        DBUGLN("Clean DNA and NudgeMath Region Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_CleanDNADataNudgeMatchREQ_SIZE == size )
        {
          //! Clear the BM History and Write in Flash
          resetBandMatchingRegionInFlash();
          DBUGLN("Clean DNA and NudgeMath Region");
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGLN("Data length not matched");
          status = STATUS_FAILURE;
        }        
        band_cmd_base_rsp rsp(BC(CleanDNADataNudgeMatch, RSP), status);

        DBUGLN("Send Clean DNA and NudgeMath Region Response to Central Device ");
        bleSerial.write((uint8_t *)&rsp, sizeof(band_cmd_base_rsp));
      }
      break;
      case BandCommand_GetNumberOfLifeStyleHisory_V2REQ:
      {
        DBUGLN("Get lifestyle History Count Request from Mobile App");
        uint8_t numOfRecords = 0;
        //! Check whether we have received required data bytes or not?
        if(BandCommand_GetNumberOfLifeStyleHisoryREQ_V2_SIZE == size )
        {
          numOfRecords = lifeStyle->getLifeStyleHistoryCount(); //MIN(lifeStyle->getLifeStyleHistoryCount(), lifeStyle->getMaxLifeStyleHistoryCount());
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGF("Band lenth fomate not matched %d",BandCommand_GetNumberOfLifeStyleHisoryREQ_V2_SIZE);
          status = STATUS_FAILURE;
          numOfRecords = 0xFF;
        }        

        get_number_of_lifestyle_hisory_V2_rsp rsp(status,numOfRecords);
        DBUGVAR(numOfRecords);
        DBUGLN("Send Get lifestyle History Count Response to Central Device");
        bleSerial.write((uint8_t *)&rsp, sizeof(get_number_of_lifestyle_hisory_V2_rsp));
      }
      break;
      case BandCommand_GetLifeStyleHisoryIndex_V2REQ:
      {
        //Init return value
        uint8_t historyindex = 0xFF;
        uint32_t timestamp = 0xFFFFFFFF;
        uint8_t penaltyPoint = 0xFF;
        uint32_t stepCount = 0xFFFFFFFF;
        DBUGLN("Get life style History By Index Request from Mobile App");
        status = STATUS_FAILURE;
        if(BandCommand_GetLifeStyleHisoryIndex_V2_SIZE == size )
        {
          historyindex = buf[BAND_CMD_DATA_POS];
          DBUG("Requested Index : ");
          DBUGLN(historyindex);
          if (historyindex < lifeStyle->getMaxLifeStyleHistoryCount() && historyindex >= 0)
          {
            lifeStyle->getLifeStyleHistory(historyindex, &timestamp, &penaltyPoint, &stepCount);
            DBUGVAR(penaltyPoint);
            DBUGVAR(timestamp);
            DBUGVAR(stepCount);
            status = STATUS_SUCCESS;
          }
          else
          {
            DBUGLN("Index Out of Range.");
          }
        }
        else
        {
          DBUGF("Band lenth fomate not matched %d",BandCommand_GetLifeStyleHisoryIndex_V2_SIZE);
        }
        
        get_lifestyle_hisory_index_V2_rsp rsp(status,historyindex,timestamp,penaltyPoint,stepCount);

        bleSerial.write((uint8_t *)&rsp, sizeof(get_lifestyle_hisory_index_V2_rsp));
      }
      break;  
      case BandCommand_CleanLifeStyleHisory_V2REQ:
      {
        DBUGLN("Clear life style History Request from Mobile App");
        //! Check whether we have received required data bytes or not?
        if(BandCommand_CleanLifeStyleHisory_V2_SIZE == size )
        {
          
          lifeStyle->eraseLifeStyleHistory();
          DBUGLN("Clear the lifestyle History");
          status = STATUS_SUCCESS;
        }
        else
        {
          DBUGF("Band lenth fomate not matched %d",BandCommand_CleanLifeStyleHisory_V2_SIZE);
          status = STATUS_FAILURE;
        }
        clean_lifestyle_hisory_V2_rsp rsp(status);

        DBUGLN("Send Clear life style History Response to Central Device ");
        bleSerial.write((uint8_t *)&rsp, sizeof(clean_lifestyle_hisory_V2_rsp));
      }
      break; 
         
      
#ifdef ENABLE_TEST_RECOMMENDATION
      case BandCommand_VerifyRecommendationREQ:
      {
        // DBUGLN("Verify User Recommendation Request from Mobile App");
        verify_user_recomm_data req;

        //! Check whether we have received required data bytes or not?
        if(sizeof(verify_user_recomm_data) == size)
        {
          memcpy(&req, buf, size);
          // DBUGF("enableUserSlot[0x%lx] : 0x%x\r", req.sampleId, req.enable);
          Barcode barcode((char*)req.barcode);
          if(barcodeManager->searchBarcode(barcode, true)) 
          {
            Recommendation recommendation = barcode.getRecommendation();
            if (recommendation == req.result) {
              led->setGood();
              status = 0;  
            } else {
              led->setBad();
              DBUGVAR(req.result);
              DBUGVAR(recommendation);
              DBUG("Verify barcode failed:");
              DBUGVAR((char*)req.barcode);
              status = STATUS_FAILURE;  
            }
            // bool allergy = barcode.getAllergy();

            // DBUGVAR(recommendation);
            // DBUGVAR(allergy);
          }
        }
        else
        {
          DBUGLN("Data length not matched 15");
          status = STATUS_FAILURE;
        }
        band_cmd_base_rsp cmd(BandCommand_VerifyRecommendationRSP, status);

        // DBUGLN("Send Set Enable User Slot Response to Central Device ");
        bleSerial.write((uint8_t *)&cmd, sizeof(band_cmd_base_rsp));
      }
#endif
      break;

      default:
        uint8_t failed = 1;
        bleSerial.write((uint8_t *)&failed, 1);
        DBUGLN("Invalid Command");
        break;
      }
    }
    else
    {
      DBUGLN("Data Length not matched 16");
    }
  }
  else
  {
    //! Not a Band matching command
    //DBUGLN("Not a Band matching command");
    if (BleManagerTask::eBandMatchingState == BM_STATE_CENTRAL_WAIT_FOR_DNA_DATA) {
      DBUGLN("DNA Data Request V2 not recognised. Resend V1 request format.");
      SendDNADataRequestToPeripheralDevice();
    }
  }

  return handled;
}

//! Get 4 Bit DNA Trait from DNA data array
uint8_t BleManagerTask::Get_Next_DNATrait(uint8_t *DNAData, uint8_t index)
{
  if (0 == (index & 0x1)) {
    return ((*(DNAData + index/2) & 0xF0) >> 4);
  }
  else {
    return (*(DNAData + index/2) & 0x0F);
  }
}

//! Search for the device name in Bandmatch History
bool BleManagerTask::SearchBandmatchingHistory(char *deviceName, uint8_t *matchStatus)
{
  uint8_t index = 0;
  bool found = false;

  for (index = 0; index < strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards; index++)
  {
    if (0 == strncmp(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index].peerBandName,
                     deviceName, DEVICE_NAME_SIZE))
    {
      //! Device name is found
      *matchStatus = *strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index].matchStatus;
      found = true;
      break;
    }
  }
  return found;
}
//! Clear Bandmatching History
bool BleManagerTask::ClearBandmatchingHistory(void)
{
  //! Clear Bandmatching History
  memset(&strBandMatchFlashTable.strBandMatchHistory, 0xFF, sizeof(TSBandMatchHistory));
  strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards = 0;

  //! Update Bandmatching History and DNA Data into Flash
  return deviceManager->UpdateBandMatchingHistoryAndDNADataToFlash(&strBandMatchFlashTable);
}

//! Update Bandmatching History
void BleManagerTask::UpdateBandmatchingHistory(char *deviceName, const uint8_t *matchStatus, uint8_t groupCount)
{
  uint8_t index = 0;
  uint32_t time = 0;
  uint8_t prevMatchStatus = 0;
  if (true != dataStore->getTime(time))
  {
    time = 0;
  }

  //! Search in Bandmatch History
  //! if not found then add it to Flash
  if (false == SearchBandmatchingHistory(deviceName, &prevMatchStatus))
  {
    //! Update the Data
    if (MAX_BANDMATCHING_RECORDS > strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards)
    {
      // data not full
      memcpy(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards].matchStatus, matchStatus, groupCount);
      memcpy(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards].peerBandName,
             deviceName, DEVICE_NAME_SIZE);
      strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards].time = time;
      strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards++;
    }
    else
    {
      for (index = 0; index < (MAX_BANDMATCHING_RECORDS - 1); index++)
      {
        memcpy(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index].matchStatus, strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index + 1].matchStatus, sizeof(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index].matchStatus));
        memcpy(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index].peerBandName, strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index + 1].peerBandName, DEVICE_NAME_SIZE);
        strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index].time = strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[index + 1].time;
      }

      memcpy(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[(MAX_BANDMATCHING_RECORDS - 1)].matchStatus, matchStatus, groupCount);
      memcpy(strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[(MAX_BANDMATCHING_RECORDS - 1)].peerBandName,
             deviceName, DEVICE_NAME_SIZE);
      strBandMatchFlashTable.strBandMatchHistory.strBandMatchStatus[(MAX_BANDMATCHING_RECORDS - 1)].time = time;
      strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards++;
    }

    //! Update Bandmatching History and DNA Data into Flash
    deviceManager->UpdateBandMatchingHistoryAndDNADataToFlash(&strBandMatchFlashTable);
  }
  else
  {
    DBUG("Bandmatching Already Done for this Band : ");
    DBUGLN(deviceName);
    DBUG("Prev Bandmatch Status (first group) : ");
    DBUGLN(prevMatchStatus);
    DBUG("Current Bandmatch Status (first group): ");
    DBUGLN(matchStatus[0]);
  }
}

//! Update DNA Data to Flash
bool BleManagerTask::UpdateDNADataIntoFlash(uint8_t *dnaData, uint16_t dnaDataSize)
{
  DBUGLN("Inside setDNAData ...");
  if (0 == dnaData[0]) {
    DBUGLN("Clean DNA Data");
    strBandMatchFlashTable.dnaDataSetInFlash = BAND_DNA_DATA_NOT_SET_IN_FLASH;
    strBandMatchFlashTable.bandDnaDataSize = 0;
    memset(strBandMatchFlashTable.bandDnaData, 0xFF, sizeof(strBandMatchFlashTable.bandDnaData));
  }
  else {
    DBUGLN("Set DNA Data");
    DBUGVAR(dnaDataSize);
    strBandMatchFlashTable.dnaDataSetInFlash = BAND_DNA_DATA_SET_IN_FLASH;
    strBandMatchFlashTable.bandDnaDataSize = dnaDataSize;
    memcpy(strBandMatchFlashTable.bandDnaData, dnaData, dnaDataSize);
  }

  //! Clear Bandmatching History
  memset(&strBandMatchFlashTable.strBandMatchHistory, 0xFF, sizeof(TSBandMatchHistory));
  strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards = 0;

  //! Update Bandmatching History and DNA Data into Flash
  return deviceManager->UpdateBandMatchingHistoryAndDNADataToFlash(&strBandMatchFlashTable);
}

//! Update Sleep Timeout to Flash
bool BleManagerTask::UpdateSleepTimeoutIntoFlash(uint8_t sleepTimeout)
{
  DBUGLN("Inside setSleepTimeout ...");
  DBUGVAR(sleepTimeout);
  
  strBandMatchFlashTable.autoSleepTime = sleepTimeout;
  //! Update Bandmatching History and DNA Data into Flash
  return deviceManager->UpdateBandMatchingHistoryAndDNADataToFlash(&strBandMatchFlashTable);
}

//! Update RSSI Threshold to Flash
bool BleManagerTask::UpdateRSSIThresholdIntoFlash(int8_t rssiThreshold)
{
  DBUGLN("Inside setRSSIThreshold ...");
  DBUGVAR(rssiThreshold);

  strBandMatchFlashTable.rssiThreshold = rssiThreshold;
  //! Update Bandmatching History and DNA Data into Flash
  return deviceManager->UpdateBandMatchingHistoryAndDNADataToFlash(&strBandMatchFlashTable);
}

//! Get Bandmatching History and DNA Data from Flash
bool BleManagerTask::GetBandMatchingHistoryandDNADataFromFlash(TSBandMatchFlashTable *strptrBandMatchFlashTable)
{

  DBUGLN("Inside GetBandMatchingHistory ...");
  //! Wait for Previous Operation completes
  if (flashManager->ReadConfigRegion(BAND_MATCHING_DNA_DATA_REGION_START,
                                    strptrBandMatchFlashTable, sizeof(TSBandMatchFlashTable)))
  {

    DBUGLN("Bandmatch History and DNA Data Read Success");
    return true; // success
  }
  DBUGLN("Bandmatch History and DNA Data Read Failed");
  return false;
}
#endif //BLE_BAND_MATCHING

void BleManagerTask::setDeviceName()
{
  char tempDeviceName[64];
  char IdPrefix[15];
  char BandId[15];

  memset(tempDeviceName, 0, sizeof(tempDeviceName));
  memset(IdPrefix, 0, sizeof(IdPrefix));
  memset(BandId, 0, sizeof(BandId));

  uint8_t status = deviceManager->getBandIdStatus();

  //! If IDPrefix is set then Get IDPrefix
  if (status & IDPREFIX_SET_CODE)
  {
    deviceManager->getIdPrefix(IdPrefix);
  }

  //! If the prefix ID is not set or the size is Zero set the default prefix
  if (0 == strlen(IdPrefix))
  {
    strcpy(IdPrefix, "Nudge");
  }

  //! If the BandID is Set then Get the BandID
  if (status & BANDID_SET_CODE)
  {
    deviceManager->getBandId(BandId);
  }

  //! If the BandId is set and size is nonzero then update bandid with idprefix
  //! else set Device name to idprefix
  if (0 != strlen(BandId))
  {
    #ifdef BLE_PREFIX
      strcpy(tempDeviceName, IdPrefix);
      strcat(tempDeviceName, "-");
      strcat(tempDeviceName, BandId);
    #else
      strcpy(tempDeviceName, BandId);
    #endif
  }
  else
  {
    DBUGLN("<<< Setting BLE Device name to default >>>");
    strcpy(tempDeviceName, IdPrefix);
  }
  
  #ifdef OVERRIDE_BANDID
    strcpy(tempDeviceName, STR(OVERRIDE_BANDID));
  #endif

  //! Update the new device name
  memset(DeviceName, 0, DEVICE_NAME_SIZE + 1);
  strncpy(DeviceName, tempDeviceName, DEVICE_NAME_SIZE);

  DBUG("Device Name : ");
  DBUGLN(DeviceName);

  //! Set BLE Device Name
  bleSerial.setDeviceName(DeviceName);
}

//! Recieve function for Peripheral mode device
void BleManagerTask::onReceive(const uint8_t *buf, size_t size)
{
  bool handled = false;
  bool setStatus = false;
  static bool printOnce = false;
  int op = buf[0];

  size_t maxBlePayloadSize = 0;

#ifdef BLE_BAND_MATCHING
  //! Check whether this is the request/response for the Band matching or not
  handled = onBandMatchReqRspReceive(buf, size);
#else
  if (xTimerIsTimerActive( *sleepAlarm ) != pdFALSE) {
    xTimerReset(*sleepAlarm, 0);
  }
#endif //BLE_BAND_MATCHING

  //! If the messages is not handled by the Band match Req/Rsp function
  //! then it may the mobile app function
  //! handle it here
  if (false == handled)
  {
    DBUGVAR(op);

    if ('A' == op || 'B' == op || 'C' == op || 'E' == op)
    {
      //! Calculate the BLE Payload Size
      //! If the mtu_size is not set from App then set to default MTU size
      if(0 == mtu_size)
      {
        mtu_size = DEFAULT_MTU_SIZE;
      }
      
      maxBlePayloadSize = (mtu_size-3-4);
    
      if (size >= 1)
      {
        uint16_t seq = (uint16_t)buf[1] << 8 | (uint8_t)buf[2];
        responseMsg[0] = buf[1];
        responseMsg[1] = buf[2];
        responseMsg[2] = buf[3];

        DBUGVAR(seq);
        uint8_t count = buf[3];
        DBUGVAR(count);

        const uint8_t *data = buf + 4;

        // Binary message
        if ('C' == op) {
          // Bulk barcode download
          if (size >= 4 + sizeof(uint64_t) * count)
          {
            if (printOnce == false)
            {
              printOnce = true;
              DBUGLN("Operation C");
              DBUG("Size: ");
              DBUG(size);
              DBUG(" Count: ");
              DBUGLN(count);
              #ifdef ENABLE_DEBUG_BLE
              const uint32_t *bc = (uint32_t *)data;
              DBUG(" data[0] = 0x");
              DBUG(bc[0], HEX);
              DBUG(" data[1] = 0x");
              DBUG(bc[1], HEX);
              DBUG(" data[2] = 0x");
              DBUG(bc[2], HEX);
              DBUG(" data[3] = 0x");
              DBUGLN(bc[3], HEX);
              #endif //ENABLE_DEBUG_BLE
            }
            // return;
            // barcodeManager->setBoth((const uint64_t *)data, seq, count);
          }
        }
        else if ('E' == op) {
          DBUGLN("Operation E");
          DBUG("Size: ");
          DBUG(size);
          DBUG("Remaining Count: ");
          DBUGLN(count);
          // count should contain the remaining number of allergies/ambers for a particular seq number
          // current_count contains the number of allergies/ambers in the currrent ble packet
          uint8_t current_count = size - 4;

          if (barcodeManager->readyToReceive) {
            // check the first byte to verify whether it is allergy or lifestyle
            bool isAllergy = ((data[0] >> (ALLERGY_BIT_INDEX+1)) | LIFESTYLE_MASK) == 0xFF;
            if (isAllergy) DBUGVAR(allergySlotIndex);

            // this block is needed for backwards compatibility with previous allergies flow
            if (isAllergy && allergySlotIndex == 0xFF) {
              allergySlotIndex = 0x00;
              deviceManager->setAllergySlotIndex(allergySlotIndex);
              barcodeManager->updateAllergySlotIndex();
              scanner->updateAllergySlotIndex();
              DBUGVAR(allergySlotIndex);
            }
            
            // This check is needed, so that duplicate packets are not processed.
            // The check should be (preAllergyCount != count) but count from the app does not decrease as it should. 
            // (preAllergyCount != current_count) works if the same seq number is split into 2 packets max (no more)
            if (preAllergyCount != current_count) { 
              // Bulk Allergy download
              // Count is from App and counts down.
              barcodeManager->readyToReceive = false;
              isSendReponse = true;
              bool endOfBlock;
              // maxBlePayloadSize = ATT MTU - 3 bytes ATT header - 4 bytes command header
              // this check is used to verify if this is the last packet for a seq number
              if (current_count == maxBlePayloadSize && current_count!=count) { 
                DBUGF("%s\r", isAllergy ? "Start setAllergy" : "Start setLifestyle");
                endOfBlock = false;
                // save the current_count for use next time
                preAllergyCount = current_count;
              } 
              else {
                DBUGF("%s\r", isAllergy ? "Start setAllergy (last packet)" : "Start setLifestyle (last packet)");
                endOfBlock = true;
                // Reset preAllergyCount
                preAllergyCount = 0;
              }
              barcodeManager->setAllergyRecommendations(isAllergy ? WRITE_ALLERGY : WRITE_LIFESTYLE, data, current_count, seq, current_count, endOfBlock);
            }
          }
          else { // (false == barcodeManager->readyToReceive) {
            DBUGLN("Received E Command packet overflow");
          }

          handled = true;
        }
        else if ('B' == op) {
          DBUGLN("Operation B");
          DBUG("Size: ");
          DBUG(size);
          DBUG("Remaining Count: ");
          DBUGLN(count);
          // count should contain the remaining number of allergies/ambers for a particular seq number
          // current contains the number of allergies/ambers in the currrent ble packet
          uint8_t current_count = size - 4;

          if (barcodeManager->readyToReceive) {
            // this check is needed, so that duplicate packets are not processed
            // The check should be (preCount != count) but count from the app does not decrease as it should. 
            // (preCount != current_count) works if the same seq number is split into 2 packets max (no more)
            if (preCount != current_count) {
              // Bulk Allergy download
              // Count is from App and counts down.
              barcodeManager->readyToReceive = false;
              isSendReponse = true;
              bool endOfBlock;
              // maxBlePayloadSize = ATT MTU - 3 bytes ATT header - 4 bytes command header
              // this check is used to verify if this is the last packet for a seq number
              if (current_count == maxBlePayloadSize && current_count!=count) {
                DBUGLN("Start setRecommendations");
                endOfBlock = false;
                // save the current_count for use next time
                preCount = current_count;
              } 
              else {
                DBUGLN("Start setRecommendations (last packet)");
                endOfBlock = true;
                // Reset preCount
                preCount = 0;
              }
              barcodeManager->setAllergyRecommendations(WRITE_RECOMMENDATION, data, current_count, seq, current_count, endOfBlock);
            }
          }
          else { // (false == barcodeManager->readyToReceive) {
            DBUGLN("Received B Command packet overflow");
          }

          handled = true;
        } 
        else if ('A' == op) {
          // Bulk barcode download with connection negotiated MTU = [23, 247]
          // Rx size = [20,244]. 
          // 1-byte op + 2-bytes seq + 1-byte count  = 4 bytes header
          // [16, 240] bytes / 8-bytes = [2,30] barcodes.

          // DBUG("[A] seq = ");
          // DBUG(seq);
          // DBUG(", count = ");
          // DBUG(count);
          // DBUG(", size = ");
          // DBUGLN(size);
          
          if ((size >= 4 + sizeof(uint64_t) * count) && (barcodeManager->readyToReceive)) {
            barcodeManager->readyToReceive = false;
            isSendReponse = true;
            barcodeManager->setBarcodes((const uint64_t *)data, seq, count);
          }
          else if (false == barcodeManager->readyToReceive) {
            DBUGLN("Received A Command packet overflow");
          }
          else {
            DBUGLN("Received A Command packet size wrong");
          }

          handled = true;
        }
      }

      if (isSendReponse && barcodeManager->readyToReceive)
      {
        DBUGLN("Reponse Sent");
        uint16_t resp = (uint16_t)responseMsg[0] << 8 | (uint8_t)responseMsg[1];
        DBUG("response = ");
        DBUGLN(resp);

        bleSerial.write(responseMsg, 3);  
        
        isSendReponse = false;
      }
    }
    else
    {
      // Text message, parse the generic format

      char copy[BLE_ATTRIBUTE_MAX_VALUE_LENGTH + 1];

      // Copy the data so we can corrupt it
      memcpy(copy, buf, size + 1);
      copy[size] = '\0';

      DBUG(F("BleManagerTask::onReceive("));
      DBUG(copy);
      DBUGLN(F(")"));

      char *path = &copy[1];
      char *data = NULL;
      for (size_t i = 1; i < size; i++)
      {
        if (':' == copy[i])
        {
          copy[i] = '\0';
          data = copy + i + 1;
          break;
        }
      }

      DBUGVAR(path);
      DBUGVAR(data);

      if (0 == strncmp("#", path, 1))
      {
        char *barcodeArg = path + 1;
        char *slotArg = NULL;
        char *recommendationArg = data;

        for (size_t i = barcodeArg - copy; i < size; i++)
        {
          if ('/' == copy[i])
          {
            copy[i] = '\0';
            slotArg = &copy[i + 1];
            break;
          }
        }

        DBUGVAR(barcodeArg);
        DBUGVAR(slotArg);
        DBUGVAR(recommendationArg);

        Barcode barcode(barcodeArg);

        // int slot = atoi(slotArg);
        // DBUGVAR(slot);

        switch (op)
        {
        case 'G':
        {
          char ret[] = "U";
          if (dataStore->getBarcode(barcode))
          {
            switch (barcode.getRecommendation())
            {
            case Recommendation_Good:
              ret[0] = '1';
              break;
            case Recommendation_Bad:
              ret[0] = '0';
              break;
            case Recommendation_NotSet:
              ret[0] = 'N';
              break;
            case Recommendation_BadSlotNumber:
              ret[0] = 'S';
              break;
            }
          }
          DBUGVAR(ret);
          bleSerial.write((uint8_t *)ret, 1);
          handled = true;
        }
        break;
        case 'S':
        {
          if (NULL != recommendationArg && '\0' == recommendationArg[1] &&
              ('0' == recommendationArg[0] || '1' == recommendationArg[0]))
          {
            char ret[] = "1";

            Recommendation recommendation = '1' == recommendationArg[0]
                                                ? Recommendation_Good
                                                : Recommendation_Bad;

            barcode.setRecommendation(recommendation);
            if (dataStore->setBarcode(barcode))
            {
              ret[0] = '0';
            }
            DBUGVAR(ret);
            bleSerial.write((uint8_t *)ret, 1);
            handled = true;
          }
        }
        break;
        }
      }
      else if (0 == strncmp("/history", path, 8))
      {
        switch (op)
        {
        case 'G':
          led->setSedAlert(false);
          lifeStyle->isSedAlertAcked = true;
          if (8 == strlen(path))
          {
            uint32_t length;
            uint32_t baseTime;
            if (dataStore->getHistoryInfo(length, baseTime))
            {
              char msg[MAX_MSG_LEN];
              int len = snprintf(msg, sizeof(msg), "%lu:%lu", length, baseTime);
              bleSerial.write((uint8_t *)msg, len);
              handled = true;
            }
          }
          else if (strlen(path) > 9 && '/' == path[8])
          {
            uint32_t index = atoi(path + 9);
            uint32_t offset;
            Barcode barcode;
            HistoryState state;
            uint8_t userIndexes = 0;
            DBUGLN("GetHist");
            if (dataStore->getHistory(index, offset, barcode, state, userIndexes))
            {
              char buy = 'U';
              switch (state)
              {
              case HistoryState_Buy:
                buy = '1';
                break;
              case HistoryState_NotBuy:
                buy = '0';
                break;
              case HistoryState_NotSet:
                buy = 'N';
                break;
              case HistoryState_Unknown:
                buy = 'U';
                break;
              case HistoryState_AmberNotBuy:
                buy = 'P';
                break;
              case HistoryState_AmberBuy:
                buy = 'Q';
                break;
              }
              // TODO: Update correct offset when we have RTC
              offset = 0;
              bool hasUserEnabled = false;
              String msg;
              msg.concat(barcode.toString().c_str());
              msg.concat(":");
              msg.concat(buy);
              msg.concat("{");
              
              for (int userIndex = 0; userIndex < 5; userIndex++) {
                if ((userIndexes & (0x1 << userIndex)) > 0) {
                  uint64_t sampleId = deviceManager->getUserSampleId(userIndex);
                  if (sampleId != UINT64_MAX) {
                    char sid_c[sizeof(uint64_t)+1] = {
                                  (uint8_t)(0xff & (sampleId >> 0)), 
                                  (uint8_t)(0xff & (sampleId >> 8)), 
                                  (uint8_t)(0xff & (sampleId >> 16)), 
                                  (uint8_t)(0xff & (sampleId >> 24)), 
                                  (uint8_t)(0xff & (sampleId >> 32)),
                                  (uint8_t)(0xff & (sampleId >> 40)),
                                  (uint8_t)(0xff & (sampleId >> 48)),
                                  (uint8_t)(0xff & (sampleId >> 56)), 0};
                    msg.concat(sid_c);
                    msg.concat(',');
                    hasUserEnabled = true;
                  }                  
                }
              }
              if (hasUserEnabled) {
                msg.remove(msg.length() - 1);
              }
              msg.concat(String("}"));
              int len = msg.length();
              DBUGVAR(msg);
              int index = 0;
              while(len > mtu_size - 3) {
                  bleSerial.write((uint8_t *)msg.c_str() + index * (mtu_size - 3), mtu_size - 3);
                  index++;
                  len -= (mtu_size - 3);
              }
              if (len > 0) {
                  bleSerial.write((uint8_t *)msg.c_str() + index * (mtu_size - 3), len);
              }
              
              // bleSerial.write((uint8_t *)test, strlen(test));
              handled = true;
                                 
            }
            
          }
          break;
        case 'D':
          char ret[] = "1";

          if (dataStore->clearHistory())
          {
            ret[0] = '0';
          }
          DBUGVAR(ret);
          bleSerial.write((uint8_t *)ret, 1);
          handled = true;
          break;
        }
      }
      else if (0 == strcmp("/time", path))
      {
        uint32_t time;
        switch (op)
        {
        case 'G':
          if (dataStore->getTime(time))
          {
            char msg[MAX_MSG_LEN];
            int len = snprintf(msg, sizeof(msg), "%lu", time-946684800);
            DBUG("Time is ");
            DBUGLN(msg);
            bleSerial.write((uint8_t *)msg, len);
            handled = true;
          }
          break;
        case 'S':
          time = atoi(data) + 946684800;
          char ret[] = "1";
          if (dataStore->setTime(time))
          {
            ret[0] = '0';
          }
          DBUGVAR(ret);
          if (true == lifeStyle->getLifeStyleEnableStatus()) {
            lifeStyle->updateEvaluationCycle();
          }
          bleSerial.write((uint8_t *)ret, 1);
          handled = true;
          break;
        }
      }      
      else if (0 == strncmp("/test/", path, 6))
      {
        char *test = path + 6;
        switch (op)
        {
        case 'G':
          if (0 == strcmp("led", test))
          {

            //DBUGLN("LED Test");
            led->test();
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
          else if (0 == strcmp("scan", test))
          {
            DBUGLN("--BLE Triggers Scanner!--");
            scanner->trigger();
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
          else if (0 == strcmp("cal", test))
          {
            mpu->recalibrate();
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
          break;
        }
      }
      else if (0 == strncmp("/off/", path, 5))
      {
        char *test = path + 5;
        switch (op)
        {
        case 'S':
          if (0 == strcmp("1", strtok(test, "/")))
          {
            // DBUG("Setting mpu variable 1");
            char *tmp = path + 7;
            mpu->setoffset1(tmp);
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
          else if (0 == strcmp("2", strtok(test, "/")))
          {
            char *tmp = path + 7;
            mpu->setoffset2(tmp);
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
        }
      }
      else if (0 == strncmp("/mpu/", path, 5))
      {
        char *test = path + 5;
        switch (op)
        {
        case 'S':
          if (0 == strcmp("1", strtok(test, "/")))
          {
            // DBUG("Setting mpu variable 1");
            char *tmp = path + 7;
            mpu->setarg1(tmp);
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
          else if (0 == strcmp("2", strtok(test, "/")))
          {
            char *tmp = path + 7;
            mpu->setarg2(tmp);
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
          else if (0 == strcmp("3", strtok(test, "/")))
          {
            char *tmp = path + 7;
            bleSerial.write((const uint8_t *)tmp, 5);
            mpu->setarg3(tmp);
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
        }
      }
      else if (0 == strncmp("/flash/", path, 7))
      {
        char *test = path + 7;
        switch (op)
        {
        case 'S':
          if (0 == strcmp("rstBarcodes", test))
          {
            deviceManager->setDbFormatted(true);
            deviceManager->setDatabaseVersion((uint32_t)0xFFFFFF);
            allergySlotIndex = 0xFF;
            deviceManager->setAllergySlotIndex(allergySlotIndex);
            barcodeManager->updateAllergySlotIndex();
            scanner->updateAllergySlotIndex();
            barcodeManager->eraseAllBarcodes();
            bleSerial.write((const uint8_t *)"0", 1);
            handled = true;
          }
        case 'G':
          if (0 == strcmp("isRstDone", test))
          {
            if (!barcodeManager->isErasing())
            {
              bleSerial.write((uint8_t *)"1", 1);
              DBUGLN("1");
            }
            else
            {
              bleSerial.write((uint8_t *)"0", 1);
              DBUGLN("0");
            }
            handled = true;
          }
          #ifdef GENOTYPE_BLE
          else if (0 == strcmp("genotype", test))
          {
            uint8_t index = 0;
            uint8_t ble_DNAData[BAND_DNA_DATA_SIZE];
            uint8_t ble_DNATrait = 0;

            memcpy(ble_DNAData, strBandMatchFlashTable.bandDnaData, BAND_DNA_DATA_SIZE);

            String dnaTrait[BAND_DNA_TRAITS_COUNT] = {
              "Stretch marks",
              "Crow's feet",
              "Eyelid Sagging",
              "Loss of Hydra.",
              "Skin Irrita.",
              "Photoageing",
              "Oxidative Stress",
              "Collagen Degrad.",
              "Deep Wrinkles",
              "Fine lines",
              "Food Addic.",
              "Appetite Ctrl",
              "Alcohol Flush",
              "Sweet Taste",
              "Obesity Risk",
              "Calorie Sensi",
              "Fat Sensi",
              "Sat.Fat Sensi",
              "Carbohydrate Sensi",
              "Sugar Sensi",
              "Salt Sensi",
              "Caffeine Meta"
            };

            String msg = "Cosmetic:\n";
            DBUGLN("Cosmetic:");
            uint16_t offset = 2 * BAND_DNA_DATA_SIZE - BAND_DNA_TRAITS_COUNT;
            for (index = 0; index < BAND_DNA_TRAITS_COUNT_COSMETIC; index++)
            {
              ble_DNATrait = Get_Next_DNATrait(ble_DNAData, index + offset);

              msg.concat("\n");
              msg.concat(dnaTrait[index]);
              msg.concat("(");
              msg.concat((ble_DNATrait & 0x08)>>3);
              msg.concat(")");
              msg.concat(":");
              msg.concat(ble_DNATrait & 0x07);
              DBUGF("%s(%d):%d\r\n",dnaTrait[index].c_str(), (ble_DNATrait & 0x08)>>3, ble_DNATrait & 0x07);
            }

            bleSerial.write((uint8_t *)msg.c_str(), msg.length());

            msg = "Nutrition:\n";
            DBUGLN("Nutrition:");
            offset = 2 * BAND_DNA_DATA_SIZE - BAND_DNA_TRAITS_COUNT + BAND_DNA_TRAITS_COUNT_COSMETIC;
            for (index = 0; index < BAND_DNA_TRAITS_COUNT_NUTRITION; index++)
            {
              ble_DNATrait = Get_Next_DNATrait(ble_DNAData, index + offset);

              msg.concat("\n");
              msg.concat(dnaTrait[index + BAND_DNA_TRAITS_COUNT_COSMETIC]);
              msg.concat("(");
              msg.concat((ble_DNATrait & 0x08)>>3);
              msg.concat(")");
              msg.concat(":");
              msg.concat(ble_DNATrait & 0x07);
              DBUGF("%s(%d):%d\r\n",dnaTrait[index + BAND_DNA_TRAITS_COUNT_COSMETIC].c_str(), (ble_DNATrait & 0x08)>>3, ble_DNATrait & 0x07);
            }

            bleSerial.write((uint8_t *)msg.c_str(), msg.length());

            handled = true;
          }
          break;
          #endif //GENOTYPE_BLE
        }
      }
      else if (0 == strncmp("/dev/", path, 5))
      { // device command decoder
        char *test = path + 5;
        switch (op)
        {
        case 'S':
          if (0 == strncmp("bi/", test, 3))
          { // set band id
            const char *bandid = test + 3;
            DBUGVAR(bandid);

            //! default fail
            setStatus = false;

            if ((0 != strlen(bandid)) && (MAX_BANDID_SIZE >= strlen(bandid)))
            {
              if (true == deviceManager->setBandId(bandid))
              {
                //! success
                setStatus = true;
              }
              else
              {
                DBUGLN("Bandid write in flash failed");
              }
            }
            else
            {
              if (0 == strlen(bandid))
              {
                DBUGLN("Bandid empty");
              }
              else
              {
                DBUG("Bandid too long. Max allowed : ");
                DBUGLN(MAX_BANDID_SIZE);
              }
            }
            //! Write Response
            if (true == setStatus)
            {
              bleSerial.write((const uint8_t *)"0", 1);
              DBUGLN("bandId written");

              //! Reset the System
              resetSystem = true;
            }
            else
            {
              bleSerial.write((const uint8_t *)"1", 1);
              DBUGLN("BandId write failed");
            }

            handled = true;
          }
          else if(0 == strncmp("st/", test, 3))
          { //set sleep time for lifestyle
            char *st = test + 3;
            lifeStyle->setLifeStyleSleepTime(st);
          }
          else if (0 == strncmp("pc/", test, 3))
          { // set passcode
            const char *passcode = test + 3;
            DBUGVAR(passcode);

            //! default fail
            setStatus = false;

            if ((0 != strlen(passcode)) && (MAX_PASSCODE_SIZE >= strlen(passcode)))
            {
              if (true == deviceManager->setPassCode(passcode))
              {
                //! success
                setStatus = true;
              }
              else
              {
                DBUGLN("Passcode write in flash failed");
              }
            }
            else
            {
              if (0 == strlen(passcode))
              {
                DBUGLN("Passcode empty");
              }
              else
              {
                DBUG("Passcode too long. Max allowed : ");
                DBUGLN(MAX_PASSCODE_SIZE);
              }
            }
            //! Write Response
            if (true == setStatus)
            {
              bleSerial.write((const uint8_t *)"0", 1);
              DBUGLN("PassCode written");
            }
            else
            {
              bleSerial.write((const uint8_t *)"1", 1);
              DBUGLN("PassCode write failed");
            }
            handled = true;
          }
          else if (0 == strncmp("pf/", test, 3))
          { // set prefix
            const char *prefix = test + 3;
            DBUGVAR(prefix);

            //! default fail
            setStatus = false;

            if ((0 != strlen(prefix)) && (MAX_IDPREFIX_SIZE >= strlen(prefix)))
            {
              if (true == deviceManager->setIdPrefix(prefix))
              {
                //! success
                setStatus = true;
              }
              else
              {
                DBUGLN("idprefix write in flash failed");
              }
            }
            else
            {
              if (0 == strlen(prefix))
              {
                DBUGLN("idprefix empty");
              }
              else
              {
                DBUG("idprefix too long. Max allowed : ");
                DBUGLN(MAX_IDPREFIX_SIZE);
              }
            }
            //! Write Response
            if (true == setStatus)
            {
              bleSerial.write((const uint8_t *)"0", 1);
              DBUGLN("Prefix written");
              //! Reset the System
              resetSystem = true;
            }
            else
            {
              bleSerial.write((const uint8_t *)"1", 1);
              DBUGLN("Prefix write failed");
            }
            handled = true;
          }
          #ifdef ENABLE_DEBUG_BLE
          else if (0 == strncmp("asi/", test, 4))
          { // set allergy slot index
            const char *allergy_slot_index = test + 4;

            //! default fail
            setStatus = false;

            if (0 != strlen(allergy_slot_index))
            {
              uint8_t index = allergy_slot_index[0] - 48;
              if (true == deviceManager->setAllergySlotIndex(index))
              {
                //! success
                setStatus = true;
                allergySlotIndex = index;
                barcodeManager->updateAllergySlotIndex();
                scanner->updateAllergySlotIndex();
                DBUGVAR(allergySlotIndex);
              }
              else
              {
                DBUGLN("Allergy slot index write in flash failed");
              }
            }
            else
            {
              DBUGLN("allergy slot index empty");
            }
            //! Write Response
            if (true == setStatus)
            {
              bleSerial.write((const uint8_t *)"0", 1);
              DBUGLN("Allergy slot index written");
            }
            else
            {
              bleSerial.write((const uint8_t *)"1", 1);
              DBUGLN("Allergy slot index write failed");
            }
            handled = true;
          }
          #endif
          break;
        case 'G':
          if (0 == strcmp("bi", test))
          {
            int len = 0;
            char bandid[MAX_BANDID_LEN + 1];
            memset(bandid, 0, sizeof(bandid));

            //! If BandID is set in Flash then Read BandID From Flash
            if ((deviceManager->getBandIdStatus() & BANDID_SET_CODE))
            {
              deviceManager->getBandId(bandid);
            }

            //! If it is set and size is non zero then return bandid
            //! else return failure
            len = strlen(bandid);
            if (len > 0)
            {
              bleSerial.write((uint8_t *)bandid, len);
              DBUGVAR(bandid);
              DBUGLN();
            }
            else
            {
              bleSerial.write((uint8_t *)"1", 1);
              DBUGLN("Get BandId failed");
            }

            handled = true;
          }
          else if (0 == strcmp("status", test))
          {
            char msg[MAX_MSG_LEN];
            uint8_t status;
            status = deviceManager->getBandIdStatus();
            int len = snprintf(msg, sizeof(msg), "%d", status);
            DBUGVAR(status);
            bleSerial.write((uint8_t *)msg, len);
            handled = true;
          }
          else if (0 == strcmp("batstat", test))
          { // get battery status
            char batVolt[MAX_MSG_LEN];
            float batteryReading = 0.0;
            memset(batVolt, 0, MAX_MSG_LEN);
            batteryReading = batteryMonitor->ReadBatteryVoltage();
            int len = snprintf(batVolt, sizeof(batVolt), "%2.2f", batteryReading);
            DBUGVAR(batVolt);
            bleSerial.write((uint8_t *)batVolt, len);
            handled = true;
          } else if (0 == strcmp("mtu", test)) {
              #ifdef ADAFRUIT_NRF52_CORE
              mtu_size = bleSerial.getMTU();
              #else
              mtu_size = DEFAULT_MTU_SIZE; //! return default size
              #endif //ADAFRUIT_NRF52_CORE
              DBUGVAR(mtu_size);
              bleSerial.write((uint8_t *)(&mtu_size), 2);
              handled = true;
          }
          #ifdef ENABLE_DEBUG_BLE
          else if (0 == strcmp("asi", test))
          { // get allergy slot index
            DBUGVAR(allergySlotIndex);
            const uint8_t msg = allergySlotIndex + 48;
            bleSerial.write(&msg, 1);
            handled = true;
          }
          #endif
          break;
        case 'R':
          if (0 == strncmp("bi/", test, 3))
          { // reset band id
            char *passcode = test + 3;
            DBUGVAR(passcode);
            if (deviceManager->resetBandId(passcode) == true)
            {
              bleSerial.write((const uint8_t *)"0", 1);

              DBUGLN("BandID Reset Seccessful");

              //! Reset the System
              resetSystem = true;
            }
            else
            {
              bleSerial.write((const uint8_t *)"1", 1);
              DBUGLN("BandID Reset failed");
            }
            handled = true;
          }
          break;
        }
      }
    }
  }
  if (false == handled)
  {
    bleSerial.write((uint8_t *)"error", 5);
  }
}

void BleManagerTask::enabled(bool enable) 
{
  if(this->enable != enable) 
  {
    this->enable = enable;
     if(enable)
    {
      bleSerial.begin();
    }
    else
    {
      bleSerial.end();
    }
  }
}

uint8_t BleManagerTask::getDnaDataSetStatus() {
  return strBandMatchFlashTable.dnaDataSetInFlash;
}

bool BleManagerTask::resetBandMatchingRegionInFlash()
{
  DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);

  memset(&strBandMatchFlashTable, 0xFF, sizeof(TSBandMatchFlashTable));
  strBandMatchFlashTable.dnaDataSetInFlash = BAND_DNA_DATA_NOT_SET_IN_FLASH;
  strBandMatchFlashTable.autoSleepTime = DEFAULT_AUTO_SLEEP_TIME;
  strBandMatchFlashTable.rssiThreshold = BLE_DUAL_RSSI_THRESHOLD;
  strBandMatchFlashTable.bandDnaDataSize = 0;
  strBandMatchFlashTable.strBandMatchHistory.u8NumOfRecards = 0;

  //! Update Bandmatching History and DNA Data into Flash
  return deviceManager->UpdateBandMatchingHistoryAndDNADataToFlash(&strBandMatchFlashTable);
}