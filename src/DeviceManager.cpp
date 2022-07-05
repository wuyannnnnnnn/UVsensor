#ifndef ENABLE_DEBUG_DEVICE_MANAGER
#undef ENABLE_DEBUG
#endif

#include "DeviceManager.h"
#include "debug.h"
#include "FlashManagerTask.h"

#ifdef BLE_BAND_MATCHING
#include "BleManagerTask.h"
#endif //BLE_BAND_MATCHING
#define XSTR(x) #x
#define STR(x) XSTR(x)

DeviceManager::DeviceManager(FlashManagerTask *flashManager)
    : flashManager(flashManager), DeviceStatus(0xFF), userSlotIndex(0) {
    }

bool DeviceManager::eraseBandId() {
  if (flashManager->ReadConfigRegion(MPU_CONFIG_ADDRESS_START, localStore, MPU_STORE_SIZE)
      && flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig))
      && flashManager->ReadConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache))) 
  // if (0 == isCacheDirty)  // Always make sure cache is clean before erase!
  {
    memset(&bandConfig.bandIdStatus, 0xFF, sizeof(bandConfig.bandIdStatus));
    memset(&bandConfig.IdPrefix, 0xFF, sizeof(bandConfig.IdPrefix)*(MAX_BANDID_LEN + 1));
    memset(&bandConfig.BandId, 0xFF, sizeof(bandConfig.BandId)*(MAX_BANDID_LEN + 1));
    memset(&bandConfig.PassCode, 0xFF, sizeof(bandConfig.PassCode)*(MAX_BANDID_LEN + 1));

    if (false == flashManager->EraseConfigRegion(0, 0x1000)) {
      return false;
    };

    // Restore the data
    if (flashManager->WriteConfigRegion(MPU_CONFIG_ADDRESS_START, localStore, MPU_STORE_SIZE)) {
      #ifdef BLE_BAND_MATCHING
      uint8_t *ptrBandmatchStore = NULL;
      uint32_t remLen = 0, updateLen = 0, offset = 0;
      //! Get Bandmatch History Flash Table 
      ptrBandmatchStore = (uint8_t *)BleManagerTask::getBandmatchHistoryDataFlashTable();

      if(NULL != ptrBandmatchStore)
      {
        //! Write Bandmatching History and DNA Data
        remLen = sizeof(TSBandMatchFlashTable);
        while(remLen != 0)
        {
          if(remLen > MAX_FLASH_WRITE_BYTES)
          {
            updateLen = MAX_FLASH_WRITE_BYTES;
          }
          else
          {
            updateLen = remLen;
          }

          if (flashManager->WriteConfigRegion((BAND_MATCHING_DNA_DATA_REGION_START+offset), (ptrBandmatchStore+offset), updateLen)) 
          {
            //! Flash Write Success
            //! Continue
          }
          else
          {
            //! Flash Write failed
            break;
          }
          offset += updateLen;
          remLen -= updateLen;
        }

        //! Clear the pointer
        ptrBandmatchStore = NULL;
      }

      if (0 == remLen) 
      {
        DBUGLN("Bandmatch History Write Success");
        state = DeviceManagerState_Idle; // write successful
      }
      else
      {
        DBUGLN("Bandmatch History Write Failed");
      }

      // Writeback other regions
      flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig));
      flashManager->WriteConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache));

      #else
      {
        state = DeviceManagerState_Idle; // write successful
      }
      #endif //BLE_BAND_MATCHING
    }
    return true;
  }
  return false;
}

bool DeviceManager::getBandId(char *BandId) {
  DBUGLN("Inside getBandId ...");
  char bandId[MAX_BANDID_LEN + 1];
  memset(bandId, 0, sizeof(bandId));
  
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START + BANDID_OFFSET,
                                    bandId, (MAX_BANDID_LEN + 1))) {
    DBUG("bandId: ");
    DBUGLN(bandId);
    #ifdef OVERRIDE_BANDID
    strcpy(BandId, STR(OVERRIDE_BANDID));
    #else
    if(0 != strlen(bandId))
    {
     strcpy(BandId, bandId);
    }
    #endif
    DBUG("BandId: ");
    DBUGLN(BandId);
    return true; // success
  }
  return false;
}

bool DeviceManager::getIdPrefix(char *IdPrefix) {
  char idPrefix[MAX_BANDID_LEN + 1];
  memset(idPrefix, 0, sizeof(idPrefix));

  DBUGLN("Inside getIdPrefix ...");
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START +
                                        IdPrefix_STATUS_OFFSET,
                                    idPrefix, (MAX_BANDID_LEN + 1))) {
    DBUG("idPrefix: ");
    DBUGLN(idPrefix);
    if(0 != strlen(idPrefix))
    {
      strcpy(IdPrefix, idPrefix);
    }
    return true; // success
  }
  return false;
}

bool DeviceManager::setBandId(const char *BandId) {
  // make sure band has not already been paired
  DBUGLN("Inside setBandId ...");
  DBUG("len BandId: ");
  int len = strlen(BandId);
  if (len > (MAX_BANDID_LEN)) {
    len = (MAX_BANDID_LEN + 1);
  }
  DBUGLN(len);
  DBUG("BandId: ");
  DBUGLN(BandId);
  if ((getBandIdStatus() & BANDID_SET_CODE) == 0 &&
      flashManager->WriteConfigRegion(
          DEVICE_CONFIG_ADDRESS_START + BANDID_OFFSET, BandId, len + 1)) {
    DeviceStatus &= BANDID_SET_CODE ^ BANDID_SET_MASK;
    DBUG("DeviceStatus: ");
    DBUGLN(DeviceStatus);
    if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START +
                                           BANDID_STATUS_OFFSET,
                                       &DeviceStatus, (1))) {
      return true;
    }
  }
  return false;
}

bool DeviceManager::setPassCode(const char *passcode) {
  // make sure band has not already been paired
  DBUGLN("Inside setPassCode ...");
  DBUG("len passcode: ");
  int len = strlen(passcode);
  if (len > (MAX_BANDID_LEN)) {
    len = (MAX_BANDID_LEN + 1);
  }
  DBUGLN(len);
  DBUG("passcode: ");
  DBUGLN(passcode);
  if ((getBandIdStatus() & PASSCODE_SET_CODE) == 0 &&
      flashManager->WriteConfigRegion(
          DEVICE_CONFIG_ADDRESS_START + PASSCODE_OFFSET, passcode, len + 1)) {
    // if passcode successfully written then set the band status id
    DeviceStatus &= PASSCODE_SET_CODE ^ BANDID_SET_MASK;
    DBUG("DeviceStatus: ");
    DBUGLN(DeviceStatus);
    if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START +
                                           BANDID_STATUS_OFFSET,
                                       &DeviceStatus, (1))) {
      return true;
    }
  }
  return false;
}
bool DeviceManager::setIdPrefix(const char *idPrefix) {
  // make sure prefix has not already been paired
  DBUGLN("Inside setIdPrefix ...");
  DBUG("len idPrefix: ");
  int len = strlen(idPrefix);
  if (len > (MAX_BANDID_LEN)) {
    len = (MAX_BANDID_LEN + 1);
  }
  DBUGLN(len);
  DBUG("idPrefix: ");
  DBUGLN(idPrefix);
  if ((getBandIdStatus() & IDPREFIX_SET_CODE) == 0 &&
      flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START +
                                         IdPrefix_STATUS_OFFSET,
                                     idPrefix, len + 1)) {
    DeviceStatus &= IDPREFIX_SET_CODE ^ BANDID_SET_MASK;
    DBUG("DeviceStatus: ");
    DBUGLN(DeviceStatus);
    if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START +
                                           BANDID_STATUS_OFFSET,
                                       &DeviceStatus, (1))) {
      return true;
    }
  }
  return false;
}

uint8_t DeviceManager::getBandIdStatus() {
  DBUGLN("Inside getBandIdStatus ...");
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START +
                                        BANDID_STATUS_OFFSET,
                                    localStore, (1))) {
    return localStore[0] ^ BANDID_SET_MASK; // XOR with BANDID_SET_MASK
  }
  return uint8_t(0);
}

// this method hamdles the reset of the bandid
bool DeviceManager::resetBandId(const char *passcode) {
  DBUGLN("Inside resetBandId ...");
  char rdPassCode[MAX_BANDID_LEN + 1];
  DBUG("passcode: ");
  DBUGLN(passcode);
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START +
                                        PASSCODE_OFFSET,
                                    rdPassCode, (MAX_BANDID_LEN + 1))) {
    // DBUG("localStore: ");
    // DBUGLN((char *)localStore);
    // rdPassCode = (char *)localStore;
    DBUG("rdPassCode: ");
    DBUGLN(rdPassCode);
    if (0 == strncmp(rdPassCode, passcode,
                     strlen(passcode))) { // if passcode matches
      eraseBandId();
      DeviceStatus = 0xFF;
      return true;
    }
  }
  return false;
}

ErrorCode DeviceManager::setDeviceConfig(DeviceConfig deviceCfg, uint8_t* value, uint8_t length, uint64_t sampleId) {
    // First copy the MPU configuration data
    DBUGF("setDeviceConfig[%s]: 0x%x\r", (uint8_t *)&sampleId, value[0]);
    //! Copy Device Configuration Data
    memset(&bandConfig, 0, sizeof(BandConfig));
    if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                    sizeof(BandConfig))) 
    {
      DBUGLN("Read MPUConfig Success");

      // check before write
      switch(deviceCfg) {
        case DeviceConfig_DBFormatted:
          if(bandConfig.common.dbFormated == (value[0] ^ 0x1)) {
            return ErrorCode_Success;
          }
        break;
        case DeviceConfig_DBVersion:
          if (length != 4) {
            return ErrorCode_DataLengthNotMatch;
          }
        break;
        /*case DeviceConfig_LifeStyleSyncStatus:
          if(bandConfig.common.enableLifeStyleInactive == (value[0] ^ 0x1)) {
            return ErrorCode_Success;
          }*/
        break;
        case DeviceConfig_UserSampleId: {
          userSlotIndex = -1;
          bool availableSlot = false;
          uint64_t _sampleId = 0;
          memcpy(&_sampleId, value, SAMPLE_ID_SIZE);
          DBUGF("sampleId:%s\r", (uint8_t *)(&_sampleId));
          for (int index = 0; index < MAX_SHARED_RECOMMENDATIONS+1; index++) {
            if (bandConfig.userInfo[index].userSampleId == _sampleId) {
              // You cannot add same sample Id again, just return user exist
              DBUGVAR(bandConfig.userInfo[index].enableNudgeshare);
              if((bandConfig.userInfo[index].enableNudgeshare ^ NUDGESHARE_WRITE_MASK) != NudgeshareWriteStatus_Incompleted) {
                DBUGLN("User already exist\r");
                return ErrorCode_UserAlreadyExist;
              }
              DBUGLN("User data incompleted\r");
              userSlotIndex = index;
              availableSlot = true;
              break;
            }
            if (bandConfig.userInfo[index].userSampleId == 0xffffffffffffffff) {
              DBUGF("Check UserSampleId[%d]: 0x%s\r", index, (uint8_t *)&value);
              availableSlot = true;
              userSlotIndex = index;
              break;
            }
            DBUGF("memory sampleId:%s\r", (uint8_t *)&bandConfig.userInfo[index].userSampleId);
          }
          if (!availableSlot) {
            return ErrorCode_UserSlotFull;
          }
        } break;
        case DeviceConfig_NudgeshareEnabled: {
          bool userExist = false;
          for (int index = 0; index < MAX_SHARED_RECOMMENDATIONS+1; index++) {
            if (bandConfig.userInfo[index].userSampleId == sampleId) {
              userExist = true;
              break;
            }
          }
          if (!userExist) {
            return ErrorCode_UserNotExist;
          }
        } break;
        case DeviceConfig_CartridgeId:
          if (length == CARTRIDGE_ID_SIZE) {
            bool sameCartridge = true;
            for (int i = 0;i < CARTRIDGE_ID_SIZE;i++) {
              if(bandConfig.cartridgeId[i] != (value[i] & 0xff)) {
                sameCartridge = false;
                break;
              }
            }
            if(sameCartridge) {
              return ErrorCode_Success;
            }
          } else {
            return ErrorCode_DataLengthNotMatch;
          }
        break;
        case DeviceConfig_TestId:
          if (length != TEST_ID_SIZE) {
            return ErrorCode_DataLengthNotMatch;
          }
        break;
        case DeviceConfig_AllergySlotIndex:
          if (value[0] > MAX_ALLERGY_SLOT_INDEX)
            return ErrorCode_AllergySlotFull;
        break;

      }
      memset(mpuConfigStore, 0, LOCAL_STORE_SIZE);
      if (flashManager->ReadConfigRegion(MPU_CONFIG_ADDRESS_START, mpuConfigStore,
                                        LOCAL_STORE_SIZE)) 
      {
        DBUGLN("Read DevConfig Success");
  

        if (flashManager->ReadConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache)))
        {
          DBUGLN("Read LifeStyle Region Success");

          //! Erase Configuration Memory 
          if( flashManager->EraseConfigRegion(MPU_CONFIG_ADDRESS_START, 0x1000))
          {
            DBUGLN("Erase Flash Success");
  
            //! Write MPU Configuration Data
            if (flashManager->WriteConfigRegion(MPU_CONFIG_ADDRESS_START,
                                            mpuConfigStore, LOCAL_STORE_SIZE)) 
            {
              DBUGLN("Write MPUConfig Success");
              switch(deviceCfg) {
                case DeviceConfig_DBFormatted:
                  bandConfig.common.dbFormated = (value[0] ^ 0x1);
                  DBUGF("DBFormated: 0x%x\r", bandConfig.common.dbFormated);
                break;
                case DeviceConfig_DBVersion:
                    bandConfig.dBVersion.year = value[2] & 0xff;
                    bandConfig.dBVersion.month = value[1] & 0xff;
                    bandConfig.dBVersion.major = value[0] & 0xff;  
                break;
                case DeviceConfig_RecVersion: 
                  for (int i=0; i<MAX_SHARED_RECOMMENDATIONS + 1; i++) {
                    if (sampleId == bandConfig.userInfo[i].userSampleId) {
                      bandConfig.userInfo[i].recVersion.year = value[3] & 0xff;
                      bandConfig.userInfo[i].recVersion.month = value[2] & 0xff;
                      bandConfig.userInfo[i].recVersion.major = value[1] & 0xff;
                      bandConfig.userInfo[i].recVersion.ageGroup = (value[0] & 0x30) >> 4;
                      bandConfig.userInfo[i].recVersion.version = value[0] & 0x0f;
                      break;
                    }
                  }
                break;
                case DeviceConfig_LifeStyleSyncStatus:
                  bandConfig.common.lifeStyleSyncStatus = (value[0] ^ 0xf);
                  DBUGF("LifeStyleInactive: 0x%x\r", bandConfig.common.lifeStyleSyncStatus);
                break;
                case DeviceConfig_UserSampleId: {
                  memcpy(&bandConfig.userInfo[userSlotIndex].userSampleId, value, SAMPLE_ID_SIZE);        
                  // Mark it as incompleted           
                  bandConfig.userInfo[userSlotIndex].enableNudgeshare = NudgeshareWriteStatus_Incompleted ^ NUDGESHARE_WRITE_MASK;
                  DBUGF("Add UserSampleId[%d]: %s\r", userSlotIndex, value);
                  DBUGVAR(bandConfig.userInfo[userSlotIndex].enableNudgeshare ^ NUDGESHARE_WRITE_MASK);
                } break;
                case DeviceConfig_NudgeshareEnabled: {
                  for (int index = 0; index < MAX_SHARED_RECOMMENDATIONS+1; index++) {
                    if (bandConfig.userInfo[index].userSampleId == sampleId) {
                      bandConfig.userInfo[index].enableNudgeshare = (value[0] ^ NUDGESHARE_WRITE_MASK);
                      DBUGF("UserSlotEnabled[%d]: 0x%x\r", index, bandConfig.userInfo[index].enableNudgeshare);
                      break;
                    }
                  }
                } break;
                case DeviceConfig_CartridgeId: {
                  memcpy(bandConfig.cartridgeId, value, CARTRIDGE_ID_SIZE);
                } break;
                case DeviceConfig_TestId: {
                  memcpy(bandConfig.testId, value, TEST_ID_SIZE);
                } break;
                case DeviceConfig_AllergySlotIndex: {
                  bandConfig.allergySlotIndex = value[0];
                } break;
              }

              //! Write Device Configuration Data
              if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START,
                                            &bandConfig, sizeof(BandConfig))) 
              {
                //DBUGLN("Write DevConfig Success");
                      //! allocate memory to store Bandmatching history results
                uint8_t* ptrBandmatchStore = NULL;
                ptrBandmatchStore = (uint8_t *)BleManagerTask::getBandmatchHistoryDataFlashTable();

                //DBUG("Bandmatch History Size : ");
                //DBUGLN((sizeof(TSBandMatchFlashTable)+1));

                flashManager->WriteConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache));

                if(NULL != ptrBandmatchStore) {
                  uint32_t remLen = 0, updateLen = 0, offset = 0;
                  remLen = sizeof(TSBandMatchFlashTable);
                  while(remLen != 0)
                  {
                    //DBUGVAR(remLen);

                    if(remLen > MAX_FLASH_WRITE_BYTES)
                    {
                      updateLen = MAX_FLASH_WRITE_BYTES;
                    }
                    else
                    {
                      updateLen = remLen;
                    }
                    
                    //DBUGVAR(updateLen);

                    if (flashManager->WriteConfigRegion((BAND_MATCHING_DNA_DATA_REGION_START+offset), (ptrBandmatchStore+offset), updateLen)) 
                    {
                      //! Flash Write Success
                      //! Continue

                                }
                    else
                    {
                      //! Flash Write failed
                      break;
                    }
                    //DBUGLN("Write Bandmatching data");

                    offset += updateLen;
                    remLen -= updateLen;
                  }

                  if (0 == remLen) 
                  {
                    DBUGLN("Bandmatch History and DNA Data Write Success");
                    return ErrorCode_Success;
                  }
                  else
                  {
                    DBUGLN("Bandmatch History and DNA Data Write Failed");
                  }
                } else {
                  return ErrorCode_Success;
                }
              }

            }
            else
            {
              flashManager->WriteConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache));
            }
            
          }
        }
      }
    } else {
      DBUGLN("Read MPUConfig Failed");
    }
    return ErrorCode_Failed;
}

ErrorCode DeviceManager::setDbFormatted(bool formatted) {
  uint8_t value = formatted ? 0x1 : 0x0;
  return setDeviceConfig(DeviceConfig_DBFormatted, &value, 1);
}

ErrorCode DeviceManager::setEnabledUserSlotConfig(uint64_t sampleId, bool enabled) {
  uint8_t value = enabled ? NudgeshareWriteStatus_Enabled : NudgeshareWriteStatus_Disabled;
  return setDeviceConfig(DeviceConfig_NudgeshareEnabled, &value, 1, sampleId);
}

ErrorCode DeviceManager::setUserSampleId(uint64_t sampleId) {
  uint64_t _sampleId = sampleId;
  return setDeviceConfig(DeviceConfig_UserSampleId, (uint8_t*)&_sampleId, sizeof(uint64_t));
}

ErrorCode DeviceManager::setCartridgeId(uint8_t* barcode, int length) {
#ifdef ENABLE_SAVE_CARID_FLASH
  return setDeviceConfig(DeviceConfig_CartridgeId, barcode, length);
#else
  memcpy(cartridgeId, barcode, CARTRIDGE_ID_SIZE);
  return ErrorCode_Success;
#endif
}

ErrorCode DeviceManager::setTestId(uint8_t* testId, int length) {
  return setDeviceConfig(DeviceConfig_TestId, testId, length);
}

ErrorCode DeviceManager::setDatabaseVersion(uint32_t version) {
  return setDeviceConfig(DeviceConfig_DBVersion, (uint8_t*)&version, sizeof(uint32_t));
}

ErrorCode DeviceManager::setUserRecVersion(uint32_t version, uint64_t sampleID) {
  return setDeviceConfig(DeviceConfig_RecVersion, (uint8_t*)&version, sizeof(uint32_t), sampleID);
}

ErrorCode DeviceManager::setAllergySlotIndex(uint8_t allergySlotIndex) {
  return setDeviceConfig(DeviceConfig_AllergySlotIndex, &allergySlotIndex, sizeof(uint8_t));
}

ErrorCode DeviceManager::disableAllUserSlotConfig() {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
    sizeof(BandConfig))) {
      for (int index = 0; index < MAX_SHARED_RECOMMENDATIONS+1; index++) {
        if ((bandConfig.userInfo[index].enableNudgeshare ^ NUDGESHARE_WRITE_MASK) == NudgeshareWriteStatus_Enabled) {
          bandConfig.userInfo[index].enableNudgeshare = (NudgeshareWriteStatus_Disabled ^ NUDGESHARE_WRITE_MASK);
        }
      }
      return ErrorCode_Success;
    }
  return ErrorCode_Failed;
}

bool DeviceManager::isLifeStyleSynced() {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                        sizeof(BandConfig))) {
                                          DBUGF("LifeStyle Synced : %s\r", (bandConfig.common.lifeStyleSyncStatus ^ 0x1) == 0x1 ? "true" : "false");
                                          return (bandConfig.common.lifeStyleSyncStatus ^ 0x1) == 0x1;
                                        }
    return false;
}

ErrorCode DeviceManager::setLifeStyleSyncStatus(bool synced) {
  return setDeviceConfig(DeviceConfig_LifeStyleSyncStatus, (uint8_t*)&synced, sizeof(bool));
}

bool DeviceManager::getDBFormated() {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig))) {
                                        DBUGF("DBFormated : %s\r", (bandConfig.common.dbFormated ^ 0x1) == 0x1 ? "true" : "false");
                                        return (bandConfig.common.dbFormated ^ 0x1) == 0x1;
                                      }
  return false;
}

bool DeviceManager::getEnabledUserSlotConfig(uint64_t sampleId) {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig))) {
                                        for (int index = 0; index < MAX_SHARED_RECOMMENDATIONS+1; index++) {
                                          if (bandConfig.userInfo[index].userSampleId == sampleId) {
                                              DBUGF("UserSlotEnabled[%d]: 0x%x\r", index, bandConfig.userInfo[index].enableNudgeshare);
                                              return ((bandConfig.userInfo[index].enableNudgeshare ^ NUDGESHARE_WRITE_MASK) == NudgeshareWriteStatus_Enabled);
                                          }
                                        }
                                      }
  return false;
}

uint8_t DeviceManager::getEnabledIndexes() {
  uint8_t indexes = 0;
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig))) {
                                        for (int index = 0; index < MAX_SHARED_RECOMMENDATIONS+1; index++) {
                                          if ((bandConfig.userInfo[index].enableNudgeshare ^ NUDGESHARE_WRITE_MASK) == NudgeshareWriteStatus_Enabled) {
                                              indexes |= 1L << index;
                                          }
                                        }
                                      }
  return indexes;
}

uint64_t DeviceManager::getUserSampleId(int index) {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig)) && index >= 0 && index < MAX_SHARED_RECOMMENDATIONS+1) {
                                        return bandConfig.userInfo[index].userSampleId;
                                      }
  return UINT64_MAX;
}

bool DeviceManager::getCartridgeId(uint8_t* _cartridgeId) {
  #ifdef ENABLE_SAVE_CARID_FLASH
    if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig))) {
                                        memcpy(cartrigdeId, bandConfig.cartridgeId, CARTRIDGE_ID_SIZE);
                                        return true;
                                      }
    return false;
  #else
  memcpy(_cartridgeId, cartridgeId, CARTRIDGE_ID_SIZE); 
  return true;
  #endif
}

bool DeviceManager::getTestId(uint8_t* testId) {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig))) {
                                        memcpy(testId, bandConfig.testId, TEST_ID_SIZE);
                                        return true;
                                      }
  return false;
}

uint32_t DeviceManager::getDatabaseVersion() {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig))) {
                                        uint32_t version = bandConfig.dBVersion.year << 16 | bandConfig.dBVersion.month << 8 | bandConfig.dBVersion.major;
                                        return version;
                                      }
  return 0;
}

uint32_t DeviceManager::getUserRecVersion(uint64_t _sampleID) {
  if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig))) {
    for (int i=0; i<MAX_SHARED_RECOMMENDATIONS + 1; i++) {
      if (_sampleID == bandConfig.userInfo[i].userSampleId) {
        uint32_t version = bandConfig.userInfo[i].recVersion.year << 24 | bandConfig.userInfo[i].recVersion.month << 16 
                        | bandConfig.userInfo[i].recVersion.major << 8 | bandConfig.userInfo[i].recVersion.ageGroup << 4
                        | bandConfig.userInfo[i].recVersion.version;
        return version;
      }
    }
  }
  return 0;
}

uint8_t DeviceManager::getAllergySlotIndex() {
 if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                      sizeof(BandConfig))) {
                                        uint8_t index = bandConfig.allergySlotIndex;
                                        return index;
                                      }
  return 8;  
}

#ifdef BLE_BAND_MATCHING

bool DeviceManager::clearUserRelatedData() {
 
  memset(mpuConfigStore, 0, LOCAL_STORE_SIZE);
  if (flashManager->ReadConfigRegion(MPU_CONFIG_ADDRESS_START, mpuConfigStore,
                                    LOCAL_STORE_SIZE)) 
  {
    DBUGLN("Read MPU Config Success");

    //! Erase Configuration Memory 
    if (flashManager->EraseConfigRegion(MPU_CONFIG_ADDRESS_START, 0x1000))
    {
      DBUGLN("Erase Flash Success");

      //! Write MPU Configuration Data
      if (flashManager->WriteConfigRegion(0, mpuConfigStore, sizeof(mpuConfigStore))) 
      {
        DBUGLN("Write MPUConfig Success");
        return ErrorCode_Success;
      }
    }
  }
  else 
  {
    DBUGLN("Read MPU config Failed");
  }
  return ErrorCode_Failed;  
}

bool DeviceManager::eraseUserConfig() {
    memset(&bandConfig, 0, sizeof(BandConfig));
    if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig,
                                    sizeof(BandConfig))) 
    {
      DBUGLN("Read MPUConfig Success");

      memset(&bandConfig.common, 0xff, sizeof(Common));
      memset(&bandConfig.userInfo, 0xff, sizeof(UserInfo)*5);
      memset(&bandConfig.dBVersion, 0xff, sizeof(DatabaseVersion));
      bandConfig.allergySlotIndex = 0xff;
      memset(mpuConfigStore, 0, LOCAL_STORE_SIZE);
      if (flashManager->ReadConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache)))
      {
        DBUGLN("Read DevConfig Success");
  
        //! Erase Configuration Memory 
        if( flashManager->EraseConfigRegion(MPU_CONFIG_ADDRESS_START, 0x1000))
        {
          DBUGLN("Erase Flash Success");

          //! Write MPU Configuration Data
          if (flashManager->WriteConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache))) 
          {
            DBUGLN("Write LifeStyle and MPUConfig Success");
  
            //! Write Device Configuration Data
            if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START,
                                          &bandConfig, sizeof(BandConfig))) 
            {
              DBUGLN("Write DevConfig Success");
                    return ErrorCode_Success;
            }
          }
        }
        else
        {
          DBUGLN("Erase Configuration Table failed");
        }
      }
      else
      {
          DBUGLN("Read Configuration Table Failed");
      }
    } else {
      DBUGLN("Read MPUConfig Failed");
    }
    return ErrorCode_Failed;
}

//! Update Bandmatching History And DNA Data to Flash
bool DeviceManager::UpdateBandMatchingHistoryAndDNADataToFlash(TSBandMatchFlashTable *strptrBandMatchFlashTable)
{
  bool status = false;
  uint32_t remLen = 0, updateLen = 0, offset = 0;
  uint8_t *ptrBandmatchStore = NULL;

  DBUGLN("Inside UpdateBandMatchingHistoryandDNAData ...");
  DBUG("Bandmatch History Count : ");
  DBUG(strptrBandMatchFlashTable->strBandMatchHistory.u8NumOfRecards);
  DBUGLN();

  //! allocate memory to store Bandmatching history results
  ptrBandmatchStore = (uint8_t *)strptrBandMatchFlashTable;
  
  //DBUG("Bandmatch History Size : ");
  //DBUGLN((sizeof(TSBandMatchFlashTable)+1));

  if(NULL != ptrBandmatchStore)
  {
    // First copy the MPU configuration data
    memset(mpuConfigStore, 0, LOCAL_STORE_SIZE);
    if (flashManager->ReadConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache))) 
    {
      DBUGLN("Read LifeStyle and MPUConfig Success");

      //! Copy Device Configuration Data
      memset(&bandConfig, 0, sizeof(BandConfig));
      if (flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig))) 
      {
        //! Erase Configuration Memory 
        if( flashManager->EraseConfigRegion(MPU_CONFIG_ADDRESS_START, 0x1000))
        {
          //DBUGLN("Erase Flash Success");

          //! Write MPU Configuration Data
          if (flashManager->WriteConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache))) 
          {
            DBUGLN("Write LifeStyle and MPUConfig Success");
  
            //! Write Device Configuration Data
            if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig))) 
            {
              //DBUGLN("Write DevConfig Success");
    
              remLen = sizeof(TSBandMatchFlashTable);
              while(remLen != 0)
              {
                //DBUGVAR(remLen);

                if(remLen > MAX_FLASH_WRITE_BYTES)
                {
                  updateLen = MAX_FLASH_WRITE_BYTES;
                }
                else
                {
                  updateLen = remLen;
                }
                
                //DBUGVAR(updateLen);

                if (flashManager->WriteConfigRegion((BAND_MATCHING_DNA_DATA_REGION_START+offset), (ptrBandmatchStore+offset), updateLen)) 
                {
                  //! Flash Write Success
                  //! Continue
                }
                else
                {
                  //! Flash Write failed
                  break;
                }
                //DBUGLN("Write Bandmatching data");

                offset += updateLen;
                remLen -= updateLen;
              }

              if (0 == remLen) 
              {
                DBUGLN("Bandmatch History and DNA Data Write Success");
                status = true; // success
              }
              else
              {
                DBUGLN("Bandmatch History and DNA Data Write Failed");
              }

            }
            else
            {
              DBUGLN("Write Device Configuration Failed. Cannot Write Bandmatch History and DNA Data");
            }
          }
          else
          {
            DBUGLN("Write LifeStyle and MPUConfig Failed. Cannot Write Bandmatch History and DNA Data");
          }
        }
        else
        {
          DBUGLN("Erase Configuration Memory Failed. Cannot Write Bandmatch History and DNA Data");
        }
      }
      else
      {
        DBUGLN("Read Device Configuration Failed. Cannot Write Bandmatch History and DNA Data");
      }
    }
    else
    {
      DBUGLN("Read MPU Configuration Failed. Cannot Write Bandmatch History and DNA Data");
    }
    //! Clear the Pointer
    ptrBandmatchStore = NULL;
  }
  else
  {
    DBUGLN("Bandmatch History and DNA Data Malloc Failed");
  }

  return status;
}
#endif //BLE_BAND_MATCHING

void DeviceManager::UpdateMPUCalibration(const int16_t* offset, const uint8_t size)
{
    // including mpu, lifestyle(config, history), bandmatching(dna, history)
    flashManager->ReadConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache));
    flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig));

    DBUGLN("Erase 4k Config table");
    // erase the first 4k region
    flashManager->EraseConfigRegion(0, 0x1000);
    memcpy(lifeStyleCache, offset, size);
    uint16_t upperBoundary = LIFESTYLE_HISTORY_REGION_START + 0x10 + 5 *(0xFFFFFFFF - *(uint32_t *)(&lifeStyleCache[0x100]));

    upperBoundary = (upperBoundary >> 2) << 2;
    DBUGVAR(upperBoundary);
    // while(flashManager->isFlash_busy()) {DBUGLN("Flash is busy erasing life style region");}
    flashManager->WriteConfigRegion(0, lifeStyleCache, upperBoundary);

        //! Write Device Configuration Data
    if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig))) 
    {
      //DBUGLN("Write DevConfig Success");

      //! allocate memory to store Bandmatching history results
      uint8_t* ptrBandmatchStore = NULL;
      ptrBandmatchStore = (uint8_t *)BleManagerTask::getBandmatchHistoryDataFlashTable();

      //DBUG("Bandmatch History Size : ");
      //DBUGLN((sizeof(TSBandMatchFlashTable)+1));

      if(NULL != ptrBandmatchStore) {
        uint32_t remLen = 0, updateLen = 0, offset = 0;
        remLen = sizeof(TSBandMatchFlashTable);
        while(remLen != 0)
        {
          //DBUGVAR(remLen);

          if(remLen > MAX_FLASH_WRITE_BYTES)
          {
            updateLen = MAX_FLASH_WRITE_BYTES;
          }
          else
          {
            updateLen = remLen;
          }
          
          //DBUGVAR(updateLen);

          if (flashManager->WriteConfigRegion((BAND_MATCHING_DNA_DATA_REGION_START+offset), (ptrBandmatchStore+offset), updateLen)) 
          {
            //! Flash Write Success
            //! Continue

            }
          else
          {
            //! Flash Write failed
            break;
          }
          //DBUGLN("Write Bandmatching data");

          offset += updateLen;
          remLen -= updateLen;
        }

        if (0 == remLen) 
        {
          DBUGLN("Bandmatch History and DNA Data Write Success");
          // return ErrorCode_Success;
        }
        else
        {
          DBUGLN("Bandmatch History and DNA Data Write Failed");
        }
      } else {
        DBUGLN("Bandmatch History and DNA Data Write Success 2");
        // return ErrorCode_Success;
      }
    }
    else
    {
      DBUGLN("Writing Configure Table failed");
    }
}

void DeviceManager::updateLifeStyleRegion(bool resetConfig, bool resetHistory, bool updateHistory, uint32_t timeStamp, uint8_t penaltyPoints, uint32_t stepCount, bool updateConfig) 
{
    uint16_t upperBoundary= 0;  // The minimum flash memory area needs to be updated. Depending on number of records
    // copy over first 4k region 
    // including mpu, lifestyle(config, history), bandmatching(dna, history)
    flashManager->ReadConfigRegion(0, lifeStyleCache, sizeof(lifeStyleCache));
    flashManager->ReadConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig));

    DBUGLN("Erase 4k Config table");
    // erase the first 4k region
    flashManager->EraseConfigRegion(0, 0x1000);

    if (true == resetConfig || true == resetHistory)
    {
        //Reset behaviour
        if (true == resetConfig) 
        {
            DBUGLN("RESETTING LIFESTYLE CONFIG");
            // reset the lift style region
            memset(lifeStyleCache+0x50, 0xFF, 0xB0);
            // update the required part
            *((uint16_t *)(&lifeStyleCache[0x50])) = LS_CONF_MAGIC_WORD; // Mark that LifeStyleRegion has been reseted
            // lifeStyleCache[0x52] = 0; //isPenaltyActive;
            lifeStyleCache[0x53] = 0; //isLifeStyleEnabled;
            lifeStyleCache[0x54] = 1; //isSleepTimeEnabled;
            *((uint32_t *)(&lifeStyleCache[0x55])) = 22*60*60;  // sleep at 10 PM.
            *((uint32_t *)(&lifeStyleCache[0x59])) = 86400 + 6 * 60 *60;    // wake up at 6 AM.
            // *((uint32_t *)(&lifeStyleCache[0x5D])) = 0xFFFFFFFF; // PenaltyValidPeriod;
            *((uint32_t *)(&lifeStyleCache[0x61])) = 0xFFFFFFFF; // evaluationEndTime
            *((uint8_t *)(&lifeStyleCache[0x65])) = LS_DEFAULT_PENALTY_LIMITS; // Default penalty limits
            *((uint8_t *)(&lifeStyleCache[0x66])) = 0; // Default last penalty points
            *((uint32_t *)(&lifeStyleCache[0x67])) = 0; // Default last step count
            *((uint16_t *)(&lifeStyleCache[0x6B])) = STEP_REQUIRED_PER_POINT; // Default is 2000 steps/penalty point
            upperBoundary = LIFESTYLE_CONFIG_REGION_START + LIFESTYLE_CONFIG_REGION_SIZE;
        }

        if (true == resetHistory) 
        {
            DBUGLN("RESETTING LIFESTYLE HISTORY");
            // reset history region
            memset(lifeStyleCache+0x100, 0xFF, 0x300);
            upperBoundary = LIFESTYLE_HISTORY_REGION_START + LIFESTYLE_HISTORY_REGION_SIZE;
        }
    }
    
    if (true == updateHistory)
    {   
        DBUGLN("ADDING LIFESTYLE HISTORY");
        uint32_t *availableSlots = (uint32_t *)(&lifeStyleCache[0x100]);
        DBUGVAR(*availableSlots);
        // add the record
//        *((uint32_t *)(&lifeStyleCache[0x110 + 5 * ((0xFFFFFFFF - *availableSlots) % MAX_NUMBER_LIFESTYLE_RECORDS) ])) = timeStamp;
//        *(&lifeStyleCache[0x110 + 5 *(0xFFFFFFFF - *availableSlots) + 4])  = penaltyPoints;
//        *(&lifeStyleCache[0x110 + 5 *(0xFFFFFFFF - *availableSlots) + 5])  = stepCount;

        *( (uint32_t *)(&lifeStyleCache[ 0x110 + 9 * ((0xFFFFFFFF - *availableSlots) % MAX_NUMBER_LIFESTYLE_RECORDS) ]) ) = timeStamp;
        *(&lifeStyleCache[ 0x110 + 9 * ((0xFFFFFFFF - *availableSlots) % MAX_NUMBER_LIFESTYLE_RECORDS) + 4 ])  = penaltyPoints;
        *((uint32_t *)(&lifeStyleCache[0x110 + 9 * ((0xFFFFFFFF - *availableSlots) % MAX_NUMBER_LIFESTYLE_RECORDS) + 5]))  = stepCount;
     
        DBUGVAR(timeStamp);
        DBUGVAR(penaltyPoints);
        DBUGVAR(stepCount);
        // update availableSlots
        *availableSlots -= 1;

        if ((0xFFFFFFFF - *availableSlots) < MAX_NUMBER_LIFESTYLE_RECORDS) 
        {
            // No circular happens, boundary is the last record
            upperBoundary = LIFESTYLE_HISTORY_REGION_START + 0x10 + 9 *(0xFFFFFFFF - *availableSlots);
        }
        else
        {
            // Circular overflow, boundary is the whole history region
            upperBoundary = LIFESTYLE_HISTORY_REGION_START + LIFESTYLE_HISTORY_REGION_SIZE;
        }
    }
    
    if (true == updateConfig)
    {
        DBUGLN("SAVING LIFESTYLE CONFIGURATION");
        // update the required part
        *((uint16_t *)(&lifeStyleCache[0x50])) = LS_CONF_MAGIC_WORD; // Mark that LifeStyleRegion is valid
        // lifeStyleCache[0x52] = lifeStyleConfig.isPenaltyActive;
        lifeStyleCache[0x53] = lifeStyleConfig.isLifeStyleEnabled;
        lifeStyleCache[0x54] = lifeStyleConfig.isSleepTimeEnabled;
        *((uint32_t *)(&lifeStyleCache[0x55])) = lifeStyleConfig.sleepStartTime;
        *((uint32_t *)(&lifeStyleCache[0x59])) = lifeStyleConfig.sleepEndTime;
        // *((uint32_t *)(&lifeStyleCache[0x5D])) = lifeStyleConfig.penaltyEndTime;
        *((uint32_t *)(&lifeStyleCache[0x61])) = lifeStyleConfig.evaluationEndTime;
        *((uint8_t *)(&lifeStyleCache[0x65])) = lifeStyleConfig.penaltyLimits;
        *((uint8_t *)(&lifeStyleCache[0x66])) = lifeStyleConfig.penaltyPoints;
        *((uint32_t *)(&lifeStyleCache[0x67])) = lifeStyleConfig.stepCount;
        *((uint16_t *)(&lifeStyleCache[0x6B])) = lifeStyleConfig.equivalentSteps;

        uint32_t *availableSlots = (uint32_t *)(&lifeStyleCache[0x100]);          
        if ((0xFFFFFFFF - *availableSlots) < MAX_NUMBER_LIFESTYLE_RECORDS) 
        {
            // No circular happens, boundary is the last record
            upperBoundary = LIFESTYLE_HISTORY_REGION_START + 0x10 + 9 *(0xFFFFFFFF - *availableSlots);
        }
        else
        {
            // Circular overflow, boundary is the whole history region
            upperBoundary = LIFESTYLE_HISTORY_REGION_START + LIFESTYLE_HISTORY_REGION_SIZE;
        }    
    }

    // Align upperBoundary to 256
    //upperBoundary = (upperBoundary >> 8) << 8;
    DBUGVAR(upperBoundary);
    // while(flashManager->isFlash_busy()) {DBUGLN("Flash is busy erasing life style region");}
    flashManager->WriteConfigRegion(0, lifeStyleCache, upperBoundary);

    // Write back DEVICE CONFIG
    // block = 0;
    // while (block < ((CONFIG_REGION_SIZE - DEVICE_CONFIG_ADDRESS_START) / 0x100)) {
    //     // Break writing into 256Bytes per write
    //     DBUGLN("Writing back DEVICE CONFIG and BAND MATCHING");
    //     flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START + block*256, lifeStyleCache + DEVICE_CONFIG_ADDRESS_START + block*256, 256);
    //     block++;
    //     DBUGVAR(block*256);
    //     // while(flashManager->isFlash_busy()) {DBUGLN("Flash is busy writing");}
    // }
    //! Write Device Configuration Data
    if (flashManager->WriteConfigRegion(DEVICE_CONFIG_ADDRESS_START, &bandConfig, sizeof(BandConfig))) 
    {
      DBUGLN("Write DevConfig Success");

      //! allocate memory to store Bandmatching history results
      uint8_t* ptrBandmatchStore = NULL;
      ptrBandmatchStore = (uint8_t *)BleManagerTask::getBandmatchHistoryDataFlashTable();

      //DBUG("Bandmatch History Size : ");
      //DBUGLN((sizeof(TSBandMatchFlashTable)+1));

      if(NULL != ptrBandmatchStore) {
        uint32_t remLen = 0, updateLen = 0, offset = 0;
        remLen = sizeof(TSBandMatchFlashTable);
        while(remLen != 0)
        {
          //DBUGVAR(remLen);

          if(remLen > MAX_FLASH_WRITE_BYTES)
          {
            updateLen = MAX_FLASH_WRITE_BYTES;
          }
          else
          {
            updateLen = remLen;
          }
          
          //DBUGVAR(updateLen);

          if (flashManager->WriteConfigRegion((BAND_MATCHING_DNA_DATA_REGION_START+offset), (ptrBandmatchStore+offset), updateLen)) 
          {
            //! Flash Write Success
            //! Continue

            }
          else
          {
            //! Flash Write failed
            break;
          }
          //DBUGLN("Write Bandmatching data");

          offset += updateLen;
          remLen -= updateLen;
        }

        if (0 == remLen) 
        {
          DBUGLN("Bandmatch History and DNA Data Write Success");
          // return ErrorCode_Success;
        }
        else
        {
          DBUGLN("Bandmatch History and DNA Data Write Failed");
        }
      } else {
        DBUGLN("Bandmatch History and DNA Data Write Success 2");
        // return ErrorCode_Success;
      }
    }
    else
    {
      DBUGLN("Writing Configure Table failed");
    }
}
