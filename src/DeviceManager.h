#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include "FlashManagerTask.h"
#include "config.h"
#include <Arduino.h>

#define MAX_BANDID_LEN 11
#define LOCAL_STORE_SIZE 64                 // 64 bytes local storage
#define MPU_STORE_SIZE LOCAL_STORE_SIZE / 2 // 1/2 of local storage
#define BAND_STATUS_BYTES 2                 // 2 bytes local storage
#define BANDID_SET_MASK 0xFF                // 1st BYTE is band id status
#define BANDID_SET_CODE 0x01                // 1st BYTE is band id status
#define IDPREFIX_SET_CODE 0x02              // 1st BYTE is band id status
#define PASSCODE_SET_CODE 0x04              // 1st BYTE is band id status
#define SAMPLE_ID_SIZE 8
#define CARTRIDGE_ID_SIZE 16
#define TTP_CARTRIDGE_ID_SIZE 10
#define TEST_ID_SIZE 16
#define MAC_ADDRESS_SIZE 6
#define MAX_ALLERGY_SLOT_INDEX 7            // allergies can be written up to 8 times

#define NUDGESHARE_WRITE_MASK 0x3

#define LS_CONF_MAGIC_WORD 0x53CA
#ifdef ENABLE_TEST_LIFESTYLE
#define MAX_PENALTY_POINTS_ALLOWED 3
#else
#define LS_MAX_PENALTY_POINTS_ALLOWED 48   // Max allowed 24 hours
#define LS_DEFAULT_PENALTY_LIMITS 12
#endif
#define LS_PERSONAL_GOAL_4HRS 8
#define LS_PERSONAL_GOAL_6HRS 12
#define LS_PERSONAL_GOAL_8HRS 16

struct DatabaseVersion {
  uint8_t year;
  uint8_t month;
  uint8_t major;
};

struct RecomVersion {
  uint8_t year;
  uint8_t month;
  uint8_t major;
  uint8_t ageGroup:4;
  uint8_t version:4;
};

enum NudgeshareWriteStatus : uint8_t {
  NudgeshareWriteStatus_Incompleted = 0,
  NudgeshareWriteStatus_Enabled,
  NudgeshareWriteStatus_Disabled,
};

struct Common {
  uint8_t dbFormated:1;                       // flag to show whether database formated or not
  uint8_t lifeStyleSyncStatus:1;          // flag to show whether lifestyle active or not
  uint8_t reserved:6;                         // reserved for future use
};

struct UserInfo {
  uint64_t userSampleId;
  RecomVersion recVersion;
  uint8_t enableNudgeshare:2; // value 0 = disable, 1 = enable, 2 = write Incompleted
  uint8_t reserved:6;
};

struct CacheStatus {
  uint8_t dirtyMPUConf:1;
  uint8_t dirtyLSConf:1;
  uint8_t dirtyLSHistory:1;
  uint8_t dirtyDeviceConf:1;
  uint8_t dirtyBMData:1;
  uint8_t dirtyBMHisotry:1;
};

struct BandConfig {
  uint8_t bandIdStatus;
  Common common;                 // config for the band
  char IdPrefix[MAX_BANDID_LEN + 1]; // allow one character for null termination
  char BandId[MAX_BANDID_LEN + 1];   // allow one character for null termination
  char PassCode[MAX_BANDID_LEN + 1]; // allow one character for null termination
  DatabaseVersion dBVersion;          // database version
  UserInfo userInfo[4+1];        // should be same as MAX_SHARED_RECOMMENDATIONS + 1
  uint8_t cartridgeId[CARTRIDGE_ID_SIZE];
  uint8_t testId[TEST_ID_SIZE];
  uint8_t allergySlotIndex;
} __attribute__((packed));

struct LifeStyleConfig {
  uint16_t configStamp; // Mark that LifeStyleRegion is valid
  bool isPenaltyActive;  // is sedentary rule met
  bool isLifeStyleEnabled; // lifestyle master switch
  bool isSleepTimeEnabled; // sleep time master switch.current linked with isLifeStyleEnabled
  uint32_t sleepStartTime;  // sleep start time
  uint32_t sleepEndTime;  // sleep end time
  uint32_t penaltyEndTime; // penalty valid till this time point
  uint32_t evaluationEndTime; // current evaluation end time
  uint8_t penaltyLimits;  // i.e. personal goal
  uint8_t penaltyPoints;  // last recorded penalty points. This will be loaded everytime the band wake up
  uint32_t stepCount;
  uint16_t equivalentSteps;
} __attribute__((packed));

struct LifeStyleHistoryRecord
{
   uint32_t timeStamp; 
   uint8_t penaltyPoints; 
   uint32_t stepCount;
}__attribute__((packed));

struct lifeStyleHistory
{
  uint32_t HistoryCount;
  uint32_t HistoryAvailable;

  struct LifeStyleHistoryRecord record[7];   //7 history records

}__attribute__((packed));

struct MpuConfig {
  uint16_t calibrationStamp;  // Calibration Stamp
  int16_t offset; // Offset: AX, AY, AZ, GX, GY, GZ
};

#define BANDID_STATUS_OFFSET (0)
#define CONFIG_OFFSET (1)
#define IdPrefix_STATUS_OFFSET (2)
#define BANDID_OFFSET (IdPrefix_STATUS_OFFSET + (MAX_BANDID_LEN + 1))
#define PASSCODE_OFFSET (BANDID_OFFSET + (MAX_BANDID_LEN + 1))
#define USER_SLOT_OFFSET (PASSCODE_OFFSET + (MAX_BANDID_LEN + 1))
#define USER_SLOT_ALLOCATED_OFFSET (USER_SLOT_OFFSET + 1)
#define USER_SAMPLE_ID (USER_SLOT_ALLOCATED_OFFSET + 1)
#define DB_VERSION_OFFSET (USER_SAMPLE_ID + 4*6)

enum DeviceManagerState {
  DeviceManagerState_Idle,
  DeviceManagerState_eraseBandId,
  DeviceManagerState_restoreData
};

enum DeviceConfig {
  DeviceConfig_DBFormatted = 0,
  DeviceConfig_DBVersion,
  DeviceConfig_LifeStyleSyncStatus,
  DeviceConfig_NudgeshareEnabled,
  DeviceConfig_UserSampleId,
  DeviceConfig_CartridgeId,
  DeviceConfig_TestId,
  DeviceConfig_AllergySlotIndex,
  DeviceConfig_RecVersion
};

enum ErrorCode {
  ErrorCode_AllergySlotFull,
  ErrorCode_DataLengthNotMatch,
  ErrorCode_UserSlotFull,
  ErrorCode_UserAlreadyExist,
  ErrorCode_UserNotExist,
  ErrorCode_Failed = 0,
  ErrorCode_Success = 1,
};

class DeviceManager {
public:
  DeviceManager(FlashManagerTask *flashManager);
  void setup();
  void loop();
  bool eraseBandId();
  bool getBandId(char *BandId);
  bool getIdPrefix(char *BandId);
  bool setBandId(const char *BandId);
  bool setPassCode(const char *passcode);
  bool setIdPrefix(const char *idPrefix);
  uint8_t getBandIdStatus();
  bool resetBandId(const char *passcode);

  /* Firmware V1.5.0 features >>>START<<< */
  ErrorCode setDeviceConfig(DeviceConfig deviceCfg, uint8_t* value, uint8_t length = 1, uint64_t sampleId = 0);
  ErrorCode setDbFormatted(bool formatted);
  ErrorCode setEnabledUserSlotConfig(uint64_t sampleId, bool enabled);
  ErrorCode setUserSampleId(uint64_t sampleId = 0);
  ErrorCode setCartridgeId(uint8_t* barcode, int length);
  ErrorCode setTestId(uint8_t* testId, int length);
  ErrorCode setAllergySlotIndex(uint8_t allergySlotIndex);
  ErrorCode setDatabaseVersion(uint32_t version);
  ErrorCode setUserRecVersion(uint32_t version, uint64_t sampleID);
  ErrorCode disableAllUserSlotConfig();
  
  bool isLifeStyleSynced();
  ErrorCode setLifeStyleSyncStatus(bool synced);
  bool getDBFormated();
  bool getEnabledUserSlotConfig(uint64_t sampleId = 0);
  uint8_t getEnabledIndexes();
  uint64_t getUserSampleId(int index = 0);
  /* Please allocate 15 bytes array */ 
  bool getCartridgeId(uint8_t* _cartrigdeId);
  /* Please allocate 16 bytes array */ 
  bool getTestId(uint8_t* testId);
  uint32_t getDatabaseVersion();
  uint32_t getUserRecVersion(uint64_t _sampleID);
  uint8_t getAllergySlotIndex();
  /* Firmware V1.5.0 features >>>END<<< */
  
  bool clearUserRelatedData();
  bool eraseUserConfig();
  #ifdef BLE_BAND_MATCHING
  //! Update Bandmatching History And DNA Data to Flash
  bool UpdateBandMatchingHistoryAndDNADataToFlash(TSBandMatchFlashTable *strptrBandMatchFlashTable);
  #endif //BLE_BAND_MATCHING
  uint8_t getUserSlotIndex() { return userSlotIndex; };
  void UpdateMPUCalibration(const int16_t* offset, const uint8_t size);
  void updateLifeStyleRegion(bool resetConfig, bool resetHistory, bool updateHistory, uint32_t timeStamp, uint8_t penaltyPoints, uint32_t stepCount, bool updateConfig);
  
  LifeStyleConfig lifeStyleConfig;
  lifeStyleHistory LifeStyleHistory;

private:
  FlashManagerTask *flashManager;
  DeviceManagerState state;
  int run_itration;
  uint8_t localStore[LOCAL_STORE_SIZE];
  #ifdef BLE_BAND_MATCHING
  uint8_t mpuConfigStore[LOCAL_STORE_SIZE];
  // uint8_t *mpuConfigStore;
  // uint8_t devConfigStore[LOCAL_STORE_SIZE];
  BandConfig bandConfig;
  uint8_t DeviceStatus;
  #endif //BLE_BAND_MATCHING
  uint8_t userSlotIndex;
  uint8_t cartridgeId[CARTRIDGE_ID_SIZE];

  // uint8_t localCache[0x1000]; // 4KB cache
  uint8_t lifeStyleCache[LIFESTYLE_CONFIG_REGION_START+LIFESTYLE_CONFIG_REGION_SIZE+LIFESTYLE_HISTORY_REGION_SIZE];
  // CacheStatus isCacheDirty;
};

#endif // !DEVICE_MANAGER_H