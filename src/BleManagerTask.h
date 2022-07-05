#ifndef BLE_MANAGER_TASK_H
#define BLE_MANAGER_TASK_H

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include "BLESerial.h"
#include "BarcodeManagerTask.h"
#include "BatteryMonitor.h"
#include "DataStore.h"
#include "DeviceManager.h"
#include "LedManagerTask.h"
#include "MpuManagerTask.h"
#include "ScannerTask.h"
#include "ExtRTCTask.h"
#include "LifeStyle.h"

#ifdef BLE_BAND_MATCHING
#include "FlashManagerTask.h"
#endif //BLE_BAND_MATCHING

#define DNANUDGE_BLE_COMPANY_ID   0x0847  // 2119 Decimal

#ifdef BLE_BAND_MATCHING

#define TRAIT_COUNT_MASK    (0x1F)

//! Band matching timeouts
#define MAX_BAND_MATCHING_TIMEOUT        (60000) //! 60 seconds
#define MAX_BAND_MATCH_DNA_DATA_TIMEOUT  (1000)  //! 3 seconds
#define MAX_BAND_MATCH_DNA_DATA_RETRY    10       //! 3 Times
#define MAX_BAND_MATCH_LED_ACK_TIMEOUT   (3000)  //! 3 seconds
#define MAX_BAND_MATCH_LED_ACK_RETRY     3       //! 3 Times

//! DNA Band Matching Request and Response
//! message data types and formats
#define BAND_DNA_MATCH_STATUS_SIZE      1
#define BAND_DNA_DATA_STATUS_SIZE       1
#define BAND_CMD_STATUS_SIZE            1
#define SLEEP_TIMEOUT_SIZE              1
#define RSSI_THRESHOLD_SIZE             1
#define VERSION_NUMBER_SIZE             3
#define USER_SLOT_STATUS_SIZE           1

#define BAND_CMD_SOF_1_POS  0
#define BAND_CMD_SOF_2_POS  1
#define BAND_CMD_LEN_POS    2
#define BAND_CMD_CMD_POS    3
#define BAND_CMD_DATA_POS   4

#define BAND_CMD_HDR_SIZE                    3
#define BAND_CMD_CMDID_SIZE                  1
#define BAND_CMD_STATUS_SIZE                  1
#define BAND_CMD_CRC32_SIZE                  4
#define BAND_CMD_INDEX_SIZE                  1
#define BAND_MATCHING_HISTORY_COUNT_SIZE     1
#define BAND_MATCHING_HISTORY_TIMESTAMP_SIZE 4
#define SLEEP_TIME_SIZE     4
#define PENALTY_LIMITS_SIZE 1

#define BAND_CMD_SOF_VALUE 0xFEFE
#define BAND_CMD_SOF_1_VALUE    0xFE
#define BAND_CMD_SOF_2_VALUE    0xFE

#define BAND_DNA_DATA_NOT_MATCHED   0x00
#define BAND_DNA_DATA_MATCHED       0x01
#define BAND_DNA_OVERALL_MATCHED    0x02
#define BAND_DNA_AMBER_MATCHED      0x03

#define GET_DNA_DATA_STATUS_SUCCESS    0x00
#define GET_DNA_DATA_STATUS_FAILURE    0x01

#define BAND_STEP_COUNT_SIZE     4
#define BAND_PENALTY_POINT_SIZE     1
#define BAND_PENALTY_TIMESTAMP_SIZE    4
#define REQ
#define RSP
#define BC(name, type) BandCommand_##name##type

enum BandCommand {
  BC(BandGetDNAData, REQ) = 1,
  BC(BandGetDNAData, RSP),
  BC(BandUpdateLedStatus, REQ),
  BC(BandUpdateLedStatus, RSP),
  BC(GetDataDNAData, REQ),
  BC(GetDataDNAData, RSP),
  BC(SetDataDNAData, REQ),
  BC(SetDataDNAData, RSP),
  BC(GetBMHistoryCount, REQ),
  BC(GetBMHistoryCount, RSP),
  BC(GetBMHistoryByIndex, REQ),
  BC(GetBMHistoryByIndex, RSP),
  BC(ClearBMHistory, REQ),
  BC(ClearBMHistory, RSP),
  BC(GetSleepTimeout, REQ),
  BC(GetSleepTimeout, RSP),
  BC(SetSleepTimeout, REQ),
  BC(SetSleepTimeout, RSP),  
  BC(GetRssiThreshold, REQ),
  BC(GetRssiThreshold, RSP),
  BC(SetRssiThreshold, REQ),
  BC(SetRssiThreshold, RSP),
  BC(GetVersionNo, REQ),
  BC(GetVersionNo, RSP),
  BC(GetSampleId, REQ),
  BC(GetSampleId, RSP),
  BC(SetSampleId, REQ),
  BC(SetSampleId, RSP),
  BC(GetEnabledUserSlot, REQ),
  BC(GetEnabledUserSlot, RSP),
  BC(SetEnabledUserSlot, REQ),
  BC(SetEnabledUserSlot, RSP),
  BC(ClearUserSlots, REQ),
  BC(ClearUserSlots, RSP),  
  BC(SetUserId, REQ),
  BC(SetUserId, RSP),
  BC(GetDbVersionNo, REQ),
  BC(GetDbVersionNo, RSP),
  BC(SetDbVersionNo, REQ),
  BC(SetDbVersionNo, RSP),
  BC(GetLifeStyleHistCount, REQ),               // 0x29
  BC(GetLifeStyleHistCount, RSP),               // 0x2A
  BC(GetPenaltyHisByIndex, REQ),          // 0x2B
  BC(GetPenaltyHisByIndex, RSP),          // 0x2C
  BC(CleanPenaltyHis, REQ),               // 0x2D
  BC(CleanPenaltyHis, RSP),               // 0x2E
  BC(SetLifestyleSleepTime, REQ),         // 0x2F
  BC(SetLifestyleSleepTime, RSP),         // 0x30
  BC(GetLifestyleSleepTime, REQ),         // 0x31
  BC(GetLifestyleSleepTime, RSP),         // 0x32 
  BC(SetLifestyleSleepTimeEnabled, REQ),  // 0x33
  BC(SetLifestyleSleepTimeEnabled, RSP),  // 0x34
  BC(GetLifestyleSleepTimeEnabled, REQ),  // 0x35
  BC(GetLifestyleSleepTimeEnabled, RSP),  // 0x36
  BC(SetLifestyleEnabled, REQ),           // 0x37
  BC(SetLifestyleEnabled, RSP),           // 0x38
  BC(GetLifestyleEnabled, REQ),           // 0x39
  BC(GetLifestyleEnabled, RSP),           // 0x3A
  BC(SetLifestylePenaltyLimits, REQ),     // 0x3B
  BC(SetLifestylePenaltyLimits, RSP),     // 0x3C
  BC(GetLifestylePenaltyLimits, REQ),     // 0x3D
  BC(GetLifestylePenaltyLimits, RSP),     // 0x3E
  BC(SetLifestyleSyncStatus, REQ),        // 0x3F
  BC(SetLifestyleSyncStatus, RSP),        // 0x40
  BC(GetLifestyleSyncStatus, REQ),        // 0x41
  BC(GetLifestyleSyncStatus, RSP),        // 0x42
  BC(GetLifestyleRunningStatus, REQ),     // 0x43
  BC(GetLifestyleRunningStatus, RSP),     // 0x44
  BC(GetLifestylePenEndTime, REQ),        // 0x45
  BC(GetLifestylePenEndTime, RSP),        // 0x46
  BC(GetSittingStepCount, REQ),           // 0x47
  BC(GetSittingStepCount, RSP),           // 0x48
  BC(GetTestId, REQ),                     // 0x49
  BC(GetTestId, RSP),                     // 0x4A
  BC(GetMacAddress, REQ),                 // 0x4B
  BC(GetMacAddress, RSP),                 // 0x4C
  BC(SetEquivalentSteps, REQ),            // 0x4D
  BC(SetEquivalentSteps, RSP),            // 0x4E
  BC(GetEquivalentSteps, REQ),            // 0x4F
  BC(GetEquivalentSteps, RSP),            // 0x50
  BC(GetAllergySlotIndex, REQ),           // 0x51
  BC(GetAllergySlotIndex, RSP),           // 0x52
  BC(SetAllergySlotIndex, REQ),           // 0x53
  BC(SetAllergySlotIndex, RSP),           // 0x54
  BC(GetUserRecVersion, REQ),           // 0x55
  BC(GetUserRecVersion, RSP),           // 0x56
  BC(SetUserRecVersion, REQ),           // 0x57
  BC(SetUserRecVersion, RSP),           // 0x58
  BC(VirtualNudgeMatch, REQ) = 0x5F,             // 0x5F
  BC(VirtualNudgeMatch, RSP),           // 0x60
  BC(BandGetDNAData_V2, REQ) = 0x61,            
  BC(BandGetDNAData_V2, RSP),
  BC(BandUpdateLedStatus_V2, REQ),
  BC(GetDataDNAData_V2, RSP),
  BC(SetDataDNAData_V2, REQ),
  BC(CleanDNADataNudgeMatch, REQ),
  BC(CleanDNADataNudgeMatch, RSP),
  BC(GetNumberOfLifeStyleHisory_V2,REQ) = 0x68,// 0x68
  BC(GetNumberOfLifeStyleHisory_V2, RSP),      // 0x69
  BC(GetLifeStyleHisoryIndex_V2,REQ),          // 0x6A
  BC(GetLifeStyleHisoryIndex_V2, RSP),         // 0x6B
  BC(CleanLifeStyleHisory_V2,REQ),             // 0x6C
  BC(CleanLifeStyleHisory_V2, RSP),            // 0x6D

#ifdef ENABLE_TEST_RECOMMENDATION
  BC(VerifyRecommendation, REQ) = 0xf0,
  BC(VerifyRecommendation, RSP),
#endif
};

#define BandCommand_BandGetDNADataREQ_SIZE                    (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_BandGetDNADataRSP_SIZE                    (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_DNA_DATA_STATUS_SIZE+BAND_DNA_DATA_SIZE)
#define BandCommand_BandGetDNADataRSP_HDR_SIZE                    (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_DNA_DATA_STATUS_SIZE)
#define BandCommand_BandUpdateLedStatusREQ_SIZE                   (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_DNA_MATCH_STATUS_SIZE+DEVICE_NAME_SIZE)
#define BandCommand_BandUpdateLedStatusRSP_SIZE               (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+DEVICE_NAME_SIZE)
#define BandCommand_GetBMHistoryCountRSP_SIZE             (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_DNA_DATA_STATUS_SIZE+BAND_MATCHING_HISTORY_COUNT_SIZE)
#define APP_CMD_GET_BM_HISTORY_BY_INDEX_SUCCESS_RSP_SIZE  (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+DEVICE_NAME_SIZE+BAND_DNA_DATA_STATUS_SIZE+BAND_MATCHING_HISTORY_TIMESTAMP_SIZE)
#define APP_CMD_GET_BM_HISTORY_BY_INDEX_FAILURE_RSP_SIZE  (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_DNA_DATA_STATUS_SIZE)
#define BandCommand_ClearBMHistoryRSP_SIZE                 (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_DNA_DATA_STATUS_SIZE)
#define BandCommand_GetDataDNADataRSP_SIZE                         (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_STATUS_SIZE+SLEEP_TIMEOUT_SIZE)
#define BandCommand_SetDataDNADataRSP_SIZE                         (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_STATUS_SIZE)
#define BandCommand_GetVersionNoRSP_SIZE               (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_STATUS_SIZE+VERSION_NUMBER_SIZE)
#define BandCommand_GetEnabledUserSlotRSP_SIZE             (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_STATUS_SIZE+USER_SLOT_STATUS_SIZE)
#define BandCommand_GetLifeStyleHistCountREQ_SIZE                    (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetPenaltyHisByIndexREQ_SIZE                    (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_PENALTY_POINT_SIZE)
#define BandCommand_CleanPenaltyHisREQ_SIZE                    (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetLifeStyleHistCountRSP_SIZE                     (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_PENALTY_POINT_SIZE+BAND_STEP_COUNT_SIZE)
#define BandCommand_GetPenaltyHisByIndexRSP_SIZE                (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_PENALTY_POINT_SIZE+BAND_STEP_COUNT_SIZE+BAND_PENALTY_TIMESTAMP_SIZE)        
#define BandCommand_CleanPenaltyHisRSP_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_PENALTY_POINT_SIZE)
#define BandCommand_SetLifestyleSleepTimeREQ_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+SLEEP_TIME_SIZE+SLEEP_TIME_SIZE)
#define BandCommand_GetLifestyleSleepTimeRSP_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_STATUS_SIZE+SLEEP_TIME_SIZE+SLEEP_TIME_SIZE)
#define BandCommand_GetLifestyleSleepTimeREQ_SIZE         (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)  
#define BandCommand_SetLifestyleSleepTimeRSP_SIZE         (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_STATUS_SIZE)
#define BandCommand_SetLifestylePenaltyLimitsREQ_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+PENALTY_LIMITS_SIZE)
#define BandCommand_SetLifestylePenaltyLimitsRSP_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_STATUS_SIZE)
#define BandCommand_GetLifestylePenaltyLimitsREQ_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetLifestylePenaltyLimitsRSP_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+PENALTY_LIMITS_SIZE+BAND_CMD_STATUS_SIZE)
#define BandCommand_GetEquvalentStepsREQ_SIZE                      (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetAllergySlotIndexREQ_SIZE                    (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE) 

#define BandCommand_GetDataDNADataREQ_SIZE                     (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_SetDataDNADataREQ_HDR_SIZE                     (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetBMHistoryCountREQ_SIZE             (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetBMHistoryByIndexREQ_SIZE          (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_MATCHING_HISTORY_COUNT_SIZE)
#define BandCommand_ClearBMHistoryREQ_SIZE                 (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetSleepTimeoutREQ_SIZE                (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_SetSleepTimeoutREQ_SIZE                (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+SLEEP_TIMEOUT_SIZE)
#define BandCommand_GetRssiThresholdREQ_SIZE               (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_SetRssiThresholdREQ_SIZE               (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+RSSI_THRESHOLD_SIZE)
#define BandCommand_GetVersionNoREQ_SIZE               (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetEnabledUserSlotREQ_SIZE             (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_SetEnabledUserSlotREQ_SIZE             (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+USER_SLOT_STATUS_SIZE)
#define BandCommand_CleanDNADataNudgeMatchREQ_SIZE          (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetNumberOfLifeStyleHisoryREQ_V2_SIZE          (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_CleanLifeStyleHisory_V2_SIZE          (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE)
#define BandCommand_GetLifeStyleHisoryIndex_V2_SIZE          (BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+BAND_CMD_INDEX_SIZE)
typedef enum
{
 //! Common State
 BM_STATE_READY,
 BM_STATE_WAIT_FOR_CONNECTION,

 //! Central Mode State
 BM_STATE_CENTRAL_CONNECTED,  //! Band in Central Mode
 BM_STATE_CENTRAL_WAIT_FOR_DNA_DATA,
 BM_STATE_CENTRAL_UPDATE_LED_STATUS,
 BM_STATE_CENTRAL_WAIT_FOR_UPDATE_LED_ACK,

 //! Peripheral Mode State
 BM_STATE_PERIPHERAL_CONNECTED, //! Band in Peripheral Mode
 BM_STATE_PERIPHERAL_UPDATE_DNA_DATA,
 BM_STATE_PERIPHERAL_PROCESS_LED_STATUS,

}TEBandMatchingState;

#endif //BLE_BAND_MATCHING

#define MAX_IDPREFIX_SIZE       5
#define MAX_BANDID_SIZE         5
#define MAX_PASSCODE_SIZE       DEVICE_NAME_SIZE

struct band_cmd_base {
    uint16_t sof;
    uint8_t length;
    uint8_t cmd;
    band_cmd_base() : sof(BAND_CMD_SOF_VALUE) {};
    band_cmd_base(uint8_t cmd) : sof(BAND_CMD_SOF_VALUE), length(BAND_CMD_CMDID_SIZE), cmd(cmd) {      
    };
    band_cmd_base(uint8_t length, uint8_t cmd) : sof(BAND_CMD_SOF_VALUE), length(length+1), cmd(cmd) {      
    };
} __attribute__((packed));

struct band_cmd_dna_data_req {
    band_cmd_base base;
    uint8_t srcDNADataFieldSize;
    band_cmd_dna_data_req(uint8_t cmd, uint8_t srcDNADataFieldSize) : base(1, cmd), srcDNADataFieldSize(srcDNADataFieldSize) {};
} __attribute__((packed));

struct band_cmd_base_rsp {
    band_cmd_base base;
    uint8_t status;
    band_cmd_base_rsp(uint8_t cmd, uint8_t status) : base(band_cmd_base(1, cmd)), status(status) {}
    band_cmd_base_rsp(uint8_t length, uint8_t cmd, uint8_t status) : base(band_cmd_base(length+1, cmd)), status(status) {}
} __attribute__((packed));

struct band_update_led_status_data {
  band_cmd_base base;
  uint8_t status;
  uint8_t deviceName[DEVICE_NAME_SIZE];
  band_update_led_status_data() : base(BAND_DNA_MATCH_STATUS_SIZE + DEVICE_NAME_SIZE, BandCommand_BandUpdateLedStatusREQ) {};
} __attribute__((packed));

struct band_update_led_status_data_v2 {
  band_cmd_base base;
  uint8_t status;
  uint8_t deviceName[DEVICE_NAME_SIZE];
  uint8_t matchScore[BAND_DNA_TRAITS_MAX_GROUP_COUNT];
  band_update_led_status_data_v2(uint8_t length) : base(length, BandCommand_BandUpdateLedStatus_V2REQ) {};
} __attribute__((packed));

struct band_matching_history_rsp {
    band_cmd_base base;
    uint8_t deviceName[DEVICE_NAME_SIZE];
    uint8_t matchScore[BAND_DNA_TRAITS_MAX_GROUP_COUNT];
    uint32_t timestamp;
    band_matching_history_rsp() : base(BAND_CMD_HDR_SIZE+BAND_CMD_CMDID_SIZE+DEVICE_NAME_SIZE+BAND_DNA_TRAITS_MAX_GROUP_COUNT+BAND_MATCHING_HISTORY_TIMESTAMP_SIZE) {};
} __attribute__((packed));

struct band_dna_data {
  band_cmd_base base;
  uint8_t status;
  uint8_t dnaData[BAND_DNA_DATA_SIZE];
  band_dna_data(uint8_t cmd, uint8_t status) : base(BAND_DNA_MATCH_STATUS_SIZE + BAND_DNA_DATA_SIZE, cmd), status(status) {};
} __attribute__((packed));

struct band_dna_data_v2 {
  band_cmd_base base;
  uint8_t status;
  uint8_t dnaData[BAND_MATCHING_DNA_DATA_REGION_SIZE];
  band_dna_data_v2(uint8_t cmd, uint8_t status, uint8_t dnaDataSize) : base(BAND_DNA_MATCH_STATUS_SIZE + dnaDataSize, cmd), status(status) {};
} __attribute__((packed));

struct band_get_sample_id_data {
  band_cmd_base base;
  uint8_t index;
  band_get_sample_id_data() : base(BAND_DNA_MATCH_STATUS_SIZE + 1, BandCommand_GetSampleIdREQ) {};
} __attribute__((packed));

struct band_set_sample_id_data {
  band_cmd_base base;
  uint64_t sampleId;
  band_set_sample_id_data() : base(BAND_DNA_MATCH_STATUS_SIZE + DEVICE_NAME_SIZE, BandCommand_SetSampleIdREQ) {};
} __attribute__((packed));

#ifdef NO_USE
struct history_count_data {
    band_cmd_base base;
    uint8_t status;
    uint8_t count;
} __attribute__((packed));;

struct history_index_data {
    band_cmd_base base;
    char bandId[11];
    uint8_t status;
    uint32_t timestamp;
} __attribute__((packed));;

struct sleep_timeout_rsp_data {
    band_cmd_base base;
    uint8_t status;
    uint8_t value;
} __attribute__((packed));
#endif

struct band_req_get_enable_user_recomm_data {
    band_cmd_base base;
    uint64_t sampleId;
    band_req_get_enable_user_recomm_data() : base(0, 0) {};
} __attribute__((packed));

struct band_req_set_enable_user_recomm_data {
    band_cmd_base base;
    uint8_t enable;
    uint64_t sampleId;
    band_req_set_enable_user_recomm_data() : base(0, 0) {};
} __attribute__((packed));

struct get_sample_id_rsp_data {
  band_cmd_base base;
  uint8_t status;
  uint64_t sampleId;
  get_sample_id_rsp_data(uint8_t cmd, uint8_t _status, uint64_t _sampleId) : base(BAND_CMD_STATUS_SIZE + sizeof(uint64_t), cmd), status(_status), sampleId(_sampleId) {};
} __attribute__((packed));

struct set_lifestyle_sleeping_time_req {
    band_cmd_base base;
    uint32_t start_time;
    uint32_t end_time;
} __attribute__((packed));

struct get_lifestyle_history_rsp {
    band_cmd_base_rsp base;
    uint8_t penaltyPoint;
    uint32_t stepCount;
    uint32_t timestamp;
    get_lifestyle_history_rsp(uint8_t penaltyPoint, uint32_t stepCount, uint32_t timestamp, uint8_t status) : base(0x1, BC(GetLifeStyleHistCount, RSP), status), penaltyPoint(penaltyPoint), stepCount(stepCount), timestamp(timestamp) {}
} __attribute__((packed));

struct get_lifestyle_history_count_rsp {
    band_cmd_base_rsp base;
    uint8_t count;
    get_lifestyle_history_count_rsp(uint8_t count, uint8_t status) : base(0x1, BC(GetLifeStyleHistCount, RSP), status), count(count) {}
} __attribute__((packed));

struct get_lifestyle_sleeping_time_rsp {
    band_cmd_base_rsp base;
    uint32_t start_time;
    uint32_t end_time;
    get_lifestyle_sleeping_time_rsp(uint32_t start_time, uint32_t end_time, uint8_t status) : base(0x8, BC(GetLifestyleSleepTime, RSP), status), start_time(start_time), end_time(end_time) {}
} __attribute__((packed));

struct set_lifestyle_enable_req {
    band_cmd_base base;
    uint8_t enabled;
} __attribute__((packed));

struct get_lifestyle_enable_rsp {
    band_cmd_base_rsp base;
    uint8_t enabled;
    get_lifestyle_enable_rsp(uint8_t enabled, uint8_t status) : base(0x1, BC(GetLifestyleEnabled, RSP), status), enabled(enabled) {}
    get_lifestyle_enable_rsp(uint8_t cmd, uint8_t enabled, uint8_t status) : base(0x1, cmd, status), enabled(enabled) {}
} __attribute__((packed));

struct set_lifestyle_penalty_limits_req {
    band_cmd_base base;
    uint8_t maxPenaltyPoints;
} __attribute__((packed));

struct get_lifestyle_penalty_limits_rsp {
    band_cmd_base_rsp base;
    uint8_t maxPenaltyPoints;
    get_lifestyle_penalty_limits_rsp(uint8_t maxPenaltyPoints, uint8_t status) : base(0x1, BC(GetLifestylePenaltyLimits, RSP), status), maxPenaltyPoints(maxPenaltyPoints) {}
} __attribute__((packed));

struct get_lifestyle_penalty_endtime_rsp {
    band_cmd_base_rsp base;
    uint32_t penaltyEndTime;
    get_lifestyle_penalty_endtime_rsp(uint32_t penaltyEndTime, uint8_t status) : base(0x4, BC(GetLifestylePenEndTime, RSP), status), penaltyEndTime(penaltyEndTime) {}
} __attribute__((packed));

struct get_sitting_step_count_rsp {
    band_cmd_base_rsp base;
    uint8_t penaltyPoint;
    uint8_t personalGoal;
    uint32_t stepCount;
    get_sitting_step_count_rsp(uint8_t penaltyPoint, uint8_t personalGoal, uint32_t stepCount, uint8_t status) : base(0x6, BC(GetSittingStepCount, RSP), status), penaltyPoint(penaltyPoint), personalGoal(personalGoal), stepCount(stepCount) {}
} __attribute__((packed));

struct set_equivalent_step_count_req {
    band_cmd_base base;
    uint16_t equivalentSteps;
} __attribute__((packed));

struct get_equivalent_step_count_rsp {
    band_cmd_base_rsp base;
    uint16_t equivalentSteps;
    get_equivalent_step_count_rsp(uint16_t equivalentSteps, uint8_t status) : base(0x2, BC(GetEquivalentSteps, RSP), status), equivalentSteps(equivalentSteps) {}
} __attribute__((packed));

struct get_test_id_rsp {
    band_cmd_base_rsp base;
    uint8_t testId[TEST_ID_SIZE];
    get_test_id_rsp(uint8_t* _testId, uint8_t status) : base(TEST_ID_SIZE, BC(GetTestId, RSP), status) {
      memcpy(testId, _testId, TEST_ID_SIZE);
    }
} __attribute__((packed));

struct get_db_version_rsp {
    band_cmd_base_rsp base;
    uint8_t year;
    uint8_t month;
    uint8_t major;
    get_db_version_rsp(uint8_t status, uint8_t year, uint8_t month, uint8_t major) : base(0x3, BC(GetDbVersionNo, RSP), status), year(year), month(month), major(major) {}
} __attribute__((packed));

struct set_db_version_req {
    band_cmd_base base;
    uint8_t year;
    uint8_t month;
    uint8_t major;
} __attribute__((packed));

struct get_rec_version_req {
    band_cmd_base base;
    uint64_t sampleID;
} __attribute__((packed));

struct get_rec_version_rsp {
    band_cmd_base_rsp base;
    uint8_t year;
    uint8_t month;
    uint8_t major;
    uint8_t ageGroup:4;
    uint8_t version:4;
    uint64_t sampleID;
    get_rec_version_rsp(uint8_t status, uint8_t year, uint8_t month, uint8_t major, uint8_t ageGroup, uint8_t version, uint64_t sampleID) : base(0xC, BC(GetUserRecVersion, RSP), status), year(year), month(month), major(major), ageGroup(ageGroup), version(version), sampleID(sampleID) {}
} __attribute__((packed));

struct set_rec_version_req {
    band_cmd_base base;
    uint8_t year;
    uint8_t month;
    uint8_t major;
    uint8_t ageGroup:4;
    uint8_t version:4;
    uint64_t sampleID;
} __attribute__((packed));

struct get_mac_address_rsp {
    band_cmd_base_rsp base;
    uint8_t mac_address[MAC_ADDRESS_SIZE];
    get_mac_address_rsp(uint8_t* _mac_address, uint8_t status) : base(MAC_ADDRESS_SIZE, BC(GetMacAddress, RSP), status) {
      memcpy(mac_address, _mac_address, MAC_ADDRESS_SIZE);
    }
} __attribute__((packed));

struct get_allergy_slot_index_rsp {
    band_cmd_base_rsp base;
    uint8_t index;
    get_allergy_slot_index_rsp(uint8_t index, uint8_t status) : base(0x1, BC(GetAllergySlotIndex, RSP), status), index(index) {}
} __attribute__((packed));

struct set_allergy_slot_index_req {
    band_cmd_base base;
    uint8_t index;
} __attribute__((packed));

struct version_rsp_v2 {
    band_cmd_base_rsp base;
    uint8_t major;
    uint8_t minor;
    uint8_t sub;
    uint16_t build;
    version_rsp_v2(uint8_t status, uint8_t major, uint8_t minor, uint8_t sub, uint16_t build) : base(0x5, BC(GetVersionNo, RSP), status), major(major), minor(minor), sub(sub), build(build) {}
} __attribute__((packed));


struct virtual_nudgeMatch_rsp {
    band_cmd_base_rsp base;
    uint8_t matchScore[BAND_DNA_TRAITS_MAX_GROUP_COUNT];
    virtual_nudgeMatch_rsp(uint8_t length, uint8_t status) : base(length, BC(VirtualNudgeMatch, RSP), status) {}
} __attribute__((packed));

struct get_number_of_lifestyle_hisory_V2_rsp {
    band_cmd_base_rsp base;
    uint8_t count;
    get_number_of_lifestyle_hisory_V2_rsp(uint8_t status,uint8_t count ) : base(0x1, BC(GetNumberOfLifeStyleHisory_V2, RSP), status),count(count) {}
} __attribute__((packed));


struct get_lifestyle_hisory_index_V2_rsp {
    band_cmd_base_rsp base;
    uint8_t index;
    uint32_t timestamp;
    uint8_t penaltyPoint;
    uint32_t stepCount;
    get_lifestyle_hisory_index_V2_rsp(uint8_t status,uint8_t index,uint32_t timestamp, uint8_t penaltyPoint, uint32_t stepCount) : base(0x0A, BC(GetLifeStyleHisoryIndex_V2, RSP), status),index(index),timestamp(timestamp),penaltyPoint(penaltyPoint),stepCount(stepCount) {}
} __attribute__((packed));

struct clean_lifestyle_hisory_V2_rsp {
    band_cmd_base_rsp base;

    clean_lifestyle_hisory_V2_rsp(uint8_t status) : base(0x0, BC(CleanLifeStyleHisory_V2, RSP), status){}

} __attribute__((packed));

#ifdef NO_USE
struct rssi_threshold_rsp_data {
    band_cmd_base base;
    uint8_t status;
    int8_t value;
} __attribute__((packed));
#endif

#ifdef ENABLE_TEST_RECOMMENDATION
struct verify_user_recomm_data {
  band_cmd_base base;
  uint8_t barcode[16];
  uint8_t result;
} __attribute__((packed));
#endif


class BleManagerTask {
private:
  BLESerial bleSerial;
  DataStore *dataStore; 
  LedManagerTask *led; 
  ScannerTask *scanner;
  MpuManagerTask *mpu;
  BarcodeManagerTask *barcodeManager;
  TimerHandle_t *sleepAlarm;
  DeviceManager *deviceManager;
  BatteryMonitor *batteryMonitor;
  FlashManagerTask *flashManager;
  LifeStyle *lifeStyle;
  uint8_t preCount;
  uint8_t preAllergyCount;
  uint16_t mtu_size;
  uint8_t allergySlotIndex;
  bool enable;

  bool bPeripheralConnected;

  #ifdef BLE_BAND_MATCHING
  //! To Store Bandmatching State
  static TEBandMatchingState eBandMatchingState;

  //! To Store DNA Match Score
  uint8_t dnaMatchScore[BAND_DNA_TRAITS_MAX_GROUP_COUNT];

  //! To Store the Bandmatch start time
  uint32_t bandMatchStartTime;

  //! To Store Get DNA Data Timout
  uint32_t bandMatchDNADataStartTime;
  uint8_t bandMatchDNADataRetry;

  //! To Store Led Ack Timeout 
  uint32_t bandMatchLedAckStartTime;
  uint8_t bandMatchLedAckRetry;

  //! To Store Bandmatching History and DNA Data
  static TSBandMatchFlashTable strBandMatchFlashTable;

  uint16_t traitGroupCount;
  uint16_t traitCount;
  uint16_t traitStartPos;
  uint16_t activeTraitCount;
  double totalScore;
  #endif //BLE_BAND_MATCHING

  char DeviceName[DEVICE_NAME_SIZE+1];

  bool resetSystem;
  bool isSendReponse;
  uint8_t responseMsg[3];

  // Scheduler interval when disconnected.
  // Wake up time is around 5-10 ms.
  // Needs to adjust according to FIFO rate of MPU6050 to avoid FIFO overflow.
  static const uint IDLE_POLL_INTERVAL = 10; //
  static const uint BUSY_POLL_INTERVAL = 0; // Scheduler interval when connected

  void onReceive(const uint8_t *data, size_t size);
  void setDeviceName();

  #ifdef BLE_BAND_MATCHING
  //! Central Connection Callback
  void BleCentral_connect_callback(void);
  //! Central DisConnection Callback
  void BleCentral_disconnect_callback(void);
  //! Peripheral Connection Callback
  void BlePeripheral_connect_callback(void);
  //! Peripheral DisConnection Callback
  void BlePeripheral_disconnect_callback(void);

  //! Central Receive Callback
  void BleCentral_Receive_callback(const uint8_t *buf, size_t size);
  //! Double Tap Detection Callback
  void DoubleTapDetectedCallback(void);
  //! Handle Band matching Request/Response message
  bool onBandMatchReqRspReceive(const uint8_t *buf, size_t size);
  //! Send Get DNA Data Request to Peripheral Device
  void SendDNADataRequestToPeripheralDevice();
  void SendDNADataRequestToPeripheralDevice_v2();
  //! Send Update LED Status Request to Peripheral Device
  void SendUpdateLEDStatusRequestToPeripheralDevice();
  void SendUpdateLEDStatusRequestToPeripheralDevice_v2();
  //! Send DNA Data Response to Central Device
  void SendDNADataResponseToCentralDevice(uint8_t cmdID, uint8_t getDNADataStatus);
  void SendDNADataResponseToCentralDevice_v2(uint8_t cmdID, uint8_t getDNADataStatus);
  //! Search for the device name in Bandmatch History
  bool SearchBandmatchingHistory(char *deviceName, uint8_t *matchStatus);
  //! Update Bandmatching Status History
  void UpdateBandmatchingHistory(char *deviceName, const uint8_t *matchStatus, uint8_t groupCount);
  
  //! Update Sleep Timeout to Flash
  bool UpdateSleepTimeoutIntoFlash(uint8_t sleepTimeout);
  //! Update RSSI Threshold to Flash
  bool UpdateRSSIThresholdIntoFlash(int8_t rssiThreshold);
  //! Get Bandmatching History from Flash
  bool GetBandMatchingHistoryandDNADataFromFlash(TSBandMatchFlashTable *strptrBandMatchFlashTable);
  //! Read and Restore BandMatching History and DNA Data from Flash
  void RestoreBandMatchingHistoryAndDNADataFromFlash(void);
  //! Check For Bandmatching command timeouts
  void CheckForBandmatchingCommandTimeout(void);
  //! Set Auto Sleep Timeout
  uint8_t setAutoSleepTimemout(uint8_t autoSleepTimeout);
  //! Get 4 Bit DNA Trait from DNA data
  uint8_t Get_Next_DNATrait(uint8_t *DNAData, uint8_t index);
  #endif //BLE_BAND_MATCHING
  
public:
#ifdef BLE_BAND_MATCHING
  BleManagerTask(DataStore *dataStore, LedManagerTask *led,
                 ScannerTask *scanner, MpuManagerTask *mpu,
                 BarcodeManagerTask *barcodeManager,
                 DeviceManager *deviceManager, TimerHandle_t *sleepAlarm,
                 BatteryMonitor *batteryMonitor, FlashManagerTask *flashManager,
                 LifeStyle *lifeStyle);
  //! Update DNA Data to Flash
  bool UpdateDNADataIntoFlash(uint8_t *dnaData, uint16_t dnaDataSize);
  bool resetBandMatchingRegionInFlash();
#else
BleManagerTask(DataStore *dataStore, LedManagerTask *led,
                 ScannerTask *scanner, MpuManagerTask *mpu,
                 BarcodeManagerTask *barcodeManager,
                 DeviceManager *deviceManager, TimerHandle_t *sleepAlarm,
                 BatteryMonitor *batteryMonitor, LifeStyle *LifeStyle);
#endif //BLE_BAND_MATCHING                 
  ~BleManagerTask();

  void setup();
  void enabled(bool enable);
  void StopBandMatching();
  uint8_t getDnaDataSetStatus();

  #ifdef BLE_BAND_MATCHING
  //! Get Bandmatching State
  static TEBandMatchingState getBandmatchingState(void);
  //! Get Bandmatch History Flash Table
  static TSBandMatchFlashTable *getBandmatchHistoryDataFlashTable(void);
  //! Clear Bandmatching History
  bool ClearBandmatchingHistory(void);
  #endif //BLE_BAND_MATCHING
};

#endif //  BLE_MANAGER_TASK_H
