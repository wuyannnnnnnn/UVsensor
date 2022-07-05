//
//  LifeStyle.h
//
//  Created by Wong Kin Yu on 9/18/18.
//  Modified by Song Luan on 11/12/18.
//  Copyright © 2018 Wong Kin Yu. All rights reserved.
//

#ifndef LIFE_STYLE_H
#define LIFE_STYLE_H
#include "config.h"
#include "debug.h"
#include "LedManagerTask.h"
#include "ExtRTCTask.h"
#include "FlashManagerTask.h"
#include "DeviceManager.h"

#undef min
#undef max
#include <functional>
#include <vector>
#include <queue>

using namespace std;

typedef enum : uint8_t {
    LifeStylePersonalGoal4Hrs = 1,
    LifeStylePersonalGoal6Hrs = 1 << 1,
    LifeStylePersonalGoal8Hrs = 1 << 2,
} LifeStylePersonalGoal;

#define LIFE_STYLE_BUFFER_SIZE (40)           // 10 sample points at 10Hz gives 1 second resolution
#define LIFE_STYLE_PREDICTION_COUNT (60)      // evaulation time slice. 60 seconds
#define staticThreshold (0.004)   // Or 0.015
#define slowWalkingThreshold (0.04)   // Or 0.13
#define style_perc (0.50) // active decision threshold requires tuning. 0.5 represents simple majority
#define LIFESTYLE_RECORD_SIZE (9) // timestamp(4) + penaltypoints(1) + stepcount(4)
#define MAX_NUMBER_LIFESTYLE_RECORDS ((LIFESTYLE_HISTORY_REGION_SIZE - 0x10) / LIFESTYLE_RECORD_SIZE) // 0x10 is offset of history starts
#define STEP_REQUIRED_PER_POINT (1000)
#define REFRESH_CYCLE_NUMBER    (40)
#define SAMPLE_DIFF_LOW_LIMIT     (8)
#define SAMPLE_DIFF_HIGH_LIMIT    (80)
#define STEPCOUNT_CORRECTION_STEPS (10)
#define DETECTION_PRECISION (isLowPowerMode?0.04:0.1)
#define ZERO_MOTION_MAX_SAMPLES (5*60)  // in seconds
#define ZERO_MOTION_THRESHOLD (0.00001)
#define ZERO_MOTION_RESTORE_TIME (1000*60*5) // in ms
#define RHYTHM_VIOLATION_TOLERANCE (40)
#define MAX_LIFESTYLE_HISTORY_COUNT 7  //last 7 days' history, index 0-6

enum ActivityType {
    ActivityType_UNKNOWN = -1,
    ActivityType_LOW,
    ActivityType_MOD,
    ActivityType_HIGH,
    ActivityType_SITTING
};

enum LifeStyleType {
    LifeStyleType_Data_Collecting = -1,
    LifeStyleType_Active,
    LifeStyleType_Inactive,
    LifeStyleType_Sitting,
    LifeStyleType_UNKNOWN
};

class BandModeManager;
class BatteryMonitor;
class MpuManagerTask;

class LifeStyle
{

friend BandModeManager;
friend BatteryMonitor;
friend MpuManagerTask;

private:
    LedManagerTask* led;
    ExtRTCTask *rtc;
    BandModeManager *bandMode;
    FlashManagerTask *flashManager;
    DeviceManager *deviceManager;
    TimerHandle_t *LSsleepAlarm;
    TimerHandle_t *LSevalAlarm;
    MpuManagerTask *mpuManager;
    BatteryMonitor *batteryMonitor;

    // uint32_t evaluationEndTime;
    // uint32_t PenaltyValidPeriod;
    // bool isLifeStyleEnabled;
    bool isLifeStyleRunning;
    // bool isSleepTimeEnabled;
    // uint32_t sleepStartTime;
    // uint32_t sleepEndTime;
    // uint8_t historyRegionBuf[0x300];
    // uint8_t penaltyLimits;

    double filtered_sample[3];
    double sample_new;   // New sample 1,2,3 = x,y,z respectively
    double sample_old;   // Old sample 1,2,3 = x,y,z respectively
    uint8_t sample_cnt = 50;
    double euclideanNormInASecond[50];
    double threshold;    // Dynamic threshold
    double min_value = 1;    // Minimum value updated every 50 samples
    double max_value = -1;    // Minimum value updated every 50 samples
    double precision = DETECTION_PRECISION;       // User defined precision.
    uint8_t axis_idx, lock_axis_idx;       // Axis index that have the max difference
    uint16_t temp_stepCnt;
    uint8_t interval_cnt = 0;  // number of samples between two valid steps.
    uint32_t sum_interval = 0;
    uint8_t avg_interval = 0;
    bool isBadStep;     // Mark if a step is an outlier
    bool isFirstSteps = true;
    uint8_t tempStepCount;

    int index;
    // uint8_t countIndex;
    
    // uint8_t mod_count;
    // uint8_t high_count;
    uint8_t predictionIndex;    // data index per second within evaluation time (LIFE_STYLE_PREDICTION_COUNT)

    uint8_t mvpa_count; // in seconds
    uint8_t low_count;  // in seconds
    uint8_t sit_count;  // in seconds

    uint8_t sittingDuration;    // in minutes
    uint8_t activeDuration;     // in minutes
    uint8_t inActiveDuration;     // in minutes
    // uint8_t penaltyPoints;  // 0 to 12.
    // bool isPenaltyActive;
    uint8_t LifeStyleConfWord[2];
    uint16_t zeroMotSampleCnt;
    bool isZeroMotion;
    bool isLowPowerMode;
    uint32_t lastZeroMotionTime = 0;
    double variance;

    bool isSitting; // state described in Level 2 lifestyle implementation
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t _xHigherPriorityTaskWoken = pdFALSE;
    double lowPassFilter(double input, double filterFactor, double previousValue);
    
    void feedAccelerationData(double ax, double ay, double az);
    
    double calculateEuclideanNorm(double xi, double yi, double zi);
    
    void collectEuclideanNorm(double euclideanNorm);

    double sumOfArray(double* values);

    double calculateVariance();
    
    ActivityType determinePedestrianStatusAndStepCount(double variance);
    
    LifeStyleType lifestyle_decision_rules(double mvpa_active_time, double low_active_time, double sit_time);

    void initStepCountAlgorithm();

    void penalty_decision();

    void checkSedRules();

public:
    LifeStyle(LedManagerTask *led, ExtRTCTask *rtc, FlashManagerTask *flashManager, DeviceManager *deviceManager, TimerHandle_t *LSsleepAlarm, TimerHandle_t *LSevalAlarm);
    
    void run(int ax, int ay, int az, bool _isLowPowerMode);

    void updateEvaluationCycle(bool isInISR = false);    // for restarting lifestyle
    void evalAlarmReceived();
    void alarmReceived();    // for handeling RTC alarm
    void sleepAlarmReceived();
    
    uint8_t getPenaltyPoints(); // For scanner task to get current penalty points
    bool getPenaltyStatus();    
    bool getRunningStatus(); // true means lifestyle is running
    void setRunningStatus(bool run, bool resetZeroMotion = false);
    bool getLifeStyleEnableStatus();
    void setLifeStyleEnableStatus(bool _enabled);
    void loadLifeStyleConfigs();
    void saveLifeStyleConfigs();

    bool setSleepingTime(uint32_t _startTime, uint32_t _endTime);
    void getSleepingTime(uint32_t *_startTime, uint32_t *_endTime);
    void enableSleepTime(bool enabled);
    bool getSleepTimeEnabled();
    void resetLifeStyleRegion();

    void eraseLifeStyleHistory();
    void saveLifeStyleHistory( uint32_t timeStamp, uint8_t penaltyPoints, uint32_t stepCount);
    void LoadLifeStyleHistoryRegion(); 
    void getLifeStyleHistory ( uint8_t historyIndex, uint32_t *timestamp, uint8_t *penalty_point, uint32_t *stepCount );
    uint32_t getLifeStyleHistoryCount();

    int8_t isLifeStyleConfigured();
    bool setEquivalentSteps(uint16_t steps);
    uint16_t getEquivalentSteps();

    bool setLifeStyleSleepTime(char *sleeptime);
    void updateRunningStatus(bool updateAlarm = true, bool isInISR = false);
    bool setPenaltyLimits(uint8_t maxPenaltyPoints);
    uint8_t getPenaltyLimits();
    uint8_t getPersonalGoal() { // xxxy yyyy, we only take 3 bits from the penalty point history
        if (deviceManager->lifeStyleConfig.penaltyLimits <= LS_PERSONAL_GOAL_4HRS) {
            return LifeStylePersonalGoal4Hrs;
        } else if (deviceManager->lifeStyleConfig.penaltyLimits > LS_PERSONAL_GOAL_4HRS && deviceManager->lifeStyleConfig.penaltyLimits <= LS_PERSONAL_GOAL_6HRS) {
            return LifeStylePersonalGoal6Hrs;
        } else {
            return LifeStylePersonalGoal8Hrs;
        }
     };

    uint8_t getMaxLifeStyleHistoryCount() { return MAX_LIFESTYLE_HISTORY_COUNT; };
    // void checkEvaulationPeriod(DateTime currentTime);
    void resetDurations() { sittingDuration = 0; activeDuration = 0; inActiveDuration = 0; }
    uint32_t getPenaltyEndTime();

    bool isSedAlertAcked = false;
};

#endif //  LIFE_STYLE_H



