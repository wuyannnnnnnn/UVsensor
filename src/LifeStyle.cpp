//
//  LifeStyle.h
//
//  Created by Wong Kin Yu on 9/18/18.
//  Modified by Song Luan on 11/12/18.
//  Copyright © 2018 Wong Kin Yu. All rights reserved.
//
#ifndef ENABLE_DEBUG_LIFESTYLE
#undef ENABLE_DEBUG
#endif

#define MAX(a, b)         (((a) > (b)) ? (a) : (b))
#define MIN(a, b)         (((a) < (b)) ? (a) : (b))

#include "LifeStyle.h"
#include "BandModeManager.h"
#include "BatteryMonitor.h"

double LifeStyle::lowPassFilter(double input, double filterFactor, double previousValue) {
    double firstSection = previousValue * filterFactor / 100.0;
    double secondSection = (input * (1 - filterFactor / 100.0));
    return firstSection + secondSection;
}

void LifeStyle::feedAccelerationData(double ax, double ay, double az) {
    filtered_sample[0] = lowPassFilter(ax, 15, filtered_sample[0]);
    filtered_sample[1] = lowPassFilter(ay, 15, filtered_sample[1]);
    filtered_sample[2] = lowPassFilter(az, 15, filtered_sample[2]);
    // DBUG("x,y,z:");
    // DBUG(filteredX);
    // DBUG(",");
    // DBUG(filteredY);
    // DBUG(",");
    // DBUGLN(filteredZ);
    double euclideanNorm = calculateEuclideanNorm(filtered_sample[0], filtered_sample[1], filtered_sample[2]);
    collectEuclideanNorm(euclideanNorm);

    if (false == getRunningStatus()) return;

    // For step Count
    if (0 == sample_cnt) {
        // Find max and min value
        max_value = MAX(max_value, euclideanNorm);
        min_value = MIN(max_value, euclideanNorm); 
        threshold = (max_value + min_value) / 2;
        // DBUGF("MIN = %f, MAX = %f, THR = %f \r\n", min_value, max_value, threshold);
        // Reset temporary buffer
        max_value = -1;
        min_value = 1; 
        sample_old = threshold;
        sample_new = threshold;
        sample_cnt = REFRESH_CYCLE_NUMBER;
    }
    else {
        sample_cnt--;
        max_value = MAX(max_value, euclideanNorm);
        min_value = MIN(max_value, euclideanNorm); 
    }

    interval_cnt++;

    // Decide if sample new needs update
    if (abs(euclideanNorm - sample_new) > DETECTION_PRECISION) {
        // change is larger than precision required 
        double diff_value = 0;
        if (abs(euclideanNorm - sample_new) > diff_value) {
            diff_value = abs(euclideanNorm - sample_new);
        }
        sample_old = sample_new;
        sample_new = euclideanNorm;

        // DBUGF("axis = %d | lock_axis = %d | %f | %f | %f | %f | %d \r\n", axis_idx, lock_axis_idx, sample_new[axis_idx], threshold[axis_idx], sample_old[axis_idx], sample_new[axis_idx] - sample_old[axis_idx], deviceManager->lifeStyleConfig.stepCount);

        if ((sample_new <= threshold) && (threshold <= sample_old)) 
        {
            if (SAMPLE_DIFF_LOW_LIMIT <= interval_cnt && interval_cnt <= SAMPLE_DIFF_HIGH_LIMIT) {
                if (true == isFirstSteps) {
                    sum_interval += interval_cnt;
                    if (STEPCOUNT_CORRECTION_STEPS <= tempStepCount) {
                        deviceManager->lifeStyleConfig.stepCount+=tempStepCount;
                        isFirstSteps = false;
                        avg_interval = sum_interval / STEPCOUNT_CORRECTION_STEPS;
                    }
                    else {
                        tempStepCount++;
                    }
                }
                else if (abs(interval_cnt - avg_interval) < RHYTHM_VIOLATION_TOLERANCE){
                    DBUGLN("TRUE STEP");
                    deviceManager->lifeStyleConfig.stepCount++;
                    avg_interval = (avg_interval + interval_cnt) / 2;
                }
                else {
                    DBUGF("Violate rhythm %d vs %d(avg) \r\n", interval_cnt, avg_interval);
                    isFirstSteps = true;
                    tempStepCount = 0;
                }
                DBUGF("Rthym %d vs %d(avg) \r\n", interval_cnt, avg_interval);
                interval_cnt = 0;
            }
            else {
                DBUGLN("BAD STEP!");
                DBUGVAR(interval_cnt);
                if (interval_cnt > SAMPLE_DIFF_HIGH_LIMIT || STEPCOUNT_CORRECTION_STEPS - 1 == tempStepCount) {
                    isFirstSteps = true;
                    tempStepCount = 0;
                    sum_interval = 0;
                }
                interval_cnt = 0;
                
            }
        }
        DBUGF("Steps = %d(%d) \r\n", deviceManager->lifeStyleConfig.stepCount, tempStepCount);
    }
    else {
        // change is not large enough
        sample_old = sample_new;
    }
}

double LifeStyle::calculateEuclideanNorm(double xi, double yi, double zi) {
    //'''This function calculates the magnitude of the acceleration vector (xi,yi,zi).
    //since the modulus corresponds to the L2-norm of the data, it rectifies the signal
    //(as the calculation corresponds to the sum of the absolute values of each axis point)'''
    
    return sqrt((xi*xi)+(yi*yi)+(zi*zi));
}

void LifeStyle::collectEuclideanNorm(double euclideanNorm) {
    euclideanNormInASecond[index] = euclideanNorm;
    index++;
    // DBUGVAR(index);

    if ( LIFE_STYLE_BUFFER_SIZE <= index) {
        index = 0; 
        if (true == isLowPowerMode) {
            // Check if the lifestyle is not running due to other reason
            // If yes, then do not evaluate zero motion
            if (false == getRunningStatus() && false == isZeroMotion) {
                zeroMotSampleCnt = 0;
                return;
            }

            variance = calculateVariance();
            // index=0;
            if (false == isZeroMotion) {
                // Not zero motion
                if (variance <= ZERO_MOTION_THRESHOLD) {            
                    zeroMotSampleCnt++;
                }
                else {
                    zeroMotSampleCnt = 0;
                }
                DBUGVAR(zeroMotSampleCnt);

                if (zeroMotSampleCnt >= ZERO_MOTION_MAX_SAMPLES) {
                    isZeroMotion = true;
                    setRunningStatus(false);
                    lastZeroMotionTime = millis();
                    DBUGLN("No motion detected for 5 minutes");
                }
            }
            else {
                // Already zero motion
                if (variance > ZERO_MOTION_THRESHOLD) {      
                    zeroMotSampleCnt = 0;
                    DBUGLN("Motion detected");
                    updateRunningStatus(false,false);
                    isZeroMotion = false;
                    // lifeStyle->setRunningStatus(true);
                    if (millis() - lastZeroMotionTime >= ZERO_MOTION_RESTORE_TIME) {
                        resetDurations();
                    }
                }
                else {
                    // Do not do further processing if currently is in zero motion state
                    return;
                }
            }
        }
        else {
            variance = calculateVariance();
        }

        ActivityType activityType = determinePedestrianStatusAndStepCount(variance);
        for (int j=0;j<LIFE_STYLE_BUFFER_SIZE;j++)
        {
            euclideanNormInASecond[j] = 0.0; 
        }  
        switch(activityType) {
            case ActivityType_HIGH:
            case ActivityType_MOD:
                mvpa_count++;
                // DBUGVAR(mvpa_count);
            break;
            case ActivityType_LOW:
                low_count++;
                // DBUGVAR(low_count);
            break;
            case ActivityType_SITTING:
                sit_count++;
                // DBUGVAR(sit_count);
            break;
            case ActivityType_UNKNOWN:
                DBUGLN("ActivityType_UNKNOWN");
            break;
        };
        predictionIndex++;
        // DBUGVAR(predictionIndex);
        if (predictionIndex >= LIFE_STYLE_PREDICTION_COUNT) {
            double mvpaActivePercent = (mvpa_count / (double)LIFE_STYLE_PREDICTION_COUNT) * 100.0;
            double lowActivePercent = (low_count / (double)LIFE_STYLE_PREDICTION_COUNT) * 100.0;
            double sitPercent = (sit_count / (double)LIFE_STYLE_PREDICTION_COUNT) * 100.0;
            LifeStyleType led_status = lifestyle_decision_rules(mvpaActivePercent, lowActivePercent, sitPercent);
            DBUG("LifeStyleType:");
            switch(led_status)
            {
                case LifeStyleType_Active:
                    activeDuration++;
                    if (isSitting) {
                        if (activeDuration + inActiveDuration >= 2) {
                            // sittingDuration = 0;
                            inActiveDuration = 0;
                            isSitting = false;  // State transition
                            DBUGLN("State is active");
                        }
                        else {
                            sittingDuration++;
                            checkSedRules();
                        }
                    }
                    else {
                        // sittingDuration = 0;
                        if (activeDuration % 5 == 0) {saveLifeStyleConfigs();}
                    }
                    DBUGLN("Active");
                break;
                case LifeStyleType_Inactive:
                    inActiveDuration++;
                    if (isSitting) {
                        if (activeDuration + inActiveDuration >= 2) {
                            activeDuration = 0;
                            // sittingDuration = 0;
                            isSitting = false;  // State transition
                            DBUGLN("State is static");
                        }
                        else {
                            sittingDuration++;
                            checkSedRules();
                        }
                    }
                    else {
                        // sittingDuration = 0;
                    }
                    DBUGLN("Inactive");
                break;
                case LifeStyleType_Sitting:
                    sittingDuration++;
                    checkSedRules();
                    DBUGVAR(sittingDuration);
                    if (isSitting) {
                        activeDuration = 0;
                        inActiveDuration = 0;
                        checkSedRules();
                    }
                    else {
                        if (sittingDuration == 2) {
                            activeDuration = 0;
                            inActiveDuration = 0;
                            isSitting = true;   // State transition
                            DBUGLN("State is sitting");
                            saveLifeStyleConfigs();
                        }
                    }

                    DBUGLN("Sitting");
                break;
                case LifeStyleType_Data_Collecting:
                    DBUGLN("Forbidden -- Data Collecting");
                break;
                case LifeStyleType_UNKNOWN:
                    DBUGLN("Forbidden -- UNKNOWN");
                break;
            }

            // reset all values
            predictionIndex = 0;
            mvpa_count = 0;
            low_count = 0;
            sit_count = 0;
        }
    }
}
void LifeStyle::checkSedRules() {
    #ifdef ENABLE_TEST_LIFESTYLE
    if (sittingDuration >= 1) {
    #else
    if (sittingDuration == 30) {
    #endif
        sittingDuration = 0;
        deviceManager->lifeStyleConfig.penaltyPoints++;
        DBUGVAR(deviceManager->lifeStyleConfig.penaltyPoints);
        saveLifeStyleConfigs();
        if (true == getPenaltyStatus()) {   // Penalty Points cannot exceed this
            // setRunningStatus(false); // Sedentary rules met (Level 3)
            // setPenaltyStatus(true);
            if (false == isSedAlertAcked) {
                led->setSedAlert(true);
            }
            // led->setLifeStyle(getPenaltyStatus());

            // #ifdef ENABLE_TEST_LIFESTYLE
            // deviceManager->lifeStyleConfig.penaltyEndTime = (rtc->getCurrentTime() + TimeSpan(0,0,3,0)).unixtime();  // locked for 3 mins
            // #else
            // DateTime tmp = rtc->getCurrentTime();
            // DBUGLN("Set Penalty Period till the end of tomorrow");
            // deviceManager->lifeStyleConfig.penaltyEndTime = (DateTime(tmp.year(), tmp.month(), tmp.day()) + TimeSpan(2,0,0,0)).unixtime();
            // #endif

            // // Save settings first to Memory
            // saveLifeStyleConfigs();
            // // Enter deep sleep to save power only if it is in low power mode
            // if (mpuManager->getLowPowerStatus()) {
            //     bandMode->sleep(true, false);
            // }
        }
        else {
            isSedAlertAcked = false;
        }
    }
}
double LifeStyle::sumOfArray(double* values) {
    double sum = 0;
    for(int i=0;i<LIFE_STYLE_BUFFER_SIZE;i++) {
        sum+=values[i];
    }
    return sum;
}
double LifeStyle::calculateVariance() {
    double totalEuclideanNorm = sumOfArray(euclideanNormInASecond);
    // DBUG("totalEuclideanNorm:");
    // DBUGLN(totalEuclideanNorm);
    // let euclideanNormInASecondCount = Double(euclideanNormInASecond.count)
    double euclideanNormMean = (totalEuclideanNorm / LIFE_STYLE_BUFFER_SIZE);
    
    double total = 0.0;
    for (int i = 0;i<LIFE_STYLE_BUFFER_SIZE;i++) {
        total += ((euclideanNormInASecond[i] - euclideanNormMean) * (euclideanNormInASecond[i] - euclideanNormMean));
    }
    
    return total / LIFE_STYLE_BUFFER_SIZE;
}

ActivityType LifeStyle::determinePedestrianStatusAndStepCount(double variance) {
    // DBUG("variance:");
    // DBUGLN(variance, 5);
    if (variance < staticThreshold) {
        if ((filtered_sample[0] >= filtered_sample[1]) && (filtered_sample[0] >= -filtered_sample[2])) {
            // DBUGLN("Static");
            return ActivityType_LOW;
        }
        else
        {
            // DBUGLN("Sitting");
            return ActivityType_SITTING;
        }
    } else if ((staticThreshold <= variance)
                &&
                (variance <= slowWalkingThreshold))
    {
        // pedestrianStatus = "Slow Walking"
        DBUGLN("Slow Walking");
        // deviceManager->lifeStyleConfig.stepCount += 1;
        // DBUGVAR(deviceManager->lifeStyleConfig.stepCount);
        return ActivityType_MOD;
    } else if (slowWalkingThreshold < variance) {
        // pedestrianStatus = "Fast Walking"
        DBUGLN("Fast Walking");
        // deviceManager->lifeStyleConfig.stepCount += 1;
        // DBUGVAR(deviceManager->lifeStyleConfig.stepCount);
        return ActivityType_HIGH;
    }
    return ActivityType_UNKNOWN;
}

LifeStyleType LifeStyle::lifestyle_decision_rules(double mvpa_active_time, double low_active_time, double sit_time) {    
    //    #mod_th = (150/(24*7*60))*100 # real factor
    double mod_th = 60 * style_perc; // simulation factor
    
    // double high_th = mod_th/2;
    
    //    print(mod_th, high_th)
    
    if (mvpa_active_time > mod_th) {
        return LifeStyleType_Active;
    } else if(low_active_time > mod_th) {
        return LifeStyleType_Inactive;
    } else if(sit_time > mod_th) {
        return LifeStyleType_Sitting;
    }
    return LifeStyleType_UNKNOWN;
}

void LifeStyle::initStepCountAlgorithm()
{
    DBUGLN("RESET ALGORITHM");
    for (int j=0;j<LIFE_STYLE_BUFFER_SIZE;j++)
       {
        euclideanNormInASecond[j] = 0.0; 
       }  
    index = 0; 

    threshold = 0;    // Dynamic threshold

    min_value = 1;    // Minimum value updated every 50 samples

    max_value = -1;    // Minimum value updated every 50 samples

    interval_cnt = 0;  // number of samples between two valid steps.
    isBadStep = false;     // Mark if a step is an outlier
}

LifeStyle::LifeStyle(LedManagerTask* _led, ExtRTCTask* _rtc, FlashManagerTask *flashManager, DeviceManager *deviceManager, TimerHandle_t *LSsleepAlarm, TimerHandle_t *LSevalAlarm) :
led(_led),
rtc(_rtc),
bandMode(NULL),
flashManager(flashManager),
deviceManager(deviceManager),
LSsleepAlarm(LSsleepAlarm),
LSevalAlarm(LSevalAlarm),
mpuManager(NULL),
batteryMonitor(NULL),
index(0),
// countIndex(0),
predictionIndex(0),
mvpa_count(0),
low_count(0),
sit_count(0),
sittingDuration(0),
activeDuration(0),
// penaltyPoints(0),
isSitting(false) {
    // Default scenario is NOT sitting
    // To-do: Reload isSitting state and penalty points from Flash
    // Hookup RTC callback.
    rtc->setExtRTCAlarmCallback([this]() {this->alarmReceived();});
};

void LifeStyle::run(int ax, int ay, int az, bool _isLowPowerMode) {
    // Check if the lifestyle is not running due to other reason
    // If yes, then do not evaluate zero motion
    if (false == getRunningStatus() && false == isZeroMotion) {
        zeroMotSampleCnt = 0;
    }
    else {
        feedAccelerationData(ax / 32768.0, ay / 32768.0, az / 32768.0);
        isLowPowerMode = _isLowPowerMode;
    }
    // Note, it is not possible to have zero 
}

void LifeStyle::updateEvaluationCycle(bool isInISR) 
{
    DBUGLN("Updating Evaluation Cycle...");
    if (false == getLifeStyleEnableStatus()) return;
    // Restart evaluation
    // setRunningStatus(true);  // restart evaulation
    DateTime tmp = rtc->getCurrentTime(isInISR);
    // Based on old evaluationEndTime
    
    
    
    if (0xFFFFFFFF == deviceManager->lifeStyleConfig.evaluationEndTime 
    || deviceManager->lifeStyleConfig.evaluationEndTime -60 <= tmp.unixtime()) 
    {
        if ( deviceManager->lifeStyleConfig.evaluationEndTime -60 <= tmp.unixtime() && 
             tmp.unixtime() - deviceManager->LifeStyleHistory.record[0].timeStamp > 1000 )
        {
            
            DBUGLN("Add a record to Life Style History Region");
            saveLifeStyleHistory( deviceManager->lifeStyleConfig.evaluationEndTime, deviceManager->lifeStyleConfig.penaltyPoints, deviceManager->lifeStyleConfig.stepCount);
            
            for( uint8_t i=6 ; i>0 ; i-- )
            {
                deviceManager->LifeStyleHistory.record[i] = deviceManager->LifeStyleHistory.record[i-1];
            }      
            
            deviceManager->LifeStyleHistory.record[0].timeStamp     = deviceManager->lifeStyleConfig.evaluationEndTime;
            deviceManager->LifeStyleHistory.record[0].penaltyPoints = deviceManager->lifeStyleConfig.penaltyPoints;
            deviceManager->LifeStyleHistory.record[0].stepCount     = deviceManager->lifeStyleConfig.stepCount;
            
            deviceManager->LifeStyleHistory.HistoryCount++;
            deviceManager->LifeStyleHistory.HistoryAvailable = deviceManager->LifeStyleHistory.HistoryCount > 7 ? 7 : deviceManager->LifeStyleHistory.HistoryCount;

            for (uint8_t i=0;i<7;i++)
            {
                DBUGVAR(deviceManager->LifeStyleHistory.record[i].timeStamp);
                DBUGVAR(deviceManager->LifeStyleHistory.record[i].penaltyPoints);
                DBUGVAR(deviceManager->LifeStyleHistory.record[i].stepCount);
            }

            DBUGVAR(deviceManager->LifeStyleHistory.HistoryCount);
            DBUGVAR(deviceManager->LifeStyleHistory.HistoryAvailable);


        }

        DBUGLN("Reset penalty points");
        deviceManager->lifeStyleConfig.penaltyPoints = 0;
        deviceManager->lifeStyleConfig.stepCount = 0;
    }
    // Update evaluationEndTime
    deviceManager->lifeStyleConfig.evaluationEndTime = (DateTime(tmp.year(), tmp.month(), tmp.day())+TimeSpan(1,0,0,5)).unixtime();
    DBUGVAR(deviceManager->lifeStyleConfig.evaluationEndTime);
    DBUGVAR(tmp.unixtime());
    DBUGVAR((deviceManager->lifeStyleConfig.evaluationEndTime - tmp.unixtime())* configTICK_RATE_HZ);
    if (false == isInISR) {
        if ( xTimerIsTimerActive( *LSevalAlarm ) != pdFALSE ) { xTimerStop(*LSevalAlarm, 0); }
        xTimerChangePeriod(*LSevalAlarm, (deviceManager->lifeStyleConfig.evaluationEndTime - tmp.unixtime())* configTICK_RATE_HZ, 0);
        xTimerStart(*LSevalAlarm, 0);
    }
    else
    {
        xHigherPriorityTaskWoken = pdFALSE;
        _xHigherPriorityTaskWoken = pdFALSE;
        if ( xTimerIsTimerActive( *LSevalAlarm ) != pdFALSE ) { xTimerStopFromISR(*LSevalAlarm, &_xHigherPriorityTaskWoken);}
        xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
        _xHigherPriorityTaskWoken = pdFALSE;
        xTimerChangePeriodFromISR(*LSevalAlarm, (deviceManager->lifeStyleConfig.evaluationEndTime - tmp.unixtime())*configTICK_RATE_HZ, &_xHigherPriorityTaskWoken);
        xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
        _xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(*LSevalAlarm, &_xHigherPriorityTaskWoken);
        xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
    #ifdef ENABLE_DEBUG_LIFESTYLE
    DBUGLN("Current Time:");
    rtc->printTime(tmp);

    DBUGLN("Current evaluation cycle ends on:");
    rtc->printTime(DateTime(deviceManager->lifeStyleConfig.evaluationEndTime));
    #endif
    
    saveLifeStyleConfigs();

    resetDurations();
    updateRunningStatus(true, isInISR);
}

void LifeStyle::evalAlarmReceived() {
    DBUGLN("Evaluation End Alarm Called");
    updateEvaluationCycle(true);
}

void LifeStyle::alarmReceived() {
    // This callback will not be called from deep sleep.
    // It is called when user set wake up alarm is triggered
    // if (penaltyPoints < deviceManager->lifeStyleConfig.penaltyLimits) {
    //     setPenaltyStatus(false);
    //     penaltyPoints = 0;
    // }
    DBUGLN("RTC wake up called");
    // restartLifeStyle();
    // applySleepTime();
    if (false == batteryMonitor->getChargerStatus()) {
        setRunningStatus(true);
    }
    else
    {
        setRunningStatus(false);
    }
}

bool LifeStyle::getRunningStatus() {
    // For barcodemanager to check if it needs to check if lifestyle is enalbed
    return isLifeStyleRunning;
}

void LifeStyle::setRunningStatus(bool run, bool resetZeroMotion) {
    if (false == getLifeStyleEnableStatus()) {
        isLifeStyleRunning = false;
    }
    else {
        isLifeStyleRunning = run;
    }
    DBUGVAR(isLifeStyleRunning);
    if (true == resetZeroMotion) {
        isZeroMotion = false;
        zeroMotSampleCnt = 0;
    }
}

uint8_t LifeStyle::getPenaltyPoints() {
    return deviceManager->lifeStyleConfig.penaltyPoints;
}

bool LifeStyle::getPenaltyStatus() {
    // return deviceManager->lifeStyleConfig.isPenaltyActive;
    if ((int)deviceManager->lifeStyleConfig.penaltyPoints
        - (int)deviceManager->lifeStyleConfig.stepCount/deviceManager->lifeStyleConfig.equivalentSteps 
        >= deviceManager->lifeStyleConfig.penaltyLimits)
    {
        return true;
    }

    return false;
}

bool LifeStyle::getLifeStyleEnableStatus() {
    return (deviceManager->lifeStyleConfig.isLifeStyleEnabled);
}

void LifeStyle::setLifeStyleEnableStatus(bool _enabled) {
    deviceManager->lifeStyleConfig.isLifeStyleEnabled = _enabled;
    initStepCountAlgorithm();
    enableSleepTime(_enabled);
    if (false == _enabled) {
        setRunningStatus(_enabled);
        saveLifeStyleConfigs();
    }
    else {
        updateEvaluationCycle();
    }
}

void LifeStyle::loadLifeStyleConfigs() 
{
    uint8_t buf[27];
    flashManager->ReadLifeStyleConfigRegion(2, buf, sizeof(buf)); // The first two bytes are not loaded
    // deviceManager->lifeStyleConfig.isPenaltyActive = buf[0];
    deviceManager->lifeStyleConfig.isLifeStyleEnabled = deviceManager->isLifeStyleSynced() ? buf[1] : 0;
    deviceManager->lifeStyleConfig.isSleepTimeEnabled = buf[2];
    deviceManager->lifeStyleConfig.sleepStartTime = *((uint32_t *)(&buf[3]));
    deviceManager->lifeStyleConfig.sleepEndTime = *((uint32_t *)(&buf[7]));
    // deviceManager->lifeStyleConfig.penaltyEndTime = 0xFFFFFFFF; // *((uint32_t *)(&buf[11]));
    deviceManager->lifeStyleConfig.evaluationEndTime = *((uint32_t *)(&buf[15]));
    deviceManager->lifeStyleConfig.penaltyLimits = *((uint8_t *)(&buf[19]));
    deviceManager->lifeStyleConfig.penaltyPoints = *((uint8_t *)(&buf[20]));
    deviceManager->lifeStyleConfig.stepCount = *((uint32_t *)(&buf[21])) == 0xFFFFFFFF ? 0 : *((uint32_t *)(&buf[21]));
    deviceManager->lifeStyleConfig.equivalentSteps = *((uint16_t *)(&buf[25])) == 0xFFFF ? STEP_REQUIRED_PER_POINT : *((uint16_t *)(&buf[25]));

    DBUGVAR(deviceManager->lifeStyleConfig.isPenaltyActive);
    DBUGVAR(deviceManager->lifeStyleConfig.isLifeStyleEnabled);
    DBUGVAR(deviceManager->lifeStyleConfig.isSleepTimeEnabled);
    DBUGVAR(deviceManager->lifeStyleConfig.sleepStartTime);
    DBUGVAR(deviceManager->lifeStyleConfig.sleepEndTime);
    DBUGVAR(deviceManager->lifeStyleConfig.penaltyEndTime);
    DBUGVAR(deviceManager->lifeStyleConfig.evaluationEndTime);
    DBUGVAR(deviceManager->lifeStyleConfig.penaltyLimits);
    DBUGVAR(deviceManager->lifeStyleConfig.penaltyPoints);
    DBUGVAR(deviceManager->lifeStyleConfig.stepCount);
    DBUGVAR(deviceManager->lifeStyleConfig.equivalentSteps);

    if (deviceManager->lifeStyleConfig.penaltyLimits > LS_MAX_PENALTY_POINTS_ALLOWED) 
    {
        deviceManager->lifeStyleConfig.penaltyLimits = LS_MAX_PENALTY_POINTS_ALLOWED;
    }

    if (deviceManager->lifeStyleConfig.penaltyPoints > LS_MAX_PENALTY_POINTS_ALLOWED) 
    {
        // Probably now initialised if it is higher than allowed
        deviceManager->lifeStyleConfig.penaltyPoints = 0;
        deviceManager->lifeStyleConfig.stepCount = 0;
    }
}

void LifeStyle::LoadLifeStyleHistoryRegion() 
{
    DBUGLN("Load lifestyle history region...");
    uint8_t buf[LIFESTYLE_HISTORY_REGION_SIZE];
    flashManager -> ReadLifeStyleHistoryRegion (0, buf, sizeof(buf));

    uint32_t HistoryCount = 0xFFFFFFFF - *((uint32_t *)(&buf[0]));
    deviceManager->LifeStyleHistory.HistoryCount = HistoryCount;
    deviceManager->LifeStyleHistory.HistoryAvailable = HistoryCount > 7 ? 7 : HistoryCount;
    
    //get last 7 days' record
    for ( uint8_t i=0 ; i<deviceManager->LifeStyleHistory.HistoryAvailable ; i++ )
    {
        deviceManager->LifeStyleHistory.record[i].timeStamp     = *( (uint32_t *)( &buf[ 0x10 + 9 * ( (HistoryCount - i) % MAX_NUMBER_LIFESTYLE_RECORDS - 1 )     ]) );
        deviceManager->LifeStyleHistory.record[i].penaltyPoints = *( (uint8_t *)(  &buf[ 0x10 + 9 * ( (HistoryCount - i) % MAX_NUMBER_LIFESTYLE_RECORDS - 1 ) + 4 ]) );
        deviceManager->LifeStyleHistory.record[i].stepCount     = *( (uint32_t *)( &buf[ 0x10 + 9 * ( (HistoryCount - i) % MAX_NUMBER_LIFESTYLE_RECORDS - 1 ) + 5 ]) );
    }

    DBUGVAR(deviceManager->LifeStyleHistory.HistoryCount);
    DBUGVAR(deviceManager->LifeStyleHistory.HistoryAvailable);

    DBUGVAR(deviceManager->LifeStyleHistory.record[6].timeStamp);
    DBUGVAR(deviceManager->LifeStyleHistory.record[6].penaltyPoints);
    DBUGVAR(deviceManager->LifeStyleHistory.record[6].stepCount);

    DBUGVAR(deviceManager->LifeStyleHistory.record[5].timeStamp);
    DBUGVAR(deviceManager->LifeStyleHistory.record[5].penaltyPoints);
    DBUGVAR(deviceManager->LifeStyleHistory.record[5].stepCount);

    DBUGVAR(deviceManager->LifeStyleHistory.record[4].timeStamp);
    DBUGVAR(deviceManager->LifeStyleHistory.record[4].penaltyPoints);
    DBUGVAR(deviceManager->LifeStyleHistory.record[4].stepCount);

    DBUGVAR(deviceManager->LifeStyleHistory.record[3].timeStamp);
    DBUGVAR(deviceManager->LifeStyleHistory.record[3].penaltyPoints);
    DBUGVAR(deviceManager->LifeStyleHistory.record[3].stepCount);

    DBUGVAR(deviceManager->LifeStyleHistory.record[2].timeStamp);
    DBUGVAR(deviceManager->LifeStyleHistory.record[2].penaltyPoints);
    DBUGVAR(deviceManager->LifeStyleHistory.record[2].stepCount);

    DBUGVAR(deviceManager->LifeStyleHistory.record[1].timeStamp);
    DBUGVAR(deviceManager->LifeStyleHistory.record[1].penaltyPoints);
    DBUGVAR(deviceManager->LifeStyleHistory.record[1].stepCount);

    DBUGVAR(deviceManager->LifeStyleHistory.record[0].timeStamp);
    DBUGVAR(deviceManager->LifeStyleHistory.record[0].penaltyPoints);
    DBUGVAR(deviceManager->LifeStyleHistory.record[0].stepCount);

}

void LifeStyle::eraseLifeStyleHistory() 
{
    deviceManager->updateLifeStyleRegion(false,true,false,0,0,0,false);

    for ( uint8_t i=0 ; i<7; i++ )
    {
        deviceManager->LifeStyleHistory.record[i].timeStamp = 0;
        deviceManager->LifeStyleHistory.record[i].penaltyPoints = 0;
        deviceManager->LifeStyleHistory.record[i].stepCount = 0;
    }
    deviceManager->LifeStyleHistory.HistoryCount = 0;
    deviceManager->LifeStyleHistory.HistoryAvailable = 0;
}

void LifeStyle::saveLifeStyleHistory ( uint32_t timeStamp, uint8_t penaltyPoints, uint32_t stepCount)
{
    deviceManager->updateLifeStyleRegion(false,false,true,timeStamp, penaltyPoints, stepCount,false);
}

void LifeStyle::getLifeStyleHistory ( uint8_t historyIndex, uint32_t *timestamp, uint8_t *penalty_point, uint32_t *stepCount )
{
    *timestamp     = deviceManager->LifeStyleHistory.record[historyIndex].timeStamp;
    *penalty_point = deviceManager->LifeStyleHistory.record[historyIndex].penaltyPoints;
    *stepCount     = deviceManager->LifeStyleHistory.record[historyIndex].stepCount;
}

uint32_t LifeStyle::getLifeStyleHistoryCount()
{
    return deviceManager->LifeStyleHistory.HistoryAvailable;
}

void LifeStyle::saveLifeStyleConfigs() {
    deviceManager->updateLifeStyleRegion(false,false,false,0,0,0,true);
}

bool LifeStyle::setSleepingTime(uint32_t _startTime, uint32_t _endTime) {
    // if (false == isSleepTimeEnabled) return false;
    // start time & end time are time without date. 
    // Both are in minutes. Range: [0,24*60)
    DBUG("SETTING SLEEP PERIOD ... ");
    if ((_startTime<0 || _startTime>=1440) || (_endTime<0 || _endTime>=1440)) return false;
    
    deviceManager->lifeStyleConfig.sleepStartTime = _startTime * 60;

    if (_endTime <= _startTime) {
        // Next day
        deviceManager->lifeStyleConfig.sleepEndTime = _endTime * 60 + 24*60*60;
    }
    else {
        // Same day
        deviceManager->lifeStyleConfig.sleepEndTime = _endTime * 60;
    }

    saveLifeStyleConfigs();
    updateRunningStatus(true, false);

    DBUGLN("DONE");

    return true;
}

void LifeStyle::updateRunningStatus(bool updateAlarm, bool isInISR) {
    if (false == getSleepTimeEnabled() || false == getLifeStyleEnableStatus()) { return; }
    // Mapping to time around current time
    DateTime now = (rtc->getCurrentTime(isInISR));
    uint32_t dayOffset = DateTime(now.year(), now.month(), now.day(),0,0,0).unixtime();
    uint32_t preDayOffset = dayOffset - 86400;

    if (((deviceManager->lifeStyleConfig.sleepStartTime + dayOffset) <= now.unixtime() && now.unixtime() < (deviceManager->lifeStyleConfig.sleepEndTime + dayOffset)))  
    {
        DBUGLN("Current time is inside the sleeping period of today");
        if (true == updateAlarm) {
            // Set wakeup alarm to came out of sleep
            rtc->disableAlarm(isInISR);
            rtc->setAlarm(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + dayOffset), isInISR);
        
            // Set sleep alarm  to enter sleep
            // sleepAlarm.Clear();
            // sleepAlarm.Set((deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*1000, false);
            if (false == isInISR) {
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStop(*LSsleepAlarm, 0); }
                xTimerChangePeriod(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*configTICK_RATE_HZ, 0);
                xTimerStart(*LSsleepAlarm, 0);
            }
            else {
                xHigherPriorityTaskWoken = pdFALSE;
                _xHigherPriorityTaskWoken = pdFALSE;
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStopFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);}
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerChangePeriodFromISR(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*configTICK_RATE_HZ, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            

            #ifdef ENABLE_DEBUG_LIFESTYLE
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400));
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + dayOffset));
            #endif
        }

        setRunningStatus(false);
    }
    else if (((deviceManager->lifeStyleConfig.sleepStartTime + preDayOffset) <= now.unixtime() && now.unixtime() < (deviceManager->lifeStyleConfig.sleepEndTime + preDayOffset)))
    {
        DBUGLN("Current time is inside the sleeping period set across days");
        if (true == updateAlarm) {
            // Set wakeup alarm to came out of sleep
            rtc->disableAlarm(isInISR);
            rtc->setAlarm(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + preDayOffset), isInISR);
        
            // Set sleep alarm  to enter sleep
            // sleepAlarm.Clear();
            // sleepAlarm.Set((deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*1000, false);
            if (false == isInISR) {
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStop(*LSsleepAlarm, 0); }
                xTimerChangePeriod(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*configTICK_RATE_HZ, 0);
                xTimerStart(*LSsleepAlarm, 0);
            }
            else {
                xHigherPriorityTaskWoken = pdFALSE;
                _xHigherPriorityTaskWoken = pdFALSE;
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStopFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);}
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerChangePeriodFromISR(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*configTICK_RATE_HZ, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            
            #ifdef ENABLE_DEBUG_LIFESTYLE
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepStartTime + dayOffset  + 86400));
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + preDayOffset));
            #endif
        }
            
        setRunningStatus(false);
    }
    else if ((deviceManager->lifeStyleConfig.sleepStartTime + dayOffset) > now.unixtime())
    {
        DBUGLN("Current time is before the sleeping period of today");
        if (true == updateAlarm) {
            // Set wakeup alarm to came out of sleep
            rtc->disableAlarm(isInISR);
            rtc->setAlarm(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + dayOffset), isInISR);
        
            // Set sleep alarm  to enter sleep
            // sleepAlarm.Clear();
            // sleepAlarm.Set((deviceManager->lifeStyleConfig.sleepStartTime + dayOffset - now.unixtime())*1000, false);
            if (false == isInISR) {
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStop(*LSsleepAlarm, 0); }
                xTimerChangePeriod(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset - now.unixtime())*configTICK_RATE_HZ, 0);
                xTimerStart(*LSsleepAlarm, 0);
            }
            else {
                xHigherPriorityTaskWoken = pdFALSE;
                _xHigherPriorityTaskWoken = pdFALSE;
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStopFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);}
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerChangePeriodFromISR(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset - now.unixtime())*configTICK_RATE_HZ, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            #ifdef ENABLE_DEBUG_LIFESTYLE
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepStartTime + dayOffset));
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + dayOffset));
            #endif
        }

        if (false == batteryMonitor->getChargerStatus()) {
            setRunningStatus(true);
        }
        else
        {
            setRunningStatus(false);
        }
    }
    else
    {
        DBUGLN("Current time is beyond the sleeping period of today");
        if (true == updateAlarm) {
            // Set wakeup alarm to came out of sleep
            rtc->disableAlarm(isInISR);
            rtc->setAlarm(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + dayOffset + 86400), isInISR);
        
            // Set sleep alarm  to enter sleep
            // sleepAlarm.Clear();
            // sleepAlarm.Set((deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*1000, false);
            if (false == isInISR) {
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStop(*LSsleepAlarm, 0); }
                xTimerChangePeriod(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*configTICK_RATE_HZ, 0);
                xTimerStart(*LSsleepAlarm, 0);
            }
            else {
                xHigherPriorityTaskWoken = pdFALSE;
                _xHigherPriorityTaskWoken = pdFALSE;
                if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStopFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);}
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerChangePeriodFromISR(*LSsleepAlarm, (deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400 - now.unixtime())*configTICK_RATE_HZ, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                _xHigherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(*LSsleepAlarm, &_xHigherPriorityTaskWoken);
                xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }

            #ifdef ENABLE_DEBUG_LIFESTYLE
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepStartTime + dayOffset + 86400));
            rtc->printTime(DateTime(deviceManager->lifeStyleConfig.sleepEndTime + dayOffset + 86400));
            #endif
        }

        if (false == batteryMonitor->getChargerStatus()) {
            setRunningStatus(true);
        }
        else
        {
            setRunningStatus(false);
        }
    }
}

void LifeStyle::getSleepingTime(uint32_t *_startTime, uint32_t *_endTime) {
    DBUGVAR(deviceManager->lifeStyleConfig.sleepStartTime);
    DBUGVAR(deviceManager->lifeStyleConfig.sleepEndTime);
    *_startTime = (deviceManager->lifeStyleConfig.sleepStartTime / 60)%1440;
    *_endTime = (deviceManager->lifeStyleConfig.sleepEndTime / 60)%1440;
}

void LifeStyle::sleepAlarmReceived() {
    // Called when it's time to enter sleep or wake up
    DBUGLN("Sleep Alarm Called");
    saveLifeStyleConfigs();
    setRunningStatus(false);

    if (mpuManager->getLowPowerStatus()) {
        bandMode->sleep();
    }
    
}

void LifeStyle::enableSleepTime(bool enabled) {
    deviceManager->lifeStyleConfig.isSleepTimeEnabled = enabled;
    if (false == deviceManager->lifeStyleConfig.isSleepTimeEnabled)
    {
        // clear all the alarms
        // sleepAlarm.Clear();
        if ( xTimerIsTimerActive( *LSsleepAlarm ) != pdFALSE ) { xTimerStop(*LSsleepAlarm, 0); }
        if ( xTimerIsTimerActive( *LSevalAlarm ) != pdFALSE ) { xTimerStop(*LSevalAlarm, 0); }
        rtc->disableAlarm();
    }
}

bool LifeStyle::getSleepTimeEnabled() {
    return deviceManager->lifeStyleConfig.isSleepTimeEnabled;
}

void LifeStyle::resetLifeStyleRegion() {
    deviceManager->updateLifeStyleRegion(true, true, false, 0, 0, 0, false);
}

int8_t LifeStyle::isLifeStyleConfigured() {
    uint16_t buf;
    if (true == flashManager->ReadConfigRegion(0x50, &buf, 2)) {
        if (LS_CONF_MAGIC_WORD == buf)
        {
            return 1;
        }
        else
        {
            DBUGLN("Lifestyle configure magic word not found");
            DBUGVAR(buf);
            // Double check again
            if (true == flashManager->ReadConfigRegion(0x50, &buf, 2))
            {
                DBUGVAR(buf);
                return (LS_CONF_MAGIC_WORD == buf)?1:0;
            }
        }
    }
    DBUGLN("Problem accessing flash content");
    return -1;
}


bool LifeStyle::setLifeStyleSleepTime(char *sleeptime)
{
    char *tmp;
    tmp = strtok(sleeptime, "/");
    deviceManager->lifeStyleConfig.sleepStartTime = atoi(tmp);
    tmp = strtok(NULL, "/");
    deviceManager->lifeStyleConfig.sleepEndTime = atoi(tmp);
    return setSleepingTime(deviceManager->lifeStyleConfig.sleepStartTime, deviceManager->lifeStyleConfig.sleepEndTime);  
}

bool LifeStyle::setPenaltyLimits(uint8_t maxPenaltyPoints)
{
    if (maxPenaltyPoints > 0 && maxPenaltyPoints <= LS_MAX_PENALTY_POINTS_ALLOWED && deviceManager->lifeStyleConfig.penaltyLimits!=maxPenaltyPoints) {
        deviceManager->lifeStyleConfig.penaltyLimits = maxPenaltyPoints;
        saveLifeStyleConfigs();
        checkSedRules();
        // if (deviceManager->lifeStyleConfig.penaltyLimits > deviceManager->lifeStyleConfig.penaltyPoints) {
        //     setRunningStatus(true);
        // }
        return true;
    }
    return false;
}

uint8_t LifeStyle::getPenaltyLimits()
{
    return deviceManager->lifeStyleConfig.penaltyLimits;
}

uint32_t LifeStyle::getPenaltyEndTime()
{
    return (0xFFFFFFFF == deviceManager->lifeStyleConfig.penaltyEndTime?0:deviceManager->lifeStyleConfig.penaltyEndTime);
}

bool LifeStyle::setEquivalentSteps(uint16_t steps) {
    deviceManager->lifeStyleConfig.equivalentSteps = steps;
    saveLifeStyleConfigs();
    checkSedRules();
    return true;
}

uint16_t LifeStyle::getEquivalentSteps() {
    return deviceManager->lifeStyleConfig.equivalentSteps;
}
