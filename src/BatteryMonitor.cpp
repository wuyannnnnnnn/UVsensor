#ifndef ENABLE_DEBUG_BATTERY
#undef ENABLE_DEBUG
#endif

#include "BatteryMonitor.h"
#include "ArduinoLowPower.h"
#include "BandModeManager.h"
#include "BleManagerTask.h"
#include "config.h"
#include "debug.h"

bool isRunning = true;

BatteryMonitor::BatteryMonitor(LedManagerTask *led, TimerHandle_t *sleepAlarm,  LifeStyle *lifeStyle, TimerHandle_t *battTimer, DataStore *dataStore) :
  led(led),
  sleepAlarm(sleepAlarm),
  lifeStyle(lifeStyle),
  bandMode(NULL),
  bleManager(NULL),
  battTimer(battTimer),
  dataStore(dataStore)
{
  lifeStyle->batteryMonitor = this;
  isStartup = true;
}

void BatteryMonitor::checkState()
{
  if (false == isRunning) return;
  pullUp = digitalRead(PIN_STAT);
  pinMode(PIN_STAT, INPUT_PULLDOWN);
  DBUGLN("Pulldown for 500ms");
  delay(500);

  pullDown = digitalRead(PIN_STAT);
  pinMode(PIN_STAT, INPUT_PULLUP);
  DBUGLN("Pullup");

  DBUGVAR(pullUp);
  DBUGVAR(pullDown);
  // LowPower.enableWakeupFrom(GPIO_WAKEUP, PIN_STAT, LOW);
  if (false == isStartup) {
    if(pullUp == pullDown) {
      // It's charging/charged, keep detecing on every 1 second for charger disconnection
      led->setCharging(pullUp ? ChargeState_Charged : ChargeState_Charging);
      if (false == isChargerOn) {
        DBUGLN("Charger Connected");
        isChargerOn = true;
        // This line will prevent it from going to deep sleep (immediate startup) while charging
        if ( xTimerIsTimerActive( *sleepAlarm ) != pdFALSE ) {
          xTimerStop(*sleepAlarm, 0);
        }
        lifeStyle->setRunningStatus(false, true);
      }
      xTimerStop(*battTimer, 0);
      xTimerChangePeriod(*battTimer, CHARGER_ON_CHECK_INTERVAL * configTICK_RATE_HZ / 1000, 0);
    }
    else {
      if (true == isChargerOn) {
        DBUGLN("Charger Disconnected");
        voltageCheckCounter = 0;
        isChargerOn = false;
        // PIN_STAT is Hi-Z. No battery or no charger.
        led->setCharging(ChargeState_Off);
        #ifndef DISABLE_DEVICE_SLEEP
        xTimerReset(*sleepAlarm, 0);
        #endif
        // lifeStyle->setRunningStatus(true);
        lifeStyle->updateRunningStatus(false, false);
      }

      #ifdef BATTERY_LOW_MONITOR
      if (false == isChargerOn && voltageCheckCounter-- <= 0) {
        // Check battery voltage every 5 minutes
        voltageCheckCounter = 200;
        float votlage = ReadBatteryVoltage(true);
        // DBUGF("Battery Voltage = %2.2f\r\n", votlage);
        if ( votlage <= 3.60) {
          DBUGLN("Voltage is too low");
          lifeStyle->setRunningStatus(false);
          bandMode->sleep();
        }
      }
      #endif
      xTimerStop(*battTimer, 0);
      xTimerChangePeriod(*battTimer, CHARGER_OFF_CHECK_INTERVAL * configTICK_RATE_HZ / 1000, 0);
    }
  }
  else {
    isStartup = false;
    voltageCheckCounter = 1; // Startup calibration should be done after 1 seconds
  }
}

bool BatteryMonitor::getChargerStatus() {
  DBUGVAR(isChargerOn);
  return isChargerOn;
}

float BatteryMonitor::ReadBatteryVoltage(bool isCalibrate)
{
  if (true == isFirstMeasure) {
    isCalibrate = true;
  }
  isFirstMeasure = false;
  uint32_t avgResult = 0;
  volatile uint16_t tmpResult = 0;
  float precise_result = 0;

  NRF_SAADC->TASKS_STOP = 1;
  while (NRF_SAADC->EVENTS_STOPPED == 0);
  NRF_SAADC->EVENTS_STOPPED = 0;

  // Configure SAADC singled-ended channel, Internal reference (0.6V) and 1/6 gain.
  NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos) |
                            (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos) |
                            (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                            (SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) |
                            (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) |
                            (SAADC_CH_CONFIG_TACQ_40us       << SAADC_CH_CONFIG_TACQ_Pos);// | (SAADC_CH_CONFIG_BURST_Enabled	 << SAADC_CH_CONFIG_BURST_Pos);

  // Configure the SAADC channel with VDD as positive input, no negative input(single ended).
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput6 << SAADC_CH_PSELP_PSELP_Pos;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;

  // Configure the SAADC resolution.
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos;

  // Configure result to be put in RAM at the location of "result" variable.
  NRF_SAADC->RESULT.MAXCNT = 1;
  NRF_SAADC->RESULT.PTR = (uint32_t)&tmpResult;
  // NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over128x << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;
  // No automatic sampling, will trigger with TASKS_SAMPLE.
  NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task;

  // Enable SAADC (would capture analog pins if they were used in CH[0].PSELP)
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  if (true == isCalibrate) {
    // Calibrate the SAADC (only needs to be done once in a while)
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy <<SAADC_STATUS_STATUS_Pos));

    NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0);
    NRF_SAADC->EVENTS_STOPPED = 0;
  }

  for (int idx=0; idx<NUMBER_OF_BATTERY_SAMPLE; idx++) {
    // Start the SAADC and wait for the started event.
    NRF_SAADC->TASKS_START = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0);
    NRF_SAADC->EVENTS_STARTED = 0;

    // Do a SAADC sample, will put the result in the configured RAM buffer.
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_END == 0);
    NRF_SAADC->EVENTS_END = 0;

    // Stop the SAADC, since it's not used anymore.
    NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0);
    NRF_SAADC->EVENTS_STOPPED = 0;
    avgResult += tmpResult;
    nrf_delay_ms(5);
  }


  // Convert the result to voltage
  // Result = [V(p) - V(n)] * GAIN/REFERENCE * 2^(RESOLUTION)
  // Result = (VDD - 0) * ((1/6) / 0.6) * 2^14
  // VDD = Result / 4551.1
  avgResult = avgResult / NUMBER_OF_BATTERY_SAMPLE;
  // Serial.println(avgResult / NUMBER_OF_BATTERY_SAMPLE);
  precise_result = (float)avgResult / 4551.1f; //284.444f;
  precise_result = ((2*precise_result*100 / 0.996272684f + 0.5) / 100);
  // Serial.printf("Battery Voltage = %2.3f\r\n", precise_result);
  return precise_result;
}

void BatteryMonitor::stopTask() {
  xTimerStop(*battTimer, 0);
  isRunning = false; // Stop checking battery status in timer callback
  delay(CHARGER_ON_CHECK_INTERVAL);  // Wait for the last timer call back to finish
  uint8_t dnaDataStatus = bleManager->getDnaDataSetStatus();
  DBUGVAR(dnaDataStatus);
  if (0xFF == dnaDataStatus) {
    // Shipping mode
    DBUGLN("Shipping mode");
    // dataStore->clearHistory();
    // bleManager->ClearBandmatchingHistory();
    pinMode(PIN_STAT, INPUT_PULLDOWN);
  }
  else
  {
    DBUGLN("User mode");
    NRF_GPIO->PIN_CNF[PIN_STAT] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                          | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                          | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)
                          | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                          | ((uint32_t)GPIO_PIN_CNF_SENSE_Low   << GPIO_PIN_CNF_SENSE_Pos);
  }
}