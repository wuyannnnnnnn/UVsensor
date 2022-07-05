#ifndef ENABLE_DEBUG_LED
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>

#include "config.h"
#include "debug.h"
#include "LedManagerTask.h"

#define MAX_BM_FADE_LED   255

#define MAX_CHARGING_LED  255
#define FADE_STEP         16
#define FADE_DELAY        50

#define FLASH_TIME        150
#define FLASH_COUNT       5 // Number of steps, on, off, on, off, on, will back to off/charging after

#define ALLERGY_FLASH_TIME        200
#define ALLERGY_FLASH_COUNT       (5*10)+1 // Number of steps, on, off, on, off, on, will back to off/charging after
#define HALF_ALLERGY_FLASH_COUNT       (5*5) // Number of steps, on, off, on, off, on, will back to off/charging after

#define BANDMATCH_FLASH_TIME        100
#define BANDMATCH_FLASH_COUNT       (2*10)+1 // Number of steps, on, off, on, off, on, will back to off/charging after

#define UNKNOWN_TIMEOUT        2000
#define TEST_LED_TIME           500

#define CAL_FLASH_TIME          2000
#define CAL_FLASH_COUNT         5 // Number of steps, on, off, on, off, on, will back to off/charging after
#define SED_ALERT_FLASH_TIME    500
#define CAL_DONE_TIMEOUT        2000

// https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
const uint8_t gamma8[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
 90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

TaskHandle_t LedManagerTask::xThisTask = NULL;

LedManagerTask::LedManagerTask() :
  state(LedState_Test_Red)
#ifdef ENABLE_NUDGESHARE
  , familyScanEnabled(false)
#endif // ENABLE_NUDGESHARE
{
}

// static uint16_t _chargedLed[31][4] = {
//   {0,0,0,0},
//   {0,1,0,0},
//   {0,2,0,0},
//   {0,5,0,0},
//   {0,10,0,0},
//   {0,16,0,0},
//   {0,25,0,0},
//   {0,36,0,0},
//   {0,50,0,0},
//   {0,68,0,0},
//   {0,89,0,0},
//   {0,114,0,0},
//   {0,142,0,0},
//   {0,175,0,0},
//   {0,213,0,0},
//   {0,255,0,0},
//   {0,213,0,0},
//   {0,175,0,0},
//   {0,142,0,0},
//   {0,114,0,0},
//   {0,89,0,0},
//   {0,68,0,0},
//   {0,50,0,0},
//   {0,36,0,0},
//   {0,25,0,0},
//   {0,16,0,0},
//   {0,10,0,0},
//   {0,5,0,0},
//   {0,2,0,0},
//   {0,1,0,0},
//   {0,0,0,0},
// };

static uint16_t _chargingLed[63][4] = {
  {0,0,0,0},
  {0,0,0,0},
  {1,1,0,0},
  {1,1,0,0},
  {2,2,0,0},
  {2,2,0,0},
  {3,3,0,0},
  {5,5,0,0},
  {7,7,0,0},
  {10,10,0,0},
  {13,13,0,0},
  {16,16,0,0},
  {20,20,0,0},
  {25,25,0,0},
  {30,30,0,0},
  {36,36,0,0},
  {42,42,0,0},
  {50,50,0,0},
  {59,59,0,0},
  {68,68,0,0},
  {78,78,0,0},
  {89,89,0,0},
  {101,101,0,0},
  {114,114,0,0},
  {128,128,0,0},
  {142,142,0,0},
  {160,160,0,0},
  {175,175,0,0},
  {194,194,0,0},
  {213,213,0,0},
  {234,234,0,0},
  {255,255,0,0},
  {234,234,0,0},
  {213,213,0,0},
  {194,194,0,0},
  {175,175,0,0},
  {160,160,0,0},
  {142,142,0,0},
  {128,128,0,0},
  {114,114,0,0},
  {101,101,0,0},
  {89,89,0,0},
  {78,78,0,0},
  {68,68,0,0},
  {59,59,0,0},
  {50,50,0,0},
  {42,42,0,0},
  {36,36,0,0},
  {30,30,0,0},
  {25,25,0,0},
  {20,20,0,0},
  {16,16,0,0},
  {13,13,0,0},
  {10,10,0,0},
  {7,7,0,0},
  {5,5,0,0},
  {3,3,0,0},
  {2,2,0,0},
  {2,2,0,0},
  {1,1,0,0},
  {1,1,0,0},
  {0,0,0,0},
  {0,0,0,0},
};

uint16_t LedManagerTask::_singleColor[4] = {0,0,0,0};

void LedManagerTask::setup()
{
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  #ifndef HIGH_LED_DRIVE_STRENGTH
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  #else
  NRF_GPIO->PIN_CNF[GREEN_LED] = ((uint32_t)GPIO_PIN_CNF_DIR_Output        << GPIO_PIN_CNF_DIR_Pos)
                      | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
                      | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled      << GPIO_PIN_CNF_PULL_Pos)
                      | ((uint32_t)GPIO_PIN_CNF_DRIVE_H0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                      | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);

  NRF_GPIO->PIN_CNF[BLUE_LED] = ((uint32_t)GPIO_PIN_CNF_DIR_Output        << GPIO_PIN_CNF_DIR_Pos)
                    | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
                    | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled      << GPIO_PIN_CNF_PULL_Pos)
                    | ((uint32_t)GPIO_PIN_CNF_DRIVE_H0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                    | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
  #endif
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);

  // Initialize Registers
  NRF_PWM1->MODE            = PWM_MODE_UPDOWN_Up;
  NRF_PWM1->COUNTERTOP      = 255; // default is 255 (8 bit), can be configured before begin()
  NRF_PWM1->PRESCALER       = PWM_PRESCALER_PRESCALER_DIV_128;
  NRF_PWM1->DECODER         = PWM_DECODER_LOAD_Individual;

  NRF_PWM1->SEQ[0].PTR      = (uint32_t) _chargingLed;
  NRF_PWM1->SEQ[0].CNT      = 4 * 63; // default mode is Individual --> count must be 4
  NRF_PWM1->SEQ[0].REFRESH  = 15;
  NRF_PWM1->SEQ[0].ENDDELAY = 15;

  NRF_PWM1->SEQ[1].PTR      = (uint32_t) _chargingLed;
  NRF_PWM1->SEQ[1].CNT      = 4 * 63;
  NRF_PWM1->SEQ[1].REFRESH  = 15;
  NRF_PWM1->SEQ[1].ENDDELAY = 15;

  NRF_PWM1->PSEL.OUT[0] = RED_LED;
  NRF_PWM1->PSEL.OUT[1] = GREEN_LED;
  NRF_PWM1->PSEL.OUT[2] = BLUE_LED;

  // HwPWM1.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_128);    //125kHz
  // HwPWM1.addPin(RED_LED);
  // HwPWM1.addPin(GREEN_LED);
  // HwPWM1.addPin(BLUE_LED);
}

bool LedManagerTask::isLEDIdle() {
  return m_isLEDIdle;
}

void LedManagerTask::loop()
{
  DBUG("LED manager woke: ");
  DBUGLN(LedState_Off == state ? "LedState_Off" :
         LedState_Test_Red == state ? "LedState_Test_Red" :
         LedState_Test_Green == state ? "LedState_Test_Green" :
         LedState_Test_Blue == state ? "LedState_Test_Blue" :
         LedState_NudgeBar == state ? "LedState_NudgeBar" :
         LedState_Good == state ? "LedState_Good" :
         LedState_Amber == state ? "LedState_Amber":
         LedState_Bad == state ? "LedState_Bad" :
         LedState_Good_Allergy == state ? "LedState_Good_Allergy" :
         LedState_Amber_Allergy == state ? "LedState_Amber_Allergy" :
         LedState_Bad_Allergy == state ? "LedState_Bad_Allergy" :
         LedState_Unknown == state ? "LedState_Unknown" :
         LedState_Buy == state ? "LedState_Buy" :
         LedState_NotBuy == state ? "LedState_NotBuy" :
         LedState_Charging == state ? "LedState_Charging" :
         LedState_Calibration_Start == state ? "LedState_Calibration_Start" :
         LedState_Calibration_Done == state ? "LedState_Calibration_Done" :
         LedState_IdelPressed == state ? "LedState_IdelPressed" :
         LedState_Idel == state ? "LedState_Idel" :
         LedState_Shopping == state ? "LedState_Shopping" :
         LedState_LongPress == state ? "LedState_LongPress" :
         LedState_FlashOrange == state ? "LedState_FlashOrange" :
         LedState_SedAlert == state ? "LedState_SedAlert" :
         LedState_FlashYellow == state ? "LedState_FlashYellow" :
         LedState_FlashBlue == state ? "LedState_FlashBlue" :
         LedState_FlashGreen == state ? "LedState_FlashGreen" :
         LedState_FlashRed == state ? "LedState_FlashRed" :
         LedState_FlashMagenta == state ? "LedState_FlashMagenta" :
         LedState_FlashCyan == state ? "LedState_FlashCyan" :
         LedState_SetOrange == state ? "LedState_SetOrange" :
         LedState_SetYellow == state ? "LedState_SetYellow" :
         LedState_SetBlue == state ? "LedState_SetBlue" :
         LedState_SetGreen == state ? "LedState_SetGreen" :
         LedState_SetRed == state ? "LedState_SetRed" :
         LedState_SetMagenta == state ? "LedState_SetMagenta" :
         LedState_SetCyan == state ? "LedState_SetCyan" :

         #ifdef BLE_BAND_MATCHING
         LedState_BM_Enter == state ? "LedState_BM_Enter" :
         LedState_FamilyScan == state ? "LedState_FamilyScan" :
         LedState_BM_Unknown == state ? "LedState_BM_Unknown":
         LedState_BM_Matched == state ? "LedState_BM_Matched" :
         LedState_BM_Not_Matched == state ? "LedState_BM_Not_Matched" :
         LedState_BM_MatchingNotAllowed == state ? "LedState_BM_MatchingNotAllowed" :
         LedState_BM_OverallMatched == state ? "LedState_BM_OverallMatched" :
         LedState_BM_AmberMatched == state ? "LedState_BM_AmberMatched" :
         LedState_BM_Exit == state ? "LedState_BM_Exit" :
         #endif //BLE_BAND_MATCHING
         "NOT DEFINED");

  if (NULL == xThisTask) {xThisTask = xTaskGetCurrentTaskHandle();}
  m_isLEDIdle = false;

  switch(state)
  {
    case LedState_Off:
      if (1 == NRF_PWM1->ENABLE) {
        NRF_PWM1->EVENTS_STOPPED = 0;
        NRF_PWM1->TASKS_STOP = 1;
        while (0 == NRF_PWM1->EVENTS_STOPPED) {delay(1);}
        NRF_PWM1->ENABLE = 0;
      }
      setLed(0, 0, 0);
      m_isLEDIdle = true;
      state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      if (LedState_Off == state) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      }
      break;

    case LedState_Test_Red:
      setLed(255, 0, 0);
      state = LedState_Test_Green;
      ulTaskNotifyTake(pdTRUE, TEST_LED_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Test_Green:
      setLed(0, 255, 0);
      state = LedState_Test_Blue;
      ulTaskNotifyTake(pdTRUE, TEST_LED_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Test_Blue:
      setLed(0, 0, 255);
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
      if (familyScanEnabled) {
          state = LedState_FamilyScan;
      }
      #endif // ENABLE_NUDGESHARE
      ulTaskNotifyTake(pdTRUE, TEST_LED_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_NudgeBar:
      if (isAmber) {
        setLed(255, 170, 0);
      }
      else {
        setLed(0, 255, 0);
      }
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
      if (familyScanEnabled) {
          state = LedState_FamilyScan;
      }
      #endif // ENABLE_NUDGESHARE
      ulTaskNotifyTake(pdTRUE, NUDGE_BAR_QUERY_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Good:
      setLed(0, 255, 0);
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
      if (familyScanEnabled) {
          state = LedState_FamilyScan;
      }
      #endif // ENABLE_NUDGESHARE
      ulTaskNotifyTake(pdTRUE, _triggerByButton ? HALF_RECOMENDATION_TIMEOUT * configTICK_RATE_HZ / 1000 : RECOMENDATION_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Amber:
      setLed(255, 170, 0);
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
      if (familyScanEnabled) {
          state = LedState_FamilyScan;
      }
      #endif // ENABLE_NUDGESHARE
      ulTaskNotifyTake(pdTRUE, _triggerByButton ? HALF_RECOMENDATION_TIMEOUT * configTICK_RATE_HZ / 1000 : RECOMENDATION_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Bad:
      setLed(255, 0, 0);
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
      if (familyScanEnabled) {
          state = LedState_FamilyScan;
      }
      #endif // ENABLE_NUDGESHARE
      ulTaskNotifyTake(pdTRUE, _triggerByButton ? HALF_RECOMENDATION_TIMEOUT * configTICK_RATE_HZ / 1000 : RECOMENDATION_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Bad_Allergy:
    case LedState_Good_Allergy:
    case LedState_Amber_Allergy:
      if(flashCount % 2)
      {
        if(LedState_Bad_Allergy == state)
        {
          setLed(255, 0, 0);
        }
        else if (LedState_Good_Allergy == state)
        {
          setLed(0, 255, 0);
        }
        else if (LedState_Amber_Allergy == state)
        {
          setLed(255,170,0);
        }
      } else {
        setLed(0, 0, 0);
      }
      if(0 == --flashCount) {
        // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
        state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
        if (familyScanEnabled) {
            state = LedState_FamilyScan;
        }
      #endif // ENABLE_NUDGESHARE
      }
      ulTaskNotifyTake(pdTRUE, ALLERGY_FLASH_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Unknown:
      setLed(0, 0, 255);
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
        if (familyScanEnabled) {
            state = LedState_FamilyScan;
        }
      #endif // ENABLE_NUDGESHARE
      ulTaskNotifyTake(pdTRUE, UNKNOWN_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    #ifdef BLE_BAND_MATCHING
    case LedState_BM_Enter:
    case LedState_FamilyScan:
      // Fade into purple
      // setLed(128, 0, bmFadeValue); 
	    // //! Fade Purple LED
      // bmFadeValue = FadeLed(bmFadeValue, bmFadeDir);
      // ulTaskNotifyTake(pdTRUE, FADE_DELAY * configTICK_RATE_HZ / 1000);
      // break;
      m_isLEDIdle = true;

      // Set Purple
      setLed(128, 0, 255);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;
    case LedState_BM_Unknown:
      setLed(0, 0, 255);  //! Blue
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      ulTaskNotifyTake(pdTRUE, BAND_MATCH_STATUS_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;
    case LedState_BM_Matched:
      setLed(0, 255, 0);  //! Green
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      ulTaskNotifyTake(pdTRUE, BAND_MATCH_STATUS_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;
    
    case LedState_BM_Not_Matched:
      setLed(255, 0, 0);  //! Red
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      ulTaskNotifyTake(pdTRUE, BAND_MATCH_STATUS_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    case LedState_BM_MatchingNotAllowed:
      setLed(0, 0, 255);  //! Blue
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      ulTaskNotifyTake(pdTRUE, BAND_MATCH_STATUS_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    case LedState_BM_OverallMatched:
      if(flashCount % 2)
      {
        setLed(0, 255, 0);
      } else {
        setLed(0, 0, 0);
      }
      if(0 == --flashCount) {
        // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
        state = LedState_Off;
      }
      ulTaskNotifyTake(pdTRUE, BANDMATCH_FLASH_TIME * configTICK_RATE_HZ / 1000);
      break;
    case LedState_BM_AmberMatched:
      setLed(255, 170, 0);  //! Orange
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      ulTaskNotifyTake(pdTRUE, BAND_MATCH_STATUS_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;
    case LedState_BM_Exit:
      setLed(0, 0, 0);  //! LED Off
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      ulTaskNotifyTake(pdTRUE, BAND_MATCH_EXIT_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;
      
    #endif //BLE_BAND_MATCHING

    case LedState_Buy:
    case LedState_NotBuy:
      if(flashCount % 2)
      {
        setLed(LedState_Buy == state ? 0 : 255,
               LedState_Buy == state ? 0 : 0,
               LedState_Buy == state ? 255 : 0);
      } else {
        setLed(0, 0, 0);
      }
      if(0 == --flashCount) {
        // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
        state = LedState_Off;
      #ifdef ENABLE_NUDGESHARE
        if (familyScanEnabled) {
            state = LedState_FamilyScan;
        }
      #endif // ENABLE_NUDGESHARE
      }
      ulTaskNotifyTake(pdTRUE, FLASH_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Idel:
    case LedState_Shopping:
      if(flashCount % 2)
      {
        setLed(LedState_Shopping == state ? 0 : 255,
               LedState_Shopping == state ? 255 : 0,
               0);
      } else {
        setLed(0, 0, 0);
      }
      if(0 == --flashCount) {
        // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
        state = LedState_Off;
      }
      ulTaskNotifyTake(pdTRUE, FLASH_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_LongPress:
      if(flashCount % 2)
      {
        setLed(255, 0, 0);
      } else {
        setLed(0, 0, 0);
      }
      if(0 == --flashCount) {
        // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
        state = LedState_Off;
      }
      ulTaskNotifyTake(pdTRUE, FLASH_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Charging:
      m_isLEDIdle = true;
      // setLed(ChargeState_Charged == ChargeState_Off != charging ? 0 : chargingVal, chargingVal, 0);
      if (1 == NRF_PWM1->ENABLE) {
        NRF_PWM1->EVENTS_STOPPED = 0;
        NRF_PWM1->TASKS_STOP = 1;
        while (0 == NRF_PWM1->EVENTS_STOPPED) {delay(1);}
        NRF_PWM1->ENABLE = 0;
      }

      if (ChargeState_Charged == charging && _chargingLed[1][0] == _chargingLed[1][1]) {
        // Update charging led to charged
        for (int i=0; i<63;i++) {
          _chargingLed[i][0] = 0;
        }
      }
      else if (ChargeState_Charging == charging && _chargingLed[1][0] != _chargingLed[1][1]) {
        for (int i=0; i<63;i++) {
          _chargingLed[i][0] = _chargingLed[i][1];
        }
      }

      NRF_PWM1->SEQ[0].PTR = (uint32_t)_chargingLed;
      NRF_PWM1->SEQ[0].CNT = 63 * 4;
      NRF_PWM1->SEQ[0].REFRESH  = 15;
      NRF_PWM1->SEQ[0].ENDDELAY = 15;

      NRF_PWM1->SEQ[1].PTR = (uint32_t)_chargingLed;
      NRF_PWM1->SEQ[1].CNT = 63 * 4;
      NRF_PWM1->SEQ[1].REFRESH  = 15;
      NRF_PWM1->SEQ[1].ENDDELAY = 15;

      NRF_PWM1->LOOP = 0xFFFFFFFF;
      NRF_PWM1->ENABLE = 1;
      NRF_PWM1->TASKS_SEQSTART[0] = 1;

      // chargingVal += chargingDir;
      // if(chargingVal >= MAX_CHARGING_LED) 
      // {
      //   chargingVal = MAX_CHARGING_LED;
      //   chargingDir = -FADE_STEP; 
      // } else if(chargingVal <= 0) {
      //   chargingVal = 0; 
      //   chargingDir = FADE_STEP;
      // }
      // return FADE_DELAY;
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_Calibration_Start:
      if(!isCalLedOn)
      {
        setLed(255, 255, 0);
        isCalLedOn = true;
      } else {
        setLed(0, 0, 0);
        isCalLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, 2 * CAL_FLASH_TIME * configTICK_RATE_HZ / 1000);
      break;

    case LedState_Calibration_Done:
      setLed(0, 255, 0);
      // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
      state = LedState_Off;
      ulTaskNotifyTake(pdTRUE, 2 * CAL_DONE_TIMEOUT * configTICK_RATE_HZ / 1000);
      break;

    case LedState_IdelPressed:
      setLed(0, 0, 100);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_FlashOrange:
      if(!isFlashLedOn)
      {
        setLed(255, 140, 0);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, (CAL_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_SedAlert:
      m_isLEDIdle = true;
      if(!isFlashLedOn)
      {
        setLed(255, 170, 0);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      state = isreachSedLimit ? LedState_SedAlert : LedState_Off;
      ulTaskNotifyTake(pdTRUE, (SED_ALERT_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_FlashYellow:
      if(!isFlashLedOn)
      {
        setLed(255, 255, 0);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, (CAL_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_FlashBlue:
      if(!isFlashLedOn)
      {
        setLed(0, 0, 255);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, (CAL_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_FlashGreen:
      if(!isFlashLedOn)
      {
        setLed(0, 255, 0);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, (CAL_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_FlashRed:
      if(!isFlashLedOn)
      {
        setLed(255, 0, 0);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, (CAL_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_FlashMagenta:
      if(!isFlashLedOn)
      {
        setLed(255, 0, 255);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, (CAL_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_FlashCyan:
      if(!isFlashLedOn)
      {
        setLed(0, 255, 255);
        isFlashLedOn = true;
      } else {
        setLed(0, 0, 0);
        isFlashLedOn = false;
      }
      ulTaskNotifyTake(pdTRUE, (CAL_FLASH_TIME * configTICK_RATE_HZ / 1000)/2);
      break;

    case LedState_SetOrange:
      setLed(255, 140, 0);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_SetYellow:
      setLed(255, 255, 0);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_SetBlue:
      setLed(0, 0, 255);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_SetGreen:
      setLed(0, 255, 0);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_SetRed:
      setLed(255, 0, 0);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_SetMagenta:
      setLed(255, 0, 255);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;

    case LedState_SetCyan:
      setLed(0, 255, 255);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;    
  }
}

int LedManagerTask::FadeLed(int fadeValue, int FadeDir)
{
  fadeValue += FadeDir;
  if(fadeValue >= MAX_BM_FADE_LED) 
  {
    fadeValue = MAX_BM_FADE_LED;
    FadeDir = -FADE_STEP; 
  } else if(fadeValue <= 0) {
    fadeValue = 0; 
    FadeDir = FADE_STEP;
  }
  return fadeValue;
}

void LedManagerTask::setLed(uint8_t red, uint8_t green, uint8_t blue)
{
  DBUG("LED R:");
  DBUG(red);
  DBUG(" G:");
  DBUG(green);
  DBUG(" B:");
  DBUGLN(blue);

  if (1 == NRF_PWM1->ENABLE) {
    NRF_PWM1->EVENTS_STOPPED = 0;
    NRF_PWM1->TASKS_STOP = 1;
    while (0 == NRF_PWM1->EVENTS_STOPPED) {delay(1);}
    NRF_PWM1->ENABLE = 0;
  }
  

  if (
    ((0 == red) || (255 == red)) &&
    ((0 == green) || (255 == green)) &&
    ((0 == blue) || (255 == blue))
  )
  {
    digitalWrite(RED_LED, 0==red?HIGH:LOW);
    digitalWrite(GREEN_LED, 0==green?HIGH:LOW);
    digitalWrite(BLUE_LED, 0==blue?HIGH:LOW);      
  }
  else
  {    
    _singleColor[0] = gamma8[red];
    _singleColor[1] = gamma8[green];
    _singleColor[2] = gamma8[blue];

    NRF_PWM1->SEQ[0].PTR = (uint32_t)(_singleColor);
    NRF_PWM1->SEQ[0].CNT = 4;
    NRF_PWM1->SEQ[0].REFRESH  = 0;
    NRF_PWM1->SEQ[0].ENDDELAY = 0;

    NRF_PWM1->SEQ[1].PTR = (uint32_t)(_singleColor);;
    NRF_PWM1->SEQ[1].CNT = 4;
    NRF_PWM1->SEQ[1].REFRESH  = 0;
    NRF_PWM1->SEQ[1].ENDDELAY = 0;

    NRF_PWM1->LOOP = 0;
    NRF_PWM1->ENABLE = 1;
    NRF_PWM1->TASKS_SEQSTART[0] = 1;
  }
}
#ifdef BLE_BAND_MATCHING
void LedManagerTask::setBM_Entered()
{
  // bmFadeValue = 0;
  // bmFadeDir = FADE_STEP;
  state = LedState_BM_Enter;
#ifdef ENABLE_NUDGESHARE
  familyScanEnabled = true;
#endif // ENABLE_NUDGESHARE
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBM_Unkonwn()
{
  state = LedState_BM_Unknown;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBM_Matched()
{
  state = LedState_BM_Matched;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBM_NotMatched()
{
  state = LedState_BM_Not_Matched;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBM_Exited()
{
#ifdef ENABLE_NUDGESHARE
  familyScanEnabled = false;
#endif // ENABLE_NUDGESHARE
  state = LedState_BM_Exit;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBM_MatchingNotAllowed()
{
  state = LedState_BM_MatchingNotAllowed;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBM_OverallMatched()
{
  flashCount = BANDMATCH_FLASH_COUNT;
  state = LedState_BM_OverallMatched;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBM_AmberMatched()
{
  state = LedState_BM_AmberMatched;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

#endif //BLE_BAND_MATCHING

void LedManagerTask::setNudgeBar(bool _isAmber)
{
  state = LedState_NudgeBar;
  isAmber = _isAmber;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setGood()
{
  state = LedState_Good;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setAmber()
{
  state = LedState_Amber;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBad()
{
  state = LedState_Bad;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setGoodAllergy()
{
  flashCount = _triggerByButton ? HALF_ALLERGY_FLASH_COUNT : ALLERGY_FLASH_COUNT;
  state = LedState_Good_Allergy;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setAmberAllergy()
{
  flashCount = _triggerByButton ? HALF_ALLERGY_FLASH_COUNT : ALLERGY_FLASH_COUNT;
  state = LedState_Amber_Allergy;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBadAllergy()
{
  flashCount = _triggerByButton ? HALF_ALLERGY_FLASH_COUNT :ALLERGY_FLASH_COUNT;
  state = LedState_Bad_Allergy;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setUnknown()
{
  state = LedState_Unknown;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBuy()
{
  state = LedState_Buy;
  flashCount = FLASH_COUNT;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setNotBuy()
{
  state = LedState_NotBuy;
  flashCount = FLASH_COUNT;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setIdlePressed(bool pressed)
{
  if(pressed) {
    state = LedState_IdelPressed;
    if (NULL != xThisTask) {
      xTaskNotifyGive(xThisTask);
    }
  } else if (LedState_IdelPressed == state) {
    // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
    state = LedState_Off;
    if (NULL != xThisTask) {
      xTaskNotifyGive(xThisTask);
    }
  }
}

void LedManagerTask::setIdle()
{
  state = LedState_Idel;
  flashCount = 3;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setShopping()
{
  state = LedState_Shopping;
  flashCount = FLASH_COUNT;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setLongPress()
{
  state = LedState_LongPress;
  flashCount = 1000;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::clear()
{
  // state = ChargeState_Off != charging ? LedState_Charging : isreachSedLimit ? LedState_SedAlert : LedState_Off;
  state = LedState_Off;
  if (familyScanEnabled) {
    state = LedState_FamilyScan;
  }
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setCharging(ChargeState charging)
{
  if(this->charging != charging)
  {
    this->charging = charging;
    // chargingVal = 0;
    // chargingDir = FADE_STEP;
    if(ChargeState_Off != charging && (LedState_Off == state || LedState_Charging == state))
    {
      state = LedState_Charging;
      if (NULL != xThisTask) {
        xTaskNotifyGive(xThisTask);
      }
    }
    else if(ChargeState_Off == charging && LedState_Charging == state)
    {
      state = LedState_Off;
      if (NULL != xThisTask) {
        xTaskNotifyGive(xThisTask);
      }
    }
  }
}

void LedManagerTask::test()
{
  state = LedState_Test_Red;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setCalStart()
{
  state = LedState_Calibration_Start;
  flashCount = CAL_FLASH_COUNT;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setCalDone()
{
  state = LedState_Calibration_Done;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::flashOrange()
{
  state = LedState_FlashOrange;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setSedAlert(bool _state)
{
  if (true == _state) {
    isreachSedLimit = true;
    state = LedState_SedAlert;
    if (NULL != xThisTask) {
      xTaskNotifyGive(xThisTask);
    }
  }
  else {
    isreachSedLimit = false;
    if (NULL != xThisTask && state == LedState_SedAlert) {
      xTaskNotifyGive(xThisTask);
    }
  }
}

void LedManagerTask::flashYellow()
{
  state = LedState_FlashYellow;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::flashBlue()
{
  state = LedState_FlashBlue;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::flashGreen()
{
  state = LedState_FlashGreen;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::flashRed()
{
  state = LedState_FlashRed;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::flashMagenta()
{
  state = LedState_FlashMagenta;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::flashCyan()
{
  state = LedState_FlashCyan;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setOrange()
{
  state = LedState_SetOrange;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setYellow()
{
  state = LedState_SetYellow;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setBlue()
{
  state = LedState_SetBlue;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setGreen()
{
  state = LedState_SetGreen;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setRed()
{
  state = LedState_SetRed;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setMagenta()
{
  state = LedState_SetMagenta;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setCyan()
{
  state = LedState_SetCyan;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::setOff()
{
  state = LedState_Off;
  if (NULL != xThisTask) {
    xTaskNotifyGive(xThisTask);
  }
}

void LedManagerTask::stopTask()
{
  if (NULL != xThisTask) {
    vTaskSuspend(xThisTask);
  }

  xThisTask = NULL;

  if (1 == NRF_PWM1->ENABLE) {
    NRF_PWM1->EVENTS_STOPPED = 0;
    NRF_PWM1->TASKS_STOP = 1;
    while (0 == NRF_PWM1->EVENTS_STOPPED) {nrf_delay_ms(1);}
    NRF_PWM1->ENABLE = 0;
  }
}