#ifndef LED_MANAGER_TASK_H
#define LED_MANAGER_TASK_H

#include "DataStore.h"
#include "nrf.h"

enum LedState
{
  LedState_Off,
  LedState_Test_Red,
  LedState_Test_Green,
  LedState_Test_Blue,
  LedState_NudgeBar,
  LedState_Good,
  LedState_Amber,
  LedState_Bad,
  LedState_Good_Allergy,
  LedState_Amber_Allergy,
  LedState_Bad_Allergy,
  LedState_Unknown,
  LedState_Buy,
  LedState_NotBuy,
  LedState_Charging,
  LedState_Calibration_Start,
  LedState_Calibration_Done,
  LedState_IdelPressed,
  LedState_Idel,
  LedState_Shopping,
  LedState_LongPress,
  LedState_FlashOrange,
  LedState_SedAlert,
  LedState_FlashYellow,
  LedState_FlashBlue,
  LedState_FlashGreen,
  LedState_FlashRed,
  LedState_FlashMagenta,
  LedState_FlashCyan,
  LedState_SetOrange,
  LedState_SetYellow,
  LedState_SetBlue,
  LedState_SetGreen,
  LedState_SetRed,
  LedState_SetMagenta,
  LedState_SetCyan,
  #ifdef BLE_BAND_MATCHING
  LedState_BM_Enter,
  LedState_BM_Unknown,
  LedState_BM_Matched,
  LedState_BM_Not_Matched,
  LedState_BM_MatchingNotAllowed,
  LedState_BM_OverallMatched,
  LedState_BM_AmberMatched,
  LedState_BM_Exit,
  LedState_FamilyScan,
  #endif //BLE_BAND_MATCHING
};

enum ChargeState
{
  ChargeState_Off = 0,
  ChargeState_Charging,
  ChargeState_Charged
};

class LedManagerTask
{
  private:
    LedState state;
    ChargeState charging;
    bool isreachSedLimit;
    bool m_isLEDIdle;
    int chargingVal;
    int chargingDir;

    int bmFadeValue;
    int bmFadeDir;

    int flashCount;
    bool isCalLedOn;
    bool isFlashLedOn;
    bool _triggerByButton;
    static uint16_t _singleColor[4];
    bool isAmber;

#ifdef ENABLE_NUDGESHARE
    bool familyScanEnabled;
#endif // ENABLE_NUDGESHARE
    void setLed(uint8_t red, uint8_t green, uint8_t blue);
    int FadeLed(int fadeValue, int FadeDir);

    static TaskHandle_t xThisTask;
  public:
    LedManagerTask();

    void setup();
    void loop();
    bool isLEDIdle();

    void setNudgeBar(bool _isAmber);
    void setGood();
    void setAmber();
    void setBad();
    void setGoodAllergy();
    void setAmberAllergy();
    void setBadAllergy();
    void setUnknown();
    
#ifdef BLE_BAND_MATCHING
    void setBM_Entered();
    void setBM_Unkonwn();
    void setBM_Matched();
    void setBM_NotMatched();
    void setBM_Exited();
    void setBM_MatchingNotAllowed();
    void setBM_OverallMatched();
    void setBM_AmberMatched();
#endif //BLE_BAND_MATCHING

#ifdef ENABLE_NUDGESHARE
    bool isFamilyScanEnabled() { return familyScanEnabled; };
#endif // ENABLE_NUDGESHARE

    void setBuy();
    void setNotBuy();

    void setCharging(ChargeState charging);

    void clear();

    void test();

    void setCalStart();
    void setCalDone();

    void setIdlePressed(bool pressed);
    void setIdle();
    void setShopping();

    void setLongPress();

    void flashOrange();
    void setSedAlert(bool _state);
    void flashYellow();
    void flashBlue();
    void flashGreen();
    void flashRed();
    void flashMagenta();
    void flashCyan();

    void setOrange();
    void setYellow();
    void setBlue();
    void setGreen();
    void setRed();
    void setMagenta();
    void setCyan();
    void setOff();
    void stopTask();
    
    void setTriggerByButton(bool triggerByButton) {
      _triggerByButton = triggerByButton;
    }
  };

#endif //  LED_MANAGER_TASK_H
