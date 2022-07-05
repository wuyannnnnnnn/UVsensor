#ifndef ENABLE_DEBUG_BANDMODE_MANAGER
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>
#include <ArduinoLowPower.h>
#include <nrf_nvic.h>
#include <bluefruit.h>

#include "config.h"
#include "debug.h"

#include "BandModeManager.h"

#define SLEEP_TIMEOUT (3 * 150) + 50
TaskHandle_t BandModeManager::xThisTask = NULL;
SoftwareTimer BandModeManager::btnTimer;

BandModeManager::BandModeManager(LedManagerTask *led, ScannerTask *scanner, MpuManagerTask *mpuManager, 
  TimerHandle_t *sleepAlarm, LifeStyle *lifeStyle, BleManagerTask *bleManager, BatteryMonitor *battery, FlashManagerTask *flashManager)
    : led(led), scanner(scanner), mpuManager(mpuManager), 
      sleepAlarm(sleepAlarm), lifeStyle(lifeStyle), bleManager(bleManager), battery(battery), flashManager(flashManager),
      mode(Mode_Shopping), buttonPressedTime(millis()),
      longPress(false), goToSleep(false), externalSleep(false)
{
  // Bit of a hack, but easy way to resolve circular dependency
  lifeStyle->bandMode = this;
  battery->bandMode = this;
}

void BandModeManager::vButton_ISR() {
  // All interrupts should channeled to bandmode manager
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  xTaskNotifyFromISR(BandModeManager::xThisTask, digitalRead(PIN_BUTTON1), 
    eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void BandModeManager::vBtnTimer_ISR(TimerHandle_t xTimerID) {
  (void) xTimerID;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  xTaskNotifyFromISR(BandModeManager::xThisTask, 3, 
    eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void BandModeManager::setup() {
  pinMode(PIN_BUTTON1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON1), BandModeManager::vButton_ISR, CHANGE);

  // Setup a timer for checking button hold
  BandModeManager::btnTimer.begin(MODE_SWITCH_TIMEOUT, vBtnTimer_ISR);
  BandModeManager::btnTimer.stop();
}

void BandModeManager::loop() {
  DBUG("Band mode: ");
  DBUGLN(Mode_Shopping == mode ? "Mode_Shopping" :
       Mode_Idel == mode ? "Mode_Idel" :
       Mode_MAX == mode ? "Mode_MAX" :
       "UNKNOWN");

  if (NULL == xThisTask) {xThisTask = xTaskGetCurrentTaskHandle();}

  uint32_t _intState = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  DBUGVAR(_intState);
  if (0x4 == _intState) {
    // Only valid if current mode is not already in Idle mode

    mode = Mode_Idel;

    
    if ((true== lifeStyle->getRunningStatus()) 
    && (true == lifeStyle->getLifeStyleEnableStatus()))
    {
      mpuManager->enterLowPowerMode(true);
      delay(25);
    }
    else
    {
      mpuManager->enabled(false);
    }
    
    DBUGLN("Turn off Band matching");
    bleManager->StopBandMatching();
    DBUGLN("Turn off BLE");

    DBUGLN("Set LED");
    led->setIdle();
    delay(25);

    CRITICAL_REGION_ENTER();
    Bluefruit.Advertising.restartOnDisconnect(false);
    if (Bluefruit.connected()) {
      DBUGLN("Closing Existing connection"); 
      Bluefruit.disconnect();
    }

    DBUGLN("Turning off ble scanning"); 
    Bluefruit.Scanner.stop();

    DBUGLN("Turning off ble advertising"); 
    Bluefruit.Advertising.stop();
    CRITICAL_REGION_EXIT();

    if ((false == lifeStyle->getRunningStatus()) 
    || (false == lifeStyle->getLifeStyleEnableStatus()))
    {
      while (false == led->isLEDIdle()) {
        delay(1);
      }
      led->stopTask();
      battery->stopTask();
      pinMode(PIN_SCANNER_POWER, INPUT_PULLUP);
      // pinMode(TRIGGER_MODULE, OUTPUT);
      digitalWrite(TRIGGER_MODULE, LOW);

      flashManager->Uninit();
      Wire.end();
      DBUG("Enter deep sleep...");
      spi_pulldown();
      DBUG("SPI pulldown...");
      // Make sure all LEDs are off
      digitalWrite(RED_LED, HIGH);
      pinMode(RED_LED, INPUT_PULLUP);
      digitalWrite(GREEN_LED, HIGH);
      pinMode(GREEN_LED, INPUT_PULLUP);
      digitalWrite(BLUE_LED, HIGH);
      pinMode(BLUE_LED, INPUT_PULLUP);
      // Make sure Exposed UART pins are input
      pinMode(PIN_MCU_RX, INPUT_PULLDOWN);
      pinMode(PIN_MCU_TX, INPUT_PULLDOWN);
      CRITICAL_REGION_ENTER();
      __set_FPSCR(__get_FPSCR() & ~(0x0000009F));
      (void) __get_FPSCR();
      NVIC_ClearPendingIRQ(FPU_IRQn);
      CRITICAL_REGION_EXIT();
      LowPower.deepSleep(false);
    }
    else
    {
      DBUG("Enter low power idle...");
      while (false == led->isLEDIdle()) {
        delay(1);
      }
      
      pinMode(PIN_SCANNER_POWER, INPUT_PULLUP);
      // pinMode(TRIGGER_MODULE, OUTPUT);
      digitalWrite(TRIGGER_MODULE, LOW);

      DBUGLN("Done");
    }
  }
  else if (0x3 == _intState) {
    // Wake up by timer and check for long press
    if (BUTTON_PRESSED == digitalRead(PIN_BUTTON1)) {
      if (longPress) {
        DBUGLN("Reboot");
        sd_nvic_SystemReset();
      } else {
        DBUGLN("Long press start");
        longPress = true;
        led->setLongPress();
        BandModeManager::btnTimer.setPeriod(REBOOT_TIMEOUT);
        BandModeManager::btnTimer.start();
      }
    }
  }
  else {
    if (_intState != (uint32_t)digitalRead(PIN_BUTTON1)) {
      DBUGLN("Button Glitch?!");
    }
    else {
      if (BUTTON_PRESSED == _intState) {
        // Button pressed
        DBUGLN("BUTTON PRESSED");
        led->setSedAlert(false);
        if (lifeStyle->getPenaltyStatus())
        {
          lifeStyle->isSedAlertAcked = true;
        }
        buttonPressedTime = millis();
        #ifdef ENABLE_BUTTON_PRESS_LED
        led->setIdelPressed(button->IsPressed());
        #endif
        BandModeManager::btnTimer.setPeriod(MODE_SWITCH_TIMEOUT);
        BandModeManager::btnTimer.start();
      }
      else {
        // Button released
        DBUGLN("BUTTON RELEASED");
        if (false == longPress) {
          // Button was short pressed
          switch (mode) {
          case Mode_Shopping:
            #ifdef ENABLE_BUTTON_TRIGGER
            DBUGLN("Trigger scanner (button)");
            scanner->trigger(true);
            #endif
            break;
          case Mode_Idel:
            // HACK: return 0 here, will cause the device to immediateley go to sleep, rather than wait for
            //       the flashing LED
            mode = Mode_Shopping;
            led->test();
            #ifndef DISABLE_DEVICE_SLEEP
            xTimerReset(*sleepAlarm, 0);
            #endif
            mpuManager->enterLowPowerMode(false);
            
            // pinMode(TRIGGER_MODULE, OUTPUT);
            digitalWrite(TRIGGER_MODULE, HIGH);
            // pinMode(PIN_SCANNER_POWER, OUTPUT);
            // digitalWrite(PIN_SCANNER_POWER, HIGH);
            // analogWrite(PIN_SCANNER_POWER, 255);  
            pinMode(PIN_SCANNER_POWER, INPUT_PULLUP);

            DBUGLN("Turn BLE advertising back on");
            if (false == Bluefruit.Advertising.isRunning()) {
              Bluefruit.Advertising.start();
              Bluefruit.Advertising.restartOnDisconnect(true);
            }
            // MicroTask.wakeTask(mpuManager);
            // return 0;
            break;
          case Mode_MAX:
            break;
          }
        }
        else {
          // Button was long pressed but released before REBOOT_TIMEOUT
          longPress = false;  
          sleep();
        }
        BandModeManager::btnTimer.stop();
      }
    }
  }
}

void BandModeManager::sleep() {
  DBUGLN("BandModeManager::sleep called");
  xTaskNotify(BandModeManager::xThisTask, 4, eSetValueWithOverwrite);
  if ( xTimerIsTimerActive( *sleepAlarm ) != pdFALSE ) {
    xTimerStop(*sleepAlarm, 0);
  }
}

void BandModeManager::spi_pulldown() {
  pinMode(PIN_SPI_MISO, INPUT_PULLDOWN);
  pinMode(PIN_SPI_MOSI, INPUT_PULLDOWN);
  pinMode(PIN_SPI_SCK, INPUT_PULLDOWN);
}
