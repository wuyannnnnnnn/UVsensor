#ifndef ENABLE_DEBUG_SCANNER
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>


#include "config.h"
#include "debug.h"
#include "ScannerTask.h"
#include "LifeStyle.h"
#include "UARTBoxComTask.h"

#define TRIGGER_TIME 5000
#ifdef ENABLE_SERIAL_MT15
#include <SoftwareSerial.h>
extern SoftwareSerial ScannerSerial;
#else 
#define ScannerSerial Serial
#endif

TaskHandle_t ScannerTask::xThisTask = NULL;

void ScannerTask::vDecode_ISR() {
  // All interrupts should channeled to bandmode manager
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Single shot interrupt
  detachInterrupt(digitalPinToInterrupt(PIN_SCANNER_DECODE));
  xTaskNotifyFromISR(ScannerTask::xThisTask, ScannerState_Decoded, 
    eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// #define ScannerSerial Serial
ScannerTask::ScannerTask(DeviceManager *deviceManager, DataStore *dataStore, LedManagerTask *led, BarcodeManagerTask *barcodeManager, 
                         LifeStyle *lifeStyle, TimerHandle_t *sleepAlarm) :
  deviceManager(deviceManager),
  barcodeManager(barcodeManager),
  dataStore(dataStore),
  lifeStyle(lifeStyle),
  led(led),
  sleepAlarm(sleepAlarm),
  timeout(0),
  detected_scanning_callback(0),
  scanner_flag(true),
  uartBoxCom(NULL)
{
}

void ScannerTask::setup()
{
  pinMode(TRIGGER_MODULE, OUTPUT);
  digitalWrite(TRIGGER_MODULE, HIGH);
  // NRF_GPIO->PIN_CNF[g_ADigitalPinMap[PIN_SCANNER_POWER]] = ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)
  //                       | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
  //                       | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
  //                       | ((uint32_t)GPIO_PIN_CNF_DRIVE_H0H1       << GPIO_PIN_CNF_DRIVE_Pos)
  //                       | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
  Pwr_pinMode(PIN_SCANNER_POWER, INPUT_PULLUP);
  // digitalWrite(PIN_SCANNER_POWER, HIGH);
  // analogOutputInit();
  // analogWriteResolution();

  pinMode(PIN_SCANNER_DECODE, INPUT);
  allergySlotIndex = deviceManager->getAllergySlotIndex();
}

#ifdef ENABLE_MT15_TEST
bool find(uint8_t* line, int lineLength, uint8_t* sWord, int sWordLength)
{
    bool flag = false;
    int index = 0, i, helper = 0;
    for (i = 0; i < lineLength; i++)
    {
        if (sWord[index] == line[i])
        {
            if (flag == false)
            {
                flag = true;
                helper = i;
            }
            index++;
        }
        else
        {
            flag = false;
            index = 0;
        }
        if (index == sWordLength)
        {
            break;
        }
    }
    if ((i+1-helper) == index)
    {
        return true;
    }
    return false;
}

static int gstate = 0;
static int good = 0;
static int bad = 0;
#endif

void ScannerTask::loop()
{
  Recommendation recommendation = Recommendation_NotSet;
  bool allergy = false;
  uint32_t _intState;
  if (NULL == xThisTask) {xThisTask = xTaskGetCurrentTaskHandle();}
  
  // Waiting for trigger
  if (false == forceTrigger) {
    _intState = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  forceTrigger = false;
  DBUGVAR(_intState);
      
  #ifdef PIN_SCANNER_POWER
  DBUGLN("Power on scanner");
  // digitalWrite(TRIGGER_MODULE, HIGH);
  // nrf_delay_ms(100);
  // digitalWrite(PIN_SCANNER_POWER, LOW);
  // for (int i=255; i>=0; i--) {
  //   analogWrite(PIN_SCANNER_POWER, i);
  //   nrf_delay_us(1);
  // }
  Pwr_pinMode(PIN_SCANNER_POWER, INPUT_PULLDOWN);
  delay(310);
  // _pauseTime = millis();
  #endif
  if (detected_scanning_callback) {
    detected_scanning_callback();
  }
  // If button presseed, trigger the Scanner module
  DBUGLN("Start scan");
  // flush out anything in the buffer
  // while(ScannerSerial.available())
  // {
  //   #ifdef ENABLE_DEBUG_SCANNER
  //   int code = ScannerSerial.read();
  //   DBUG("Ignoring: ");
  //   DBUGLN(code, HEX);
  //   #endif
    ScannerSerial.flush();
  // }

  // Trigger the scanner module
  DBUGLN("Laser on");
  isScanning = true;
  digitalWrite(TRIGGER_MODULE, LOW);

  delay(20);
  attachInterrupt(digitalPinToInterrupt(PIN_SCANNER_DECODE), ScannerTask::vDecode_ISR, RISING);
  // timeout = millis() + TRIGGER_TIME;
  // DBUGVAR(timeout);

  // Wait for the interrupt until it times out

  if(ScannerState_Decoded == ulTaskNotifyTake(pdTRUE, TRIGGER_TIME * configTICK_RATE_HZ / 1000))  {
    DBUGLN("Barcode detected by scanner");

    //! Reset the sleep timer only if there is some scanned data
    if (xTimerIsTimerActive( *sleepAlarm ) != pdFALSE) { 
      xTimerReset(*sleepAlarm, 0);
    }

    String code = ScannerSerial.readStringUntil('\r');
    // NRF_UART0->RESERVED1[3] = 0x1;
    DBUGLN(code);
    DBUGLN(code.length());
    DBUGVAR(NRF_UART0->EVENTS_ERROR);
    DBUGVAR(NRF_UART0->INTENCLR);
    DBUGVAR(NRF_UART0->ERRORSRC);
    DBUGVAR(NRF_UART0->EVENTS_RXDRDY);
    DBUGVAR(NRF_UART0->INTENSET);
    DBUGVAR(NRF_UART0->INTENCLR);
    if (NRF_UART0->ERRORSRC & 0x1) { 
      NRF_UART0->RESERVED1[3] = 0x1;
      NRF_UART0->EVENTS_ERROR = 0x0;
      NRF_UART0->ERRORSRC = NRF_UART0->ERRORSRC;
      DBUGLN("RESET ScannerSerial");
      ScannerSerial.end();
      ScannerSerial.begin(115200);
    }

    #ifdef ENABLE_MT15
      int newLine = ScannerSerial.read();
      digitalWrite(TRIGGER_MODULE, HIGH);
      #endif

      #ifdef PIN_SCANNER_POWER
      // digitalWrite(PIN_SCANNER_POWER, HIGH);
      Pwr_pinMode(PIN_SCANNER_POWER, INPUT_PULLUP);
      digitalWrite(TRIGGER_MODULE, HIGH);
      isScanning = false;
      // DBUGVAR(digitalRead(TRIGGER_MODULE));
    #endif

    if (0 != code.length()) {
      Barcode barcode(code);

      if ( ((code.length() == CARTRIDGE_ID_SIZE) || (code.length() == TTP_CARTRIDGE_ID_SIZE)) && 
          (code[0] == 'D') && (code[1] == 'N') ) {
        uint8_t cartrID[CARTRIDGE_ID_SIZE] = {0};
        memcpy(cartrID, code.c_str(), code.length());
        lifeStyle->setLifeStyleEnableStatus(false); //This will turn off lifestyle and sleeptime
        deviceManager->setCartridgeId(cartrID, CARTRIDGE_ID_SIZE);
        scanner_flag = false;
        uartBoxCom->startBoxCom();
      }
      else if(barcodeManager->searchBarcode(barcode)) {
        int penaltypoint_compensated = (int)deviceManager->lifeStyleConfig.penaltyPoints
          - (int)deviceManager->lifeStyleConfig.stepCount / deviceManager->lifeStyleConfig.equivalentSteps;          
        recommendation = barcode.getRecommendation();
        allergy = (allergySlotIndex != 0xFF) ? (((barcode.getAllergy() >> allergySlotIndex) & 0x1) == 0) : 0;

        uint8_t amberFlag = barcode.getLifeStyle();
        if (0 == amberFlag || 0xFF == amberFlag) {
          isLifeStyle = false;
        }
        else {
          isLifeStyle = true;
        }

        if (isLifeStyle){
          switch(lifeStyle->getPersonalGoal()){
            case LifeStylePersonalGoal4Hrs:
              if (penaltypoint_compensated >= 8)
                isPenaltied = true;
              else if (penaltypoint_compensated >= amberFlag){
                isPenaltied = true;
              }
              else{
                isPenaltied = false;
              }
              break;
            case LifeStylePersonalGoal6Hrs:
              if (penaltypoint_compensated >= 12)
                isPenaltied = true;
              else if (penaltypoint_compensated >= amberFlag){
                isPenaltied = true;
              }
              else{
                isPenaltied = false;
              }
              break;
            case LifeStylePersonalGoal8Hrs:
              if (penaltypoint_compensated >= 16)
                isPenaltied = true;
              else if ((penaltypoint_compensated > 0) && (penaltypoint_compensated <= 4) && (amberFlag == 1)){
                isPenaltied = true;
              }
              else if ((penaltypoint_compensated > 4) && (penaltypoint_compensated >= (amberFlag + 3))){
                isPenaltied = true;
              }
              else
                isPenaltied = false;
              break;
          }
        }
                      
        DBUGVAR((int)deviceManager->lifeStyleConfig.penaltyPoints);
        DBUGVAR((int)deviceManager->lifeStyleConfig.stepCount / deviceManager->lifeStyleConfig.equivalentSteps);
        DBUGVAR((int)deviceManager->lifeStyleConfig.penaltyLimits);
        DBUGVAR(amberFlag);
        DBUGVAR(isLifeStyle);
        DBUGVAR(isPenaltied);
        DBUGVAR(recommendation);
        DBUGVAR(allergy);
        DBUGVAR(_intState);
        led->setTriggerByButton(ScannerState_TriggerByButton == _intState);
        if (allergy) {
          if(Recommendation_Good == recommendation) 
            {
              if (true == isPenaltied && true == isLifeStyle) {
                led->setAmberAllergy();   
              }
              else {
                led->setGoodAllergy();
              }
            } 
            else if(Recommendation_Bad == recommendation)
            {
              led->setBadAllergy();
            }
            else
            {
              led->setUnknown();
            }
        } else {
          if(Recommendation_Good == recommendation) {
            if (true == isPenaltied && true == isLifeStyle) {
              led->setAmber();   
            }
            else {
              led->setGood();
            }
          } 
          else if(Recommendation_Bad == recommendation)
          {
            led->setBad();
          }
          else
          {
            led->setUnknown();
          }
        }

        lastCode = barcode;
        // Wait for buy or not buy
        _intState = ulTaskNotifyTake(pdTRUE, ((ScannerState_TriggerByButton == _intState) ? HALF_RECOMENDATION_TIMEOUT : RECOMENDATION_TIMEOUT) * configTICK_RATE_HZ / 1000);

        if ( ScannerState_TriggerByButton == _intState) {
          DBUGLN("Buy");
          if (xTimerIsTimerActive( *sleepAlarm ) != pdFALSE) { 
            xTimerReset(*sleepAlarm, 0);
          }

          if (true == isPenaltied && true == isLifeStyle) {
            dataStore->addHistory(lastCode, HistoryState_AmberBuy, led->isFamilyScanEnabled());
          }
          else {
            dataStore->addHistory(lastCode, HistoryState_Buy, led->isFamilyScanEnabled());
          }
          led->setBuy();
        }
        else if (ScannerState_TriggerByGesture == _intState) {
          if (true == isPenaltied && true == isLifeStyle) {
            dataStore->addHistory(lastCode, HistoryState_AmberNotBuy, led->isFamilyScanEnabled());
          }
          else {
            dataStore->addHistory(lastCode, HistoryState_NotBuy, led->isFamilyScanEnabled());
          }
          DBUGLN(F("Not buy (trigger)"));
          forceTrigger = true;
          led->clear();
        }
        else {
          if (true == isPenaltied && true == isLifeStyle) {
            dataStore->addHistory(lastCode, HistoryState_AmberNotBuy, led->isFamilyScanEnabled());
          }
          else {
            dataStore->addHistory(lastCode, HistoryState_NotBuy, led->isFamilyScanEnabled());
          }
          DBUGLN(F("Buy/Not Buy timeout"));
        }
      }
      else 
      {
        led->setUnknown();
        if(deviceManager->getDBFormated() != true)
        {
          DBUGLN(deviceManager->getDatabaseVersion());
          dataStore->addHistory(barcode, HistoryState_Unknown, led->isFamilyScanEnabled());
        }
      }
    }
  }
  else {
    DBUGLN(F("Scanner timeout"));
    detachInterrupt(digitalPinToInterrupt(PIN_SCANNER_DECODE));
    #ifdef ENABLE_MT15
    digitalWrite(TRIGGER_MODULE, HIGH);
    isScanning = false;
    #endif

    #ifdef PIN_SCANNER_POWER
    // digitalWrite(PIN_SCANNER_POWER, HIGH);
    Pwr_pinMode(PIN_SCANNER_POWER, INPUT_PULLUP);
    // digitalWrite(TRIGGER_MODULE, LOW);
    // DBUGVAR(digitalRead(TRIGGER_MODULE));
    #endif
  }
}

void ScannerTask::trigger(bool triggerByButton)
{
  if (true == scanner_flag && false == isScanning)
  { 
    xTaskNotify(ScannerTask::xThisTask, ((true == triggerByButton)?ScannerState_TriggerByButton:ScannerState_TriggerByGesture), eSetValueWithOverwrite);
  }
}

void ScannerTask::Pwr_pinMode( uint32_t ulPin, uint32_t ulMode )
{
  if (ulPin >= PINS_COUNT) {
    return;
  }

  ulPin = g_ADigitalPinMap[ulPin];

  // Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
  switch ( ulMode )
  {
    case INPUT:
      // Set pin to input mode
      NRF_GPIO->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    case INPUT_PULLUP:
      // Set pin to input mode with pull-up resistor enabled
      NRF_GPIO->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0D1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    case INPUT_PULLDOWN:
      // Set pin to input mode with pull-down resistor enabled
      NRF_GPIO->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pulldown    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_D0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    case OUTPUT:
      // Set pin to output mode
      NRF_GPIO->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    default:
      // do nothing
    break ;
  }
}
