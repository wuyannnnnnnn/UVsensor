#include <Arduino.h>

#include "config.h"
#include "debug.h"
#include "Version.h"
#include "ArduinoLowPower.h"
#include "BandModeManager.h"
#include "Barcode.h"
#include "BarcodeManagerTask.h"
#include "BatteryMonitor.h"
#include "BleManagerTask.h"
#include "DataStore.h"
#include "DeviceManager.h"
#include "FlashManagerTask.h"
#include "MpuManagerTask.h"
#include "SPIFlash.h"
#include "ScannerTask.h"
#include "UARTBoxComTask.h"
#include "LifeStyle.h"
#include "UVsensor.h"

#ifdef ENABLE_MT15
#include "mt15.h"
#endif

#include <nrf_ecb.h>
#include <nrf_nvic.h>

#include <projdefs.h>
#include <task.h>

#ifdef ENABLE_WATCHDOG_RESET_TASK
#include "WatchdogResetTask.h"
#endif //ENABLE_WATCHDOG_RESET_TASK

#ifndef UNIT_TEST // IMPORTANT LINE!
SoftwareTimer sleepAlarm, LSsleepAlarm, LSevalAlarm, battTimer;
TimerHandle_t xSleepAlarm, xLSsleepAlarm, xLSevalAlarm, xBattTimer; 

//! Default Flash Chip
//! CYPRESS
#define CYPRESS_FLASH_JEDEC_ID  0x0160 

//! ISSI Flash
//! jedec_MANUF_ID = 0x9D
//! jedec_MEM_TYPE = 0x60
//! jedec_CAPACITY = 0x19
#define ISSI_FLASH_JEDEC_ID     0x9D60
#define MICRON_FLASH_JEDEC_ID     0x20BA

#ifndef SERIAL_SPEED
#define SERIAL_SPEED 9600
#endif

#ifdef ENABLE_SERIAL_MT15
#ifndef SCANNER_SPEED
#define SCANNER_SPEED 9600
#endif
#endif

#ifndef BUTTON_MODE
#define BUTTON_MODE INPUT_PULLUP
#endif

#ifndef BUTTON_PRESSED
#define BUTTON_PRESSED LOW
#endif

#ifdef ENABLE_MEMORY_DEBUG
SoftwareTimer debugTimer;
#endif

#ifdef DUMP_BARCODES
bool isDumpBarcodesDone = false;
#endif

static SemaphoreHandle_t twi_comms_lock;
static SemaphoreHandle_t flash_mem_lock;

static SPIFlash flash(SS, CYPRESS_FLASH_JEDEC_ID); // 0x0160 is Cypress Flash JEDEC code

static Barcode barcode;
static LedManagerTask led;
//static UVTask sensor;
static UVTask sensor(&twi_comms_lock);
static ExtRTCTask rtc(&twi_comms_lock);
static FlashManagerTask flashManager(&flash, &flash_mem_lock);
static DeviceManager deviceManager(&flashManager);
static LifeStyle lifeStyle(&led, &rtc, &flashManager, &deviceManager, &xLSsleepAlarm, &xLSevalAlarm);
static HistoryManager historyManager(&flashManager);
static BarcodeManagerTask barcodeManager(&flashManager, &deviceManager, &led);
static DataStore dataStore(&barcode, &historyManager, &deviceManager, &rtc);

static ScannerTask scanner(&deviceManager, &dataStore, &led, &barcodeManager, &lifeStyle, &xSleepAlarm);
static MpuManagerTask mpuManager(&scanner, &led, &flashManager, &xSleepAlarm, &lifeStyle, &deviceManager, &twi_comms_lock);
static BatteryMonitor battery(&led, &xSleepAlarm, &lifeStyle, &xBattTimer, &dataStore);

#ifdef ENABLE_WATCHDOG_RESET_TASK
static WatchDogResetTask watchDogResetTsk;
#endif //ENABLE_WATCHDOG_RESET_TASK


#ifdef BLE_BAND_MATCHING
static BleManagerTask bleManager(&dataStore, &led, &scanner, &mpuManager,
                                 &barcodeManager, &deviceManager, &xSleepAlarm,
                                 &battery, &flashManager, &lifeStyle);
#else
static BleManagerTask bleManager(&dataStore, &led, &scanner, &mpuManager,
                                 &barcodeManager, &deviceManager, &xSleepAlarm,
                                 &battery);
#endif

static UARTBoxComTask uartBoxCom(&dataStore, &led, &barcodeManager, &bleManager, &deviceManager, &mpuManager, &xSleepAlarm, &scanner,&sensor);
               
static BandModeManager bandMode(&led, &scanner, &mpuManager, &xSleepAlarm, &lifeStyle, &bleManager, &battery, &flashManager);

// #define TEST_HISTORY_MANAGER
#ifdef TEST_HISTORY_MANAGER
#include "TestBarcodesSmall.h"
#include "test_historymanager.h"
#endif

bool startup = false;

#ifdef ENABLE_MEMORY_DEBUG
void debug_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  dbgMemInfo();
}
#endif

void vMpuLoop() {mpuManager.loop();}
void vBandModeLoop() {bandMode.loop();}
// void vLEDLoop() {led.loop();}
void vScannerLoop() {scanner.loop();}
void vFlashManagerLoop() {flashManager.loop();}
void vUVLoopUVI() {sensor.loopUVI();}
void vUVLoopALS() {sensor.loopALS();}

void setup() 
{
  twi_comms_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(twi_comms_lock);
  flash_mem_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(flash_mem_lock);
  
  #ifdef ENABLE_MEMORY_DEBUG
  debugTimer.begin(1000, debug_timer_callback);
  debugTimer.start();
  #endif

  battTimer.begin(CHARGER_ON_CHECK_INTERVAL, [](TimerHandle_t xTimerID){(void) xTimerID;battery.checkState();});
  battTimer.start();
  xBattTimer = battTimer.getHandle();

  sleepAlarm.begin(DEFAULT_AUTO_SLEEP_TIME*60*1000, [](TimerHandle_t xTimerID){(void) xTimerID;bandMode.sleep();});
  #ifdef DISABLE_DEVICE_SLEEP
  sleepAlarm.stop();
  #else
  sleepAlarm.start();
  #endif
  xSleepAlarm = sleepAlarm.getHandle();

  LSsleepAlarm.begin(86400*1000, [](TimerHandle_t xTimerID) {
    (void) xTimerID;
    lifeStyle.sleepAlarmReceived();
  });
  LSsleepAlarm.stop();
  xLSsleepAlarm = LSsleepAlarm.getHandle();

  LSevalAlarm.begin(86400*1000, [](TimerHandle_t xTimerID) {
    (void) xTimerID;
    lifeStyle.evalAlarmReceived();
  });
  LSevalAlarm.stop();
  xLSevalAlarm = LSevalAlarm.getHandle();

  #ifndef ENABLE_EXT_RTC
  NRF_POWER->RAM[6].POWER = 0x0003FFFF;
  NRF_POWER->RAM[7].POWER = 0x0003FFFF;
  #endif
  if (!NRF_POWER->RESETREAS || (NRF_POWER->RESETREAS == 0x00000004)) {
    // dataStore.clearHistory(); // Reset history head and tail if this is
    //    not a
    // reset or from soft reset(debugger).
    startup = true;
    pinMode(PIN_BUTTON1, INPUT);
    uint8_t _counter = 0;
    while (BUTTON_PRESSED != digitalRead(PIN_BUTTON1) && _counter < 100) {
      delay(50);
      _counter++;
    }
  }
  else
  {
    startup = false;
  }

//  Serial.setPins(PIN_SCANNER_RX, PIN_MCU_RX);
#ifdef ENABLE_REVERSE_SERIAL
  Serial.setPins(PIN_MCU_TX, PIN_MCU_RX);
#endif
  Wire.end();
  pinMode(PIN_WIRE_SDA, OUTPUT);
  pinMode(PIN_WIRE_SCL, OUTPUT); // Pull down i2c to prevent ota failing
  digitalWrite(PIN_WIRE_SCL, LOW);
  digitalWrite(PIN_WIRE_SDA,LOW);

  pinMode(PIN_SCANNER_TX, INPUT);

  // nrf_delay_ms(1000);
//  Serial.setPins(PIN_SCANNER_RX, PIN_SCANNER_TX);
  // Serial.setPins(PIN_MCU_RX, PIN_MCU_TX);
  // following gpio setup is required for correct operation of UART
  // See nRF52832 Section 50.2
  pinMode(PIN_SERIAL_RX, INPUT_PULLUP);
  // digitalWrite(PIN_SERIAL_RX, HIGH);
  #ifdef ENABLE_DEBUG
  pinMode(PIN_SERIAL_TX, OUTPUT);
  digitalWrite(PIN_SERIAL_TX, HIGH);
  #else
  Serial.setPins((uint32_t)PIN_SERIAL_RX, 0xFFFFFFFF);  //Disconnect TX line if not in debug mode
  #endif
  pinMode(PIN_MCU_RX, INPUT_PULLDOWN);
  pinMode(PIN_MCU_TX, INPUT_PULLDOWN);
  NRF_UART0->RESERVED1[3] = 0x1;
  NRF_UART0->EVENTS_ERROR = 0x0;
  NRF_UART0->ERRORSRC = NRF_UART0->ERRORSRC;
  
  Serial.begin(SERIAL_SPEED);
  DBUGLN("==========Init I2C===========");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.

  DBUGF("RESET REASON = 0x%X \r\n", NRF_POWER->RESETREAS);
  // if ((0x8 == NRF_POWER->RESETREAS)) {while(1);}  // For debug timeout
  NRF_POWER->RESETREAS = NRF_POWER->RESETREAS;

  // Serial.println("\n*******************************");
  // Serial.print("****");
  // Serial.print("    ");
  // Serial.print("NUDGEband-");
  // Serial.print(FW_MAJOR_VERSION);
  // Serial.print(".");
  // Serial.print(FW_MINOR_VERSION);
  // Serial.print(".");
  // Serial.print(FW_SUB_VERSION);
  // Serial.print("    ");
  // Serial.println("****");
  // Serial.println("*******************************\n");

	//NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
	//NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled;								//disable UART								//disable UART
	// NRF_SAADC ->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);	//disable ADC
	NRF_PWM0  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);		//disable all pwm instance
	NRF_PWM1  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
	NRF_PWM2  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
	//NRF_TWIM1 ->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);	//disable TWI Master
	//NRF_TWIS1 ->ENABLE = (TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos);	//disable TWI Slave
#ifdef ENABLE_MEMORY_DEBUG
  dbgMemInfo();
#endif
  //////////////////////////////////////////////////////////
  //
  // WARNING
  //
  // The Order of initialisation of the tasks is very impornant
  //
  // Barcode Manager must be started before BLE Manager (due to AES driver)
  // BLE Manager must be initialised before everything else (due to soft device
  // start up)
  //
  //////////////////////////////////////////////////////////

  #ifdef ENABLE_WATCHDOG_RESET_TASK
  #ifndef DUMP_BARCODES
  DBUGLN("Start task Watchdog Reset");
  watchDogResetTsk.init_watchdog();
  #endif
  #endif //ENABLE_WATCHDOG_RESET_TASK

  DBUGLN("Setup Barcode Manager");
  barcodeManager.setup();

  //! Initialize SPI
  SPI.begin();

  //! Initialize flash and Check for Cycpress Flash chip ID
  if (true == flash.initialize()) 
  {
    Serial.println("Init Flash OK!");
  }
  else
  {
    //! Check for ISSI Flash ID
    uint16_t jedecID = flash.readDeviceId();
    if ( (0 == jedecID) || (ISSI_FLASH_JEDEC_ID == jedecID) || (MICRON_FLASH_JEDEC_ID == jedecID))
    {
      Serial.println("Init Flash OK!");
    }
    else
    {    
      Serial.print("JedecID = ");
      Serial.println(jedecID);

      Serial.println("Init Flash FAIL!");
    }
  }

  //===========================================================================
  // delay 30ms to allow Flash to startup
  // This is required because the BleManager needs to read the flash via
  // the deviceManager for the BLE DeviceName
  #ifdef ADAFRUIT_NRF52_CORE
    nrf_delay_ms(30);
  #else
    delay(30);
  #endif //ADAFRUIT_NRF52_CORE
  //===========================================================================

#ifdef TEST_HISTORY_MANAGER
  test_historyManager();
#endif

  // DBUGLN("Setup task led");
  // led.setup();
  // Scheduler.startLoop(vLEDLoop, "LED", 256, TASK_PRIO_LOW);

  DBUGLN("Start Flash Manager");
  Scheduler.startLoop(vFlashManagerLoop, "Flash", 128, TASK_PRIO_NORMAL);

  DBUGLN("Start task scanner");
  scanner.setup();
  Scheduler.startLoop(vScannerLoop, "Scan", 200, TASK_PRIO_NORMAL);

  // BLE must be run first
  DBUGLN("Start task bleManager");
  bleManager.setup();

  DBUGLN("Start external RTC task");
  rtc.setup();

  DBUGLN("Start task bandMode");
  bandMode.setup();
  Scheduler.startLoop(vBandModeLoop, "BM", 128, TASK_PRIO_NORMAL);

  DBUGLN("Setup task led");
  led.setup();
  // Scheduler.startLoop(vLEDLoop, "LED", 256, TASK_PRIO_LOW);

  DBUGLN("Setup uartBoxCom");
  uartBoxCom.setup(&uartBoxCom);

  DBUGLN("Start task mpuManager");
  mpuManager.setup();
  Scheduler.startLoop(vMpuLoop, "MPU", 300, TASK_PRIO_HIGH);

  // UV sensor
  DBUGLN("Start UV sensor task");
  sensor.setup();
  #ifdef ENABLE_DEBUG_UVsensor_ALS
   Scheduler.startLoop(vUVLoopALS, "UV", 300, TASK_PRIO_NORMAL);
   #endif
  #ifdef ENABLE_DEBUG_UVsensor_UVI
   Scheduler.startLoop(vUVLoopUVI, "UV", 300, TASK_PRIO_NORMAL);
   #endif

#ifdef FIX_RTC_SQW
testRTC.setup();
#endif
#ifdef ENABLE_SERIAL_MT15
  MT15.begin(SCANNER_SPEED);
#endif

  // button1.Attach();
  //  button2.Attach();

  LowPower.enableWakeupFrom(GPIO_WAKEUP, BUTTON_1, BUTTON_PRESSED);
  // LowPower.enableWakeupFrom(GPIO_WAKEUP, PIN_STAT, LOW);
  LowPower.enableWakeupFrom(GPIO_WAKEUP, PIN_RTC_INTERRUPT, LOW);
  // NRF_POWER->RAM[6].POWER = 0xFFFF;
  // if (NRF_POWER->RAM[6].POWERSET == 0xFFFF)
  //{
  //  Serial.println("RAM Retention ON");
  //}
  //#ifdef true

  DBUGLN("Finish main setup()");

};

void loop()
{
  #ifdef DUMP_BARCODES
  if (!isDumpBarcodesDone && BUTTON_PRESSED == digitalRead(PIN_BUTTON1)) {
    vTaskSuspendAll();
    barcodeManager.outputBarcodesTable();
    isDumpBarcodesDone = true;
    xTaskResumeAll();
  }
  #endif
  
  led.loop();
}

uint32_t setLoopStacksize(void) { 
  return 256;
}
void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *) __get_MSP(); // Get stack pointer
    uint32_t ia = sp[12]; // Get instruction address from stack

    Serial.printf("Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    while(1)
        ;
}

void* _sbrk(ptrdiff_t incr) {
    extern uint32_t __HeapBase;
    extern uint32_t __HeapLimit;
    static char* heap = 0;
    if (heap == 0) heap = (char*)&__HeapBase;
    void* ret = heap;
    if (heap + incr >= (char*)&__HeapLimit) {
        ret = (void*)-1;
        DBUGLN("HEAP OVERFLOW!");
    }
    else
        heap += incr;
    return ret;
}

void rtos_idle_callback() {
  CRITICAL_REGION_ENTER();
  __set_FPSCR(__get_FPSCR() & ~(0x0000009F));
  (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);
  CRITICAL_REGION_EXIT();
}

#else
#include "./../test/test_barcode_manager.cpp"
#endif // UNIT_TEST