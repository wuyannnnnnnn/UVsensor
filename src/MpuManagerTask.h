#ifndef MPU_MANAGER_TASK_H
#define MPU_MANAGER_TASK_H

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include <MPU6050.h>
#include <SPIFlash.h>

#include "LedManagerTask.h"
#include "ScannerTask.h"
#include "FlashManagerTask.h"
#include "LifeStyle.h"
#include "DeviceManager.h"

using namespace std;

#define DEVICE_CONFIG_STORE_SIZE 64
#define DEFAULT_MPU_HZ (40)
#define DOUBLE_TAP_LOCK_TIME (700) // set doubleTap lock time (do not retrigger doubleTap) to 1s

enum MpuState
{
  MPU_Init,
  MPU_Calibration_Init,
  MPU_Calibration_Measure,  
  MPU_Calibration_Store,
  MPU_Calibration_Read,  
#if LOG_MPU_DATA
  MPU_PrepareFlashLog,
#endif
  MPU_Run
};

enum gestureState
{
  gesture_Idle,
  gesture_Lock,
  gesture_Lock_r
};

//! Double Tap Detection Callback
typedef std::function<void(void)> double_Tap_Detected_callback_t;

class MpuManagerTask;

class MpuManagerTask
{
private:
  static const int CAL_ADDR = 0;          // Start address on flash for storing calibraion value 
  static const int CAL_MAGIC_WD = 0x53CB; // Stamp for calibration status
  static const int RETRY_LIMIT = 10;      // Retry limit for flash init
  static const int NUM_READINGS = 200;    // Number of readings for average
  static const int NUM_AXIS = 6;
  static const int AX = 1;
  static const int AY = 2;
  static const int AZ = 3;
  static const int GX = 4;
  static const int GY = 5;
  static const int GZ = 6;
  static const uint32_t IDLE_TIMEOUT = 2500;  // Gesture idle timeout 5s
  unsigned char accel_fsr;
  unsigned short gyro_rate, gyro_fsr;
  inv_error_t result;
  struct rx_s
  {
      unsigned char header[3];
      unsigned char cmd;
  };
  struct hal_s
  {
      unsigned char lp_accel_mode;
      unsigned char sensors;
      unsigned char dmp_on;
      unsigned char wait_for_tap;
      volatile unsigned char new_gyro;
      unsigned char motion_int_mode;
      unsigned long no_dmp_hz;
      unsigned long next_pedo_ms;
      unsigned long next_temp_ms;
      unsigned long next_compass_ms;
      unsigned int report;
      unsigned short dmp_features;
      struct rx_s rx;
  };
  hal_s hal;

  /* Platform-specific information. Kinda like a boardfile. */
  struct platform_data_s
  {
      signed char orientation[9];
  };

  /* The sensors can be mounted onto the board in any orientation. The mounting
  * matrix seen below tells the MPL how to rotate the raw data from the
  * driver(s).
  * TODO: The following matrices refer to the configuration on internal test
  * boards at Invensense. If needed, please modify the matrices to match the
  * chip-to-body matrix for your particular set up.
  */
  platform_data_s gyro_pdata = {
      .orientation = { 1, 0, 0,
                      0, 1, 0,
                      0, 0, 1}
  };

  MPU6050_eMPL mpu;
  MpuState state;         // MPU operation states
  gestureState gstrState; // Gesture detection states
  bool dmpReady;          // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  int8_t devStatus = -1;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint8_t tryCounter;     // Initilisation retry counter
  
  // Calibration related
  int32_t average[NUM_AXIS + 1];       // the average
  int16_t offset[NUM_AXIS + 1];        // the offsets
  uint32_t readingLength = 0;
  int8_t ready;
  long buf[NUM_AXIS + 1];
  int acel_deadzone = 10; //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
  int giro_deadzone = 2; //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
  bool isRecalibration = false;
  bool isCalibrated = false;
  bool isInit_reading = true;
  bool isInitialised = false; 
  bool isDmpLoaded = false; 

  // Flash operation related
  bool isFlash_ready = false;
  bool isErease_cmd_issued = false;
  uint16_t flashOP_result = 0;
#if LOG_MPU_DATA
  uint32_t logAddr = 16;  // Default offset 16 bytes (0x10)
#endif

  char devConfigStore[DEVICE_CONFIG_STORE_SIZE];

  // Gesture related
  VectorInt16 aa;      // [x, y, z]      DMP accel sensor measurements
  VectorInt16 gyro;    // [x, y, z]      DMP gyro sensor measurements
  Quaternion q;        // [w, x, y, z]   DMP quaternion container
  VectorFloat gravity; // [x, y, z]      DMP gravity vector
  int16_t ax, ay, az;  // Raw accelerometer output
  int16_t gx, gy, gz;  // Raw gyro output
  float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  // Tap related
  unsigned short xThresh  = 300;    // Disable x-axis tap
  unsigned short yThresh  = 300;    // Disable y-axis tap
  unsigned short zThresh  = 300;  // Set z-axis tap thresh to 100 mg/ms
  unsigned char  taps     = 1;    // Set minimum taps to 2
  unsigned short tapTime  = 100;  // Set tap time to 100ms
  unsigned short tapMulti = 160; // Set multi-tap time to 1s
  unsigned char  tapDir;
  unsigned char  tapCnt;
  static uint32_t doubleTap_lock;
  uint32_t tapAccumulated = 0;

  bool idle = false;
  uint32_t idleStarted = 0;
  uint32_t idleDuration = 0;

  int   pitch_range       = 30;  //Pitch flat angle range (default:30)
  int   roll_range        = 30;  //Roll flat angle range (default:30)
  int   rotate_range      = 90;  //The rotation angle range (default: more than 30)
  int   roll_remain_range = 60;  //When rotating, the range for roll angle (default: less than 60)
  float gravity_z_range   = 0.5; //Gravity on z range(default: less than -0.5)
  
  int   pitch_range_r       = 30;  //Pitch flat angle range (default:30)
  int   roll_range_r        = 60;  //Roll flat angle range (default:30)
  int   rotate_range_r      = 90;  //The rotation angle range (default: more than 30)
  int   roll_remain_range_r = 15;  //When rotating, the range for roll angle (default: less than 60)
  float gravity_z_range_r   = 0.5; //Gravity on z range(default: less than -0.5)

  bool isZeroMotion = false;
 
  static void doubleTapDetected(unsigned char direction, unsigned char count);
  bool chk_gesture();  
  // Debug related
  int laser_cnt = 0;
  long lastReport;
  void zero_average();
  void report6axisDMP();
  void report6axisAVG();
  void report6axis();
  void report6axisOffset();
  void reportStates();  
  void reportYPRG();

  ScannerTask *scanner;
  LedManagerTask *led;
  FlashManagerTask *flash;
  TimerHandle_t *alarm;
  LifeStyle *lifeStyle;
  DeviceManager *deviceManager;
  SemaphoreHandle_t *twi_comms_lock;
  
  bool enable;
  bool interruptEnabled;
  bool isLowPowerMode;
  //! To Store Double Tap Detected Callback
  static double_Tap_Detected_callback_t  _doubleTap_Detected_CB;
  static TaskHandle_t xThisTask;  
public:
  MpuManagerTask(ScannerTask *scanner, LedManagerTask *led, FlashManagerTask *flash, TimerHandle_t *alarm, LifeStyle *lifeStyle, DeviceManager *deviceManager, SemaphoreHandle_t *twi_comms_lock);

  //! Set Double Tap Detected Callback
  void SetDoubleTapDetectedCallback   ( double_Tap_Detected_callback_t  fp);
  void init();

  void setup();
  void loop();

  void recalibrate();
  void setarg1(char *arg);
  void setarg2(char *arg);
  void setarg3(char *arg);
  void setoffset1(char *offset_var);
  void setoffset2(char *offset_var);
  void enabled(bool enable);
  void enterLowPowerMode(bool enable);
  bool getLowPowerStatus() { return isLowPowerMode; }
  bool getZeroMotionStatus();
  void resetZeroMotionStatus();
  static void vISR_MPU();
};

#endif //  MPU_MANAGER_TASK_H
