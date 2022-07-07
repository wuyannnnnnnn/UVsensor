#ifndef ENABLE_DEBUG_MPU
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>
#include <Wire.h>

#include "MpuManagerTask.h"
#include "config.h"
#include "debug.h"

#ifdef BLE_BAND_MATCHING
#include "BleManagerTask.h"
#endif //BLE_BAND_MATCHING

#include "FlashManagerTask.h"


// SPIFlash flash(SS, 0x0160); // 0x0160 is Cypress Flash JEDEC code
// extern SPIClass SPI;
double_Tap_Detected_callback_t MpuManagerTask::_doubleTap_Detected_CB;
uint32_t MpuManagerTask::doubleTap_lock = 0;

MpuManagerTask::MpuManagerTask(ScannerTask *scanner, LedManagerTask *led, FlashManagerTask *flash, TimerHandle_t *alarm, LifeStyle *lifeStyle, DeviceManager *deviceManager, SemaphoreHandle_t *twi_comms_lock) : 
  mpu(MPU_I2C_ADDRESS),
  dmpReady(false),
  scanner(scanner),
  led(led),
  flash(flash),
  alarm(alarm),
  lifeStyle(lifeStyle),
  deviceManager(deviceManager),
  twi_comms_lock(twi_comms_lock),
  enable(false)
{
  // lifeStyle.attacheLed(led);
  // lifeStyle.attacheAlarm(lifeStyleAlarm);
  lifeStyle->mpuManager = this;
  memset(&hal, 0, sizeof(hal));
}

//! Used to set Double Tap Detection Callback to MPU task
void MpuManagerTask::SetDoubleTapDetectedCallback   ( double_Tap_Detected_callback_t  fp)
{
  _doubleTap_Detected_CB = fp;
}

void MpuManagerTask::vISR_MPU () {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  configASSERT( MpuManagerTask::xThisTask != NULL );
  vTaskNotifyGiveFromISR( MpuManagerTask::xThisTask , &xHigherPriorityTaskWoken) ;
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void MpuManagerTask::setup()
{
  (void)(mpl_key);
  #ifdef ENABLE_DEBUG_MPU
  lastReport = millis();
  #endif

  // join I2C bus (I2Cdev library doesn't do this automatically)
  // Wire.begin();
  // Wire.setClock(400000); // 400kHz I2C clock.
  //Wire.setClock(100000); // 100kHz I2C clock.

  isInitialised = false;

  if (-1 == lifeStyle->isLifeStyleConfigured()) {
    DBUGLN("Cannot determine lifestyle config. Disable by default");
  }
  else {
  
    if (0 == lifeStyle->isLifeStyleConfigured()) {
      DBUGLN("Resetting lifestyle config and history region ...");
      lifeStyle->resetLifeStyleRegion();
      DBUGLN("Done");
    } 

    // Load Life Style settings from Memory
    lifeStyle->loadLifeStyleConfigs();
    // Load Life Style history to buffer
    lifeStyle->LoadLifeStyleHistoryRegion();
    
    // Update evaluation status
    lifeStyle->updateEvaluationCycle();

    #ifdef ENABLE_DEBUG
      // lifeStyle->dumpLifeStyleHistory();
    #endif
  }
}

// bool reset;
TaskHandle_t MpuManagerTask::xThisTask = NULL;

void MpuManagerTask::init() {
  if (NULL == xThisTask) {MpuManagerTask::xThisTask = xTaskGetCurrentTaskHandle();}
  
  // initialize device
  DBUGLN(F("Initializing MPU6050..."));
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  // mpu.initialize();
  mpu_init(NULL);
  // mpu.setWakeCycleEnabled(false); // don't cycle between wake and sleep while NOT in sleep mode.

  // verify connection
  DBUGLN(F("Testing device connections..."));
  DBUGLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // mpu.resetFIFO();

  inv_init_mpl();
  /* Compute 6-axis and 9-axis quaternions. */
  // inv_enable_quaternion();
  // inv_enable_9x_sensor_fusion();
  /* The MPL expects compass data at a constant rate (matching the rate
    * passed to inv_set_compass_sample_rate). If this is an issue for your
    * application, call this function, and the MPL will depend on the
    * timestamps passed to inv_build_compass instead.
    *
    * inv_9x_fusion_use_timestamps(1);
    */

  /* This function has been deprecated.
    * inv_enable_no_gyro_fusion();
    */

  /* Update gyro biases when not in motion.
    * WARNING: These algorithms are mutually exclusive.
    */
  // inv_enable_fast_nomot();
  /* inv_enable_motion_no_motion(); */
  /* inv_set_no_motion_time(1000); */

  /* Update gyro biases when temperature changes. */
  // inv_enable_gyro_tc();

  /* This algorithm updates the accel biases when in motion. A more accurate
    * bias measurement can be made when running the self-test (see case 't' in
    * handle_input), but this algorithm can be enabled if the self-test can't
    * be executed in your application.
    *
    * inv_enable_in_use_auto_calibration();
    */
#ifdef COMPASS_ENABLED
  /* Compass calibration algorithms. */
  inv_enable_vector_compass_cal();
  inv_enable_magnetic_disturbance();
#endif
  /* If you need to estimate your heading before the compass is calibrated,
    * enable this algorithm. It becomes useless after a good figure-eight is
    * detected, so we'll just leave it out to save memory.
    * inv_enable_heading_from_gyro();
    */

  /* Allows use of the MPL APIs in read_from_mpl. */
  inv_enable_eMPL_outputs();

  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED)
  {
      while (1)
      {
          DBUGLN("Not authorized.\n");
      }
  }
  if (result)
  {
      DBUGLN("Could not start the MPL.\n");
  }

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
  /* The compass sampling rate can be less than the gyro/accel sampling rate.
    * Use this function for proper power management.
    */
  mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
  /* Read back configuration in case it was set improperly. */
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
  mpu_get_compass_fsr(&compass_fsr);
#endif
  /* Sync driver configuration with MPL. */
  /* Sample rate expected in microseconds. */
  inv_set_gyro_sample_rate(1000000L / gyro_rate);
  inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
  /* The compass rate is independent of the gyro and accel rates. As long as
    * inv_set_compass_sample_rate is called with the correct value, the 9-axis
    * fusion algorithm's compass correction gain will work properly.
    */
  inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
  /* Set chip-to-body orientation matrix.
    * Set hardware units to dps/g's/degrees scaling factor.
    */
  inv_set_gyro_orientation_and_scale(
      inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)gyro_fsr << 15);
  inv_set_accel_orientation_and_scale(
      inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)accel_fsr << 15);
#ifdef COMPASS_ENABLED
  inv_set_compass_orientation_and_scale(
      inv_orientation_matrix_to_scalar(compass_pdata.orientation), (long)compass_fsr << 15);
#endif
  /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
  hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
  hal.sensors = ACCEL_ON | GYRO_ON;
#endif
  hal.dmp_on = 0;
  hal.report = 0; //PRINT_GRAVITY_VECTOR | PRINT_LINEAR_ACCEL;
  hal.rx.cmd = 0;
  hal.next_pedo_ms = 0;
  hal.next_compass_ms = 0;
  hal.next_temp_ms = 0;

  /* Compass reads are handled by scheduler. */
  //    get_tick_count(&timestamp);
  xSemaphoreGive(*twi_comms_lock);
  tryCounter = 0;
  while (tryCounter++ < RETRY_LIMIT)
  {
    // while (flash->isFlash_busy()) {
    //   delay(50);
    // }
    if (flash->ReadConfigRegion(CAL_ADDR, &flashOP_result, 2))
    {
      DBUGLN(flashOP_result, HEX);
      xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
      if (flashOP_result == CAL_MAGIC_WD && false == isRecalibration)
      {
        isCalibrated = true;         
        if (flash->ReadConfigRegion(CAL_ADDR + 2, offset + 1, 12))
        {

          /* To initialize the DMP:
          * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
          *    inv_mpu_dmp_motion_driver.h into the MPU memory.
          * 2. Push the gyro and accel orientation matrix to the DMP.
          * 3. Register gesture callbacks. Don't worry, these callbacks won't be
          *    executed unless the corresponding feature is enabled.
          * 4. Call dmp_enable_feature(mask) to enable different features.
          * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
          * 6. Call any feature-specific control functions.
          *
          * To enable the DMP, just call mpu_set_dmp_state(1). This function can
          * be called repeatedly to enable and disable the DMP at runtime.
          *
          * The following is a short summary of the features supported in the DMP
          * image provided in inv_mpu_dmp_motion_driver.c:
          * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
          * 200Hz. Integrating the gyro data at higher rates reduces numerical
          * errors (compared to integration on the MCU at a lower sampling rate).
          * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
          * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
          * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
          * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
          * an event at the four orientations where the screen should rotate.
          * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
          * no motion.
          * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
          * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
          * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
          * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
          */
          dmp_load_motion_driver_firmware();
          dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
          dmp_register_tap_cb(doubleTapDetected);
          // dmp_register_android_orient_cb(android_orient_cb);
          /*
            * Known Bug -
            * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
            * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
            * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
            * there will be a 25Hz interrupt from the MPU device.
            *
            * There is a known issue in which if you do not enable DMP_FEATURE_TAP
            * then the interrupts will be at 200Hz even if fifo rate
            * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
            *
            * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
            */
          hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT |
                              DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                              DMP_FEATURE_GYRO_CAL;
          dmp_enable_feature(hal.dmp_features);
          dmp_set_fifo_rate(DEFAULT_MPU_HZ);
          mpu_set_dmp_state(1);
          hal.dmp_on = 1;

          if (hal.dmp_on && true == isLowPowerMode) {
              unsigned short dmp_rate;
              unsigned char mask = 0;
              hal.dmp_on = 0;
              mpu_set_dmp_state(0);
              /* Restore FIFO settings. */
              if (hal.sensors & ACCEL_ON)
                  mask |= INV_XYZ_ACCEL;
              if (hal.sensors & GYRO_ON)
                  mask |= INV_XYZ_GYRO;
              if (hal.sensors & COMPASS_ON)
                  mask |= INV_XYZ_COMPASS;
              mpu_configure_fifo(mask);
              /* When the DMP is used, the hardware sampling rate is fixed at
                * 200Hz, and the DMP is configured to downsample the FIFO output
                * using the function dmp_set_fifo_rate. However, when the DMP is
                * turned off, the sampling rate remains at 200Hz. This could be
                * handled in inv_mpu.c, but it would need to know that
                * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
                * put the extra logic in the application layer.
                */
              dmp_get_fifo_rate(&dmp_rate);
              mpu_set_sample_rate(dmp_rate);
              inv_quaternion_sensor_was_turned_off();
              DBUGLN("DMP disabled.");

              mpu_lp_accel_mode(40);
              /* When LP accel mode is enabled, the driver automatically configures
                * the hardware for latched interrupts. However, the MCU sometimes
                * misses the rising/falling edge, and the hal.new_gyro flag is never
                * set. To avoid getting locked in this state, we're overriding the
                * driver's configuration and sticking to unlatched interrupt mode.
                *
                * TODO: The MCU supports level-triggered interrupts.
                */
              mpu_set_int_latched(0);
              hal.sensors &= ~(GYRO_ON|COMPASS_ON);
              hal.sensors |= ACCEL_ON;
              hal.lp_accel_mode = 1;
              inv_gyro_was_turned_off();
              inv_compass_was_turned_off();
          }
        }
      }
      else
      {
        isCalibrated = false;
        isRecalibration = false;
        offset[0] = CAL_MAGIC_WD;
        offset[AX] = 0;
        offset[AY] = 0;
        offset[AZ] = 0;
        offset[GX] = 0;
        offset[GY] = 0;
        offset[GZ] = 0;

        zero_average();
        isInit_reading = true;
        mpu.setDHPFMode(4); // 0.63Hz
        mpu.setDLPFMode(0); // 260Hz
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setSleepEnabled(false);
        mpu.setFIFOEnabled(0);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setRate(1);
        // mpu.dmpSetSampleRate(50);
        mpu.setIntEnabled(0x01);
        mpu.setDMPEnabled(false);
        led->setCalStart();
        delay(100);
        // Stop auto sleep alarm
        if ( xTimerIsTimerActive( *alarm ) != pdFALSE ) {
          xTimerStop(*alarm, 0);
        }
      }
      tryCounter = RETRY_LIMIT;  // Break;
      flashOP_result = 0;

      mpu.setXAccelOffset(offset[AX]);
      mpu.setYAccelOffset(offset[AY]);
      mpu.setZAccelOffset(offset[AZ]);
      mpu.setXGyroOffset(offset[GX]);
      mpu.setYGyroOffset(offset[GY]);
      mpu.setZGyroOffset(offset[GZ]);
      
      #ifdef ENABLE_DEBUG_MPU
        report6axisOffset();
      #endif
      xSemaphoreGive(*twi_comms_lock);
    }
  }
  isInitialised = true;
  enable = true;

  if (false == interruptEnabled) {
    DBUGLN("Setup MPU Interrupt");
    pinMode(PIN_MPU_INTERRUPT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INTERRUPT), MpuManagerTask::vISR_MPU, FALLING);
    interruptEnabled = true;
  }
}
  

  

void MpuManagerTask::loop()
{  
  // if (!reset) {
  //   lifeStyle->resetLifeStyleRegion();
  //   reset = true;
  // }
  // return MicroTask.Infinate;
  if (false == isInitialised) init();

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  {
    xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
    // get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    xSemaphoreGive(*twi_comms_lock);
    // get current FIFO count
    // fifoCount = mpu.getFIFOCount();
    // DBUGVAR(mpuIntStatus);

    // check for overflow (this should never happen unless our code is too inefficient)
    if (!isLowPowerMode && ((mpuIntStatus & 0x10) || fifoCount == 1024) && true == isCalibrated)
    {
      // reset so we can continue cleanly
      xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
      mpu.resetFIFO();
      DBUGLN(F("FIFO overflow!"));
      xSemaphoreGive(*twi_comms_lock);
      // return MicroTask.Infinate | MicroTask.WaitForEvent;
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }

    // For zero motion detection interrupt
    // DBUGF("mpuIntStatus = 0x%x\r\n", mpuIntStatus);
    // if (mpuIntStatus & 0x40) {
    //   DBUGLN("Motion detected");
    // }

    else {
      if ((mpuIntStatus & 0x01) && (false == isCalibrated)) {
        xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        xSemaphoreGive(*twi_comms_lock);        
        #ifdef ENABLE_DEBUG_MPU
          report6axis();
        #endif

        if (readingLength < 20) //Discard for 20 readings
        {
          readingLength++;
        }
        else if (readingLength < NUM_READINGS + 20)
        {
          readingLength++;
          buf[AX] += ax;
          buf[AY] += ay;
          buf[AZ] += az;
          buf[GX] += gx;
          buf[GY] += gy;
          buf[GZ] += gz;
        }
        else if (readingLength == NUM_READINGS + 20)
        {
          // Reach NUM_READINGS, calculate offset
          average[AX] = buf[AX] / NUM_READINGS;
          average[AY] = buf[AY] / NUM_READINGS;
          average[AZ] = buf[AZ] / NUM_READINGS;
          average[GX] = buf[GX] / NUM_READINGS;
          average[GY] = buf[GY] / NUM_READINGS;
          average[GZ] = buf[GZ] / NUM_READINGS;

          if (isInit_reading)
          {
            offset[AX] = -average[AX] / 8;
            offset[AY] = -average[AY] / 8;
            offset[AZ] = (16384 - average[AZ]) / 8;

            offset[GX] = -average[GX] / 4;
            offset[GY] = -average[GY] / 4;
            offset[GZ] = -average[GZ] / 4;

            isInit_reading = false;
          }
          else
          {
            if (abs(average[AX]) <= acel_deadzone)
              ready++;
            else
              offset[AX] -= average[AX] / acel_deadzone;

            if (abs(average[AY]) <= acel_deadzone)
              ready++;
            else
              offset[AY] -= average[AY] / acel_deadzone;

            if (abs(16384 - average[AZ]) <= acel_deadzone)
              ready++;
            else
              offset[AZ] += (16384 - average[AZ]) / acel_deadzone;

            if (abs(average[GX]) <= giro_deadzone)
              ready++;
            else
              offset[GX] -= average[GX] / (giro_deadzone + 1);

            if (abs(average[GY]) <= giro_deadzone)
              ready++;
            else
              offset[GY] -= average[GY] / (giro_deadzone + 1);

            if (abs(average[GZ]) <= giro_deadzone)
              ready++;
            else
              offset[GZ] -= average[GZ] / (giro_deadzone + 1);
          }

          // Set current offset
          xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
          mpu.setXAccelOffset(offset[AX]);
          mpu.setYAccelOffset(offset[AY]);
          mpu.setZAccelOffset(offset[AZ]);
          mpu.setXGyroOffset(offset[GX]);
          mpu.setYGyroOffset(offset[GY]);
          mpu.setZGyroOffset(offset[GZ]);
          xSemaphoreGive(*twi_comms_lock);

          if (ready == 6)
          {
            // Stop calibration
            offset[0] = CAL_MAGIC_WD; // Stamp the magic word to mark calibration
            DBUGLN("OFFSET STORE");
            deviceManager->UpdateMPUCalibration(offset, sizeof(offset));
            isCalibrated = true;
            led->setCalDone();
            delay(1000);
            DBUGLN("Calibration done");

            #ifdef ENABLE_DEBUG_MPU
              report6axisOffset();
            #endif

            sd_nvic_SystemReset();
          }
          else
          {
            // Do another 1000 readings
            readingLength = 0;
            ready = 0;
            // report6axisAVG();
            // report6axisOffset();
            zero_average();
            // Feed watchdog
            NRF_WDT->RR[0] = WDT_RR_RR_Reload;
          }
        }
      }
      // else if (true == enable && (mpuIntStatus & 0x1) && false == isLowPowerMode) {
        
      // }
      else if (true == enable && (mpuIntStatus & (isLowPowerMode?0x01:0x02)))//0x02)
      // if (mpuIntStatus & 0x02)
      {
        // // wait for correct available data length, should be a VERY short wait
        // while (fifoCount < packetSize)
        //   fifoCount = mpu.getFIFOCount();
        // DBUGVAR(fifoCount);
        // // read a packet from FIFO
        // mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        // fifoCount -= packetSize;
        xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
        if(isLowPowerMode) {
          // alarm->Reset(); // Do we still need the alarm?
          ax = mpu.getAccelerationX();
          ay = mpu.getAccelerationY();
          az = mpu.getAccelerationZ();
        }        
        else
        {
          mpu.dmpGetQuaternion(&ax, &ay, &az, &q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
        xSemaphoreGive(*twi_comms_lock);
        #ifdef ENABLE_DEBUG_MPU
          report6axis();
        #endif
        if (true == lifeStyle->getLifeStyleEnableStatus()) {
          lifeStyle->run(ax, ay, az, isLowPowerMode); 
        }

        #ifdef ENABLE_DEBUG_MPU
          reportYPRG();
        #endif

        if (false == isLowPowerMode && chk_gesture())
        {
          DBUG("Trigger Laser!");
          DBUGLN(++laser_cnt);
          scanner->trigger();
        }
        else if (true == lifeStyle->getLifeStyleEnableStatus() && true == isLowPowerMode && chk_gesture())
        {
          DBUGLN("QUERY LIFESTYLE STATUS");
          if ((int)deviceManager->lifeStyleConfig.penaltyPoints - (int)deviceManager->lifeStyleConfig.stepCount/deviceManager->lifeStyleConfig.equivalentSteps >0)
          {
            led->setNudgeBar(true);
          }
          else {
            led->setNudgeBar(false);
          }
        }
      }
    }
  }
}
//! Double Tap Detection Notification function 
void MpuManagerTask::doubleTapDetected(unsigned char direction, unsigned char count)
{
  // Hook up call back functions here.
  DBUGLN("Double tap detected!");

  // Do not re-trigger doubleTap for DOUBLE_TAP_LOCK_TIME
  if (doubleTap_lock <= millis())
  {
    if(_doubleTap_Detected_CB) {
        //! Call Double Tap Detection function in BLE Manager
      _doubleTap_Detected_CB();
    }

    doubleTap_lock = millis() + DOUBLE_TAP_LOCK_TIME;
  }
}

//! Gesture Detection function
bool MpuManagerTask::chk_gesture()
{
  #ifdef ENABLE_DEBUG_MPU
    //reportStates();
  #endif

  switch (gstrState)
  {
    case gesture_Idle:
      if (!isLowPowerMode && abs(ypr[1]) < pitch_range * M_PI / 180 && abs(ypr[2]) < roll_range * M_PI / 180 && gravity.z >= gravity_z_range)
      { 
        // Make sure it is flat. Don't care about X&Y.
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        // DBUGVAR(gravity.z);
        // DBUGLN("Gesture Movement detected!");
        gstrState = gesture_Lock;
        idle = false;
      }
      else if (!isLowPowerMode && abs(ypr[1]) < pitch_range_r * M_PI / 180 && abs(ypr[2]) > roll_range_r * M_PI / 180 && gravity.z <= gravity_z_range_r){
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        // DBUGVAR(gravity.z);
        // DBUGLN("TOP Gesture Movement detected!");
        gstrState = gesture_Lock_r;
        idle = false;
      }
      else if (isLowPowerMode && abs(az) > 14000 && lifeStyle->getRunningStatus()) {
        gstrState = gesture_Lock;
        idle = false;
      }
      else {
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        // DBUGVAR(gravity.z);
        // DBUGLN("Not flat");
      }
      
      return false;
    case gesture_Lock:
      if (!isLowPowerMode && gravity.z < 0.1 && ypr[1] > -rotate_range * M_PI / 180 && abs(ypr[2]) > roll_remain_range * M_PI / 180)
      {
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        gstrState = gesture_Idle;
        return true;
      } 
      else if (isLowPowerMode && az < - 20000 && lifeStyle->getRunningStatus()) {
        gstrState = gesture_Idle;
        return true;
      }
      else
      {
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        // DBUGVAR(gravity.z);
        // DBUGLN("Not Trigger");
        if (!idle)
        {
          idleStarted = millis();
          idle = true;
        }

        idleDuration = millis() - idleStarted;
        if (idleDuration >= IDLE_TIMEOUT)
        {
          gstrState = gesture_Idle;
          idleDuration = 0;
          idleStarted = 0;
        }
        return false;
      }
    case gesture_Lock_r:
      if (gravity.z < -0.4 && ypr[1] > -rotate_range_r * M_PI / 180 && abs(ypr[2]) < roll_remain_range_r * M_PI / 180)
      {
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        gstrState = gesture_Idle;
        return true;
      }
      else if (abs(ypr[1]) < pitch_range * M_PI / 180 && abs(ypr[2]) < roll_range * M_PI / 180 && gravity.z >= gravity_z_range)
      { 
        // Make sure it is flat. Don't care about X&Y.
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        // DBUGVAR(gravity.z);
        // DBUGLN("Gesture Movement detected!");
        gstrState = gesture_Lock;
        idle = false;
      }
      else
      {
        // DBUGVAR(ypr[1]);
        // DBUGVAR(ypr[2]);
        // DBUGVAR(gravity.z);
        // DBUGLN("Not Trigger");
        if (!idle)
        {
          idleStarted = millis();
          idle = true;
        }

        idleDuration = millis() - idleStarted;
        if (idleDuration >= IDLE_TIMEOUT)
        {
          gstrState = gesture_Idle;
          idleDuration = 0;
          idleStarted = 0;
        }
        return false;
      }
  }
  return false;
}

void MpuManagerTask::zero_average()
{
  // zero-fill all the arrays:
  for (uint8_t axis = 1; axis <= NUM_AXIS; axis++)
  {
    buf[axis] = 0;
    average[axis] = 0;
  }
}

void MpuManagerTask::recalibrate()
{
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  mpu.reset();
  xSemaphoreGive(*twi_comms_lock);
  isLowPowerMode = enable; 
  isInitialised = false;
  isCalibrated = false;
  isInit_reading = true;
  isRecalibration = true;
  xTaskNotifyGive(MpuManagerTask::xThisTask);
}

void MpuManagerTask::setarg1(char *arg)
{
  DBUG(" pitch_range: ");
  DBUG(pitch_range);
  DBUG(" roll_range: ");
  DBUG(roll_range);
  char *tok;
  tok = strtok(NULL, "/");
  pitch_range = atoi(tok);
  tok = strtok(NULL, "/");
  roll_range = atoi(tok);
  DBUG(" pitch_range: ");
  DBUG(pitch_range);
  DBUG(" roll_range: ");
  DBUG(roll_range);
}

void MpuManagerTask::setarg2(char *arg)
{
  char *tok;
  tok = strtok(arg, "/");
  rotate_range = atoi(tok);
  tok = strtok(NULL, "/");
  roll_remain_range = atoi(tok);
}

void MpuManagerTask::setarg3(char *arg)
{
  char *tok;
  tok = strtok(arg, "/");
  gravity_z_range = atof(tok);
}

void MpuManagerTask::setoffset1(char *offset_var)
{
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  char *tok;
  tok = strtok(offset_var, "/");
  mpu.setXAccelOffset(atoi(tok));
  tok = strtok(NULL, "/");
  mpu.setYAccelOffset(atoi(tok));
  tok = strtok(NULL, "/");
  mpu.setZAccelOffset(atoi(tok));
  xSemaphoreGive(*twi_comms_lock);
}

void MpuManagerTask::setoffset2(char *offset_var)
{
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  char *tok;
  tok = strtok(offset_var, "/");
  mpu.setXGyroOffset(atoi(tok));
  tok = strtok(NULL, "/");
  mpu.setYGyroOffset(atoi(tok));
  tok = strtok(NULL, "/");
  mpu.setZGyroOffset(atoi(tok));
  xSemaphoreGive(*twi_comms_lock);
}

void MpuManagerTask::enabled(bool enable) {
  enable = enable;
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  mpu.setSleepEnabled(!enable);
  xSemaphoreGive(*twi_comms_lock);
  if (!enable) DBUGLN("MPU turned off");
}

void MpuManagerTask::enterLowPowerMode(bool enable) {

  // if (enable) {
  //   mpu.setWakeCycleEnabled(1);
  //   mpu.setSleepEnabled(0);
  //   mpu.setTempSensorEnabled(1);
  //   mpu.setStandbyXGyroEnabled(1);
  //   mpu.setStandbyZGyroEnabled(1);
  //   mpu.setStandbyYGyroEnabled(1);
  //   mpu.setWakeFrequency(3);  //10Hz
  // }
  // else
  // {
  //   mpu.setWakeCycleEnabled(0);
  //   // May need to reinit DMP here?
  // }
  isLowPowerMode = enable; 
  isInitialised = false;
  // xTaskNotifyGive(xThisTask);
  // startup = false;
  // mpuInterrupt.Dettach();
  // mpuInterrupt.Deregister(&mpuEventListener);
  //   mpuInterrupt.Register(&mpuEventListener);
  //   mpuInterrupt.Attach();
}

#ifdef ENABLE_DEBUG_MPU
void MpuManagerTask::reportStates()
{
  DBUG("Gesture Detect State: ");
  DBUG( gesture_Idle == gstrState ? "gesture_Idle" : 
          gesture_Lock == gstrState ? "gesture_Lock" : 
          "UNKNOWN");

  DBUG(" Laser Triggered: ");
  DBUGLN(laser_cnt);
}

void MpuManagerTask::report6axisDMP()
{
  DBUG(aa.x);
  DBUG("\t");
  DBUG(aa.y);
  DBUG("\t");
  DBUG(aa.z);
  DBUG("\t");
  DBUG(gyro.x);
  DBUG("\t");
  DBUG(gyro.y);
  DBUG("\t");
  DBUG(gyro.z);
  DBUG("\t");
  DBUG(millis());
  DBUG("\t");
  DBUGLN(millis() - lastReport);
  lastReport = millis();
}

void MpuManagerTask::report6axisAVG()
{
  DBUG("AVERAGE:");
  DBUG(average[AX]);
  DBUG("\t");
  DBUG(average[AY]);
  DBUG("\t");
  DBUG(average[AZ]);
  DBUG("\t");
  DBUG(average[GX]);
  DBUG("\t");
  DBUG(average[GY]);
  DBUG("\t");
  DBUGLN(average[GZ]);
}

void MpuManagerTask::reportYPRG()
{
  DBUG("Gravity\t");
  DBUG(gravity.x);
  DBUG("\t");
  DBUG(gravity.y);
  DBUG("\t");
  DBUGLN(gravity.z);

  DBUG("YPR\t");
  DBUG(ypr[0] * 180/M_PI);
  DBUG("\t");
  DBUG(ypr[1] * 180/M_PI);
  DBUG("\t");
  DBUGLN(ypr[2] * 180/M_PI);
}

void MpuManagerTask::report6axis()
{
  DBUG(ax);
  DBUG("\t");
  DBUG(ay);
  DBUG("\t");
  DBUG(az);
  DBUG("\t");
  DBUG(gx);
  DBUG("\t");
  DBUG(gy);
  DBUG("\t");
  DBUG(gz);
  DBUG("\t");
  DBUG(millis());
  DBUG("\t");
  DBUG(millis() - lastReport);
  lastReport = millis();
  DBUG("|");
  DBUG(isCalibrated);
  DBUG("|");
  DBUGLN(readingLength);
}

void MpuManagerTask::report6axisOffset()
{
  DBUG("OFFSETS:");
  DBUG(offset[AX]);
  DBUG("\t");
  DBUG(offset[AY]);
  DBUG("\t");
  DBUG(offset[AZ]);
  DBUG("\t");
  DBUG(offset[GX]);
  DBUG("\t");
  DBUG(offset[GY]);
  DBUG("\t");
  DBUGLN(offset[GZ]);
}

#endif // ENABLE_DEBUG_MPU
