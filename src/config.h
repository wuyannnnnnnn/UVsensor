#ifndef CONFIG_H
#define CONFIG_H

#define STATUS_SUCCESS          0x00
#define STATUS_FAILURE          0x01
#define STATUS_USER_EXIST       0x02
#define STATUS_USER_NOT_EXIST   0x03
#define STATUS_USER_FULL        0x04

#ifdef ARDUINO_NRF52_DK
#define RED_LED             PIN_LED1
#define GREEN_LED           PIN_LED2
#define BLUE_LED            PIN_LED3
#define TRIGGER_MODULE      1
#define BUTTON_1            PIN_BUTTON1
#define BUTTON_2            PIN_BUTTON2
#define MPU_INTERRUPT_PIN   0
#define MPU_I2C_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#endif

#ifdef ARDUINO_PRIMO_CORE
#undef RED_LED
#define RED_LED             USER_LED
#define TRIGGER_MODULE      2
#define BUTTON_1            3
#define BUTTON_2            4
#define MPU_INTERRUPT_PIN   0
#define MPU_I2C_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#endif

#ifdef ARDUINO_NUDGE_LASER
#define TRIGGER_MODULE      PIN_LASER_TRIGGER
#define BUTTON_1            PIN_BUTTON1
#define BUTTON_2            PIN_BUTTON2
#define MPU_INTERRUPT_PIN   PIN_MPU_INTERRUPT
#define MPU_I2C_ADDRESS     MPU6050_ADDRESS_AD0_HIGH
#endif

#define NUMBER_CODES    500
#define NUMBER_HISTORY  100
// #define NUMBER_SLOTS    8

#define RECOMENDATION_TIMEOUT  (10 * 1000)
#define HALF_RECOMENDATION_TIMEOUT  (2 * 1000)
#define MODE_SWITCH_TIMEOUT (2 * 1000)
#define REBOOT_TIMEOUT (8 * 1000)
#define NUDGE_BAR_QUERY_TIMEOUT (2 * 1000)

#define MINIMUM_AUTO_SLEEP_TIME 1                //! 1 minute
#define MAXIMUM_AUTO_SLEEP_TIME 10               //! 10 minutes 
#define DEFAULT_AUTO_SLEEP_TIME 2                //! 3 minutes

#define BAND_MATCH_STATUS_TIMEOUT    2000    //! 5 seconds
#define BAND_MATCH_EXIT_TIMEOUT      10    //! 10 milliseconds
#define COMMAND_STATUS_TIMEOUT  (3*1000)   //! 3 seconds
//#define AUTO_SLEEP_TIME 10 * 1000

#endif // !CONFIG_H
