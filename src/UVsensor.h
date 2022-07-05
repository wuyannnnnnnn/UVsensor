#ifndef _UVsensor_H
#define _UVsensor_H

#include "LTR390UV.h"

//#include <functional>

//typedef std::function<void(void)> ext_RTC_alarm_callback_t;


class UVTask
{
  private:

    LTR390UV sensor;

  public:

    void printALS(uint32_t myALS);
    void printUVI(uint32_t myUVI);
    
    //void vTaskALS( void *pvParameters);
    //void vTaskUVI( void *pvParameters);

    void loopALS();
    void loopUVI();
   // uint32_t getCurrentALS(int mode = LTR390UV_MODE_ALS);
   // uint32_t getCurrentUVI(int mode = LTR390UV_MODE_UVI);
    
    void setup(); 
  };

#endif //  _UVsensor_H