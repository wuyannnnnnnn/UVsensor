
#ifndef _VERSION_H_
#define _VERSION_H_

#include <stdio.h>

//! VERSION_STRING "4.2.8"
// This will be overwritten by the tag via CI for OTA updates
#define FW_MAJOR_VERSION 1
#define FW_MINOR_VERSION 1
#define FW_SUB_VERSION   4
  
#define FW_BUILD_VERSION 1  // 1 -- release, 2 -- demo, 3 -- development, 4-- Internal testing, 99 -- ad5e383, 98 -- d72a17b

#endif //_VERSION_H_s
