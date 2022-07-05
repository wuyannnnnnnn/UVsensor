#ifndef __DEBUG_H
#define __DEBUG_H

#include <MicroDebug.h>

// ======================================================================================================

// #define TEST_BARCODE_MANAGER
// #define TEST_BARCODE_TABLE_OUTPUT
// #define TEST_HASH_INDEX_TABLE_OUTPUT
// #define TEST_BARCODE_MANAGER_WITH_CODES
// #define TEST_FLASH_BYTE_WRITE
#ifdef TEST_BARCODE_TABLE_OUTPUT
#define UNIT_TEST_ENABLED
#endif
#ifdef TEST_HASH_INDEX_TABLE_OUTPUT
#define UNIT_TEST_ENABLED
#endif
#ifdef TEST_BARCODE_MANAGER_WITH_CODES
#define UNIT_TEST_ENABLED
#endif
#ifdef TEST_BARCODE_MANAGER
#define UNIT_TEST_ENABLED
#endif

#ifdef TEST_UVsensor_MANAGER
#define UNIT_TEST_ENABLED
#endif

#endif // __DEBUG_H
