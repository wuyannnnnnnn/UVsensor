#ifndef BARCODE_MANAGER_TASK_H
#define BARCODE_MANAGER_TASK_H

#include <Arduino.h>
#include "debug.h"

#include "Barcode.h"
#include "FlashManagerTask.h"
#include "DeviceManager.h"
#include "nrf_ecb.h"

#define Rule_09572 0.948
#define Rule_Avg 0.84
#define Risk_Ratio 4.0/3.0

#define WRITE_RECOMMENDATION 0x00
#define WRITE_ALLERGY        0x01
#define WRITE_LIFESTYLE      0X02

#define HASH_ADDR_WIDTH (int)(17)
#define HASH_BUCKET_ADDR_WIDTH (FLASH_ADDRESS_WIDTH - HASH_ADDR_WIDTH)
#define NUM_OVERFLOW_HASH_BUCKETS 128

#define HASH_ADDR_MASK (0x1FFFFFF) >> (HASH_BUCKET_ADDR_WIDTH)

#define RESERVED_HDR_BYTES 14
#define MAX_SHARED_RECOMMENDATIONS 4  //6

#define ALLERGY_BIT_INDEX 0
#define ALLERGY_BIT_MASK (0x01) << (ALLERGY_BIT_INDEX)
#define LIFESTYLE_MASK 0xF0

#define HASH_BUCKET_COUNT                                        \
  ((BARCODES_DATA_TABLE_REGION_SIZE >> HASH_BUCKET_ADDR_WIDTH) - \
   NUM_OVERFLOW_HASH_BUCKETS)

#define HASH_BUCKET_SIZE_IN_BYTES (0x1 << HASH_BUCKET_ADDR_WIDTH)

// this is how many hash index table entries to read
#define HASH_INDEX_TABLE_BLOCK_WIDTH (7)
#define HASH_INDEX_TABLE_BLOCK_SIZE (0x1 << HASH_INDEX_TABLE_BLOCK_WIDTH)

typedef struct BarcodeDataStruct_t
{
  uint64_t Barcode;
  uint8_t mRecommendation;                             // main (user) recommendation
  uint8_t sRecommendation[MAX_SHARED_RECOMMENDATIONS]; // shared recommendations
  DataFlag mflag;
} BarcodeDataStruct;

typedef union BarcodeDataUnion_t {
  BarcodeDataStruct_t BarcodeData;
  uint8_t byteStream[sizeof(BarcodeDataStruct_t)];
} BarcodeDataUnion;

typedef struct HashTableHeaderStruct_t
{
  uint16_t EntryCountMap;               // this is handled as a bitmap to avoid having to
                                        // erase the entire block each time
  uint8_t reserved[RESERVED_HDR_BYTES]; // shared recommendations
} HashTableHeaderStruct;

#define MAX_BUCKET_ENTRIES                      \
  (uint16_t)((0x1 << HASH_BUCKET_ADDR_WIDTH) -  \
             sizeof(HashTableHeaderStruct_t)) / \
      (sizeof(BarcodeDataStruct_t))

#define ENTRY_COUNT_MAP_SIZE sizeof(uint16_t)

#define BARCODE_SIZE sizeof(uint64_t)

#define RECOMMENDATION_SIZE sizeof(uint8_t)

#define ALLERGY_SIZE sizeof(uint8_t)

#define LIFESTYLE_SIZE sizeof(uint8_t)

typedef struct HashBucketDataStruct_t
{
  HashTableHeaderStruct_t HashTableHeader;
  BarcodeDataStruct_t BarcodeData[MAX_BUCKET_ENTRIES];
} HashBucketDataStruct;

typedef union HashBucketDataUnion_t {
  HashBucketDataStruct_t HashBucketData;
  uint8_t byteStream[sizeof(HashBucketDataStruct_t)];
} HashBucketDataUnion;

typedef struct FlashBlockDataStruct_t
{
  HashBucketDataStruct_t
      HashTableHeader[FLASH_BLOCK_SIZE / sizeof(HashBucketDataStruct_t)];
} FlashBlockDataStruct;

typedef union FlashBlockDataUnion_t {
  FlashBlockDataStruct_t FlashBlockData;
  uint8_t byteStream[sizeof(FlashBlockDataStruct_t)];
} FlashBlockDataUnion;

typedef struct RecomendationBlock_t
{
  uint16_t sequenceNumber;
  uint8_t count;
} RecomendationBlock;

class BarcodeManagerTask
{
#ifdef UNIT_TEST_ENABLED
public:
#else
private:
#endif
  FlashManagerTask *flashManager;
  DeviceManager *deviceManager;
  void *led;
  HashBucketDataStruct_t HashBucket;

  static const uint8_t aeskey[16];
  bool initEcb;
  uint8_t allergySlotIndex;
  uint8_t allergySlotMask;

  uint8_t retryCnt = 0;
  uint8_t writeFailCnt = 0;
  uint8_t readFailCnt = 0;
  uint16_t rdback;
  bool writeVerified = false;
  uint16_t hashIndexBlock[HASH_INDEX_TABLE_BLOCK_SIZE];
  uint32_t hashIndexTableAddress;
  uint32_t flashAddress;
  uint32_t baseFlashAddress = 0;
  uint8_t recommendations_buffer[256];
  
  uint32_t getHashAddress(String barcode);

  bool getBarcode(Barcode &barcode, bool forceMode = false);
  String numToString(uint64_t barcodeNum);
  void getNextHashIndices(uint16_t *hashIndexBlock, int &blockIndex,
                          int &entryIndex);

  double recommendationDecode(uint8_t recommendation);
  Recommendation evalRecommendation(uint8_t recommendation);
  bool checkIfExists(uint64_t srcBarCode);
  uint8_t getHashIndexBlockCount(uint16_t *HashIndexTable);
  bool writeVerifyAllergyRecommendations(uint32_t flashAddress, const uint8_t *rec,
                                  uint16_t len, uint8_t category);

  Recommendation nUDGEshareAlgorithm(double *individualScores, int size) {
    Recommendation recommended = Recommendation_Bad;
    bool returnResult = false;
    int numOfSe1 = 0;
    int numOfSs1 = 0;
    double numOfSe09572 = 0;
    double numOfSs09572 = 0;
    
    for (int i = 0; i < size; i++) {
      double value = individualScores[i];
      if (value != 2.0) {
        if (value == 3.0) {
            recommended = Recommendation_Good;
            returnResult = true;
        } else if (value == 4.0) {
            recommended = Recommendation_Bad;
            returnResult = true;
        } else if (value == 1) {
            numOfSe1++;
        } else if (value < 1) {
            numOfSs1++;
            if (value >= Rule_09572) {
                numOfSe09572++;
            } else {
                numOfSs09572++;
            }
        }
      }
    }
    
    if (returnResult) {
        return recommended;
    }
    
    if (numOfSe1 > numOfSs1) {
        return Recommendation_Good;
    } else if (numOfSe1 == numOfSs1) {
        if (numOfSe09572 > numOfSs09572) {
            return Recommendation_Good;
        } else {
            double countOf09572 = 0;
            double sumOf09572 = 0;
            for (int i = 0; i < size; i++) {
                  double value = individualScores[i];
                  if (value != 2.0) {
                    if (value < Rule_09572) {
                        sumOf09572+=value;
                        countOf09572++;
                    }
                  }
            }
            double avg = sumOf09572 / countOf09572;
            if (avg >= Rule_Avg) {
                return Recommendation_Good;
            } else {
                return Recommendation_Bad;
            }
        }
    } else {
        return Recommendation_Bad;
    }
  }
  bool _isErasing;
  static TaskHandle_t xThisTask;
public:
  BarcodeManagerTask(FlashManagerTask *flashManager, DeviceManager *deviceManager, void* led);

  void setup();
  uint8_t getEntryCount(uint16_t EntryCountMap);
  bool readHashBucket(uint32_t flashAddr);
  bool writeHashBucket(uint32_t flashAddr);
  void eraseAllBarcodes();
  bool isErasing();
  bool searchBarcode(Barcode &barCode, bool forceMode = false);
  void updateAllergySlotIndex() {
    allergySlotIndex = deviceManager->getAllergySlotIndex(); 
    allergySlotMask = ~(0x01 << allergySlotIndex);
  }
  volatile bool readyToReceive;
  bool newRecomAllergyBlock;
  /*============================================================================================
  // this method is called by the BLE manager when a number of barcodes have
  been received
  // parameter *barcodes is a pointer the the array of received barcodes of type
  uint64_t
  // parameter sequenceNumber is the first Sequence number received.
  =============================================================================================*/
  bool setBarcodes(const uint64_t *barcodes, uint16_t seqNumber,
                   uint8_t numOfCodes);

  /*=================================================================================================
  // this method is called by the BLE manager when a set number of
  recommendations have been received
  // parameter *recommendations is a pointer the the array of received
  recommendations of type uint8_t
  // parameter sequenceNumber is the Sequence number for the received block.
  ===================================================================================================*/
  bool setAllergyRecommendations(uint8_t catagory, const uint8_t *recommendations, size_t numberRecommendations, 
                          uint16_t seqNumber, uint8_t num, bool endOfBlock);

  /*============================================================================================
  // this method is called by the BLE manager when a number of combined barcodes
  and recommendations have been received
  // parameter *barcodes is a pointer the the array of received barcodes of type
  uint64_t
  // parameter sequenceNumber is the first Sequence number received.
  =============================================================================================*/
  // bool setBoth(const uint64_t *barcodes, uint16_t seqNumber, uint8_t numOfCodes);

  void outputBarcodesTable(void);

};

#endif // !BARCODE_MANAGER_TASK_H
