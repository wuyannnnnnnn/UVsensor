#ifndef ENABLE_DEBUG_BARCODE_MANAGER
#undef ENABLE_DEBUG
#endif

#include "BarcodeManagerTask.h"
// #include "config.h"
// #define ENABLE_DEBUG_BARCODE_MANAGER
// #define ENABLE_BARCODE_MANAGER_VERIFY
#include "LedManagerTask.h"
#include "MpuManagerTask.h"
#include "MicroDebug.h"

#include "nrf_soc.h"

const uint8_t BarcodeManagerTask::aeskey[] = {
    0x00, 0x11, 0x22, 0x33, 0x00, 0x11, 0x22, 0x33,
    0x00, 0x11, 0x22, 0x33, 0x00, 0x11, 0x22, 0x33};
static nrf_ecb_hal_data_t encryption_parm;

TaskHandle_t BarcodeManagerTask::xThisTask = NULL;

BarcodeManagerTask::BarcodeManagerTask(FlashManagerTask *flashManager, DeviceManager *deviceManager, void* led)
    : flashManager(flashManager), deviceManager(deviceManager), led(led), initEcb(false),
      newRecomAllergyBlock(true) {}

void BarcodeManagerTask::setup() {
  readyToReceive = true;
  DBUGLN("Initialising AES driver");

  // initEcb = nrf_ecb_init();
  // nrf_ecb_set_key(aeskey);
  memset(&encryption_parm, 0, sizeof(aeskey));
  memcpy(encryption_parm.key, aeskey, sizeof(aeskey));
}

double BarcodeManagerTask::recommendationDecode(uint8_t recommendation) {
  double decodeValue;
  switch (recommendation) {
  case 0:
    decodeValue = 0.0;
  case 255:
    decodeValue = -1.0;
    break;
  case 1:
    decodeValue = 1.0;
    break;
  case 2:
    decodeValue = 2.0;
    break;
  case 3:
    decodeValue = 3.0;
    break;
  case 4:
    decodeValue = 4.0;
    break;
  default:
    decodeValue = ((double)recommendation - 5.0) / 250.0;
    break;
  }
  return decodeValue;
}

Recommendation BarcodeManagerTask::evalRecommendation(uint8_t recommendation) {
  DBUG("Found recommendation : ");
  DBUGLN(recommendation);
  double recValue = recommendationDecode(recommendation);
  DBUG("Found recValue : ");
  DBUGLN(recValue);

  if (recValue < (double)0.0) {
    return Recommendation_NotSet;
  } else if (recValue > (double)0.999 && recValue < (double)1.001) {
    return Recommendation_Good;
  } else if (recValue > (double)2.999 && recValue < (double)3.001) {
    return Recommendation_Good;
  } else if (recValue > (double)3.999 && recValue < (double)4.001) {
    return Recommendation_Bad;
  } else if (recValue < (double)1.0) {
    return Recommendation_Bad;
  } else {
    return Recommendation_NotSet;
  }
}
bool BarcodeManagerTask::getBarcode(Barcode &barcode, bool forceMode) {
  uint8_t entryCount = getEntryCount(HashBucket.HashTableHeader.EntryCountMap);
  uint64_t srcBarCode = barcode.getCode();
  for (int i = 0; i < entryCount; i++) {
    if (HashBucket.BarcodeData[i].Barcode == srcBarCode &&
        HashBucket.BarcodeData[i].mRecommendation != 0xFF) {

      // DBUG("Barcode: ");
      // DBUG(numToString(srcBarCode));
      // DBUGLN(" found!");
      //! Set Recommendation
      int activeUserCount = 0;
      double userScores[MAX_SHARED_RECOMMENDATIONS+1];
#ifdef ENABLE_NUDGESHARE      
      if (((LedManagerTask*)led)->isFamilyScanEnabled() || forceMode) {
        // DBUGLN("Enter group shopping mode");
        // check enabled user only
        for(int j = 0; j < MAX_SHARED_RECOMMENDATIONS + 1; j++) {
          uint64_t sampleId = deviceManager->getUserSampleId(j);
          if (sampleId != UINT64_MAX) {
            bool enabledUserSlot = deviceManager->getEnabledUserSlotConfig(sampleId);
            // DBUGVAR(enabledUserSlot);
            if (enabledUserSlot) {
              if (j == 0) {
                userScores[activeUserCount] = recommendationDecode(HashBucket.BarcodeData[i].mRecommendation);
              } else {
                userScores[activeUserCount] = recommendationDecode(HashBucket.BarcodeData[i].sRecommendation[j - 1]);
              }
              activeUserCount++;
            }
          }          
        }
      } 
#endif
      if (activeUserCount == 0) {
        DBUGLN("Enter individual shopping mode");
        // Active master only
        userScores[0] = recommendationDecode(HashBucket.BarcodeData[i].mRecommendation);
        activeUserCount = 1;
      }
      
      barcode.setRecommendation(nUDGEshareAlgorithm(userScores, activeUserCount));
      //! Set DataFlag
      barcode.setDataFlag(HashBucket.BarcodeData[i].mflag);
      return true;
    }
  }
  for (int i = 0; i < entryCount; i++) {
      Serial.println(numToString(HashBucket.BarcodeData[i].Barcode));
      Serial.println(HashBucket.BarcodeData[i].mRecommendation);
  }
  DBUG("Barcode: ");
  DBUG(numToString(srcBarCode));
  DBUGLN(" not found!");

  return false;
}

bool BarcodeManagerTask::checkIfExists(uint64_t srcBarCode) {
  uint8_t entryCount = getEntryCount(HashBucket.HashTableHeader.EntryCountMap);
  for (int i = 0; i < entryCount; i++) {
    if (HashBucket.BarcodeData[i].Barcode == srcBarCode) {
      return true;
    }
  }
  return false;
}

uint32_t BarcodeManagerTask::getHashAddress(String barcode) {
  // unsigned char plaintext[17];
  #ifdef ENABLE_DEBUG_BARCODE_MANAGER
  int err_code;
  #endif

  int len = barcode.length();
  for (int i = 0; i < (16 - len); i++) {
    barcode.concat('0');
  }

  // barcode.getBytes(plaintext, 17);
  memcpy(encryption_parm.cleartext, barcode.c_str(), sizeof(encryption_parm.cleartext));
  // encryption_parm
  #ifdef ENABLE_DEBUG_BARCODE_MANAGER
  err_code = 
  #endif
  sd_ecb_block_encrypt(&encryption_parm);

  // nrf_ecb_crypt(ciphertxt, plaintext);
  uint32_t *longInt = (uint32_t *)&(encryption_parm.ciphertext[0]);
  uint32_t hashAddr;
  hashAddr = (longInt[3] % HASH_BUCKET_COUNT) & HASH_ADDR_MASK;
  hashAddr <<= HASH_BUCKET_ADDR_WIDTH;
  return hashAddr;
}

uint8_t BarcodeManagerTask::getEntryCount(uint16_t EntryCountMap) {
  uint8_t entryCnt = 0;
  for (uint8_t i = 0; i < MAX_BUCKET_ENTRIES; i++) {
    if ((EntryCountMap & (0x1 << i)) == 0) {
      entryCnt = i + 1;
    } else {
      entryCnt = i;
      break;
    }
  }
  return entryCnt;
}

bool BarcodeManagerTask::readHashBucket(uint32_t flashAddr) {
  bool result;
  result = flashManager->ReadBarcodeRegion(flashAddr, &HashBucket,
                                          HASH_BUCKET_SIZE_IN_BYTES);
  return result;
}

bool BarcodeManagerTask::writeHashBucket(uint32_t flashAddr) {
  bool result;
  result = flashManager->WriteBarcodeRegion(flashAddr, &HashBucket,
                                           HASH_BUCKET_SIZE_IN_BYTES);
  return result;
}

bool BarcodeManagerTask::searchBarcode(Barcode &barcode, bool forceMode) {
  // barcode = barCode;
  // state = BarcodeManagerState_findBarcode;
  uint32_t flashAddress;
  // bool accessResult;
  bool searchResult;
  // get the flash address by hashing barcode string
  flashAddress = getHashAddress(barcode.toString());
  // DBUG("flashAddress: 0x");
  // DBUGLN(flashAddress, HEX);
  // read  the existing barcode table to get the HashBucket.
  // accessResult = 
  readHashBucket(flashAddress);
  // DBUGVAR(accessResult);
  searchResult = getBarcode(barcode, forceMode);
  return searchResult;
}

void BarcodeManagerTask::eraseAllBarcodes() {
  DBUGLN("Erase barcodes start.");
  if (false == deviceManager->eraseUserConfig()) {
    DBUGLN("Erase User Config Failed");
  }
  else {
    DBUGLN("Erase User Config Success.");
  }

  if (false == flashManager->EraseHashBarcodeRegion(0, HASH_INDEX_TABLE_REGION_SIZE + BARCODES_DATA_TABLE_REGION_SIZE)) {
    DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
  }
  else {
    DBUGLN("Erase all barcodes commands send successfully.");
  }
}

bool BarcodeManagerTask::isErasing() { return false == flashManager->isFlashReady(); }

bool BarcodeManagerTask::setBarcodes(const uint64_t *barcodes,
                                     uint16_t seqNumber, uint8_t num) {
  uint64_t barcode;
  bool accessResult;
  for (int i=0; i < num; i++) {
    barcode = *(barcodes+i);
    String bStr = numToString(barcode); // get the flash address by hashing barcode string
    flashAddress = getHashAddress(bStr);
    #ifdef ENABLE_DEBUG_BARCODE_MANAGER
      DBUG("Barcode = ");
      DBUGLN(bStr);
      DBUG("Barcode # = 0x");
      DBUG((uint32_t)longBcode[0], HEX);
      DBUG("  : 0x");
      DBUGLN((uint32_t)longBcode[1], HEX);
      DBUG("Barcode #");
      DBUG(i);
      DBUG(" : hashAddress = 0x");
      DBUGLN(flashAddress, HEX);
    #endif
    // read  the existing barcode table to get the HashBucket.
    accessResult = readHashBucket(flashAddress);

    // if the barcode already exists in HashBucket return
    if (checkIfExists(barcode) == true) {
      DBUG(" Barcode : ");
      DBUG(bStr);
      DBUGLN(" already exists! aborting!!");

      #ifdef ENABLE_DEBUG_BARCODE_MANAGER
        int entryCnt = getEntryCount(HashBucket.HashTableHeader.EntryCountMap);
        DBUGVAR(entryCnt);
        uint16_t entryCountMap = HashBucket.HashTableHeader.EntryCountMap;
        DBUG("entryCountMap = 0x");
        DBUGLN(entryCountMap, HEX);
      #endif
      continue;
    }

    int entryCnt = getEntryCount(HashBucket.HashTableHeader.EntryCountMap);
    #ifdef ENABLE_DEBUG_BARCODE_MANAGER
      DBUGVAR(entryCnt);
    #endif
    // enter new barcode into the next free location in HashBucket
    HashBucket.BarcodeData[entryCnt].Barcode = barcode;
    // update HashBucket Entry count
    HashBucket.HashTableHeader.EntryCountMap <<= 1;
    // write back the HashBucket with the inserted Barcode and updated
    // EntryCount
    accessResult = writeHashBucket(flashAddress);

    // write updated entry count into the Hash Index table
    uint16_t entryCountMap = HashBucket.HashTableHeader.EntryCountMap;
    #ifdef ENABLE_DEBUG_BARCODE_MANAGER
      DBUG("entryCountMap = 0x");
      DBUGLN(entryCountMap, HEX);
    #endif
    // get hashIndexTableAddress by shifting flashAddress by the
    // HASH_BUCKET_ADDR_WIDTH
    
    hashIndexTableAddress = (flashAddress >> HASH_BUCKET_ADDR_WIDTH) * ENTRY_COUNT_MAP_SIZE;
    // update Hash Index Table with new entryCountMap
    // while(flashManager->isFlash_busy()); // wait for flash
    while (retryCnt < 4) {
      accessResult = flashManager->WriteHashIndexRegion(hashIndexTableAddress, &entryCountMap, ENTRY_COUNT_MAP_SIZE);
      if (accessResult == false) {
        writeFailCnt++;
      }
      accessResult = flashManager->ReadHashIndexRegion(hashIndexTableAddress, &rdback, ENTRY_COUNT_MAP_SIZE);
      if (accessResult == false) {
        readFailCnt++;
      }
      if (rdback == entryCountMap) {
        writeVerified = true;
        break;
      }
      retryCnt++;
    }

    if (writeVerified == false) {
      DBUG("HITAddr = 0x");
      DBUGLN(hashIndexTableAddress, HEX);
      DBUG("Error writing HashIndex!!");
      DBUG(" - wrote: 0x");
      DBUG(entryCountMap, HEX);
      DBUG("  -  rdback = 0x");
      DBUGLN(rdback, HEX);
      DBUG("  writeFailCnt = 0x");
      DBUGLN(writeFailCnt, HEX);
      DBUG("  readFailCnt = 0x");
      DBUGLN(readFailCnt, HEX);
      return false;
    }
  }
  
  readyToReceive = true;

  return true;
}

uint16_t _count, _Idxstart; 

bool BarcodeManagerTask::setAllergyRecommendations(uint8_t category, const uint8_t *recommendations,
                                            size_t num, uint16_t seqNumber, 
                                            uint8_t count, bool endOfBlock) {
  #ifdef ENABLE_DEBUG_BARCODE_MANAGER
  int tmp = 0;
  #endif

  // DBUGVAR(uxQueueSpacesAvailable(recomAllergyBlockQueue));

  if (newRecomAllergyBlock) {
    // memcpy(currentReceiveBlock.recommendation, recommendations, num);
    // currentReceiveBlock.sequenceNumber = seqNumber;

    // recommendations_buffer contains the incoming recommendations for a certain block
    // reset the recommendations_buffer if it is a new block
    memset(recommendations_buffer, 0xFF, sizeof(recommendations_buffer));
    baseFlashAddress = seqNumber << (HASH_BUCKET_ADDR_WIDTH + HASH_INDEX_TABLE_BLOCK_WIDTH);
    flashManager->ReadHashIndexRegion(
      (baseFlashAddress >> HASH_BUCKET_ADDR_WIDTH)* ENTRY_COUNT_MAP_SIZE, hashIndexBlock,
      (HASH_INDEX_TABLE_BLOCK_SIZE * ENTRY_COUNT_MAP_SIZE));

    // currentReceiveBlock.count = num;

    // _count should contain the number of incoming recommendations for a certain block
    // num contains the the number of recommendations in *recommendations
    _count = num;
    // _Idxstart contains the starting position to fill the recommendations_buffer
    _Idxstart = 0;
    // DBUG("[newBlockFlag]currentReceiveeBlock.count = ");
    // DBUGLN(currentReceiveBlock.count);
    newRecomAllergyBlock = false;
  }
  else
  {
    // memcpy(currentReceiveBlock.recommendation+count, recommendations, num);
    // currentReceiveBlock.count += num;

    // increase _Idxstart and _count if it is not a new block
    _Idxstart += _count;
    _count += num;
    // DBUG("[newBlockFlag]currentReceiveBlock.count = ");
    // DBUGLN(currentReceiveBlock.count);
  // }
  // for (uint i = 0; i<num; i++) {
  //   xQueueSend(recomAllergyQueue, recommendations+i, portMAX_DELAY);
  }

  // fill recommendations_buffer starting from proper position
  memcpy(recommendations_buffer + _Idxstart, recommendations, num);

  // if it is the end of a block, write the contents of recommendations_buffer in the proper address 
  if (endOfBlock) {
    // DBUGLN("END OF BLOCK!");
    // DBUGVAR(currentReceiveBlock.count);
    // DBUGVAR(uxQueueSpacesAvailable - uxQueueSpacesAvailable(recomAllergyBlockQueue));

    uint16_t rdBlockCount;
    rdBlockCount = getHashIndexBlockCount(hashIndexBlock);
    if (rdBlockCount != _count) {
      Serial.println("<<< block entry count Mismatch!! >>>");
      Serial.print("Hash Addr based on Seq.Num.: 0x");
      Serial.println(baseFlashAddress, HEX);
      Serial.print("Wrote : ");
      Serial.print(_count);
      Serial.print(" - readback : ");
      Serial.println(rdBlockCount);
      newRecomAllergyBlock = true;

      return false;
    }

    newRecomAllergyBlock = true;
    
    // xQueueSend(recomAllergyBlockQueue, &currentReceiveBlock, portMAX_DELAY);
  
    // Serial.printf("_Idxstart = %d, _count = %d \r\n ", _Idxstart, _count);
    uint16_t idx = 0;
    for (uint8_t blockIndex = 0; blockIndex < HASH_INDEX_TABLE_BLOCK_SIZE; blockIndex++) {
      uint16_t curEntryCnt = getEntryCount(hashIndexBlock[blockIndex]);
      for (uint8_t entryIndex = 0; entryIndex < curEntryCnt; entryIndex++) {
        if (WRITE_RECOMMENDATION == category)
        {
          flashAddress = baseFlashAddress +
                        (blockIndex * HASH_BUCKET_SIZE_IN_BYTES +
                          sizeof(HashTableHeaderStruct_t) +
                          entryIndex * sizeof(BarcodeDataStruct_t) + BARCODE_SIZE + RECOMMENDATION_SIZE * deviceManager->getUserSlotIndex());
          writeVerifyAllergyRecommendations(flashAddress, recommendations_buffer+idx++, RECOMMENDATION_SIZE, WRITE_RECOMMENDATION);
        }
        else if (WRITE_LIFESTYLE == category) {
          flashAddress = baseFlashAddress +
                        (blockIndex * HASH_BUCKET_SIZE_IN_BYTES +
                          sizeof(HashTableHeaderStruct_t) +
                          entryIndex * sizeof(BarcodeDataStruct_t) + BARCODE_SIZE + RECOMMENDATION_SIZE * (MAX_SHARED_RECOMMENDATIONS + 1));

          uint8_t lifestyle = (*(recommendations_buffer + idx++) >> (ALLERGY_BIT_INDEX+1)) | LIFESTYLE_MASK;

          #ifdef ENABLE_DEBUG_BARCODE_MANAGER
            DBUG("Hash Addr: 0x");
            DBUGLN(flashAddress, HEX);
            DBUG("Current Block: ");
            DBUGLN(blockIndex);
            DBUG("Incoming lifestyle: ");
            DBUGLN(lifestyle);

          #endif
          writeVerifyAllergyRecommendations(flashAddress, &lifestyle, LIFESTYLE_SIZE, WRITE_LIFESTYLE);  
        }
        else if (WRITE_ALLERGY == category) {
          flashAddress = baseFlashAddress +
                        (blockIndex * HASH_BUCKET_SIZE_IN_BYTES +
                          sizeof(HashTableHeaderStruct_t) +
                          entryIndex * sizeof(BarcodeDataStruct_t) + BARCODE_SIZE + RECOMMENDATION_SIZE * (MAX_SHARED_RECOMMENDATIONS + 1) + LIFESTYLE_SIZE);

          uint8_t allergy = (((*(recommendations_buffer + idx++) & ALLERGY_BIT_MASK) >> ALLERGY_BIT_INDEX) << allergySlotIndex) | allergySlotMask; 

          #ifdef ENABLE_DEBUG_BARCODE_MANAGER
            DBUG("Hash Addr: 0x");
            DBUGLN(flashAddress, HEX);
            DBUG("Current Block: ");
            DBUGLN(blockIndex);
            DBUG("Incoming allergy: ");
            DBUGLN(allergy);

          #endif
          writeVerifyAllergyRecommendations(flashAddress, &allergy, ALLERGY_SIZE, WRITE_ALLERGY);  
        }
      }
    }
  }

  readyToReceive = true;

  return true;
}

bool BarcodeManagerTask::writeVerifyAllergyRecommendations(uint32_t flashAddress,
                                                    const uint8_t *rec,
                                                    uint16_t len,
                                                    uint8_t category) {
  uint8_t retryCnt = 0;
  uint8_t writeFailCnt = 0;
  uint8_t readFailCnt = 0;
  uint8_t rdback;
  uint8_t bitMask = 0xFF;
  bool accessResult = false;
  bool writeVerified = false;
  if (category == WRITE_RECOMMENDATION) {
    bitMask = 0x00;
  }
  else if (category == WRITE_ALLERGY) {
    bitMask = allergySlotMask;
  }
  else if (category == WRITE_LIFESTYLE) {
    bitMask = LIFESTYLE_MASK;
  }
  else {
    DBUGLN("Error!! Received byte not in any known category");
  }
  while (retryCnt < 4) {
    accessResult = flashManager->WriteBarcodeRegion(flashAddress, rec, len);
    if (accessResult == false) {
      writeFailCnt++;
    }
    accessResult = flashManager->ReadBarcodeRegion(flashAddress, &rdback, len);
    if (accessResult == false) {
      readFailCnt++;
    }
    if ( (rdback | bitMask) == *rec ) {
      writeVerified = true;
      break;
    }
    retryCnt++;
  }
  if (writeVerified == false) {
    DBUG("recAddr = 0x");
    DBUGLN(flashAddress, HEX);
    DBUGLN("Error writing Allergy/Recommendation!!");
    DBUG(" - wrote: 0x");
    DBUG(*rec, HEX);
    DBUG("  -  rdback = 0x");
    DBUGLN((rdback | bitMask), HEX);
    DBUG("  writeFailCnt = 0x");
    DBUGLN(writeFailCnt, HEX);
    DBUG("  readFailCnt = 0x");
    DBUGLN(readFailCnt, HEX);
  }
  return writeVerified;
}

String BarcodeManagerTask::numToString(uint64_t barcodeNum) {
  uint64_t val = barcodeNum;

  char digitString[20] = "0";

  // IMPROVE: do we need to support non-13 digit barcodes?
  int length = 13;
  for (int i = 0; i < length; i++) {
    uint8_t digit = val % 10;
    val /= 10;

    digitString[length - 1 - i] = '0' + digit;
  }
  digitString[length] = '\0';

  return String(digitString);
}

uint8_t BarcodeManagerTask::getHashIndexBlockCount(uint16_t *HashIndexTable) {
  uint8_t blkCount = 0;
  for (int i = 0; i < HASH_INDEX_TABLE_BLOCK_SIZE; i++) {
    blkCount += getEntryCount(HashIndexTable[i]);
  }
  return blkCount;
}

void BarcodeManagerTask::getNextHashIndices(uint16_t *_hashIndexBlock,
                                            int &blockIndex, int &entryIndex) {
  int bidx;
  int eidx;
  int curEntryCnt;
  bidx = blockIndex;
  // eidx = ++entryIndex;
  eidx = entryIndex;
  while (bidx < HASH_INDEX_TABLE_BLOCK_SIZE) {
    curEntryCnt = getEntryCount(_hashIndexBlock[bidx]);
#ifdef ENABLE_DEBUG_BARCODE_MANAGER
    // DBUGLN("=====getNextHashIndices========");
    // DBUG("entryIndex = ");
    // DBUGLN(entryIndex);
    // DBUG("bidx = ");
    // DBUGLN(bidx);
    // DBUG("curEntryCnt = ");
    // DBUGLN(curEntryCnt);
    // DBUGLN("=============");
#endif
    if (curEntryCnt == 0) {
      bidx++;
    } else {
      for (int i = eidx; i < curEntryCnt; i++) {
        blockIndex = bidx;
        entryIndex = i;
        ;
#ifdef ENABLE_DEBUG_BARCODE_MANAGER
        DBUG("eidx [on exit]= ");
        DBUGLN(eidx);
        DBUG("bidx [on exit]= ");
        DBUGLN(bidx);
#endif
        return;
      }
      bidx++;
      eidx = 0;
    }
  }

  // if (curEntryCnt == 0 ||
  //     curEntryCnt < (eidx + 1)) // empty or index overrun
  // {
  //   bidx++;   // goto next block as this is empty
  //   eidx = 0; // reset eidx
  // } else {
  //   break;
  // }
  // }

  // blockIndex = bidx;
  // entryIndex = eidx;
}

void BarcodeManagerTask::outputBarcodesTable(void) {
  //Serial.println("bucketNum, index, Barcode, Recommendation");
  Serial.printf("Bucket#, Entry#, Barcode, Recommendation,");
  for (int i=0; i<MAX_SHARED_RECOMMENDATIONS; i++) { Serial.printf(", Share[%d]", i);}
  Serial.println(", Current Allergy Bit(Index), All Allergy Slots, Lifestyle");
  for (long i = 0; i < (HASH_BUCKET_COUNT);
       i++) // go through all hash buckets in Barcode Table
  {
    if (readHashBucket(
            (uint32_t)(i * (uint32_t)HASH_BUCKET_SIZE_IN_BYTES)) == true) {
      uint16_t EntryCount, EntryCountMap;
      EntryCountMap = HashBucket.HashTableHeader.EntryCountMap;
      EntryCount = getEntryCount(EntryCountMap);
      for (int j = 0; j < EntryCount; j++) {
        String barcode = numToString(HashBucket.BarcodeData[j].Barcode);
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(",");
        Serial.print(barcode);
        Serial.print(",");
        Serial.print(HashBucket.BarcodeData[j].mRecommendation);
        for (int k=0; k<MAX_SHARED_RECOMMENDATIONS; k++) {
          Serial.print(",");
          Serial.print(HashBucket.BarcodeData[j].sRecommendation[k]);
        }
        Serial.print(",");
        Serial.printf("%d(%d)", (HashBucket.BarcodeData[j].mflag.mAllergy >> allergySlotIndex) & 0x1, allergySlotIndex);
        Serial.print(",");
        Serial.print(HashBucket.BarcodeData[j].mflag.mAllergy);
        Serial.print(",");
        Serial.println(HashBucket.BarcodeData[j].mflag.mLifeStyle);
      }
      // if (i == 4000)
      //   break;

    } else {
      Serial.print("Error Reading HashBucket #");
      Serial.println(i);
    }
  }
}
