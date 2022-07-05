#ifndef ENABLE_DEBUG_UARTBOXCOM
#undef ENABLE_DEBUG
#endif

#include "UARTBoxComTask.h"
#include "config.h"
#include "debug.h"
#include "Version.h"
#include "bluefruit.h"

static UARTBoxComTask *thisTask;

UARTBoxComTask::UARTBoxComTask(DataStore *dataStore, LedManagerTask *led, BarcodeManagerTask *barcodeManager, 
                               BleManagerTask *bleManager, DeviceManager *deviceManager, MpuManagerTask *mpuManager,
                               TimerHandle_t *sleepAlarm, ScannerTask *scanner, UVTask *sensor) :
  barcodeManager(barcodeManager),
  bleManager(bleManager),
  dataStore(dataStore),
  led(led),
  deviceManager(deviceManager),
  mpuManager(mpuManager),
  sleepAlarm(sleepAlarm),
  scanner(scanner),
  state(UARTBoxComState_Start),
  lastSeq(0),
  time_to_resend_ms(0),
  timeout_noRsp_ms(0),
  nakTimes(0),
  is_error_case(false),
  resetTimeout(true),
  ack_cap_to_box(BOX_COM_CMD_ACK, BOX_COM_SIZE_ACK, BOX_COM_PAYLOAD_ACK),
  nak_cap_to_box(BOX_COM_CMD_NAK, BOX_COM_SIZE_NAK, BOX_COM_PAYLOAD_NAK),
  wait_cap_to_box(BOX_COM_CMD_WAIT, BOX_COM_SIZE_WAIT, BOX_COM_PAYLOAD_WAIT),
  rdy_cap_to_box(BOX_COM_CMD_RDY, BOX_COM_SIZE_RDY, BOX_COM_PAYLOAD_RDY),
  hello_cap_to_box(BOX_COM_CMD_SEND_HELLO, BOX_COM_SIZE_SEND_HELLO, BOX_COM_PAYLOAD_HELLO),
  cap_info_cap_to_box(BOX_COM_CMD_CAPS_INFO, BOX_COM_SIZE_CAPS_INFO),
  start_cap_to_box(BOX_COM_CMD_START, BOX_COM_SIZE_START, BOX_COM_PAYLOAD_START),
  rdy_req_cap_to_box(BOX_COM_CMD_RDY_REQUEST, BOX_COM_SIZE_RDY_REQUEST),
  error_cap_to_box(BOX_COM_CMD_ERROR, BOX_COM_SIZE_ERROR)
{
  scanner->uartBoxCom = this;
}

void UARTBoxComTask::vBoxComLoop() {thisTask->loop();}

void UARTBoxComTask::setup(UARTBoxComTask *_hook) {
  /* save the ble_mac_address to use for cap_info, rdy_req and error messages */
  Bluefruit.Gap.getAddr(macAddr);
  /* construct the payload of the rdy_req message since only ble_mac_address was the missing bit,
     better to build it here once, instead of every time before it is sent */
  memcpy(rdy_req_cap_to_box.payload, macAddr, MAC_ADDRESS_SIZE);
  memcpy(rdy_req_cap_to_box.payload + MAC_ADDRESS_SIZE, BOX_COM_PAYLOAD_RDY, sizeof(BOX_COM_PAYLOAD_RDY));
  rdy_req_cap_to_box.payload_size = MAC_ADDRESS_SIZE + sizeof(BOX_COM_PAYLOAD_RDY);

  thisTask = _hook;
}

void UARTBoxComTask::loop() {
  switch(state) {
    case UARTBoxComState_Start: 
      /* turn off sleep timer */  
      xTimerStop(*sleepAlarm, 0);

      /* Configure Serial port to MCU as it is shared with scanner */
      Serial.end();
      pinMode(PIN_MCU_RX, INPUT_PULLUP); // digitalWrite(PIN_MCU_RX, HIGH);
      pinMode(PIN_MCU_TX, OUTPUT);
      digitalWrite(PIN_MCU_TX, HIGH);
      Serial.setPins((uint8_t)PIN_MCU_RX, (uint8_t)PIN_MCU_TX);
      Serial.begin(SERIAL_SPEED);

      /* Stop ble as it may interfere with the serial port */
      Bluefruit.Advertising.stop();

      /* turn led into flashing orange to indicate that capsule got into UARTBoxCom mode */
      led->flashOrange();

      /* specify the message to be sent and proceed to Send state*/
      cmd_to_send = BOX_COM_CMD_SEND_HELLO;
      state = UARTBoxComState_Send;
      break;

    case UARTBoxComState_Send: {
      /* proceed to the corresponding actions depending on cmd_to_send */
      switch (cmd_to_send) {
        case BOX_COM_CMD_SEND_HELLO:
          /* send hello message and expect hello response */ 
          SendUartMessage(hello_cap_to_box, BOX_COM_CMD_RECEIVE_HELLO, BOX_COM_SIZE_RECEIVE_HELLO);
          break;
        case BOX_COM_CMD_CAPS_INFO: {
          /* Grab and print the info needed to send to box */
          uint8_t fwVersion[SIZE_FW_VERSION_CAPS_INFO] = {FW_MAJOR_VERSION, FW_MINOR_VERSION, FW_SUB_VERSION, FW_BUILD_VERSION, 0x00};
          uint8_t dbFormatted = deviceManager->getDBFormated();
          uint8_t cartrBrc[SIZE_CARTR_BRC_CAPS_INFO]  = {0};
          deviceManager->getCartridgeId(cartrBrc);
          DBUGF("Firmware version: %X.%X.%X(%X)\r", *fwVersion, *fwVersion+1, *fwVersion+2, *fwVersion+3);
          DBUGF("MAC address: %X.%X.%X.%X.%X.%X\r", *macAddr, *macAddr+1, *macAddr+2, *macAddr+3, *macAddr+4, *macAddr+5);
          DBUGF("Cartridge barcode: %s\r", cartrBrc);

          /* construct the payload of the message that is to be sent */
          memcpy(cap_info_cap_to_box.payload, fwVersion, SIZE_FW_VERSION_CAPS_INFO);
          size_t mac_addr_offset = SIZE_FW_VERSION_CAPS_INFO;
          memcpy(cap_info_cap_to_box.payload + mac_addr_offset, macAddr, MAC_ADDRESS_SIZE);
          size_t dbFormatted_offset = SIZE_FW_VERSION_CAPS_INFO + MAC_ADDRESS_SIZE + SIZE_RESERVED_CAPS_INFO;
          cap_info_cap_to_box.payload[dbFormatted_offset] = dbFormatted;
          size_t cartr_brc_offset = SIZE_FW_VERSION_CAPS_INFO + MAC_ADDRESS_SIZE + SIZE_RESERVED_CAPS_INFO + sizeof(dbFormatted);
          memcpy(cap_info_cap_to_box.payload + cartr_brc_offset, cartrBrc, SIZE_CARTR_BRC_CAPS_INFO);
          cap_info_cap_to_box.payload_size = cartr_brc_offset + SIZE_CARTR_BRC_CAPS_INFO;
          /* send caps info message and expect ack response */ 
          SendUartMessage(cap_info_cap_to_box, BOX_COM_CMD_ACK, BOX_COM_SIZE_ACK);
          break;
        }
        case BOX_COM_CMD_START: {
          /* send start message and expect wait response
             Since rdy_req case cannot change msg_to_receive.cmd, it is set here to RST_BRC
             The order should be start - wait_box - rdy_req - rdy_box - rdy_req - rst_brc */
            uint8_t cartrBrc[SIZE_CARTR_BRC_CAPS_INFO]  = {0};
            deviceManager->getCartridgeId(cartrBrc);
            SendUartMessage(start_cap_to_box, BOX_COM_CMD_RST_BRC, BOX_COM_SIZE_WAIT_BOX);
            if (cartrBrc[2] == 'A')
            {
              led->setMagenta();
             state = UARTBoxComState_Finish;
             break;
            }
          }

          break;
        case BOX_COM_CMD_RDY_REQUEST: {
          /* send rdy_req message */ 
          SendUartMessage(rdy_req_cap_to_box);
          
          /* leave msg_to_receive.cmd unchanged and modify msg_to_receive size and cmds.
             The order should be (rdy_req - wait_box)* - rdy_req - rdy_box - rdy_req - next msg */

          /* If no rdy has been received from the box or a short timeout is triggered, expect a rdy. */
          bool isWaitRdy = (last_valid_cmd_received != BOX_COM_CMD_RDY_BOX) || 
                           (millis() > time_to_resend_ms);  
          msg_to_receive.size = isWaitRdy ? BOX_COM_SIZE_RDY_BOX :
                /* If a rdy has been received, specify msg_to_receive size and cmds based on cmd. */
                msg_to_receive.cmd == BOX_COM_CMD_RST_BRC ? BOX_COM_SIZE_RST_BRC :
                msg_to_receive.cmd == BOX_COM_CMD_BARCODES ? BOX_COM_SIZE_BARCODES :
                msg_to_receive.cmd == BOX_COM_CMD_BC_DB_VERSION ? BOX_COM_SIZE_BC_DB_VERSION :
                msg_to_receive.cmd == BOX_COM_CMD_TEST_ID ? BOX_COM_SIZE_TEST_ID :
                msg_to_receive.cmd == BOX_COM_CMD_BLE_ID ? BOX_COM_SIZE_BLE_ID : 
                (MAX_BOX_COM_SIZE_TO_RECEIVE+1);
          /* If msg_to_receive.cmd is RDY, then either RDY or WAIT is expected.
             If msg_to_receive.cmd is BARCODES, then either BARCODES or LAST is expected.
             The messages in each pair have to have the same size */
          msg_to_receive.cmds = 
                isWaitRdy ? (1 << BOX_COM_CMD_RDY_BOX) | (1 << BOX_COM_CMD_WAIT_BOX) :
                msg_to_receive.cmd == BOX_COM_CMD_BARCODES ? (1 << BOX_COM_CMD_BARCODES) | (1 << BOX_COM_CMD_LAST) :
                (1 << msg_to_receive.cmd);
          break;
        }
        case BOX_COM_CMD_WAIT:
        {
          /* If capsule is formatting, send wait message and do not expect any message back.
             If capsule has finished formatting, set led to yellow, 
             send rdy message and expect barcodes (or last) */
          if (barcodeManager->isErasing()) {
            /* since no response is expected in this case, set the resetTimeout flag to true here */
            resetTimeout = true;
            SendUartMessage(wait_cap_to_box, (BOX_COM_CMD)NULL, MAX_BOX_COM_SIZE_TO_RECEIVE+1);
          }
          else {
            led->setYellow();
            cmd_to_send = BOX_COM_CMD_RDY;
            SendUartMessage(rdy_cap_to_box, BOX_COM_CMD_BARCODES, BOX_COM_SIZE_BARCODES);
          }
          break;
        }
        case BOX_COM_CMD_RDY:
          /* send rdy message and expect barcodes (or last) */ 
          SendUartMessage(rdy_cap_to_box, BOX_COM_CMD_BARCODES, BOX_COM_SIZE_BARCODES);
          break;
        case BOX_COM_CMD_ACK:
          /* send ack message and expect the next message that is to be received based on the FSM */
          SendUartMessage(ack_cap_to_box);
          msg_to_receive.cmd = 
                last_valid_cmd_received == BOX_COM_CMD_LAST ? BOX_COM_CMD_BC_DB_VERSION :
                last_valid_cmd_received == BOX_COM_CMD_BC_DB_VERSION ? BOX_COM_CMD_TEST_ID :
                last_valid_cmd_received == BOX_COM_CMD_TEST_ID ? BOX_COM_CMD_BLE_ID : 
                (BOX_COM_CMD)NULL;
          msg_to_receive.size = 
                last_valid_cmd_received == BOX_COM_CMD_LAST ? BOX_COM_SIZE_BC_DB_VERSION :
                last_valid_cmd_received == BOX_COM_CMD_BC_DB_VERSION ? BOX_COM_SIZE_TEST_ID :
                last_valid_cmd_received == BOX_COM_CMD_TEST_ID ? BOX_COM_SIZE_BLE_ID : 
                (MAX_BOX_COM_SIZE_TO_RECEIVE+1);
          msg_to_receive.cmds = (1 << msg_to_receive.cmd);
          /* If FSM has reached the last stage, then go to Finish state */
          if (last_valid_cmd_received == BOX_COM_CMD_BLE_ID) state = UARTBoxComState_Finish;
          break;
        case BOX_COM_CMD_NAK:
          /* send nak message and expect the same message as before (leave msg_to_receive unchanged) */ 
          SendUartMessage(nak_cap_to_box);
          break;
        case BOX_COM_CMD_ERROR: {
          /* construct the payload of the message that is to be sent */
          memcpy(error_cap_to_box.payload, macAddr, MAC_ADDRESS_SIZE);
          error_cap_to_box.payload[MAC_ADDRESS_SIZE] = msg_to_receive.cmd;
          uint16_t lastSeq_error = msg_to_receive.cmd == BOX_COM_CMD_BARCODES ? (lastSeq | 0x8000) : 0x8000;
          size_t lastSeq_error_offset = MAC_ADDRESS_SIZE + BOX_COM_SIZE_CMD;
          memcpy(error_cap_to_box.payload + lastSeq_error_offset, (uint8_t*)&lastSeq_error, sizeof(lastSeq_error));
          error_cap_to_box.payload_size = lastSeq_error_offset + sizeof(lastSeq_error);
          /* send error message and expect the same message as before (leave msg_to_receive unchanged) */
          SendUartMessage(error_cap_to_box);
          break;
        }
        default: 
          /* should not enter here */
          break;
      }

      /* specify when to resend the last message that was sent (or rdy_req)
         if no response is received from the box within this time */
      uint32_t resend_timeout_ms = 
            cmd_to_send == BOX_COM_CMD_SEND_HELLO ? BOX_COM_RESEND_HELLO_TIMEOUT_MS :
            cmd_to_send == BOX_COM_CMD_CAPS_INFO ? BOX_COM_RESEND_CAPS_INFO_TIMEOUT_MS :
            cmd_to_send == BOX_COM_CMD_WAIT ? BOX_COM_RESEND_WAIT_TIMEOUT_MS : 
            BOX_COM_RESEND_RDY_REQ_TIMEOUT_MS;
      time_to_resend_ms = millis() + resend_timeout_ms;

      /* if a valid message was received (resetTimeout = true), 
         reset the timeout for long periods of no response from the box */
      if (resetTimeout) {
        uint32_t _timeout_noRsp_ms = 
              (cmd_to_send == BOX_COM_CMD_SEND_HELLO) ? BOX_COM_SEND_HELLO_TIMEOUT_MS : 
              BOX_COM_NO_RESPONSE_TIMEOUT_MS;
        timeout_noRsp_ms = millis() + _timeout_noRsp_ms; 
        resetTimeout = false;
      }
      
      /* go to receive state if the communication has not finished */
      if (state != UARTBoxComState_Finish) state = UARTBoxComState_Receive;
      break;
    }
    case UARTBoxComState_Receive:
      /* If the expected number of bytes are received */
      if(Serial.available() >= msg_to_receive.size) {
        /* save the received message in the gbuffer and flush the serial port */
        memset(gbuffer, 0, sizeof(gbuffer));
        Serial.readBytes(gbuffer, msg_to_receive.size);
        Serial.flush();

        DBUG("Message received: ");
        for (uint16_t i=0; i<msg_to_receive.size; i++) {DBUG(gbuffer[i], 16); DBUG(" ");}
        DBUGLN();

        /* if crc is correct and cmd is valid, then a valid message has been received */
        uint8_t cmd_received = gbuffer[BOX_COM_CMD_POS];
        GenerateCrc32((uint32_t*)gbuffer, (msg_to_receive.size + BOX_COM_CRC_POS)/4);
        if ( (gbuffer[msg_to_receive.size + BOX_COM_CRC_POS]     == crc[0]) && 
             (gbuffer[msg_to_receive.size + BOX_COM_CRC_POS + 1] == crc[1]) &&
             ((1 << cmd_received) & msg_to_receive.cmds)) {
          /* save the cmd to decide later what should be the next one to be received */
          last_valid_cmd_received = (BOX_COM_CMD)cmd_received;
          /* a valid response was received, so the long timeout (timeout_noRsp) can be reset */
          resetTimeout = true;
          /* a valid message was received, so reset the nak's */
          nakTimes = 0;
          /* proceed to the corresponding actions depending on cmd_received */
          switch(cmd_received) {
            case BOX_COM_CMD_RECEIVE_HELLO:
              /* set led and specify the next message to be sent */
              led->setOrange();
              cmd_to_send = BOX_COM_CMD_CAPS_INFO;
              /* add delay for sending according to the API document*/
              delay(BOX_COM_RESEND_CAPS_INFO_TIMEOUT_MS);
              break;
            case BOX_COM_CMD_ACK:
              /* set led to flashing green, specify the next message to be sent 
                 and wait for the button to be pressed */
              led->flashGreen();
              cmd_to_send = BOX_COM_CMD_START;
              state = UARTBoxComState_WaitButton;
              break;
            case BOX_COM_CMD_WAIT_BOX:
              /* if a wait is received, it means the box is busy at the moment, 
                 ask if it is ready by sending a rdy request */
              cmd_to_send = BOX_COM_CMD_RDY_REQUEST;
              /* add delay for sending according to the API document */
              if (millis() < time_to_resend_ms)
                delay (time_to_resend_ms - millis());
              break;
            case BOX_COM_CMD_RDY_BOX:
              /* if rdy is received, it means the box can resume the communication with the capsule,
                 resend a rdy request according to the API document */
              cmd_to_send = BOX_COM_CMD_RDY_REQUEST;
              break;
            case BOX_COM_CMD_RST_BRC:
              /* start formatting the flash and set led to flashing yellow */
              barcodeManager->eraseAllBarcodes();
              led->flashYellow();
              /* start sending wait to box until formatting has finished */
              cmd_to_send = BOX_COM_CMD_WAIT;
              break;
            case BOX_COM_CMD_BARCODES: {
              /* the order of bytes in this message is: 
                 cmd[1B] 'A'[1B] seq[2B] count[1B] barcodes[247B] crc[2B] end[2B];
                  - seq is an incrementing number (the packet number) 
                  - count contains the number of barcodes in this packet
                  - barcodes region contains the barcodes (8 bytes each) */
              uint16_t seq = (uint16_t)gbuffer[2] << 8 | (uint8_t)gbuffer[3];
              uint8_t count = gbuffer[4];
              uint8_t barcodes[count*BARCODE_SIZE];
              memcpy(barcodes, gbuffer + 5, sizeof(barcodes));

              DBUGF("Operation: %c\r", (char*)(gbuffer+BOX_COM_PAYLOAD_POS));
              DBUGF("Sequence: %d\r", seq);
              DBUGF("Count: %d\r", count);

              /* save the barcodes into the flash */
              barcodeManager->readyToReceive = false;
              barcodeManager->setBarcodes((const uint64_t*)barcodes, seq, count);
              /* add a soft delay to allow the capsule to save the barcodes */ 
              uint8_t soft_delay = 0;
              while(!barcodeManager->readyToReceive) soft_delay++;
              /* save the last seq number to use in error cases, 
                 where the barcode transfer can resume from the last received barcode packet */
              lastSeq = seq;
              if ( (lastSeq == 0) || is_error_case )
                /* set the led to flashing blue if this is the first barcodes packet
                   or if barcodes transfer was resumed after an error */
                led->flashBlue();
              /* specify the next message to be sent */
              cmd_to_send = BOX_COM_CMD_RDY;
              break;
            }
            case BOX_COM_CMD_LAST:
              /* set led to blue and specify the next message to be sent */
              led->setBlue();
              cmd_to_send = BOX_COM_CMD_ACK;
              break;
            case BOX_COM_CMD_BC_DB_VERSION: {
              uint32_t db_version = gbuffer[BOX_COM_PAYLOAD_POS]   << 16 | 
                                    gbuffer[BOX_COM_PAYLOAD_POS+1] << 8  | 
                                    gbuffer[BOX_COM_PAYLOAD_POS+2];
              DBUGVAR(db_version);
              /* save the barcodes db version in the flash and specify the next message to be sent */
              if (deviceManager->setDatabaseVersion(db_version) == ErrorCode_Success)    
                cmd_to_send = BOX_COM_CMD_ACK;
              else /* if the flash write fails, raise an error */
                raiseError(ERROR_REASON_WRITE_FLASH);
              break;
            }
            case BOX_COM_CMD_TEST_ID:
              /* save the test ID in the flash specify the next message to be sent */
              if (deviceManager->setTestId(gbuffer+BOX_COM_PAYLOAD_POS, TEST_ID_LEN))
                cmd_to_send = BOX_COM_CMD_ACK;   
              else /* if the flash write fails, raise an error */
                raiseError(ERROR_REASON_WRITE_FLASH);
              break;
            case BOX_COM_CMD_BLE_ID: {
              char BandID[BLE_ID_LEN] = {0};
              memcpy(BandID, (char*)(gbuffer + BOX_COM_PAYLOAD_POS), BLE_ID_LEN);
              DBUGVAR(BandID);
              /* save the band ID in the flash, specify the next message to be sent
                 and set led to magenta to declare that all information has been received */
              if (deviceManager->setBandId(BandID)) {
                led->setMagenta();
                cmd_to_send = BOX_COM_CMD_ACK;
              }
              else /* if the flash write fails, raise an error */
                raiseError(ERROR_REASON_WRITE_FLASH);
              break;
            }
            default: 
              /* should not enter here */
              break;
            
            /* reset the is_error_case if a valid information-carrying message was received */
            if ((last_valid_cmd_received != BOX_COM_CMD_RDY_BOX) && 
                (last_valid_cmd_received != BOX_COM_CMD_WAIT_BOX))
              is_error_case = false;
          }
        }
        /* if crc is not correct or cmd is not valid, then send a nak to the box */ 
        else {
          cmd_to_send = BOX_COM_CMD_NAK;
          nakTimes++;
          /* if the number of consecutive naks passes the limit, raise an error */
          if (nakTimes > BOX_COM_NAK_LIMIT)
            raiseError(ERROR_REASON_NAK);
        }
        /* go to send state, excpet before the test starts or during an error case */
        if (state != UARTBoxComState_WaitButton) state = UARTBoxComState_Send;
      }
      /* If less than the expected number of bytes is received */
      else {
        /* check for long timeout (no response timeout) only if it has been renewed,
           and trigger an error if no response has been received during this time */
        if (!resetTimeout && (millis() > timeout_noRsp_ms))
          raiseError(ERROR_REASON_TIMEOUT);
        /* if long timeout check passes, check for short timeout (resend timeout),
           and specify the message to be sent */
        else if (millis() > time_to_resend_ms) {
          /* some messages are resent (cmd_to_send remains unchanged), 
             and for some others rdy_req is sent instead, based on the FSM */
          if ((cmd_to_send == BOX_COM_CMD_ACK) || (cmd_to_send == BOX_COM_CMD_RDY) || 
              (cmd_to_send == BOX_COM_CMD_START))
            cmd_to_send = BOX_COM_CMD_RDY_REQUEST; 
          state = UARTBoxComState_Send;
        }
        else {
          /* if short timeout check also passes, remain in the receive state 
             and wait for the expected message */
        }
      }
      break;

    case UARTBoxComState_WaitButton:
      /* wait for button to be pressed */
      if (BUTTON_PRESSED == digitalRead(PIN_BUTTON1)) {

        /* if the test starts, clear the whole the configuration area except for the mpu */
        if (cmd_to_send == BOX_COM_CMD_START) {
          deviceManager->clearUserRelatedData();
          dataStore->clearHistory();
        }
        else {
          /* otherwise it is an error case */
        }
        /* send the next message (start or error message) and turn off the led */
        state = UARTBoxComState_Send;
        led->setOff();
      }
      break;

    case UARTBoxComState_Finish:
      /* block the task indefinitely to not consume any CPU time */
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	   
      break;
  }
  delay(1);
}

void UARTBoxComTask::raiseError(ERROR_REASON reason) {
  /* print the error reason */
  DBUG("Error!!! ");
  switch(reason) {
    case ERROR_REASON_TIMEOUT:
      DBUGLN("Timeout");
      break;
    case ERROR_REASON_NAK:
      DBUGLN("Too many naks sent");
      break;
    case ERROR_REASON_WRITE_FLASH:
      DBUGF("Write in flash for CMD %d failed\r\n", gbuffer[BOX_COM_CMD_POS]);  
      break;
    default:
      DBUGLN("Unknown error reason\r");
      break;
  }
  /* turn led into flashing red to indicate error state */
  led->flashRed();
  is_error_case = true;
  /* reset any error conditions */
  resetTimeout = true;
  nakTimes = 0;
  /* define the next message to be sent and wait for button */
  cmd_to_send = BOX_COM_CMD_ERROR;
  state = UARTBoxComState_WaitButton;
}

void UARTBoxComTask::GenerateCrc32(uint32_t* data, size_t len) {
  uint32_t const CRC_POLY = 0x04C11DB7;
  uint32_t val, crc32  = 0xFFFFFFFF;
  uint8_t i; 
  
  while(len--) {
    val = crc32^(*data++);
    for (i=0; i<32; i++)
      val = val & 0x80000000 ? (val<<1)^CRC_POLY : val<<1;
    crc32 = val;
  }
  crc[1] =  crc32        & 0xFF;
  crc[0] = (crc32 >> 8)  & 0xFF;
}

void UARTBoxComTask::SendUartMessage(cap_to_box_msg msg, BOX_COM_CMD _cmd_to_receive, size_t _size_to_receive) {
  /* construct the message in the gbuffer (cmd+payload+crc+\r\n) and send it via the serial port */
  memset(gbuffer, 0, sizeof(gbuffer));
  gbuffer[BOX_COM_CMD_POS] = msg.cmd;
  memcpy(gbuffer + BOX_COM_SIZE_CMD, msg.payload, msg.payload_size);
  GenerateCrc32((uint32_t*)gbuffer, (msg.size + BOX_COM_CRC_POS)/4);
  memcpy(gbuffer + msg.size + BOX_COM_CRC_POS, crc, BOX_COM_SIZE_CRC);
  memcpy(gbuffer + msg.size + BOX_COM_END_POS, BOX_COM_MSG_END, BOX_COM_SIZE_END);
  Serial.flush();
  Serial.write(gbuffer, msg.size);
  DBUG("Message sent: ");
  for (uint16_t i=0; i<msg.size; i++) {DBUG(gbuffer[i], 16); DBUG(" ");}
  DBUGLN();

  /* check if the *_to_receive parameters have their default values */
  if (_cmd_to_receive != (BOX_COM_CMD)NULL && _size_to_receive != 0) {
    msg_to_receive.cmd = _cmd_to_receive;
    msg_to_receive.size = _size_to_receive;
    /* If msg_to_receive.cmd is RST_BRC, WAIT is expected (see BOX_COM_CMD_START for more info).
       If msg_to_receive.cmd is BARCODES, BARCODES or LAST is expected (both have same size) */
    msg_to_receive.cmds = 
      _cmd_to_receive == BOX_COM_CMD_RST_BRC ? (1 << BOX_COM_CMD_WAIT_BOX) :
      _cmd_to_receive == BOX_COM_CMD_BARCODES ? (1 << _cmd_to_receive) | (1 << BOX_COM_CMD_LAST) : 
      (1 << _cmd_to_receive);
  }
}

void UARTBoxComTask::startBoxCom () {
  mpuManager->enabled(false);
  DBUGLN("Start task uartBoxCom");
  Scheduler.startLoop(vBoxComLoop, "BCom", 256, TASK_PRIO_HIGH);
}