#ifndef UARTBOXCOM_TASK_H
#define UARTBOXCOM_TASK_H

#include "DataStore.h"
#include "LedManagerTask.h"
#include "BarcodeManagerTask.h"
#include "BleManagerTask.h"
#include "DeviceManager.h"
#include "ScannerTask.h"
#include "MpuManagerTask.h"
#include "UVsensor.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/* possible values for timeout_noRsp_ms (long timeout) */                                
#define BOX_COM_SEND_HELLO_TIMEOUT_MS      10*60*1000  
#define BOX_COM_NO_RESPONSE_TIMEOUT_MS        10*1000 

/* possible values for time_to_resend_ms (short timeout) */
#define BOX_COM_RESEND_HELLO_TIMEOUT_MS          1000 
#define BOX_COM_RESEND_CAPS_INFO_TIMEOUT_MS       500
#define BOX_COM_RESEND_WAIT_TIMEOUT_MS            500 
#define BOX_COM_RESEND_RDY_REQ_TIMEOUT_MS     02*1000 

/* nak limit that triggers an error state */
#define BOX_COM_NAK_LIMIT 2

/* position of key elements of any message sent/received */
#define BOX_COM_CMD_POS     0
#define BOX_COM_PAYLOAD_POS 1
#define BOX_COM_CRC_POS     BOX_COM_END_POS-BOX_COM_SIZE_CRC
#define BOX_COM_END_POS     -BOX_COM_SIZE_END

/* size of key elements of any message sent/received */
size_t const BOX_COM_SIZE_CMD = 1;
size_t const BOX_COM_SIZE_CRC = 2;
size_t const BOX_COM_SIZE_END = 2;

#define TEST_ID_LEN 16
#define BLE_ID_LEN  MAX_BANDID_LEN

/* size of elements contained in the caps_info message */
size_t const SIZE_FW_VERSION_CAPS_INFO = 5;
size_t const SIZE_RESERVED_CAPS_INFO   = 5;
size_t const SIZE_CARTR_BRC_CAPS_INFO  = CARTRIDGE_ID_SIZE;

/**
 * @brief CMDs of messages between capsule and box
 * 
 * These are the unique identifiers of any message/command
 * that is exchanged between capsule and box
 */
enum BOX_COM_CMD {
  /* CMD of messages from capsule to box */
  BOX_COM_CMD_ACK = 0x01, // can also be sent from box to capsule
  BOX_COM_CMD_NAK,
  BOX_COM_CMD_WAIT,
  BOX_COM_CMD_RDY,
  BOX_COM_CMD_SEND_HELLO,
  BOX_COM_CMD_CAPS_INFO = 0x07,
  BOX_COM_CMD_START,
  BOX_COM_CMD_RDY_REQUEST = 0x11,
  BOX_COM_CMD_ERROR = 0x14,

  /* CMD of messages from box to capsule */
  BOX_COM_CMD_RECEIVE_HELLO = 0x06,
  BOX_COM_CMD_RST_BRC = 0x09,
  BOX_COM_CMD_BARCODES,
  BOX_COM_CMD_TEST_ID,
  BOX_COM_CMD_BLE_ID = 0x0E,
  BOX_COM_CMD_LAST = 0x10,
  BOX_COM_CMD_WAIT_BOX = 0x12,
  BOX_COM_CMD_RDY_BOX,
  BOX_COM_CMD_BC_DB_VERSION = 0x15
};

/* size of messages from capsule to box (CMD + payload + CRC + \r\n) */
size_t const BOX_COM_SIZE_ACK         = 8; // can also be sent from box to capsule
size_t const BOX_COM_SIZE_NAK         = 8;
size_t const BOX_COM_SIZE_WAIT        = 12;  
size_t const BOX_COM_SIZE_RDY         = 8;
size_t const BOX_COM_SIZE_SEND_HELLO  = 12;
size_t const BOX_COM_SIZE_CAPS_INFO   = 40; 
size_t const BOX_COM_SIZE_START       = 12;
size_t const BOX_COM_SIZE_RDY_REQUEST = 16;
size_t const BOX_COM_SIZE_ERROR       = 16;
size_t const MAX_BOX_COM_SIZE_TO_SEND = 40;

/* size of messages from box to capsule (CMD + payload + CRC + \r\n) */
size_t const BOX_COM_SIZE_RECEIVE_HELLO  = 12;
size_t const BOX_COM_SIZE_RST_BRC        = 24;
size_t const BOX_COM_SIZE_BARCODES       = 256;
size_t const BOX_COM_SIZE_TEST_ID        = 24; 
size_t const BOX_COM_SIZE_BLE_ID         = 16;
size_t const BOX_COM_SIZE_LAST           = 256;
size_t const BOX_COM_SIZE_WAIT_BOX       = 12;
size_t const BOX_COM_SIZE_RDY_BOX        = 12;
size_t const BOX_COM_SIZE_BC_DB_VERSION  = 8;
size_t const MAX_BOX_COM_SIZE_TO_RECEIVE = 256;

/* fixed payload of messages from capsule to box */
uint8_t const BOX_COM_PAYLOAD_ACK[]   = {0x61, 0x63, 0x6B}; // can also be sent from box to capsule
uint8_t const BOX_COM_PAYLOAD_NAK[]   = {0x6E, 0x61, 0x6B};
uint8_t const BOX_COM_PAYLOAD_WAIT[]  = {0x77, 0x61, 0x69, 0x74};
uint8_t const BOX_COM_PAYLOAD_RDY[]   = {0x72, 0x64, 0x79};
uint8_t const BOX_COM_PAYLOAD_HELLO[] = {0x68, 0x65, 0x6C, 0x6C, 0x6F};
uint8_t const BOX_COM_PAYLOAD_START[] = {0x73, 0x74, 0x61, 0x72, 0x74};

/* ending characters of any messsage exchanged between capsule and box (\r\n) */
uint8_t const BOX_COM_MSG_END[] = {0x0D, 0x0A};

/* states of the capsule-box communication FSM */
enum UARTBoxComState {
  /* starting point of the FSM, the capsule enter this state when startBoxCom is called */
  UARTBoxComState_Start,
  /* messages to box are sent in this state and timeouts are reset if needed */
  UARTBoxComState_Send,
  /* messages from the box are received and handled in this state,
     errors can also be triggered from this state as described in ERROR_REASON */
  UARTBoxComState_Receive,
  /* wait for the button to be pressed to carry on,
     enter this state right before a test starts or when an error is triggered */
  UARTBoxComState_WaitButton,
  /* enter this IDLE state when the capsule-box communication is successfully over */
  UARTBoxComState_Finish,
};

/**
 * @brief This struct is used to build all possible capsule to box messages
 * 
 * @cmd contains the unique cmd of the message
 * @size contains the size of the message
 * @payload contains the actual payload of the message. Its length is
 * statically assigned to be equal to the maximum possible payload_size.
 * Part of it is used to save the actual payload.
 * @payload_size contains the size of the actual payload.
 * 
 * If the message has a fixed payload, then the first constructor is used,
 * where payload is specified.
 * 
 * If the message has a variable payload, then the second constructor is used,
 * and the payload is specified later.
 * 
 * All messages are constructed during class initialization.
 */
struct cap_to_box_msg {
  BOX_COM_CMD const cmd;
  size_t const size;
  uint8_t payload[MAX_BOX_COM_SIZE_TO_SEND-BOX_COM_SIZE_CMD-BOX_COM_SIZE_CRC-BOX_COM_SIZE_END];
  uint8_t payload_size;
  cap_to_box_msg(BOX_COM_CMD _cmd, const size_t _size, const uint8_t* _payload): 
                      cmd(_cmd), size(_size), payload_size(sizeof(_payload)) {
    memcpy(payload, _payload, payload_size);
  }
  cap_to_box_msg(BOX_COM_CMD _cmd, const size_t _size): cmd(_cmd), size(_size) {}
};

class UARTBoxComTask {
  private:
    BarcodeManagerTask *barcodeManager;
    BleManagerTask *bleManager;
    DataStore *dataStore;
    LedManagerTask *led;
    DeviceManager *deviceManager;
    MpuManagerTask *mpuManager;
    TimerHandle_t *sleepAlarm;
    ScannerTask *scanner;
    UVTask *sensor;
    UARTBoxComState state;

    /* Enumeration of reasons that trigger an error state */
    enum ERROR_REASON {
      /* long periods without response from the box */
      ERROR_REASON_TIMEOUT,
      /* too many (more than NAK_LIMIT) invalid messages from the box */
      ERROR_REASON_NAK,
      /* capsule could not write data into the flash */
      ERROR_REASON_WRITE_FLASH
    };

    /* specifies the next message to be sent according to the FSM */
    BOX_COM_CMD cmd_to_send; 
    
   /**
    * @brief Defines the next message to be received according to the FSM
    * 
    * @cmd contains the main cmd to be received
    * @cmds contains all the valid cmds that can be received based on cmd
    * @size contains the size of the expected message to be received 
    * 
    * All messages with cmds contained in @cmds should have the same size  
    */
   struct {
      BOX_COM_CMD cmd;
      unsigned cmds;
      size_t size;
    } msg_to_receive;
    
    /* contains the last valid cmd that was received */
    BOX_COM_CMD last_valid_cmd_received;
    
    /* contains the last seq number received during barcode transfer, is used in error handling */
    uint16_t lastSeq;
    
    /* contains the time(in ms) to resend a message after a small period without response from the box */
    uint32_t time_to_resend_ms;
    
    /* contains the time(in ms) to trigger a timeout error for long periods without response from the box */
    uint32_t timeout_noRsp_ms;
    
    /* is used to reset the timeout_noRsp when needed */
    bool resetTimeout;
    
    /* is used to trigger a nak error if it surpasses the NAK_LIMIT */
    uint8_t nakTimes;
    
    /* is used to indicate an error case */
    bool is_error_case;

    /* is used to hold the messages to be sent and to be received */
    uint8_t gbuffer[MAX(MAX_BOX_COM_SIZE_TO_RECEIVE, MAX_BOX_COM_SIZE_TO_SEND)];
    
    /* is used to hold the last 2 bytes of a 32bit crc generated by GenerateCrc32 */
    uint8_t crc[BOX_COM_SIZE_CRC];
    
    /* contains the capsule ble mac address */
    uint8_t macAddr[MAC_ADDRESS_SIZE];
    
    /* instances for all the capsule to box messages */
    cap_to_box_msg ack_cap_to_box, nak_cap_to_box, wait_cap_to_box, 
                   rdy_cap_to_box, hello_cap_to_box, cap_info_cap_to_box, 
                   start_cap_to_box, rdy_req_cap_to_box, error_cap_to_box;
        
    /**
     * @brief Triggers the error state
     * 
     * Prints the error reason, resets the error conditions, 
     * turns led into flashing red and waits for button to be pressed 
     * 
     * @param reason contains the reason that triggered the error
     */
    void raiseError(ERROR_REASON reason);
    
    /**
     * @brief Generates a 32bit crc
     * 
     * Generates a 32bit crc from data using the CRC_POLY and saves it into the crc buffer.
     * 
     * @param data contains the data to be used for crc calculation as a uint32_t pointer
     * @param len contains the length of @data to be used counting in uint32_t's
     */
    void GenerateCrc32(uint32_t* data, size_t len);

   /**
    * @brief Sends a message to box and specifies the expected message to be received
    * 
    * Writes in the gbuffer the appropriate message, sends it via the Serial port, 
    * and specifies the next message to be received via the *_to_receive parameters.
    * If the function is called with the default *_to_receive parameters, 
    * then msg_to_receive struct remains unchanged
    * 
    * @param msg contains the message to be sent from the available cap_to_box_msg's
    * @param _cmd_to_receive is copied to msg_to_receive.cmd 
    *                       (msg_to_receive.cmds is calculated based on this parameter)
    * @param _size_to_receive is copied to msg_to_receive.size
    */
    void SendUartMessage(cap_to_box_msg msg, BOX_COM_CMD _cmd_to_receive = (BOX_COM_CMD)NULL, size_t _size_to_receive = 0);
    
    static void vBoxComLoop();
  public:
    UARTBoxComTask(DataStore *dataStore, LedManagerTask *led,  BarcodeManagerTask *barcodeManager, 
                   BleManagerTask *bleManager, DeviceManager *deviceManager, MpuManagerTask *mpuManager,
                   TimerHandle_t *sleepAlarm, ScannerTask *scanner,UVTask *sensor);
    void setup(UARTBoxComTask *_hook);
    
    /* contains the FSM of the box-capsule communication */
    void loop();
    
    /* starts the task's loop when a cartridge barcode is scanned */
    void startBoxCom();
    
    /* returns the current state of the UARTBoxComTask */
    UARTBoxComState getBoxComState() {return state;};
};

#endif //  UARTBOXCOM_TASK_H