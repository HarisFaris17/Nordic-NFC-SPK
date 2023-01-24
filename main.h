#ifndef   MAIN_H__
#define   MAIN_H__

#include "eeprom.h"
#include "nfc_spk_eeprom3.h"

#define BUTTON_COUNTER_UP                   15
#define BUTTON_COUNTER_DOWN                 26
#define BUTTON_DONE                         20


#define START_ADDR_DATA                        0x0AC0                                        /* The 5 bits LSB represents the address in the page specified for the 12 bits upper the 5 bits LSB.
                                                                                             this 5 bits LSB will be incremented internally by eeprom after writing a byte (the pointer incremented)
                                                                                             . After writing in the last address of the specifed page, the pointer will be rolled over to the first
                                                                                             address in the page
                                                                                          */
#define MAX_NUMBER_OF_SPK                     10
//#define ADDR_INDEXES_ACTIVE_SPK               START_ADDR_DATA
#define   ADDR_NFC_LENGTH_NFC_ID              START_ADDR_DATA


#define ADDR_START_SPK_LIST                   (ADDR_NFC_LENGTH_NFC_ID+BYTES_PER_PAGE)//(ADDR_INDEXES_ACTIVE_SPK+BYTES_PER_PAGE)
//#define ADDR

//#define OFFSET_NFC_ID_LENGTH                  (0)
//#define OFFSET_NFC_ID                         (OFFSET_NFC_ID_LENGTH+1)
//#define OFFSET_COUNTER(nfc_id_len)            (OFFSET_NFC_ID + nfc_id_len)
//#define OFFSET_SPK(nfc_id_len)                (OFFSET_COUNTER(nfc_id_len) + sizeof(counter_t))      

#define OFFSET_COUNTER                                    0
#define OFFSET_SPK                                        (OFFSET_COUNTER + sizeof(counter_t))  
#define OFFSET_NFC_ID_LENGTH                              (OFFSET_SPK + 1)
#define OFFSET_NFC_ID                                     (OFFSET_NFC_ID_LENGTH + 1)                      

//#define OFFSET_COUNTER_IN_SPK                  (0)
////#define OFFSET_INDEXES_SPK_LIST               (OFFSET_COUNTER_ACTIVE_SPK+1)

#define DELAY_CHANGE_STATE_DISPLAY             2000                                          /// delay state change from START to COUNTING or from COUNTING DONE to IDLE
//#define INTERVAL_ADV_COUNTING_DONE             500                                            // define how long in msec interval between counting done advertisement

#define NUM_OF_TICKS_DETECTION_DELAY_BUTTON     20

#define TIMER_TICKS_PER_SHOT                    100
#define NUM_OF_TICKS_CHANGE_NFC                 10

#define APP_BLE_OBSERVER_PRIO 3

#define ADVERTISING_DURATION                    100

#endif  //MAIN_H