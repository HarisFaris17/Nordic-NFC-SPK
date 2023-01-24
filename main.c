#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#include "app_error.h"
#include "bsp.h"
#include "hardfault.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "sdk_macros.h"
#include "sdk_config.h"

#include "adafruit_pn532.h"
#include "nfc_t2t_parser.h"
#include "nfc_t4t_cc_file.h"
#include "nfc_t4t_hl_detection_procedures.h"
#include "nfc_ndef_msg_parser.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ssd1306_oled.h"

#include "app_button.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "eeprom.h"

#include "nfc_spk_eeprom3.h"

#include "nrf_sdh_ble.h"
#include "nrf_sdh.h"

#include "adv.h"

//#include "ble_advertising.h"

//#include "ble_advertising.h"

#define SEL_RES_CASCADE_BIT_NUM            3                                              /// Number of Cascade bit within SEL_RES byte.
#define SEL_RES_TAG_PLATFORM_MASK          0x60                                           /// Mask of Tag Platform bit group within SEL_RES byte.
#define SEL_RES_TAG_PLATFORM_BIT_OFFSET    5                                              /// Offset of the Tag Platform bit group within SEL_RES byte.

#define TAG_TYPE_2_UID_LENGTH              7                                              /// Length of the Tag's UID.
#define TAG_TYPE_2_DATA_AREA_SIZE_OFFSET   (T2T_CC_BLOCK_OFFSET + 2)                      /// Offset of the byte with Tag's Data size.
#define TAG_TYPE_2_DATA_AREA_MULTIPLICATOR 8                                              /// Multiplicator for a value stored in the Tag's Data size byte.
#define TAG_TYPE_2_FIRST_DATA_BLOCK_NUM    (T2T_FIRST_DATA_BLOCK_OFFSET / T2T_BLOCK_SIZE) /// First block number with Tag's Data.
#define TAG_TYPE_2_BLOCKS_PER_EXCHANGE     (T2T_MAX_DATA_EXCHANGE / T2T_BLOCK_SIZE)       /// Number of blocks fetched in single Tag's Read command.

#define TAG_TYPE_4_NDEF_FILE_SIZE           255                                           /// Size of the buffer for NDEF file.
#define TAG_TYPE_4_NLEN_FIELD_SIZE          2                                             /// Size of NLEN field inside NDEF file.


#define BUZZER 31


#define STORE_VAR_START_ADDR_FLASH          0x3e000                                       /// starting address to store data in flash
#define STORE_VAR_END_ADDR_FLASH            0x3ffff                                       /// end address (exclusive) to store data in flash
#define OFFSET_ADDR_COUNTER                 0x0                                           /// offset address of counter reference to STORE_VAR_START_ADDR_FLASH in flash to store the counter 
#define OFFSET_ADDR_LENGTH_RFID_ID          (OFFSET_ADDR_COUNTER+4)                       /// offset address of length of RFID id in flash reference to OFFSET_ADDR_COUNTER (note : the counter is 4-byte, i.e. uint32_t)
#define OFFSET_ADDR_RFID_ID                 (OFFSET_ADDR_LENGTH_RFID_ID+4)                /* offset address of RFID id in flash reference OFFSET_ADDR_LENGTH_RFID_ID 
                                                                                          (note : one can't write only 1 byte to the flash, the flash only allow 
                                                                                          at least a page of bytes, our hypothesis one page is 4 bytes @ ref to
                                                                                          nrf_storage_nvmc*/

//#define INDEX_OF_COUNTER                    0
//#define INDEX_OF_LENGTH_NFC                 4
//#define INDEX_OF_NFC_ID                     5

#define MAX_SPK_COUNT_PER_NFC               5
#define MAX_NFC_ID_COUNT                    5



//#define ADDR_SPK_WTIH_COUNTER(SPK_ID)       (START)
// @brief
//typedef struct
//{
//    spk_id_t  spk_id;
//    counter_t counter;
//}spk_t;

//typedef struct
//{
//    //spk_id_t spk_list[MAX_NUMBER_OF_SPK];
//    uint8_t  indexes_of_spk[MAX_NUMBER_OF_SPK];
//    uint8_t  number_of_spk;
//}spk_index_list





/**
 * @brief Possible Tag Types.
 */
typedef enum
{
    NFC_T2T = 0x00,      ///< Type 2 Tag Platform.
    NFC_T4T = 0x01,      ///< Type 4A Tag Platform.
    NFC_TT_NOT_SUPPORTED ///< Tag Type not supported
} nfc_tag_type_t;

// @brief Possible state of display
typedef enum
{
    IDLE,
    CHOOSE_SPK,
    CONTINUE,
    START,
    COUNTING,
    COUNTING_DONE,
} state_type_t;


// =================Declarations=================

static inline uint16_t addr_eeprom_spk_with_counter(uint16_t spk_id);
                                        
static void update_display_counter();

static void update_display_choose_spk();

//static void display_continue();

static void button_init();

static void timer_handler(void * p_context);

static void timer_button_handler(void *p_context);

static void timer_advertising_handler(void * p_context);

static void update_display_state(state_type_t state);

static void display_counting_done();

static void display_continue();

static void display_choose_spk();

static void startup_spk_counter_eeprom();

static void save_current_counter();

static void toggle_two_times_buzzer();


//static void save_eeprom();

//static void read_eeprom();

//static void fstorage_handler(nrf_fstorage_evt_t * p_evt);

// =================================================

// @brief current display state
static state_type_t m_state;

static uint8_t m_choose_spk = 0;

// @brief object to store the received nfc
nfc_a_tag_info m_nfc_tag;

// @brief object to store current active nfc
active_nfc m_active_nfc;

// @brief object of twi
static nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);

// @brief object timer
APP_TIMER_DEF(m_timer);

// @brief object timer to count how long button has been pressed
//APP_TIMER_DEF(m_button_timer); 

// @brief object timer for advertising that counting is done
APP_TIMER_DEF(m_timer_advertising);

// @brief counter to count how many counting done advertisement has been sent.
static uint8_t m_counter_counting_done = 0;

// @brief counter to count how many counting advertisement has been sent
static uint8_t m_counter_counting = 0;

/* @brief counts how many m_button_timer has ticked. This is used to determine whether the board should be change to
          IDLE or not*/
//static uint32_t button_ticks_counter = 0;

// @brief this determine whether the m_button_timer should continue ticking or not. Since we are using 
//static bool continue_m_button_timer = false;


// @brief object to store active spk list
//static spk_index_list m_spk_list;

// @brief data to store/send data from/to EEPROM
//static eeprom_data m_data;


// object fstorage to store the counting and 
//NRF_FSTORAGE_DEF(nrf_fstorage_t m_fstorage) = {
//  // the start address in flash to store bytes
//  .start_addr = 0x3e000,
//  // the end address (the end address is exclusive) in flash to store bytes.
//  /** the total bytes that can be stored between start_addr and end_addr is 
//  *end_addr-start_addr = 0x3ffff - 0x3e000 = 0x1fff (it is more than enough to store just RFID id, 
//  the length of RFID id, and counter)
//  **/
//  .end_addr   = 0x3ffff,
//  .evt_handler= fstorage_handler,
//};

//static void fstorage_handler(nrf_fstorage_evt_t * p_evt){
//    if(p_evt->result != NRF_SUCCESS){
//      printf("The operation to fstorage failed!\n");
//      return;
//    }
//    else{
//      if(p_evt->id == NRF_FSTORAGE_EVT_WRITE_RESULT){
//        printf("Writing operation on fstorage succedd!\n");
//      }
//      else if(p_evt->id == NRF_FSTORAGE_EVT_READ_RESULT){
//        printf("Reading operation on fstorage succedd!\n");
//      }
//      else if(p_evt->id == NRF_FSTORAGE_EVT_ERASE_RESULT){
//        printf("Erase operation on fstorage succedd!\n");
//      }
//    }
//}

/**
 * @brief Macro for handling errors returne by Type 4 Tag modules.
 */
#define T4T_ERROR_HANDLE(ERR_CODE, LOG) \
    if (ERR_CODE != NRF_SUCCESS)        \
    {                                   \
        NRF_LOG_INFO(LOG, ERR_CODE);    \
        return NRF_ERROR_INTERNAL;      \
    }

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.common_evt.conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.common_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONNECTED:
            //bsp_board_led_on(CONNECTED_LED_PIN);
            //bsp_board_led_off(CONNECTABLE_ADV_LED_PIN);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            // BUZZER indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}
/**
 * @brief Function for initializations not directly related to Adafruit.
 */
void utils_setup(void)
{
    //bsp_board_init(BSP_INIT_LEDS);
    ret_code_t err;
    //err = NRF_LOG_INIT(NULL);
    err = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    //APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    button_init();

    nrf_gpio_cfg_output(BUZZER);
    //toggle_two_times_buzzer();

    err = app_timer_init();
    APP_ERROR_CHECK(err);
    nrf_delay_ms(15);
    err = app_timer_create(&m_timer,APP_TIMER_MODE_SINGLE_SHOT,timer_handler);
    APP_ERROR_CHECK(err);
    /* even though ideally we should use APP_TIMER_MODE_REPEATED, since we want to count how many tick when the 
       BUTTON_DONE pressed. but sometimes the board randomly determine who should prioritized first, sometimes it
       is m_detection_delay_timer_id from app_button module and sometimes m_button_timer. When m_button_timer prioritized 
       first and we use timer mode APP_TIMER_MODE_REPEATED and we press the BUTTON_DONE to make the m_button_timer active
       the board doesn't feel the release of BUTTON_DONE. Therefore, the m_button_timer will count ideally to infinite.\
       Therefore it is wise to used APP_TIMER_MODE_SINGLE_SHOT, hence when the handler called the timer m_detection_delay_timer_id
       automatically stopped by the board, hence the event of from BUTTON_DONE from timer m_timer can be fel by the board 
       */

    //err = app_timer_create(&m_button_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_button_handler);
    //APP_ERROR_CHECK(err);

    err = app_timer_create(&m_timer_advertising, APP_TIMER_MODE_SINGLE_SHOT, timer_advertising_handler);
    APP_ERROR_CHECK(err);

    NRF_LOG_INFO("Request");
    err = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err);
     NRF_LOG_INFO("Request succedd");

    NRF_LOG_INFO("start get");
    uint32_t ram_start = 0;
    err = nrf_sdh_ble_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err);
    NRF_LOG_INFO("succedd");
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
#if !defined (S112)
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
#endif // !defined (S112)
    err = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err);

    // Enable BLE stack.
    err = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    //uint8_t nfc_id[] = {0x44, 0x00, 0x11, 0x89};
    //NRF_LOG_INFO("Init advertising");
    //err = adv_init();
    //APP_ERROR_CHECK(err);

    //NRF_LOG_INFO("Advertising data config");
    //err = adv_data_config(1,2,3, nfc_id, 4);
    //APP_ERROR_CHECK(err);

    //NRF_LOG_INFO("Start advertising");
    //err = adv_start_or_update(1,2,3, nfc_id, 4);
    //APP_ERROR_CHECK(err);
    //NRF_LOG_INFO("Advertising started");

    //err = nrf_fstorage_init(&m_fstorage,&nrf_fstorage_nvmc,NULL);
    //APP_ERROR_CHECK(err);
}


/**
 * @brief Function for analyzing NDEF data coming either from a Type 2 Tag TLV block or
 *        Type 4 Tag NDEF file.
 */
void ndef_data_analyze(uint8_t * p_ndef_msg_buff, uint32_t nfc_data_len)
{
    ret_code_t err_code;

    uint8_t  desc_buf[NFC_NDEF_PARSER_REQIRED_MEMO_SIZE_CALC(MAX_NDEF_RECORDS)];
    uint32_t desc_buf_len = sizeof(desc_buf);

    err_code = ndef_msg_parser(desc_buf,
                               &desc_buf_len,
                               p_ndef_msg_buff,
                               &nfc_data_len);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error during parsing a NDEF message.");
    }

    ndef_msg_printout((nfc_ndef_msg_desc_t *) desc_buf);
}


/**
 * @brief Function for reading data  from a Type 2 Tag Platform.
 */
ret_code_t t2t_data_read(nfc_a_tag_info * p_tag_info, uint8_t * buffer, uint32_t buffer_size)
{
    ret_code_t err_code;
    uint8_t    block_num = 0;

    // Not enough size in the buffer to read a tag header.
    if (buffer_size < T2T_FIRST_DATA_BLOCK_OFFSET)
    {
        return NRF_ERROR_NO_MEM;
    }

    if (p_tag_info->nfc_id_len != TAG_TYPE_2_UID_LENGTH)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }

    // Read blocks 0 - 3 to get the header information.
    err_code = adafruit_pn532_tag2_read(block_num, buffer);
    if (err_code)
    {
        NRF_LOG_INFO("Failed to read blocks: %d-%d", block_num,
                     block_num + T2T_END_PAGE_OFFSET);
        return NRF_ERROR_INTERNAL;
    }

    uint16_t data_bytes_in_tag = TAG_TYPE_2_DATA_AREA_MULTIPLICATOR *
                                 buffer[TAG_TYPE_2_DATA_AREA_SIZE_OFFSET];

    if (data_bytes_in_tag + T2T_FIRST_DATA_BLOCK_OFFSET > buffer_size)
    {
        return NRF_ERROR_NO_MEM;
    }

    uint8_t blocks_to_read = data_bytes_in_tag / T2T_BLOCK_SIZE;

    for (block_num = TAG_TYPE_2_FIRST_DATA_BLOCK_NUM;
         block_num < blocks_to_read;
         block_num += TAG_TYPE_2_BLOCKS_PER_EXCHANGE)
    {
        uint16_t offset_for_block = T2T_BLOCK_SIZE * block_num;
        err_code = adafruit_pn532_tag2_read(block_num, buffer + offset_for_block);
        if (err_code)
        {
            NRF_LOG_INFO("Failed to read blocks: %d-%d",
                         block_num,
                         block_num + T2T_END_PAGE_OFFSET);
            return NRF_ERROR_INTERNAL;
        }
    }

    return NRF_SUCCESS;
}


/**
 * @brief Function for analyzing data from a Type 2 Tag Platform.
 *
 * This function parses content of a Type 2 Tag Platform and prints it out.
 */
void t2t_data_analyze(uint8_t * buffer)
{
    ret_code_t err_code;

    // Static declaration of Type 2 Tag structure.
    NFC_TYPE_2_TAG_DESC_DEF(test_1, MAX_TLV_BLOCKS);
    type_2_tag_t * test_type_2_tag = &NFC_TYPE_2_TAG_DESC(test_1);

    err_code = type_2_tag_parse(test_type_2_tag, buffer);
    if (err_code == NRF_ERROR_NO_MEM)
    {
        NRF_LOG_INFO("Not enough memory to read whole tag. Printing what've been read.");
    }
    else if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error during parsing a tag. Printing what could've been read.");
    }

    type_2_tag_printout(test_type_2_tag);

    tlv_block_t * p_tlv_block = test_type_2_tag->p_tlv_block_array;
    uint32_t      i;

    for (i = 0; i < test_type_2_tag->tlv_count; i++)
    {
        if (p_tlv_block->tag == TLV_NDEF_MESSAGE)
        {
            ndef_data_analyze(p_tlv_block->p_value, p_tlv_block->length);
            p_tlv_block++;
        }
    }
}


/**
 * @brief Function for reading and analyzing data from a Type 2 Tag Platform.
 *
 * This function reads content of a Type 2 Tag Platform, parses it and prints it out.
 */
ret_code_t t2t_data_read_and_analyze(nfc_a_tag_info * p_tag_info)
{
    ret_code_t     err_code;
    static uint8_t t2t_data[TAG_TYPE_2_DATA_BUFFER_SIZE]; // Buffer for tag data.

    err_code = t2t_data_read(p_tag_info, t2t_data, TAG_TYPE_2_DATA_BUFFER_SIZE);
    VERIFY_SUCCESS(err_code);

    t2t_data_analyze(t2t_data);

    return NRF_SUCCESS;
}


/**
 * @brief Function for reading and analyzing data from a Type 4 Tag Platform.
 *
 * This function reads content of a Type 4 Tag Platform, parses it and prints it out.
 */
ret_code_t t4t_data_read_and_analyze(nfc_a_tag_info * p_tag_info)
{
    ret_code_t err_code;

    // Static declaration of Type 4 Tag structure.
    NFC_T4T_CC_DESC_DEF(cc_file, MAX_TLV_BLOCKS);
    static uint8_t ndef_files_buffs[MAX_TLV_BLOCKS][TAG_TYPE_4_NDEF_FILE_SIZE];

    err_code = nfc_t4t_ndef_tag_app_select();
    T4T_ERROR_HANDLE(err_code, "Error (0x%X) during NDEF Tag Application Select Procedure.");

    err_code = nfc_t4t_cc_select();
    T4T_ERROR_HANDLE(err_code, "Error (0x%X) during CC Select Procedure.");

    nfc_t4t_capability_container_t * cc_file = &NFC_T4T_CC_DESC(cc_file);
    err_code = nfc_t4t_cc_read(cc_file);
    T4T_ERROR_HANDLE(err_code, "Error (0x%X) during CC Read Procedure.");

    nfc_t4t_tlv_block_t * p_tlv_block = cc_file->p_tlv_block_array;
    uint32_t              i;

    for (i = 0; i < cc_file->tlv_count; i++)
    {
        if ((p_tlv_block->type == NDEF_FILE_CONTROL_TLV) ||
            (p_tlv_block->value.read_access == CONTROL_FILE_READ_ACCESS_GRANTED))
        {
            err_code = nfc_t4t_file_select(p_tlv_block->value.file_id);
            T4T_ERROR_HANDLE(err_code, "Error (0x%X) during NDEF Select Procedure.");

            err_code = nfc_t4t_ndef_read(cc_file, ndef_files_buffs[i], TAG_TYPE_4_NDEF_FILE_SIZE);
            T4T_ERROR_HANDLE(err_code, "Error (0x%X) during NDEF Read Procedure.");
        }

        p_tlv_block++;
    }

    nfc_t4t_cc_file_printout(cc_file);

    p_tlv_block = cc_file->p_tlv_block_array;

    for (i = 0; i < cc_file->tlv_count; i++)
    {
        if ((p_tlv_block->type == NDEF_FILE_CONTROL_TLV) ||
            (p_tlv_block->value.file.p_content != NULL))
        {
            ndef_data_analyze(p_tlv_block->value.file.p_content + TAG_TYPE_4_NLEN_FIELD_SIZE,
                              p_tlv_block->value.file.len - TAG_TYPE_4_NLEN_FIELD_SIZE);
        }

        p_tlv_block++;
    }

    return NRF_SUCCESS;
}


/**
 * @brief Function for identifying Tag Platform Type.
 */
nfc_tag_type_t tag_type_identify(uint8_t sel_res)
{
    uint8_t platform_config;

    // Check if Cascade bit in SEL_RES response is cleared. Cleared bit indicates that NFCID1 complete.
    if (!IS_SET(sel_res, SEL_RES_CASCADE_BIT_NUM))
    {
        // Extract platform configuration from SEL_RES response.
        platform_config = (sel_res & SEL_RES_TAG_PLATFORM_MASK) >> SEL_RES_TAG_PLATFORM_BIT_OFFSET;
        if (platform_config < NFC_TT_NOT_SUPPORTED)
        {
            return (nfc_tag_type_t) platform_config;
        }
    }

    return NFC_TT_NOT_SUPPORTED;
}

static void twi_event_handler(nrf_drv_twi_evt_t const * p_event,void * p_context){
  if(p_event->type==NRF_DRV_TWI_EVT_ADDRESS_NACK){
    NRF_LOG_INFO("No Acknowledgement after transmitted slave address byte");
  }
  else if(p_event->type==NRF_DRV_TWI_EVT_DATA_NACK){
    NRF_LOG_INFO("No Acknowledgement after transmitted data address byte");
  }
  else if(p_event->type==NRF_DRV_TWI_EVT_DONE){
    NRF_LOG_INFO("Data byte transmitted");
  }
}

ret_code_t i2c_init(){
  ret_code_t err_code;
  nrf_drv_twi_config_t twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  twi_config.scl = SCL_I2C_PIN;
  twi_config.sda = SDA_I2C_PIN;
  printf("Creating I2C..\n");

  //@ Note : the event handler of twi should be null (don't know why)
  err_code = nrf_drv_twi_init(&m_twi_master,&twi_config,NULL,NULL);
  if(err_code != NRF_SUCCESS){
    printf("Failed to init TWI, error code : %d\n",err_code);
    return err_code;
  }

  nrf_drv_twi_enable(&m_twi_master);
  return NRF_SUCCESS;
}


ret_code_t pn532_init(){
  ret_code_t err_code;
  err_code = adafruit_pn532_init(&m_twi_master,false);
  return err_code;
}


/**
 * @brief Function for detecting a Tag, identifying its Type and reading data from it.
 *
 * This function waits for a Tag to appear in the field. When a Tag is detected, Tag Platform
 * Type (2/4) is identified and appropriate read procedure is run.
 */
ret_code_t tag_detect_and_read()
{
    ret_code_t     err_code;
    //nfc_a_tag_info t_tag;

    // Detect a NFC-A Tag in the field and initiate a communication. This function activates
    // the NFC RF field. If a Tag is present, basic information about detected Tag is returned
    // in tag info structure.
    err_code = adafruit_pn532_nfc_a_target_init(&m_nfc_tag, TAG_DETECT_TIMEOUT);
  
    if (err_code != NRF_SUCCESS)
    {
        printf("Error NRF_ERROR_NOT_FOUND\n");
        return NRF_ERROR_NOT_FOUND;
    }
    adafruit_pn532_tag_info_printout(&m_nfc_tag);
    

    nfc_tag_type_t tag_type = tag_type_identify(m_nfc_tag.sel_res);
    switch (tag_type)
    {
        case NFC_T2T:
            NRF_LOG_INFO("Type 2 Tag Platform detected. ");
            return t2t_data_read_and_analyze(&m_nfc_tag);

        case NFC_T4T:
            NRF_LOG_INFO("Type 4 Tag Platform detected. ");
            return t4t_data_read_and_analyze(&m_nfc_tag);

        default:
            printf("Tag is not type 2 nor type 4, but it is sufficient to be processed\n");
            return NRF_SUCCESS;
            //return NRF_SUCCESS;
    }
}


/**
 * @brief Function for waiting specified time after a Tag read operation.
 */
void after_read_delay(void)
{
    ret_code_t err_code;

    // Turn off the RF field.
    err_code = adafruit_pn532_field_off();
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(TAG_AFTER_READ_DELAY);
}

//================Definitions================

static inline uint16_t addr_eeprom_spk_with_counter(uint16_t spk_id)
{
    /* the spk_id starts from 1, but we want to store list of SPK(s) 
        starting at address START_ADDR_DATA in EEPROM
    */
    return ADDR_START_SPK_LIST+(spk_id-1)*BYTES_PER_PAGE;
}

static void button_event_handler(uint8_t pin_no, uint8_t action){
    ret_code_t err;
    //NRF_LOG_INFO("Current m_state ");//, m_state);
    NRF_LOG_INFO("Button event handler called with pin : %d",pin_no);
    printf("Button event handler called");
    //NRF_LOG_FLUSH();
    NRF_LOG_INFO("Current m_state %d", m_state);
    NRF_LOG_FLUSH();
    //nrf_delay_ms(500);
    //void (*function_ptr)();

    if (!m_active_nfc.active)
    {  
        printf("There is no Tag registered now\n");
        NRF_LOG_INFO("There is no Tag registered now");
        NRF_LOG_FLUSH();
        return;
    }
    
    if (action != APP_BUTTON_PUSH)
    {
        return;
    }
    //if (action != APP_BUTTON_PUSH)
    //{
    //    NRF_LOG_INFO("The button is released");
    //    NRF_LOG_FLUSH();
    //    //save_current_counter();
    //    //printf("The button is not being pushed");
    //    if  (pin_no == BUTTON_DONE)
    //    {
    //        NRF_LOG_INFO("Button ticks counter %d", button_ticks_counter);
    //        if (button_ticks_counter < NUM_OF_TICKS_CHANGE_NFC && m_state == CHOOSE_SPK)
    //        {
    //            NRF_LOG_INFO("SPK %d Selected", m_choose_spk);
    //            m_active_nfc.active = true;
    //            m_active_nfc.spk = m_choose_spk;
    //            m_active_nfc.counter=0; ////////////////// change this
    //            //m_active_nfc.nfc_id_len=0;
    //            //for(int i=0;i<MAX_NFC_A_ID_LEN;i++){
    //            //  m_active_nfc.nfc_id[i] = 0;
    //            //}
    //            eeprom_data data;

    //            //data.length = sizeof(counter_t);
    //            //memcpy(&(data.p_data[OFFSET_COUNTER_IN_SPK]), 
    //            err = eeprom_read_data(&data, addr_eeprom_spk_with_counter(m_choose_spk), sizeof(counter_t));
    //            APP_ERROR_CHECK(err);

    //            NRF_LOG_INFO("Counter : ");
    //            NRF_LOG_HEXDUMP_INFO(data.p_data, sizeof(counter_t));

    //            if (data.p_data[0] == 0xFF && data.p_data[1] == 0xFF && data.p_data[2] == 0xFF && data.p_data[3] == 0xFF)
    //            {
                    
    //                NRF_LOG_INFO("Looks like we are using new EEPROM, hence we will reset the region of memory it to 0");
    //                memset(data.p_data, 0, sizeof(counter_t));
    //                data.length = sizeof(counter_t);
    //                err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_choose_spk));
                
    //            }
    //            else
    //            {
    //                m_active_nfc.counter = CONVERT_8BIT_ARRAY_TO_32BIT(data.p_data);
    //            }
    //            nrf_gpio_pin_clear(BUZZER);
    //            update_display_state(START);
    //            //save_eeprom();
    //            err = app_timer_start(m_timer,
    //                                  APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY),
    //                                  &m_state);
    //            APP_ERROR_CHECK(err);
    //            NRF_LOG_FLUSH();
    //        }
    //        else if (button_ticks_counter < NUM_OF_TICKS_CHANGE_NFC && m_state == COUNTING)
    //        {
    //            //m_active_nfc.spk = 0;
    //            //m_active_nfc.counter = 0;
    //            //m_choose_spk = 1;
    //            m_active_nfc.active = false;
    //            m_active_nfc.nfc_id_len = 0;
    //            memset(m_active_nfc.nfc_id, 0, MAX_NFC_A_ID_LEN);
    //            err = nfc_spk_save();
    //            if (err != NRF_SUCCESS)
    //            {
    //                NRF_LOG_ERROR("Failed to save counting continue in function %s", __func__);
    //                APP_ERROR_CHECK(err);
    //            }
    //            APP_ERROR_CHECK(adv_stop());
    //            update_display_state(CONTINUE);
    //        }
    //        else if (button_ticks_counter >= NUM_OF_TICKS_CHANGE_NFC && m_state == COUNTING)
    //        {
    //            //m_choose_spk = 1;
    //            //m_state = IDLE;
    //            m_active_nfc.active = false;
    //            m_active_nfc.nfc_id_len = 0;
    //            memset(m_active_nfc.nfc_id, 0, MAX_NFC_A_ID_LEN);
    //            m_active_nfc.spk++;
    //            m_active_nfc.counter = 0;
    //            ret_code_t err;
    //            err = nfc_spk_save();
    //            if (err != NRF_SUCCESS)
    //            {
    //                NRF_LOG_ERROR("Failed to save counting done in function %s", __func__);
    //                APP_ERROR_CHECK(err);
    //            }
    //            APP_ERROR_CHECK(adv_stop());
    //            update_display_state(IDLE);

    //        }
    //        //nrf_delay_ms(10);
    //        //app_timer_stop(m_button_timer);
    //        button_ticks_counter = 0;
    //        continue_m_button_timer = false;
    //    }
    //    return;
    //}



    //if (m_state == CHOOSE_SPK)
    //{
    //    NRF_LOG_INFO("The function pointer is update_display_choose_spk");
    //    NRF_LOG_FLUSH();
    //    switch(pin_no)
    //    {
    //        case BUTTON_COUNTER_UP:
    //            if (m_choose_spk == MAX_NUMBER_OF_SPK) m_choose_spk = 1;
    //            else m_choose_spk++;

    //            NRF_LOG_INFO("Choose SPK up, current SPK %d", m_choose_spk);
    //            NRF_LOG_FLUSH();
    //            //NRF_LOG_INFO("Current counter : %d\n",m_active_nfc.counter);
    //            update_display_choose_spk();
    //            //save_eeprom();
    //            break;

    //        case BUTTON_COUNTER_DOWN:
    //            if (m_choose_spk == 1) m_choose_spk = MAX_NUMBER_OF_SPK;
    //            else m_choose_spk--;

    //            NRF_LOG_INFO("Choose SPK down, current SPK %d", m_choose_spk);
    //            NRF_LOG_FLUSH();
    //            update_display_choose_spk();
    //            break;

    //        case BUTTON_DONE:
    //            continue_m_button_timer = true;
    //            app_timer_start(m_button_timer, APP_TIMER_TICKS(TIMER_TICKS_PER_SHOT), NULL);
    //            break;
    //    }
    //}
    /*else*/ if(m_state == COUNTING)
    {
        NRF_LOG_INFO("The function pointer is update_display_choose_spk");
        NRF_LOG_FLUSH();

        ret_code_t err;
        eeprom_data data;

        data.length = sizeof(counter_t);
        //function_ptr = update_display_counter;
        switch(pin_no)
        {
            case BUTTON_COUNTER_UP:
                m_active_nfc.counter++;
                NRF_LOG_INFO("Counter up\n");
                NRF_LOG_INFO("Current counter : %d\n",m_active_nfc.counter);
                update_display_counter();
                //save_eeprom();

                //data.length = sizeof(counter_t);
                //memset(data.p_data, 0, sizeof(counter_t));
                //ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
                err = nfc_spk_save();
                if (err != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("Failed to write last data counter, in function %s",__func__);
                    APP_ERROR_CHECK(err);
                }

                err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING);
                APP_ERROR_CHECK(err);

                err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_DURATION), NULL);
                APP_ERROR_CHECK(err);

                break;

            case BUTTON_COUNTER_DOWN:
                printf("Counter down\n");
                NRF_LOG_INFO("Counter down");
                if(m_active_nfc.counter<=0)
                {
                    printf("Current counter is 0 or less\n");
                    NRF_LOG_INFO("Current counter is 0 or less");
                }

                else
                {
                    (m_active_nfc).counter--;
                    printf("Current counter : %d",m_active_nfc.counter);
                    update_display_counter();

                    //memset(data.p_data, 0, sizeof(counter_t));
                    //ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
                    //err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_active_nfc.spk));
                    err = nfc_spk_save();
                    if (err != NRF_SUCCESS)
                    {
                        NRF_LOG_ERROR("Failed to write last data counter, in function %s",__func__);
                        APP_ERROR_CHECK(err);
                    }
                    //save_eeprom();
                    err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING);
                    APP_ERROR_CHECK(err);

                    err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_DURATION), NULL);
                    APP_ERROR_CHECK(err);
                }
                break;

            case BUTTON_DONE:
                err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING_DONE);
                APP_ERROR_CHECK(err);

                m_counter_counting_done = 1;
                app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_DURATION), NULL);
                update_display_state(COUNTING_DONE);
                toggle_two_times_buzzer();
                //continue_m_button_timer = true;
                //memset(data.p_data, 0, sizeof(counter_t));
                //ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
                //err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_active_nfc.spk));
                //if (err != NRF_SUCCESS)
                //{
                //    NRF_LOG_ERROR("Failed to write last data counter, in function %s",__func__);
                //    APP_ERROR_CHECK(err);
                //}
                //app_timer_start(m_button_timer, APP_TIMER_TICKS(TIMER_TICKS_PER_SHOT), NULL);
                //printf("Counting done!\n");
                //m_active_nfc.active=false;
                //m_active_nfc.active=0;
                //m_active_nfc.counter=0;
                //m_active_nfc.nfc_id_len=0;
                //for(int i=0;i<MAX_NFC_A_ID_LEN;i++){
                //  m_active_nfc.nfc_id[i] = 0;
                //}
                //nrf_gpio_pin_clear(led);
                //update_display_state(COUNTING_DONE);
                ////save_eeprom();
                //err = app_timer_start(m_timer,
                //                      APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY),
                //                      &m_state);
                //APP_ERROR_CHECK(err);
                break;
        }
    }
    else if (m_state == CONTINUE)
    {
        // do nothing
    }
   

}

static void button_init(){
  ret_code_t err;
  static app_button_cfg_t buttons[] = {
                                      { .pin_no = BUTTON_COUNTER_UP,
                                        .active_state = APP_BUTTON_ACTIVE_LOW,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,
                                        .button_handler = button_event_handler
                                        },
                                      {
                                        .pin_no = BUTTON_COUNTER_DOWN,
                                        .active_state = APP_BUTTON_ACTIVE_LOW,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,
                                        .button_handler = button_event_handler
                                      },
                                      {
                                        .pin_no = BUTTON_DONE,
                                        .active_state = APP_BUTTON_ACTIVE_LOW,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,
                                        .button_handler = button_event_handler
                                      }
  };
  uint8_t button_count = sizeof(buttons)/sizeof(app_button_cfg_t);
  NRF_LOG_INFO("Button count %d",button_count);
  printf("Button count %d\n",button_count);
  err = app_button_init(buttons, button_count, APP_TIMER_TICKS(NUM_OF_TICKS_DETECTION_DELAY_BUTTON));
  APP_ERROR_CHECK(err);

  NRF_LOG_INFO("Button has been initialized");
  printf("Button has been initialized\n");

  err = app_button_enable();
  APP_ERROR_CHECK(err);

  NRF_LOG_INFO("Button has been enabled");
  printf("Button has been enabled\n");
  NRF_LOG_FLUSH();
}

static void after_found(){
    if (m_active_nfc.active)
    {
        printf("There is active NFC : ");
        NRF_LOG_INFO("There is activated NFC before : ");
        NRF_LOG_HEXDUMP_INFO(m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
        //for(int i=0;i<m_active_nfc.nfc_id_len;i++)
        //{
        //    printf("0x%X ",m_active_nfc.nfc_id[i]);
        //}
        //printf("\n\r");
        if (m_active_nfc.nfc_id_len == m_nfc_tag.nfc_id_len)
        {
            printf("Same length %d %d\n", m_active_nfc.nfc_id_len, m_nfc_tag.nfc_id_len);
            if (!memcmp(m_active_nfc.nfc_id, m_nfc_tag.nfc_id, m_active_nfc.nfc_id_len))
            {
                
                printf("Same NFC ID detected\n");
                NRF_LOG_ERROR("Same NFC ID detected");
                return;
            }

            for (int i = 0; i<4; i++)
                {
                    printf("0x%X  0x%X\n", m_active_nfc.nfc_id[i],  m_nfc_tag.nfc_id[i]);
                }
            printf("\n");
        }
        
        NRF_LOG_INFO("Continue counting...!");
        memcpy(m_active_nfc.nfc_id, m_nfc_tag.nfc_id, m_nfc_tag.nfc_id_len);
        update_display_state(CONTINUE);
        app_timer_start(m_timer, APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY), &m_state);
        toggle_two_times_buzzer();

    }

    else{
        ret_code_t err;
        eeprom_data data;
        
        m_active_nfc.active = true;
        // later will be changed if the operator that has the NFC select SPK that have been used before
        //m_active_nfc.counter = 0;
        m_active_nfc.nfc_id_len = m_nfc_tag.nfc_id_len;
        memset(m_active_nfc.nfc_id, 0, MAX_NFC_A_ID_LEN);
        memcpy(m_active_nfc.nfc_id, m_nfc_tag.nfc_id, m_nfc_tag.nfc_id_len);
        //for(int i = 0;  i<m_nfc_tag.nfc_id_len; i++)
        //{
        //    m_active_nfc.nfc_id[i] = m_nfc_tag.nfc_id[i];
        //}
        printf("NFC Activated\n");
        NRF_LOG_INFO("NFC Activated : ");
        NRF_LOG_HEXDUMP_INFO(m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
        //printf("Activated NFC : ");
        //for(int i=0;i<m_active_nfc.nfc_id_len;i++){
        //  printf("0x%X ",m_active_nfc.nfc_id[i]);
        //}
        //printf("\n\r");

        toggle_two_times_buzzer();
        //update_display_state(START);

        //memset(data.p_data, 0, MAX_BYTE_PER_TRX);
        //data.p_data[OFFSET_NFC_ID_LENGTH] = m_active_nfc.nfc_id_len;
        //memcpy(&(data.p_data[OFFSET_NFC_ID]), m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
        //data.length = m_active_nfc.nfc_id_len+1; //NFC ID + NFC ID length
        //err = eeprom_write_data(&data, ADDR_NFC_LENGTH_NFC_ID);
        err = nfc_spk_save();
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_INFO("Failed to save newly active NFC ID");
            APP_ERROR_CHECK(err);
        }
        update_display_state(COUNTING);

        err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING);
        APP_ERROR_CHECK(err);

        err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_DURATION), NULL);
        APP_ERROR_CHECK(err);
        //m_choose_spk = 1;
        //update_display_state(CHOOSE_SPK);
        //app_timer_start(m_timer,APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY),&m_state);

        //save_eeprom();

        //nrf_delay_ms(1000);

        //read_eeprom();
      }
}

static void timer_handler(void * p_context){
    printf("Timer handler called\n");
    NRF_LOG_INFO("Timer Handler called");
    state_type_t state = *(state_type_t *)p_context;

    if (state == START)
    {
        update_display_state(COUNTING);
    }
    else if (state == COUNTING_DONE)
    {
        update_display_state(IDLE);
    }

    else if (state == CONTINUE)
    {
        update_display_state(COUNTING);
    }
}

//static void timer_button_handler(void *p_context)
//{ 
//    app_timer_stop(m_button_timer);
//    NRF_LOG_INFO("Timer button handler called, current ticks %d", button_ticks_counter);
//    button_ticks_counter++;
//    /* sometimes when the done button (or maybe other buttons as well) is pressed, the board indeed feel the button is pressed
//        but sometimes the board doesn't feel the release of the button. This maybe caused by prioritization between
//        m_timer and m_button_timer, sometimes m_timer has higher priority than m_button_timer, sometimes otherwise.
//    */
//    if (continue_m_button_timer)
//    {
//        app_timer_start(m_button_timer, APP_TIMER_TICKS(TIMER_TICKS_PER_SHOT), NULL);
//    }
//}

static void timer_advertising_handler(void * p_context)
{
    ret_code_t err;

    NRF_LOG_INFO("Timer advertising handler called");
    
    err = app_timer_stop(m_timer_advertising);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Failed to stop timer in function %s",__func__);
        APP_ERROR_CHECK(err);
    }
    
    err = adv_stop();
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Failed to stop advertisement when counting done");
        APP_ERROR_CHECK(err);
    }

    if (m_state == COUNTING_DONE)
    {
        m_counter_counting_done++;
        if (m_counter_counting_done >= 3)
        {
            m_counter_counting_done = 0;

            err = nfc_spk_reset_and_save();
            APP_ERROR_CHECK(err);

            update_display_state(IDLE);
            return;
        }
        // if m_counter_counting_done haven't reach 3, keep advertising counting_done;
        err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING_DONE);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_INFO("Failed to restart counting done advertisement");
            APP_ERROR_CHECK(err);
        }
        err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_DURATION), NULL);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_INFO("Failed to start advertising in function %s",__func__);
            APP_ERROR_CHECK(err);
        }
    }
    else if (m_state == COUNTING)
    {
      // do nothing since the advertising has been stopped above
    }

    
}

static void display_idle(){
    ssd1306_clear_display();
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'I', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'D', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'L', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 54, SSD1306_LCDHEIGHT / 2 - 8, 'E', WHITE, BLACK, 3);
    ssd1306_display();
}

static void display_start(){
    ssd1306_clear_display();
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40, SSD1306_LCDHEIGHT / 2 - 8, 'S', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'A', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 54, SSD1306_LCDHEIGHT / 2 - 8, 'R', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 72, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, 3);
    ssd1306_display();
}

static void update_display_counter(){
    // @ this can be optimized
    static int16_t start = 6;
    ssd1306_clear_display();
    ssd1306_draw_char(start, 6, 'C', WHITE, BLACK, 1);
    ssd1306_draw_char(start + 6, 6, 'o', WHITE, BLACK, 1);
    ssd1306_draw_char(start + 12, 6, 'u', WHITE, BLACK, 1);
    ssd1306_draw_char(start + 18, 6, 'n', WHITE, BLACK, 1);
    ssd1306_draw_char(start + 24, 6, 't', WHITE, BLACK, 1);
    counter_t counter = m_active_nfc.counter;
    // maximum 4 digit counter, since the last element to store '\0' from sprintf
    char counter_string[5];
    uint8_t digits_counter = snprintf(counter_string,5,"%d",counter);
    for(int i = 0;i<digits_counter;i++){
      ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + i*18, SSD1306_LCDHEIGHT / 2 - 8, counter_string[i], WHITE, BLACK, 3);
    }
    
    ssd1306_display();
}

static void update_display_choose_spk()
{
    ssd1306_clear_display();
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'S', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'P', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'K', WHITE, BLACK, 3);
    char choose_spk_string[5];
    uint8_t digits_counter = snprintf(choose_spk_string,5,"%d",m_choose_spk);
    for (int i = 0; i< digits_counter ; i++){
        ssd1306_draw_char(SSD1306_LCDWIDTH / 2 + 10 + i*18, SSD1306_LCDHEIGHT / 2 - 8, choose_spk_string[i], WHITE, BLACK, 3);
    }
    ssd1306_display();
}
static void display_counting_done(){
    ssd1306_clear_display();
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'D', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'O', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'N', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 54, SSD1306_LCDHEIGHT / 2 - 8, 'E', WHITE, BLACK, 3);
    ssd1306_display();
}

static void display_continue()
{
    ssd1306_clear_display();
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'C', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'O', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'N', WHITE, BLACK, 3);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 54, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, 3);
    ssd1306_display();
}

//static void display_choose_spk()
//{

//}

static void update_display_state(state_type_t state){
  if  (state == START)
  {
      display_start();
      m_state = START;
  }
  else if(state == COUNTING_DONE)
  {
      display_counting_done();
      m_state = COUNTING_DONE;
  }
  else if(state == IDLE)
  {
      display_idle();
      m_state == IDLE;
  }

  else if (state == COUNTING)
  {
      update_display_counter();
      m_state = COUNTING;
  }

  else if (state == CHOOSE_SPK)
  {
      update_display_choose_spk();
      m_state = CHOOSE_SPK;
  }

  else if (state == CONTINUE)
  {
      display_continue();
      m_state = CONTINUE;
  }
}


//static void save_eeprom()
//{
//    ret_code_t err;
//    // the counter will be stored big endianly
//    ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, (&(m_data.p_data[INDEX_OF_COUNTER])));
//    m_data.p_data[INDEX_OF_LENGTH_NFC] = m_active_nfc.nfc_id_len;
//    memcpy(&(m_data.p_data[INDEX_OF_NFC_ID]), m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
//    uint8_t total_data_length = INDEX_OF_NFC_ID + m_active_nfc.nfc_id_len;
//    m_data.length = total_data_length;
    
    
//    err = eeprom_write_data(&m_data, START_ADDR_DATA);
//    APP_ERROR_CHECK(err);

//    printf("Saved data to EEPROM : ");
//    for(int i = 0; i<total_data_length; i++)
//    {
//        printf("0x%X ", m_data.p_data[i]);
//    }
//    printf("\n\r");

//}

//static void read_eeprom()
//{
//    ret_code_t err;
//    memset(m_data.p_data, 0, MAX_BYTE_PER_TRX);
//    printf("Reseted buffer : ");
//    for(int i = 0; i<MAX_BYTE_PER_TRX; i++)
//    {
//        printf("0x%X ", m_data.p_data[i]);
//    }
//    printf("\n\r");

//    err = eeprom_read_data(&m_data, START_ADDR_DATA, MAX_BYTE_PER_TRX);
//    APP_ERROR_CHECK(err);

//    printf("Received buffer : ");
//    for(int i = 0; i<MAX_BYTE_PER_TRX; i++)
//    {
//        printf("0x%X ", m_data.p_data[i]);
//    }
//    printf("\n\r");
//}

//static void startup_eeprom(){
//    printf("Retrieve data from EEPROM when starting up\n");
//    // the data retrieved from eeprom will be saved in m_data
//    read_eeprom();
    
//    if (m_data.p_data[INDEX_OF_LENGTH_NFC] == 0)
//    {
//        printf("No active data previouslly\n");
//        return;
//    }
//    else
//    {
//        printf("Assign m_data to m_active_t\n");
//        m_active_nfc.active = true;
//        m_active_nfc.counter = CONVERT_8BIT_ARRAY_TO_32BIT(m_data.p_data);
//        m_active_nfc.nfc_id_len = m_data.p_data[INDEX_OF_LENGTH_NFC];
//        memcpy(m_active_nfc.nfc_id, &(m_data.p_data[INDEX_OF_NFC_ID]), m_active_nfc.nfc_id_len);
//        printf("Retrieved active data : \n");
//        printf("Counter : %d\n",  m_active_nfc.counter);
//        printf("Length of NFC ID : %d\n", m_active_nfc.nfc_id_len);
//        printf("NFC ID : ");
//        for(int i = 0; i<m_active_nfc.nfc_id_len; i++)
//        {
//            printf("0x%X ",m_active_nfc.nfc_id[i]);
//        }
//        printf("\n\r");

//        update_display_state(DISPLAY_COUNTING);
//    }
//}

static void startup_spk_counter_eeprom()
{
    ret_code_t err;
    eeprom_data data;

    //get the previous NFC id and NFC length  
    err = eeprom_read_data(&data, ADDR_NFC_LENGTH_NFC_ID, BYTES_PER_PAGE);

    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Falied to retrieve currently active NFC");
        NRF_LOG_FLUSH();
        APP_ERROR_CHECK(err);

    }
    NRF_LOG_HEXDUMP_INFO(data.p_data, 4);
    uint8_t nfc_id_len = data.p_data[OFFSET_NFC_ID_LENGTH];

    if (nfc_id_len == EEPROM_DEFAULT_VALUE)
    {
        //NRF_LOG_INFO("Looks like the EEPROM is new since the NFC id length is 0xFF");
        //NRF_LOG_FLUSH();
        //uint8_t data8bit[] = {0x44, 0x99, 0x88, 0x11};
        //memcpy(m_active_nfc.nfc_id, data8bit, 4);
        //NRF_LOG_HEXDUMP_INFO(data8bit, 4);
        //m_active_nfc.active = true;
        //m_choose_spk = 1;
        //m_active_nfc.nfc_id_len = 4;
        //NRF_LOG_FLUSH();
        //update_display_state(IDLE);
        //nrf_ldelay_ms(2000);
        //update_display_state(CHOOSE_SPK);
        return;
    }

    if (nfc_id_len == 0)
    {
        NRF_LOG_INFO("No active NFC ID");
        NRF_LOG_FLUSH();
        return;
    }

    // counter guard
    if(nfc_id_len > MAX_NFC_A_ID_LEN)
    {
        NRF_LOG_INFO("The length of the retrieved NFC ID exceed MAX_NFC_A_ID_LEN");
        NRF_LOG_FLUSH();
        return;
    }
    
    m_active_nfc.active = true;
    m_active_nfc.nfc_id_len = nfc_id_len;
    memcpy(m_active_nfc.nfc_id, &(data.p_data[OFFSET_NFC_ID]), nfc_id_len);
    /////////////////////////////////////////////////////
    //m_state = CHOOSE_SPK;
    //m_choose_spk = 1;
    //update_display_state(CHOOSE_SPK);
    ////////////////////////////////////////////////////
    NRF_LOG_INFO("There is previously active NFC ID : ");
    NRF_LOG_HEXDUMP_INFO(m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);

    counter_t counter = CONVERT_8BIT_ARRAY_TO_32BIT((&(data.p_data[OFFSET_NFC_ID_LENGTH])));
    uint8_t offset_spk = m_active_nfc.nfc_id_len + OFFSET_NFC_ID + sizeof(counter_t);
    //memcpy(m_active_nfc.nfc_id, &(data.p_data[OFFSET_NFC_ID_LENGTH]), nfc_id_len);
    //m_active_nfc.active = true;
    //m_choose_spk = 0;
    //update_display_state(CHOOSE_SPK);
    //NRF_LOG_FLUSH();


    //memcpy(m_spk_list.indexes_of_spk, &(data.p_data[OFFSET_INDEXES_SPK_LIST]), number_of_spk);
    //m_spk_list.number_of_spk = number_of_spk;

    //0xFF default value
    //nfc_spk_reg_t * p_reg = nfc_spk_get_reg();

    //NRF_LOG_INFO("Default register");
    //default_registery();
    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Reset register");
    //nfc_spk_reset_register();
    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("save register comprehensive!");
    //NRF_LOG_FLUSH();
    //err = nfc_spk_save_comprehensive();
    //APP_ERROR_CHECK(err);
    //NRF_LOG_INFO("Register saved");
    //NRF_LOG_FLUSH();

    //nfc_spk_retrieve_all();
    
    //NRF_LOG_INFO("0xEE register");
    //nfc_spk_0xff_register();
    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Retrieve comprehensive");
    //err = nfc_spk_retrieve_comprehensive();
    //APP_ERROR_CHECK(err);
    //NRF_LOG_INFO("Comprehensive retreived");
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Count NFC %d", nfc_spk_get_nfc_count());
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //nfc_spk_print_all_active_nfc_spk();
}

static void save_current_counter()
{
    ret_code_t err;
    eeprom_data data;
    NRF_LOG_INFO("Saving current counter : %d to SPK ID %d", m_active_nfc.counter, m_active_nfc.spk);
    ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
    data.length = sizeof(counter_t);
    
    err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_active_nfc.spk));
    APP_ERROR_CHECK(err);    
}

static void toggle_two_times_buzzer()
{
    nrf_gpio_pin_set(BUZZER);
    nrf_delay_ms(100);
    nrf_gpio_pin_clear(BUZZER);
}

static void startup_nfc_spk()
{
    ret_code_t err;
    NRF_LOG_INFO("Getting NFC and SPK info");
    err = nfc_spk_retrieve_comprehensive();
    APP_ERROR_CHECK(err);
    NRF_LOG_INFO("NFC and SPK info retrieved");
    
    if (m_active_nfc.active)
    {
        update_display_state(COUNTING);

        err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING);
        APP_ERROR_CHECK(err);

        err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_DURATION), NULL);
        APP_ERROR_CHECK(err);
    }
    else
    {
        if (m_active_nfc.counter != 0) update_display_state(CONTINUE);
        else update_display_state(IDLE);
    }
}

//static void adv_data()
//{
    
//}

int main(void)
{
    ret_code_t err_code;

    utils_setup();
    
    //printf("Initialization...!\n");
    NRF_LOG_INFO("Initialization...!");
    printf("Initialization...!");
    NRF_LOG_FLUSH();

    err_code = i2c_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("I2C initialized");
    NRF_LOG_FLUSH();


    printf("SSD1306 Initializing!\n");
    ssd1306_init_i2c_2(&m_twi_master);
    printf("SSD1306 Initialized\n");
    ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, false);
    printf("Display Idle SSD1306\n");
    update_display_state(IDLE);

    err_code = pn532_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("PN532 Initialized");
    NRF_LOG_FLUSH();

    //printf("Initializing EEPROM!\n");
    //err_code = eeprom_init(&m_twi_master);
    //APP_ERROR_CHECK(err_code);
    //printf("EEPROM initialization success\n");

    NRF_LOG_INFO("Initializing EEPROM");
    printf("Initializing EEPROM");
    err_code = nfc_spk_eeprom_init(&m_twi_master, &m_active_nfc);
    APP_ERROR_CHECK(err_code);
    printf("EEPROM initialized");
    NRF_LOG_INFO("EEPROM initialized");
    NRF_LOG_FLUSH();

    startup_nfc_spk();
    //startup_eeprom();
    //startup_spk_counter_eeprom();
    //while(true);
    for (;;)
    {
        err_code = tag_detect_and_read();
        switch (err_code)
        {
            case NRF_SUCCESS:
                printf("Found\n");
                NRF_LOG_INFO("Found");
                after_found();
                
                after_read_delay();
                
                break;

            case NRF_ERROR_NO_MEM:
                NRF_LOG_INFO("Declared buffer for T2T is to small to store tag data.");
                printf("Declared buffer for T2T is to small to store tag data.\n");
                after_read_delay();
                break;

            case NRF_ERROR_NOT_FOUND:
                NRF_LOG_INFO("No Tag found.");
                printf("No Tag found.\n");
                // No delay here as we want to search for another tag immediately.
                break;

            case NRF_ERROR_NOT_SUPPORTED:
                NRF_LOG_INFO("Tag not supported.");
                printf("Tag not supported.\n");
                after_read_delay();
                break;

            default:
                NRF_LOG_INFO("Error during tag read.");
                printf("Error during tag read.\n");
                err_code = adafruit_pn532_field_off();
                break;
        }
        NRF_LOG_FLUSH();
    }
}


/** @} */ /* End of group nfc_adafruit_tag_reader_example */

