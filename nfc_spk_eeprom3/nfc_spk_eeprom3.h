#ifndef  NFC_SPK_EEPROM3__
#define  NFC_SPK_EEPROM3__

#include <stdint.h>
#include <sdk_errors.h>
#include <nrf_drv_twi.h>
#include "adafruit_pn532.h"
#include "main.h"

#define MAX_SPK_COUNT_PER_NFC               5
#define MAX_NFC_COUNT                       5

#define NO_MORE_ITEM                        -1

// @brief data type for SPK. we did this to anticipate there is a change in spk id data size
typedef uint16_t spk_id_t;

// @brief data type for counter. actually we don't really need this, but it is a sugar for syntax
typedef uint32_t counter_t;

typedef struct{
    bool active;
    uint8_t  spk;
    counter_t counter;
    uint8_t nfc_id_len;              ///< UID length.
    uint8_t nfc_id[MAX_NFC_A_ID_LEN]; ///< NFC-A UID.
}active_nfc;

// @brief
typedef struct
{
    spk_id_t  spk_id;
    counter_t counter;
}spk_t;

// @brief 
typedef struct
{
    uint8_t         nfc_id_len;              ///< UID length.
    uint8_t         nfc_id[MAX_NFC_A_ID_LEN]; ///< NFC-A UID.
    uint8_t         count_spk_list;
    uint8_t         index_spk_list[MAX_SPK_COUNT_PER_NFC];
    spk_t           spk_list[MAX_SPK_COUNT_PER_NFC];
}active_nfc_t;

// @brief
typedef struct
{
    uint8_t         count_nfc_list;
    uint8_t         index_nfc_list[MAX_NFC_COUNT];
    active_nfc_t    nfc_list[MAX_NFC_COUNT];
}nfc_spk_reg_t;

//// @brief
//typedef  struct
//{
//    bool            is_active;
//    uint8_t         active_index_nfc;
//    uint8_t         active_index_spk;
//} 

void default_registery();

static inline uint16_t nfc_spk_start_address_complete_set(uint8_t nfc_index);

static inline uint16_t nfc_spk_start_address_spk_in_complete_set(uint8_t nfc_index, uint8_t spk_index);

ret_code_t nfc_spk_eeprom_init(nrf_drv_twi_t * const p_twi_master, active_nfc * p_active_nfc);

ret_code_t nfc_spk_retrieve_comprehensive();

ret_code_t nfc_spk_save();

ret_code_t nfc_spk_retrieve_complete_set_by_nfc_index(uint8_t nfc_index);

ret_code_t nfc_spk_retrieve_spk(uint8_t nfc_index, uint8_t spk_index);

ret_code_t nfc_spk_retrieve_start_page(uint8_t * p_num_of_active_nfc, uint8_t * p_indexes_list);

ret_code_t nfc_spk_save_comprehensive();

ret_code_t nfc_spk_save_complete_set_by_nfc_index(uint8_t nfc_index);

ret_code_t nfc_spk_save_spk(uint8_t nfc_index, uint8_t spk_index);

ret_code_t nfc_spk_save_start_page();

ret_code_t nfc_spk_retrieve_all();

static ret_code_t reset_page(uint16_t start_page_address);

void nfc_spk_print_all_active_nfc_spk();

void nfc_spk_reset_register();

void nfc_spk_0xff_register();

//static ret_code_t nfc_spk_save_spk(uint8_t * p_nfc_id, uint8_t nfc_id_len);

static void delay_after_write();

uint8_t nfc_spk_get_nfc_count();



nfc_spk_reg_t * nfc_spk_get_reg();

#endif