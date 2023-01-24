#ifndef  ADV_H__
#define  ADV_H__

#include "ble_gap.h"
#include "app_util.h"
#include "sdk_errors.h"
#include "main.h"

#define DEFAULT_ADV_PARAMS    {                                                                                   \
                                  .duration = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED,                              \
                                  .interval = MSEC_TO_UNITS(ADVERTISING_DURATION, UNIT_0_625_MS),                                  \
                                  .filter_policy = BLE_GAP_ADV_FP_ANY,                                            \
                                  .p_peer_addr = NULL,                                                            \
                                  .primary_phy = BLE_GAP_PHY_1MBPS,                                               \
                                  .properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED      \
                              }

typedef enum {
    ADV_COUNTING,
    ADV_COUNTING_DONE,
}adv_type_t;


ret_code_t adv_init();

ret_code_t adv_start_or_update(uint16_t spk_id, uint32_t counter, uint8_t * const nfc_id, uint8_t nfc_id_len, adv_type_t adv_type);

static ret_code_t adv_data_config(uint16_t spk_id, uint32_t counter, uint8_t * const nfc_id, uint8_t nfc_id_len, adv_type_t adv_type);

ret_code_t adv_stop();

#endif

