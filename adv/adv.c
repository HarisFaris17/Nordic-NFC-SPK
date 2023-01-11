#include "adv.h"
#include "stdint.h"
#include "ble_advdata.h"
#include "stdio.h"
#include "nrf_log.h"



static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static ble_gap_adv_params_t m_adv_params = DEFAULT_ADV_PARAMS;
static uint8_t m_enc_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static uint8_t m_enc_scan_resp[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static bool m_adv_active = false;


static ble_gap_adv_data_t m_adv_data = 
{
    .adv_data = 
    {
      .p_data = m_enc_data,
      .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX          
    },

    .scan_rsp_data = 
    {
        .p_data = m_enc_scan_resp,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    }
};


ret_code_t adv_init()
{
    ret_code_t err;

    //err = sd_ble_gap_adv_set_configure(&m_adv_handle, )
    return NRF_SUCCESS;
}

static ret_code_t adv_data_config( uint16_t spk_id, uint32_t counter, uint8_t * const nfc_id, uint8_t nfc_id_len)
{
    ret_code_t err = NRF_SUCCESS;
    ble_advdata_t adv_data;
    uint8_t app_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    ble_advdata_manuf_data_t manuf_adv_data;
    
    memset(&adv_data, 0, sizeof(adv_data));

    uint8_t num_of_characters = snprintf(app_data, BLE_GAP_ADV_SET_DATA_SIZE_MAX, "DAT#%d#%d#", spk_id, counter);
    memcpy(&(app_data[num_of_characters]), nfc_id, nfc_id_len);
    NRF_LOG_HEXDUMP_INFO(app_data, num_of_characters + nfc_id_len);

    NRF_LOG_INFO("Number of characters %d", num_of_characters + nfc_id_len);
    manuf_adv_data.company_identifier = 0x0000;
    manuf_adv_data.data.p_data = app_data;
    manuf_adv_data.data.size = num_of_characters + nfc_id_len;

    adv_data.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    adv_data.p_manuf_specific_data = &manuf_adv_data;
    //m_adv_data

    m_adv_data.scan_rsp_data.len = 0;
    m_adv_data.scan_rsp_data.p_data = NULL;

    err = ble_advdata_encode(&adv_data, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    //VERIFY_SUCCESS(err);

    return err;
}

ret_code_t adv_start_or_update(uint16_t spk_id, uint32_t counter, uint8_t * const nfc_id, uint8_t nfc_id_len)
{
    ret_code_t err;
    m_adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    err = adv_data_config(spk_id, counter, nfc_id, nfc_id_len);
    VERIFY_SUCCESS(err);

    if (m_adv_active)
    {
        err = adv_stop();
        VERIFY_SUCCESS(err);
        err = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
        VERIFY_SUCCESS(err);

        err = sd_ble_gap_adv_start(m_adv_handle, BLE_CONN_CFG_TAG_DEFAULT);
        VERIFY_SUCCESS(err);
    }
    else
    {
        err = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
        VERIFY_SUCCESS(err);

        err = sd_ble_gap_adv_start(m_adv_handle, BLE_CONN_CFG_TAG_DEFAULT);
        VERIFY_SUCCESS(err);
    }
    


    m_adv_active = true;
    return NRF_SUCCESS;
}

ret_code_t adv_stop()
{
    ret_code_t err = NRF_SUCCESS;

    if (!m_adv_active) return NRF_SUCCESS;

    err = sd_ble_gap_adv_stop(m_adv_handle);

    m_adv_active = false;
    return err;
}