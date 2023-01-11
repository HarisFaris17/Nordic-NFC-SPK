#include "nfc_spk_eeprom.h"
#include "eeprom.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

// an SPK page contains 2 byte SPK Id plus the counter that attached to it
// consider 1 page of NFC infomation (NFC ID length, NFC ID, active SPK count, and active SPK indexes) plus the maximum number of SPK page per NFC as complete set
/* The order of page for NFC with SPK application is page 1 to store how many active NFC, indexes of every complete set of active NFC
    , then complete set NFC index 0 (page 2 - page 2 + MAX_SPK_COUNT_PER_NFC)  to store index 1 of a complete set of active NFC with SPK, then page 3 to store index 2 
    of a complete set of active NFC with SPK
*/

#define     TOTAL_BYTES_SPK                               (sizeof(spk_id_t))
#define     TOTAL_BYTES_COUNTER                           (sizeof(counter_t))
#define     TOTAL_BYTES_SPK_WITH_COUNTER                  (TOTAL_BYTES_SPK+TOTAL_BYTES_COUNTER)

//#define     MAX_SIZE_LIST_SPK_WITH_COUNTER                (MAX_SPK_COUNT_PER_NFC*sizeof(spk_t))
#define     ADDR_COUNT_NFC_LIST_WITH_INDEXES              START_ADDR_DATA                          /* this is the first page for comprehensive
                                                                                                      storing for NFC with SPK application.
                                                                                                    */
#define     PAGE_COUNT_PER_NFC_WITH_MAX_LIST_SPK          (1+MAX_SPK_COUNT_PER_NFC)                /* one page to contain NFC ID length, NFC
                                                                                                     ID, and SPK Count. after that page, reserved
                                                                                                     maximum number of SPK page per NFC. Alias
                                                                                                     this macro defines number of pages of one
                                                                                                     complete set*/
#define     COMPREHENSIVE_PAGE_COUNT                      (1+ PAGE_COUNT_PER_NFC_WITH_MAX_LIST_SPK \
                                                            * MAX_NFC_COUNT)


#define     OFFSET_COUNT_NFC_LIST                         0
#define     OFFSET_INDEXES_NFC_LIST                       (OFFSET_COUNT_NFC_LIST+1)

#define     OFFSET_NFC_ID_LENGTH                          0
#define     OFFSET_NFC_ID                                 (OFFSET_NFC_ID_LENGTH+1)
//#define     OFFSET_COUNT_SPK_PER_NFC                      (OFFSET_NFC_ID+1)
//#define     OFFSET_INDEXES_SPK_PER_NFC                    (OFFSET_COUNT_SPK_PER_NFC+1)

#define     OFFSET_SPK_ID                                 0
#define     OFFSET_COUNTER                                (OFFSET_SPK_ID+TOTAL_BYTES_SPK)   

#define     DELAY_AFTER_WRITE                             15


static nfc_spk_reg_t m_reg;
static active_nfc_t * mp_nfc_in_action = NULL;

static inline uint16_t nfc_spk_start_address_complete_set(uint8_t nfc_index)
{
    return (ADDR_COUNT_NFC_LIST_WITH_INDEXES+BYTES_PER_PAGE)+
            nfc_index*BYTES_PER_PAGE*PAGE_COUNT_PER_NFC_WITH_MAX_LIST_SPK;           /*since the very first page in eeprom of 
                                                                                                                          this application is used to store
                                                                                                                          the active NFC indexes, which means the page
                                                                                                                          is not used to store any part of complete set
                                                                                                                          but rather the first complete set (nfc index 0)
                                                                                                                          stored after that very first page (i.e. the second
                                                                                                                          page), hence it is necessary to add 1 to the
                                                                                                                          return value since the nfc index starts from 0
                                                                                                                         */
} 

static inline uint16_t nfc_spk_start_address_spk_in_complete_set(uint8_t nfc_index, uint8_t spk_index)
{
    return (nfc_spk_start_address_complete_set(nfc_index)+BYTES_PER_PAGE)
                                          +spk_index*BYTES_PER_PAGE;                                    /*since the first page of a complete 
                                                                                                          set not contains any spk, and spk(s) 
                                                                                                          starts after the first page also spk_index
                                                                                                          starts at 0, therefore it is necessary
                                                                                                          to add 1 to the addition of start address of
                                                                                                          index of specified nfc and index of spk to
                                                                                                          really return the absolute page number of
                                                                                                          the spk in eeprom  
                                                                                                        */
}

ret_code_t nfc_spk_eeprom_init(nrf_drv_twi_t * const p_twi_master)
{
    ret_code_t err;

    NRF_LOG_INFO("Initializing EEPROM");
    err = eeprom_init(p_twi_master);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed init eeprom in function %s!", __func__);
        return err;
    }
    
    if ((ADDR_COUNT_NFC_LIST_WITH_INDEXES % BYTES_PER_PAGE) != 0)
    {
        NRF_LOG_ERROR("We temporarily doesn't support start address for data to be not divisible by 32, "
                      "since it doesn't represent the start address of any page of EEPROM it will make our flow for"
                      "storing/retrieving data to/from eeprom screwed");
        return NRF_ERROR_INVALID_ADDR;
    }

    nfc_spk_reset_register();

    err = nfc_spk_retrieve_comprehensive();
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to retrieve comprehensively in function %s", __func__);
        return err;
    }


    return NRF_SUCCESS;
}

ret_code_t nfc_spk_retrieve_comprehensive()
{
    ret_code_t err;
    //uint8_t num_of_active_nfc;
    //uint8_t active_indexes_list[MAX_NFC_COUNT];

    NRF_LOG_INFO("Trying to retrieve comprehensive SPK and NFC");
    // get the number of active NFC and coresponding NFC indexes
    err = nfc_spk_retrieve_start_page(&m_reg.count_nfc_list, m_reg.index_nfc_list);
    if (err != NRF_SUCCESS) 
    {
        NRF_LOG_ERROR("Error while retrieving start page in function %s",__func__);
        return err;
    }


    for (int i = 0; i < m_reg.count_nfc_list; i++)
    {
        err = nfc_spk_retrieve_complete_set_by_nfc_index(m_reg.index_nfc_list[i]);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Failed to retrieve complete set with NFC index %d in function %s", m_reg.index_nfc_list[i], __func__);
            return err;
        }
    }

    return NRF_SUCCESS;
}

ret_code_t nfc_spk_retrieve_complete_set_by_nfc_index(uint8_t nfc_index)
{
    ret_code_t err;
    eeprom_data data;

    NRF_LOG_INFO("Trying to retrieve complete set of NFC index %d", nfc_index);
    if (nfc_index < 0 || nfc_index >= MAX_NFC_COUNT)
    {
        NRF_LOG_ERROR("The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", nfc_index, __func__);
        return NRF_ERROR_INVALID_PARAM;
    }

    //get the first page of complete set, i.e. the NFC ID length, NFC ID, count of active SPK for specified NFC ID, and indexes of active SPK
    err = eeprom_read_data(&data, nfc_spk_start_address_complete_set(nfc_index), BYTES_PER_PAGE);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Error when retrieving first page of a complete set in index NFC %d", nfc_index);
        return err;
    }
    m_reg.nfc_list[nfc_index].nfc_id_len = data.p_data[OFFSET_NFC_ID_LENGTH];
    memcpy(m_reg.nfc_list[nfc_index].nfc_id, &(data.p_data[OFFSET_NFC_ID]), m_reg.nfc_list[nfc_index].nfc_id_len);
    m_reg.nfc_list[nfc_index].count_spk_list = data.p_data[OFFSET_NFC_ID + m_reg.nfc_list[nfc_index].nfc_id_len];
    memcpy(m_reg.nfc_list[nfc_index].index_spk_list, &(data.p_data[OFFSET_NFC_ID + m_reg.nfc_list[nfc_index].nfc_id_len + 1]), m_reg.nfc_list[nfc_index].count_spk_list);

    // get each SPK id and counter in the specified nfc_index
    for (int i = 0; i < m_reg.nfc_list[nfc_index].count_spk_list; i++)
    {
        uint8_t index_in_spk_list = m_reg.nfc_list[nfc_index].index_spk_list[i];
        err = nfc_spk_retrieve_spk(nfc_index, index_in_spk_list);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Failed to retreive SPK with NFC index %d in function %s", nfc_index, __func__);
            return err;
        }
    }

    return NRF_SUCCESS;
}

ret_code_t nfc_spk_retrieve_spk(uint8_t nfc_index, uint8_t spk_index)
{
    ret_code_t err;
    eeprom_data data;
    
    NRF_LOG_INFO("Trying to retrieve SPK index %d with NFC index %d", spk_index, nfc_index);
    if (nfc_index < 0 || nfc_index >= MAX_NFC_COUNT)
    {
        NRF_LOG_ERROR("The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", nfc_index, __func__);
        return NRF_ERROR_INVALID_PARAM;
    }

    if (spk_index < 0 || spk_index >= MAX_SPK_COUNT_PER_NFC)
    {
        NRF_LOG_ERROR("The spk_index %d is not in range of [0, MAX_SPK_COUNT_PER_NFC) in function %s", spk_index, __func__);
        return NRF_ERROR_INVALID_PARAM;
    }

    err = eeprom_read_data(&data, 
                            nfc_spk_start_address_spk_in_complete_set(nfc_index, spk_index),
                            TOTAL_BYTES_SPK_WITH_COUNTER);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to retrieve SPK with index : %d in NFC index : %d, in function %s", spk_index, nfc_index, __func__);
        return err;
    }

    // remember that the counter and spk id stored in EEPROM is big endian, hence no need to swap the retrieved counter
    m_reg.nfc_list[nfc_index].spk_list[spk_index].spk_id  = CONVERT_8BIT_ARRAY_TO_16BIT((&(data.p_data[OFFSET_SPK_ID])));
    m_reg.nfc_list[nfc_index].spk_list[spk_index].counter = CONVERT_8BIT_ARRAY_TO_32BIT((&(data.p_data[OFFSET_COUNTER])));
    return NRF_SUCCESS;
}

ret_code_t nfc_spk_retrieve_start_page(uint8_t * p_num_of_active_nfc, uint8_t * p_indexes_list)
{
    ret_code_t err;
    eeprom_data data;
    
    NRF_LOG_INFO("Retrieving the first page!");
    err = eeprom_read_data(&data, ADDR_COUNT_NFC_LIST_WITH_INDEXES, 1+MAX_NFC_COUNT);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Failed to retreive the first page of NFC SPK");
        return err;
    }
    // first page retrieved
    NRF_LOG_INFO("first page retrieved! length : %d", data.length);
    NRF_LOG_FLUSH();
    NRF_LOG_HEXDUMP_INFO(data.p_data, data.length);
    *p_num_of_active_nfc = data.p_data[OFFSET_COUNT_NFC_LIST];

    for (int i = 0; i<*p_num_of_active_nfc; i++)
    {
        p_indexes_list[i] = data.p_data[OFFSET_INDEXES_NFC_LIST+i];
    }

    return NRF_SUCCESS;
}

ret_code_t nfc_spk_save_comprehensive()
{
    ret_code_t err;
    
    NRF_LOG_INFO("Trying to write to eeprom comprehensively");
    err = nfc_spk_save_start_page();
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to write start page to eeprom in function %s", __func__);
        return err;
    }
    NRF_LOG_INFO("Saved the start page");
    delay_after_write();

    for(int i = 0; i < m_reg.count_nfc_list; i++)
    {
        uint8_t index_in_nfc_list = m_reg.index_nfc_list[i];
        err = nfc_spk_save_complete_set_by_nfc_index(index_in_nfc_list);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Failed to write to eeprom with NFC index %d in function %s", index_in_nfc_list, __func__);
            return err;
        }
    }
    return NRF_SUCCESS;
}

ret_code_t nfc_spk_save_complete_set_by_nfc_index(uint8_t nfc_index)
{
    ret_code_t err;
    eeprom_data data;

    NRF_LOG_INFO("Trying to write complete set of NFC index %d", nfc_index);
    if (nfc_index < 0 || nfc_index >= MAX_NFC_COUNT)
    {
        NRF_LOG_ERROR("The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", nfc_index, __func__);
        return NRF_ERROR_INVALID_PARAM;
    }

    active_nfc_t * const p_nfc = &m_reg.nfc_list[nfc_index];

    uint8_t nfc_id_len = p_nfc->nfc_id_len;
    data.p_data[OFFSET_NFC_ID_LENGTH] = nfc_id_len;
    memcpy(&data.p_data[OFFSET_NFC_ID], p_nfc->nfc_id, nfc_id_len);

    uint8_t count_spk_list = p_nfc->count_spk_list;
    uint8_t offset_count_spk_list = OFFSET_NFC_ID + nfc_id_len;
    data.p_data[offset_count_spk_list] = count_spk_list;
    memcpy(&data.p_data[offset_count_spk_list + 1], p_nfc->index_spk_list, count_spk_list);

    data.length = offset_count_spk_list + 3;
    NRF_LOG_INFO("Trying to write first page of complete set of NFC index %d", nfc_index);
    err = eeprom_write_data(&data, nfc_spk_start_address_complete_set(nfc_index));
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to write on first page of NFC index %d",nfc_index);
        return err;
    }
    NRF_LOG_INFO("Saved the first page of complete set of NFC index %d", nfc_index);
    delay_after_write();

    for (int i = 0; i < p_nfc->count_spk_list; i++)
    {
        uint8_t index_in_spk_list = p_nfc->index_spk_list[i];
        err = nfc_spk_save_spk(nfc_index, index_in_spk_list);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Failed to write to eeprom with NFC index %d in function %s", nfc_index, __func__);
            return err;
        }
    }

    return NRF_SUCCESS;
}

ret_code_t nfc_spk_save_spk(uint8_t nfc_index, uint8_t spk_index)
{
    ret_code_t err;
    eeprom_data data;

    NRF_LOG_INFO("Trying to write SPK index %d with NFC index %d", spk_index, nfc_index);
    if (nfc_index < 0 || nfc_index >= MAX_NFC_COUNT)
    {
        NRF_LOG_ERROR("The nfc_index %d is not in range of [0, MAX_NFC_COUNT) in function %s", nfc_index, __func__);
        return NRF_ERROR_INVALID_PARAM;
    }

    if (spk_index < 0 || spk_index >= MAX_SPK_COUNT_PER_NFC)
    {
        NRF_LOG_ERROR("The spk_index %d is not in range of [0, MAX_SPK_COUNT_PER_NFC) in function %s", spk_index, __func__);
        return NRF_ERROR_INVALID_PARAM;
    }
    
    //memcpy(&(data.p_data[OFFSET_SPK_ID]),&m_reg.nfc_list[nfc_index].spk_list[spk_index].spk_id, TOTAL_BYTES_SPK);
    ASSIGN_16BIT_TO_8BIT_ARRAY(m_reg.nfc_list[nfc_index].spk_list[spk_index].spk_id, (&(data.p_data[OFFSET_SPK_ID])))
    ASSIGN_32BIT_TO_8BIT_ARRAY(m_reg.nfc_list[nfc_index].spk_list[spk_index].counter, (&(data.p_data[OFFSET_COUNTER])));
    //memcpy(&(data.p_data[OFFSET_COUNTER]),&m_reg.nfc_list[nfc_index].spk_list[spk_index].counter, TOTAL_BYTES_COUNTER);

    data.length = TOTAL_BYTES_SPK_WITH_COUNTER;
    
    err = eeprom_write_data(&data, nfc_spk_start_address_spk_in_complete_set(nfc_index, spk_index));
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to write to eeprom with SPK index %d and NFC index %d in function %s", spk_index, nfc_index, __func__);
        return err;
    }
    NRF_LOG_INFO("Saved bytes in SPK index %d and NFC index %d", spk_index, nfc_index);
    delay_after_write();
    return NRF_SUCCESS;
}

ret_code_t nfc_spk_save_start_page()
{
    ret_code_t err;
    eeprom_data data;

    NRF_LOG_INFO("Trying to save the first page to EEPROM");
    data.p_data[OFFSET_COUNT_NFC_LIST] = m_reg.count_nfc_list;
    memcpy(&data.p_data[OFFSET_INDEXES_NFC_LIST], m_reg.index_nfc_list, m_reg.count_nfc_list);
    data.length = OFFSET_INDEXES_NFC_LIST + m_reg.count_nfc_list + 1;

    err = eeprom_write_data(&data, ADDR_COUNT_NFC_LIST_WITH_INDEXES);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to write into the first page of NFC SPK");
        return err;
    }

    return NRF_SUCCESS;
}

ret_code_t nfc_spk_retrieve_all()
{
    ret_code_t err;
    eeprom_data data;
    for(int i = 0; i < COMPREHENSIVE_PAGE_COUNT; i++)
    {
        err = eeprom_read_data(&data, START_ADDR_DATA+i*BYTES_PER_PAGE, 10);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Error %s",__func__);
            NRF_LOG_FLUSH();
            return  err;
        }
        delay_after_write();
        NRF_LOG_INFO("Page %d",i);
        NRF_LOG_HEXDUMP_INFO(data.p_data, 10);
    }
    NRF_LOG_FLUSH();
    return NRF_SUCCESS;
}

void nfc_spk_print_all_active_nfc_spk()
{
    for (int i = 0; i<m_reg.count_nfc_list; i++)
    {

        uint8_t index_in_nfc_list = m_reg.index_nfc_list[i];
        active_nfc_t nfc = (m_reg.nfc_list[index_in_nfc_list]);
        NRF_LOG_HEXDUMP_INFO(nfc.nfc_id, nfc.nfc_id_len);
        for(int j = 0; j < nfc.count_spk_list; j++)
        {
            uint8_t index_in_spk_list = nfc.index_spk_list[j];
            spk_t spk = nfc.spk_list[index_in_spk_list];
            NRF_LOG_INFO("NFC index %d, SPK index %d, SPK_ID %d, Counter %d", index_in_nfc_list, index_in_spk_list, spk.spk_id, spk.counter);
        }
    }
    NRF_LOG_FLUSH();
}

//static ret_code_t nfc_spk_save_spk(uint8_t * p_nfc_id, uint8_t nfc_id_len)
//{
//    ret_code_t err;
    
//    if (m_reg.count_nfc_list == 0)
//    {
//        NRF_LOG_ERROR("Can't save SPK, since no active NFC!")
//        return NRF_ERROR_INVALID_LENGTH;
//    }

//    bool isSame = true;
//    for (int i = 0; i<m_reg.count_nfc_list; i++)
//    {
//        uint8_t index_nfc = m_reg.index_nfc_list[i];

//        // the length of NFC id of current index in register difference to that of specified in parameter
//        if (m_reg.nfc_list[index_nfc].nfc_id_len != nfc_id_len)
//        {
//            continue;
//        }

//        // the length of NFC id same but let's compare each byte of NFC id
//        isSame = memcmp(m_reg.nfc_list[index_nfc].nfc_id, p_nfc_id, nfc_id_len);
//        if (!isSame)
//        {
//            continue;
//        }

//        // the nfc id of current index of register is the same as specified in the parameter
//        break;
//    }
    
//    if (!isSame)
//    {
//        NRF_LOG_ERROR("No matching NFC ID in the register with the NFC ID specified in parameter");
//        return NRF_ERROR_INVALID_DATA;
//    }
 
//}

void nfc_spk_reset_register()
{
    memset(&m_reg, 0, sizeof(nfc_spk_reg_t));
}

void nfc_spk_0xff_register()
{
    memset(&m_reg, 0xEE, sizeof(nfc_spk_reg_t));
}

static void delay_after_write(){
    nrf_delay_ms(DELAY_AFTER_WRITE);
}

uint8_t nfc_spk_get_nfc_count()
{
    return m_reg.count_nfc_list;
}

nfc_spk_reg_t * nfc_spk_get_reg()
{
    return (nfc_spk_reg_t *)&m_reg;
}

void default_registery()
{
    memset(&m_reg, 0, sizeof(nfc_spk_reg_t));
    m_reg.count_nfc_list = 2;
    m_reg.index_nfc_list[0] = 0;
    m_reg.index_nfc_list[1] = 1;

    m_reg.nfc_list[0].count_spk_list = 2;
    m_reg.nfc_list[0].index_spk_list[0] = 0;
    m_reg.nfc_list[0].nfc_id_len = 4;
    uint8_t array[] = {0x44, 0x02, 0x01, 0x00};
    memcpy(m_reg.nfc_list[0].nfc_id, array, 4);
    m_reg.nfc_list[0].spk_list[0].counter = 10;
    m_reg.nfc_list[0].spk_list[0].spk_id = 11;

    //m_reg.nfc_list[0].count_spk_list = 1;
    m_reg.nfc_list[0].index_spk_list[1] = 1;
    //m_reg.nfc_list[0].nfc_id_len = 4;
    //uint8_t array[] = {0x19, 0x27, 0x22, 0x67};
    //memcpy(m_reg.nfc_list[0].nfc_id, array, 4);
    m_reg.nfc_list[0].spk_list[1].counter = 65;
    m_reg.nfc_list[0].spk_list[1].spk_id = 64;

    m_reg.nfc_list[1].count_spk_list = 1;
    m_reg.nfc_list[1].index_spk_list[0] = 0;
    m_reg.nfc_list[1].nfc_id_len = 4;
    uint8_t array2[] = {0x04, 0x04, 0x08, 0x07};
    memcpy(m_reg.nfc_list[1].nfc_id, array2, 4);
    m_reg.nfc_list[1].spk_list[0].counter = 90;
    m_reg.nfc_list[1].spk_list[0].spk_id = 91;
}