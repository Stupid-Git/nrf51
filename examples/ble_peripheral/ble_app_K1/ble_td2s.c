
#include "ble_td2s.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"

#define BLE_UUID_TD2S_TX_CHARACTERISTIC 0x0002                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_TD2S_RX_CHARACTERISTIC 0x0003                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_TD2S_CMD_CHARACTERISTIC 0x0004                      /**< The UUID of the Command Characteristic. */
#define BLE_UUID_TD2S_STS_CHARACTERISTIC 0x0005                      /**< The UUID of the Status Characteristic. */

#define BLE_TD2S_MAX_RX_CHAR_LEN        BLE_TD2S_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_TD2S_MAX_TX_CHAR_LEN        BLE_TD2S_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */
#define BLE_TD2S_MAX_CMD_CHAR_LEN       BLE_TD2S_MAX_DATA_LEN        /**< Maximum length of the Command Characteristic (in bytes). */
#define BLE_TD2S_MAX_STS_CHAR_LEN       BLE_TD2S_MAX_DATA_LEN        /**< Maximum length of the Status Characteristic (in bytes). */



//#define NUS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */
#define  TD1S_BASE_UUID                  {{0x00, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */
#define  TD2S_BASE_UUID                  {{0x42, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_td2s     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_td2s_t * p_td2s, ble_evt_t * p_ble_evt)
{
    p_td2s->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_td2s     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_td2s_t * p_td2s, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_td2s->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_td2s     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
#include "stdio.h"
static void on_write(ble_td2s_t * p_td2s, ble_evt_t * p_ble_evt)
{
    //char buf[10];
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

/*
    if( p_td2s->data_handler != NULL )
    {
        sprintf( buf, "0x%04x", p_evt_write->handle);
        p_td2s->data_handler(p_td2s, (uint8_t*)buf, 6);

        sprintf( buf, "0x%04x", p_td2s->cmd_handles.value_handle);
        p_td2s->data_handler(p_td2s, (uint8_t*)buf, 6);
    }
*/

    //----- RX (Notify On/Off) -----
    if(  (p_evt_write->handle == p_td2s->rx_handles.cccd_handle)  &&
         (p_evt_write->len == 2)                                      )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_td2s->is_notification_enabled = true;
        }
        else
        {
            p_td2s->is_notification_enabled = false;
        }
    }
    else

    //----- TX -----
    if(  (p_evt_write->handle == p_td2s->tx_handles.value_handle)  &&
         (p_td2s->data_handler != NULL)                               )
    {
        //ref td2s_data_handler(...)
        p_td2s->data_handler(p_td2s, p_evt_write->data, p_evt_write->len);
    }
    else
        
    //----- CMD -----
    if(  (p_evt_write->handle == p_td2s->cmd_handles.value_handle)  &&
         (p_td2s->cmd_handler != NULL)                                 )
    {
        //p_evt_write->data[1] = '-';
        //ref td2s_data_handler(...)
        p_td2s->cmd_handler (p_td2s, p_evt_write->data, p_evt_write->len);
        //p_td2s->data_handler(p_td2s, p_evt_write->data, p_evt_write->len);
    }
    else
        
    //----- STS -----
    if(  (p_evt_write->handle == p_td2s->sts_handles.value_handle)  &&
         (p_td2s->sts_handler != NULL)                                 )
    {
        //ref td2s_data_handler(...)
        p_td2s->sts_handler(p_td2s, p_evt_write->data, p_evt_write->len);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_td2s       TD2 Service structure.
 * @param[in] p_td2s_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_td2s_t * p_td2s, const ble_td2s_init_t * p_td2s_init)
{
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_td2s->uuid_type;
    ble_uuid.uuid = BLE_UUID_TD2S_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_TD2S_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_td2s->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_td2s->rx_handles);
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}


/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_td2s       Nordic UART Service structure.
 * @param[in] p_td2s_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_td2s_t * p_td2s, const ble_td2s_init_t * p_td2s_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_td2s->uuid_type;
    ble_uuid.uuid = BLE_UUID_TD2S_TX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_TD2S_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_td2s->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_td2s->tx_handles);
}

/**@brief Function for adding CMD characteristic.
 *
 * @param[in] p_td2s       TD2 Service structure.
 * @param[in] p_td2s_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cmd_char_add(ble_td2s_t * p_td2s, const ble_td2s_init_t * p_td2s_init)
{
    ble_gatts_char_md_t char_md;
  //ble_gatts_attr_md_t cccd_md;                            //****
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

  //memset(&cccd_md, 0, sizeof(cccd_md));                   //****

  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);     //****
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);    //****

  //cccd_md.vloc = BLE_GATTS_VLOC_STACK;                    //****


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
  //char_md.char_props.notify = 1;              //*****
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
  //char_md.p_cccd_md         = &cccd_md;       //****
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_td2s->uuid_type;
    ble_uuid.uuid = BLE_UUID_TD2S_CMD_CHARACTERISTIC;  //*****

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;


    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);  //==1
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_TD2S_MAX_CMD_CHAR_LEN;  //*****

    return sd_ble_gatts_characteristic_add(p_td2s->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_td2s->cmd_handles); //****
}


/**@brief Function for adding STS characteristic.
 *
 * @param[in] p_td2s       TD2 Service structure.
 * @param[in] p_td2s_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t sts_char_add(ble_td2s_t * p_td2s, const ble_td2s_init_t * p_td2s_init)
{

    ble_gatts_char_md_t char_md;
  //ble_gatts_attr_md_t cccd_md;                            //****
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

  //memset(&cccd_md, 0, sizeof(cccd_md));                   //****

  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);     //****
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);    //****

  //cccd_md.vloc = BLE_GATTS_VLOC_STACK;                    //****


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
  //char_md.char_props.notify = 1;              //*****
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
  //char_md.p_cccd_md         = &cccd_md;       //****
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_td2s->uuid_type;
    ble_uuid.uuid = BLE_UUID_TD2S_STS_CHARACTERISTIC;  //*****

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;


    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);  //==1
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_TD2S_MAX_STS_CHAR_LEN;  //*****

    return sd_ble_gatts_characteristic_add(p_td2s->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_td2s->sts_handles); //*****

}


/*
It is important to note that a notification will <b>consume an application buffer</b>, and will therefore 
 *          generate a @ref BLE_EVT_TX_COMPLETE event when the packet has been transmitted. An indication on the other hand will use the 
 *          standard server internal buffer and thus will only generate a @ref BLE_GATTS_EVT_HVC event as soon as the confirmation 
 *          has been received from the peer. Please see the documentation of @ref sd_ble_tx_buffer_count_get for more details.
*/

uint32_t ble_td2s_notify2(ble_td2s_t * p_td2s, uint8_t * p_string, uint16_t length);

void ble_td2s_on_ble_evt(ble_td2s_t * p_td2s, ble_evt_t * p_ble_evt)
{
    if ((p_td2s == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_td2s, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_td2s, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_td2s, p_ble_evt);
            break;
/*TODO
        case BLE_GATTS_EVT_READ:
            on_read(p_td2s, p_ble_evt);
            break;
*/
        
        case BLE_EVT_TX_COMPLETE:
            //p_ble_evt->evt.common_evt.conn_handle;
            //p_ble_evt->evt.common_evt.params.tx_complete.count;
            ble_td2s_notify2(p_td2s, (uint8_t*)"AACCKK", 6 );
            break;
        
        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_td2s_init(ble_td2s_t * p_td2s, const ble_td2s_init_t * p_td2s_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t td2s_base_uuid = TD2S_BASE_UUID;

    if ((p_td2s == NULL) || (p_td2s_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    // Initialize the service structure.
    p_td2s->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_td2s->data_handler            = p_td2s_init->data_handler;
    p_td2s->cmd_handler             = p_td2s_init->cmd_handler; //****
    p_td2s->sts_handler             = p_td2s_init->sts_handler; //****
    p_td2s->is_notification_enabled = false;

    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&td2s_base_uuid, &p_td2s->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_td2s->uuid_type;
    ble_uuid.uuid = BLE_UUID_TD2S_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_td2s->service_handle);
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the RX Characteristic.
    err_code = rx_char_add(p_td2s, p_td2s_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the TX Characteristic.
    err_code = tx_char_add(p_td2s, p_td2s_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the CMD Characteristic.                 //*****
    err_code = cmd_char_add(p_td2s, p_td2s_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add the STS Characteristic.                 //*****
    err_code = sts_char_add(p_td2s, p_td2s_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint8_t do_notify2 = 0;
uint32_t ble_td2s_string_send(ble_td2s_t * p_td2s, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if (p_td2s == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_td2s->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_td2s->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_TD2S_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_td2s->rx_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION; //->BLE_EVT_TX_COMPLETE

    do_notify2 = 2;
    return sd_ble_gatts_hvx(p_td2s->conn_handle, &hvx_params);
}

uint32_t ble_td2s_notify2(ble_td2s_t * p_td2s, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    if(do_notify2 == 0)
        return(NRF_SUCCESS);
    
    p_string[0] = do_notify2;
    
    if (p_td2s == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_td2s->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_td2s->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_TD2S_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_td2s->rx_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION; //->BLE_EVT_TX_COMPLETE


    if(do_notify2 > 0)
        do_notify2--;
    //do_notify2 = 0;
    
    return sd_ble_gatts_hvx(p_td2s->conn_handle, &hvx_params);
}


