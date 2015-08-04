/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"

#define USE_DM 1

#if USE_DM
#include "ble_advertising.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#endif

#include "app_gpiote.h"
#include "app_button.h"
//#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"

//*****************************************************************************
//*****************************************************************************
#define USE_TD1 0

#if USE_TD1
#include "ble_td1s.h"
static ble_td1s_t                      m_td1s;                                      /**< Structure to identify the TD1 Service. */
#endif
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
#define USE_TD2 1

#if USE_TD2
#include "ble_td2s.h"
static ble_td2s_t                      m_td2s;                                      /**< Structure to identify the TD2 Service. */
#endif
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
#define USE_NUS 0

#if USE_NUS
#include "ble_nus.h"
#endif
//*****************************************************************************
//*****************************************************************************


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define WAKEUP_BUTTON_ID                0                                           /**< Button used to wake up the application. */
#if USE_DM
#define BOND_DELETE_ALL_BUTTON_ID       1                                           /**< Button used for deleting all bonded centrals during startup. */
#endif



#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            (2 + BSP_APP_TIMERS_NUMBER)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of simultaneously GPIOTE users. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#if USE_DM
static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */
#endif

#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
#if USE_NUS
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
#endif
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */


/**@brief Function for assert macro callback.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
// https://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.generic_access.xml
// Service Name: "Generic Access", Assigned Number: 0x1800
/*
    Service Characteristics

      Name: "Device Name" , 0x2A00
        Type: org.bluetooth.characteristic.gap.device_name                                 Requirement: Mandatory
        ->@sd_ble_gap_device_name_set

      Name: "Appearance"  , 0x2A01
        Type: org.bluetooth.characteristic.gap.appearance                                  Requirement: Mandatory
        ->  sd_ble_gap_appearance_set

      Name: "Peripheral Privacy Flag" , 0x2A02
        Type: org.bluetooth.characteristic.gap.peripheral_privacy_flag                     Requirement: Optional

      Name: "Reconnection Address" , 0x2A03
        Type: org.bluetooth.characteristic.gap.reconnection_address                        Requirement: Conditional
        
      Name: "Peripheral Preferred Connection Parameters" , 0x2A04
        Type: org.bluetooth.characteristic.gap.peripheral_preferred_connection_parameters  Requirement: Optional
        ->  sd_ble_gap_ppcp_set
*/

#define td_gap_DEVICE_NAME                     "Newdick0_UART"                               /**< Name of device. Will be included in the advertising data. */
#define td_gap_MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define td_gap_MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define td_gap_SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define td_gap_CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    // Characteristic "Device Name", Assigned number: 0x2A00
    // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.device_name.xml
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) td_gap_DEVICE_NAME,
                                          strlen(td_gap_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    // Characteristic "Appearance", Assigned number: 0x2A01
    // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
    uint16_t appearance; //karel
    appearance = 0x0000; //karel
    sd_ble_gap_appearance_set(appearance); //karel
  
    // Characteristic "Peripheral Preferred Connection Parameters" Assigned number: 0x2A04
    // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.peripheral_preferred_connection_parameters.xml
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = td_gap_MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = td_gap_MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = td_gap_SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = td_gap_CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

#if USE_DM    

/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t               err_code;
    dm_init_param_t        init_data;
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    err_code = bsp_button_is_pressed(BOND_DELETE_ALL_BUTTON_ID,&(init_data.clear_persistent_data));
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}
#endif


#if USE_DM
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
#endif

/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting the advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
#if USE_DM
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
#else
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
#endif
    
    
#if USE_NUS
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};
#endif

#if USE_TD1 
    //TODO_TD1 ble_uuid_t adv_uuids[] = {{BLE_UUID_TD1S_SERVICE, m_td1s.uuid_type}};
#endif

#if USE_TD2 
    //TODO_TD2
    ble_uuid_t adv_uuids[] = {{BLE_UUID_TD2S_SERVICE, m_td2s.uuid_type}};
#endif

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = flags;
  //advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
  //advdata.uuids_complete.p_uuids  = adv_uuids;    
    
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
    
#if USE_DM
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
#if USE_NUS
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
#endif
/**@snippet [Handling the data received over BLE] */

#if USE_TD1
static void td1s_data_handler(ble_td1s_t * p_td1s, uint8_t * p_data, uint16_t length)
{
    while(app_uart_put('[') != NRF_SUCCESS);
    while(app_uart_put('T') != NRF_SUCCESS);
    while(app_uart_put('D') != NRF_SUCCESS);
    while(app_uart_put('1') != NRF_SUCCESS);
    while(app_uart_put(']') != NRF_SUCCESS);
    while(app_uart_put(' ') != NRF_SUCCESS);
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
#endif

#if USE_TD2

int32_t blk_dn_start( uint8_t *pkt );
int32_t blk_dn_add( uint8_t *pkt, uint16_t len );
int32_t blk_dn_chk();

static void td2s_data_handler(ble_td2s_t * p_td2s, uint8_t * p_data, uint16_t length)
{
    while(app_uart_put('[') != NRF_SUCCESS);
    while(app_uart_put('T') != NRF_SUCCESS);
    while(app_uart_put('X') != NRF_SUCCESS);
    while(app_uart_put(' ') != NRF_SUCCESS);
    while(app_uart_put(']') != NRF_SUCCESS);
    while(app_uart_put(' ') != NRF_SUCCESS);
    /*
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    */
    
    blk_dn_add(p_data, length);

    if( blk_dn_chk() == 0 )
        while(app_uart_put('0') != NRF_SUCCESS);
    else
        while(app_uart_put('X') != NRF_SUCCESS);        
    
    while(app_uart_put('\n') != NRF_SUCCESS);
}

static void td2s_cmd_handler(ble_td2s_t * p_td2s, uint8_t * p_data, uint16_t length)
{
    while(app_uart_put('[') != NRF_SUCCESS);
    while(app_uart_put('C') != NRF_SUCCESS);
    while(app_uart_put('M') != NRF_SUCCESS);
    while(app_uart_put('D') != NRF_SUCCESS);
    while(app_uart_put(']') != NRF_SUCCESS);
    while(app_uart_put(' ') != NRF_SUCCESS);
    /*
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    */
    
    if( (p_data[0] == 1) && (p_data[0] == 1) )
    {
        blk_dn_start( p_data );
   
        while(app_uart_put('S') != NRF_SUCCESS);
    }
    
    while(app_uart_put('\n') != NRF_SUCCESS);
}
static void td2s_sts_handler(ble_td2s_t * p_td2s, uint8_t * p_data, uint16_t length)
{
    while(app_uart_put('[') != NRF_SUCCESS);
    while(app_uart_put('S') != NRF_SUCCESS);
    while(app_uart_put('T') != NRF_SUCCESS);
    while(app_uart_put('S') != NRF_SUCCESS);
    while(app_uart_put(']') != NRF_SUCCESS);
    while(app_uart_put(' ') != NRF_SUCCESS);
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
#endif


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;

#if USE_NUS
    ble_nus_init_t   nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
#endif

    
#if USE_TD1
    ble_td1s_init_t   td1s_init;    
    memset(&td1s_init, 0, sizeof(td1s_init));

    td1s_init.data_handler = td1s_data_handler;
    
    err_code = ble_td1s_init(&m_td1s, &td1s_init);
    APP_ERROR_CHECK(err_code);
#endif

#if USE_TD2
    ble_td2s_init_t   td2s_init;    
    memset(&td2s_init, 0, sizeof(td2s_init));

    td2s_init.data_handler = td2s_data_handler;
    td2s_init.cmd_handler = td2s_cmd_handler;
    td2s_init.sts_handler = td2s_sts_handler;
    
    err_code = ble_td2s_init(&m_td2s, &td2s_init);
    APP_ERROR_CHECK(err_code);
#endif

}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;  //karel -> force negotiation see ble_conn_params.c (232)
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

#if USE_DM
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            // enable buttons to wake-up from power off
            err_code = bsp_buttons_enable( (1 << WAKEUP_BUTTON_ID)
                                         | (1 << BOND_DELETE_ALL_BUTTON_ID));
            APP_ERROR_CHECK(err_code);

            // Go to system-off mode. This function will not return; wakeup will cause a reset.
            err_code = sd_power_system_off();
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}
#endif

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_sec_keyset_t      s_sec_keyset;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();
            break;
/* NOT USE_DM - this was the last change which then enabled Windows 8.1 to pair the device
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            s_sec_keyset.keys_periph.p_enc_key = NULL;
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params,
                                                   &s_sec_keyset);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            // TODO: Adoptation to s110v8.0.0, is this needed anymore ?
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            if (s_sec_keyset.keys_periph.p_enc_key != NULL)
            {
                p_enc_info = &s_sec_keyset.keys_periph.p_enc_key->enc_info;
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device.
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;
NOT USE_DM*/
        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            { 
                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                APP_ERROR_CHECK(err_code);
                // Configure buttons with sense level low as wakeup source.
                err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
                APP_ERROR_CHECK(err_code);
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                err_code = sd_power_system_off();    
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    
#if USE_DM
    dm_ble_evt_handler(p_ble_evt);
#endif
    ble_conn_params_on_ble_evt(p_ble_evt);
    
#if USE_NUS
    ble_nus_on_ble_evt(&m_nus, p_ble_evt); // components/ble/ble_services/ble_nus/ble_nus.c(201)
        // case BLE_GAP_EVT_CONNECTED:    on_connect(p_nus, p_ble_evt);
        // case BLE_GAP_EVT_DISCONNECTED: on_disconnect(p_nus, p_ble_evt);
        // case BLE_GATTS_EVT_WRITE:      on_write(p_nus, p_ble_evt);
#endif

#if USE_TD1
    ble_td1s_on_ble_evt(&m_td1s, p_ble_evt);
#endif
    
#if USE_TD2
    ble_td2s_on_ble_evt(&m_td2s, p_ble_evt);
#endif
    
    on_ble_evt(p_ble_evt);
    
#if USE_DM
    ble_advertising_on_ble_evt(p_ble_evt);
#endif
}


#if USE_DM
/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}
#endif


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

#if USE_DM    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */

#if USE_TD1
void uart_event_handle_bogus_td1s(uint8_t *pbyte)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t err_code;

    data_array[index] = *pbyte;
    index++;
    if ((data_array[index - 1] == '\n') || (index >= (BLE_TD1S_MAX_DATA_LEN)))
    {
        err_code = ble_td1s_string_send(&m_td1s, data_array, index);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }        
        index = 0;
    }
    
}
#endif

#if USE_TD2
void uart_event_handle_bogus_td2s(uint8_t *pbyte)
{
    static uint8_t data_array[BLE_TD2S_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t err_code;

    data_array[index] = *pbyte;
    index++;
    if ((data_array[index - 1] == '\n') || (index >= (BLE_TD2S_MAX_DATA_LEN)))
    {
        err_code = ble_td2s_string_send(&m_td2s, data_array, index);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }        
        index = 0;
    }
    
}
#endif
#if USE_TD2
void uart_event_handle_td2(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_TD2S_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t err_code;
    
    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:            

        UNUSED_VARIABLE(app_uart_get(&data_array[index]));        
        index++;
        
        if ((data_array[index - 1] == '\n') || (index >= (BLE_TD2S_MAX_DATA_LEN)))
        {
            err_code = ble_td2s_string_send(&m_td2s, data_array, index);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }

            index = 0;
        }
        break;
            
    case APP_UART_COMMUNICATION_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_communication);
        break;
        
    case APP_UART_FIFO_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;
        
    default:
        break;
    }
}
/**@snippet [Handling the data received over UART] */
#endif

#if USE_NUS
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:            
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
        
#if USE_TD1
            uart_event_handle_bogus_td1s(&data_array[index]);
#endif
#if USE_TD2
            uart_event_handle_bogus_td2s(&data_array[index]);
#endif
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
            }
}
/**@snippet [Handling the data received over UART] */
#endif


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
          //UART_BAUDRATE_BAUDRATE_Baud38400
      };

#if USE_NUS
    APP_UART_FIFO_INIT( &comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle, //OK this is the callback for UART interrupts
                        APP_IRQ_PRIORITY_LOW,
                        err_code);
    APP_ERROR_CHECK(err_code);
#endif

#if USE_TD2
    APP_UART_FIFO_INIT( &comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle_td2, //OK this is the callback for UART interrupts
                        APP_IRQ_PRIORITY_LOW,
                        err_code);
    APP_ERROR_CHECK(err_code);
#endif

      
}
/**@snippet [UART Initialization] */


/**@brief  Application main function.
 */
int main(void)
{
    uint8_t start_string[] = START_STRING;
    uint32_t err_code;
    
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, NULL); //false);
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
    ble_stack_init();
    uart_init();
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        NULL);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
    APP_ERROR_CHECK(err_code);
#if USE_DM    
    device_manager_init();
#endif
    gap_params_init(); //OK
    services_init(); //OK
    advertising_init(); //OK -> either OR ish
    conn_params_init(); //OK
    sec_params_init(); //OK
    
    printf("%s",start_string);

    advertising_start(); //OK
    
    // Enter main loop.
    for (;;)
    {
        power_manage(); //OK
    }
}


/** 
 * @}
 */
