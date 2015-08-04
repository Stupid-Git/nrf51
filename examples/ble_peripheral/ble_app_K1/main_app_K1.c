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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
//#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer_appsh.h"
#include "app_gpiote.h"
#include "bsp.h"

//-----------------------------------------------------------------------------
#define USE_DIS
#define USE_BAS

#if defined(USE_BAS)

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                    81                                         /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL                    100                                        /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT              1                                          /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

static app_timer_id_t                        m_battery_timer_id;                        /**< Battery timer. */
#endif
//-----------------------------------------------------------------------------


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define WAKEUP_BUTTON_ID                0                                           /**< Button used to wake up the application. */
// YOUR_JOB: Define any other buttons to be used by the applications:
// #define MY_BUTTON_ID                   1


// YOUR_JOB: Modify these according to requirements.
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            (2 + BSP_APP_TIMERS_NUMBER)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */


#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/*
// YOUR_JOB: Uncomment this function and make it handle error situations sent back to your
//           application by the services it uses.
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
} */





/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
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

#define td_gap_DEVICE_NAME                     "T&D_app_K1"                                /**< Name of device. Will be included in the advertising data. */
//#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
//#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
//#define SLAVE_LATENCY                        0                                           /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

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
                                          (const uint8_t *)td_gap_DEVICE_NAME,
                                          strlen(td_gap_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */

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

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//#############################################################################
//##### advertising_init ######################################################
//#############################################################################

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
#if 0 // 0 don't compile

#define eg_APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define eg_NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define eg_APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define eg_APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define eg_APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define eg_APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define eg_APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define eg_APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */ 
#define eg_APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */ 
#define eg_APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                           0x45, 0x56, 0x67, 0x78, \
                                           0x89, 0x9a, 0xab, 0xbc, \
                                           0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

//#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_MAX_TIMERS            (2+BSP_APP_TIMERS_NUMBER)         /**< Maximum number of simultaneously created timers. */
//#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */

#define USE_UICR_FOR_MAJ_MIN_VALUES BOGUG_777 //karel

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define eg_MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define eg_UICR_ADDRESS                    0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t eg_m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t eg_m_beacon_info[eg_APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    eg_APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this 
                         // implementation. 
    eg_APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the 
                         // manufacturer specific data in this implementation.
    eg_APP_BEACON_UUID,     // 128 bit UUID value. 
    eg_APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons. 
    eg_APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons. 
    eg_APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in 
                         // this implementation. 
};

static void advertising_init_example_app_beacon(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = eg_APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)eg_UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)eg_UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = eg_MAJ_VAL_OFFSET_IN_BEACON_INFO;

    eg_m_beacon_info[index++] = MSB(major_value);
    eg_m_beacon_info[index++] = LSB(major_value);

    eg_m_beacon_info[index++] = MSB(minor_value);
    eg_m_beacon_info[index++] = LSB(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) eg_m_beacon_info;
    manuf_specific_data.data.size   = eg_APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&eg_m_adv_params, 0, sizeof(eg_m_adv_params));

    eg_m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND; // see ble_gap.h -> Connectable undirected /directed. Scannable undirected, Non connectable undirected.
    eg_m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    eg_m_adv_params.fp          = BLE_GAP_ADV_FP_ANY; // see ble_gap.h -> Filter scan/connect requests with whitelist yes/no/both
    eg_m_adv_params.interval    = eg_NON_CONNECTABLE_ADV_INTERVAL;
    eg_m_adv_params.timeout     = eg_APP_CFG_NON_CONN_ADV_TIMEOUT;
}
#endif //#if 0 // 0 don't compile

static void timesaver_set_m_adv_params(void);

static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags;


    // YOUR_JOB: Use UUIDs for service(s) used in your application.
    //flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    //ble_uuid_t adv_uuids[] = {{BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE}};  // 0x180F e.g.
    //ble_uuid_t adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};  // 0x180A e.g.
      ble_uuid_t adv_uuids[] = 
    {
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    ble_uuid_t  scan_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};  // 0x180A e.g.

    //===== Build and set advertising data ====================================
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;// true; // false is good for us, because it will be 0x0000
    advdata.flags                   = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    
    //===== Build and set Scan data ===========================================
    memset(&scanrsp, 0, sizeof(scanrsp));
    //scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    //scanrsp.uuids_complete.p_uuids  = adv_uuids;
    scanrsp.uuids_complete.uuid_cnt = sizeof(scan_uuids) / sizeof(scan_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = scan_uuids;
    
    err_code = ble_advdata_set(&advdata, NULL/*&scanrsp*/);
    APP_ERROR_CHECK(err_code);
   
    timesaver_set_m_adv_params(); // set now, or at least before 

}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//-----------------------------------------------------------------------------------------------------------------------------
#if defined(USE_DIS)
//#include "ble_dis.h"
#include "C:\ble_nrf51\nRF51_SDK\components\ble\ble_services\ble_dis\ble_dis.h"

//#define ble_dis_MANUFACTURER_NAME       "NordicSemiconductor"   /**< Manufacturer. Will be passed to Device Information Service. */
#define   ble_dis_MANUFACTURER_NAME       "T and D Corporation"
//#define ble_dis_MODEL_NUM               "NS-BPS-EXAMPLE"        /**< Model number. Will be passed to Device Information Service. */
#define   ble_dis_MODEL_NUM               "T&D BTLE Example"      /**< Model number. Will be passed to Device Information Service. */
#define   ble_dis_MANUFACTURER_ID         0x1122334455            /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define   ble_dis_ORG_UNIQUE_ID           0x667788                /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

static void services_init_ble_dis(void)
{
    uint32_t         err_code;

    ble_dis_init_t   dis_init;
    ble_dis_sys_id_t sys_id;

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));


    //** ble_srv_utf8_str_t             manufact_name_str;           /**< Manufacturer Name String. */
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, ble_dis_MANUFACTURER_NAME);

    //** ble_srv_utf8_str_t             model_num_str;               /**< Model Number String. */
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     ble_dis_MODEL_NUM);


    //** ble_srv_utf8_str_t             serial_num_str;              /**< Serial Number String. */
    //** ble_srv_utf8_str_t             hw_rev_str;                  /**< Hardware Revision String. */
    //** ble_srv_utf8_str_t             fw_rev_str;                  /**< Firmware Revision String. */
    //** ble_srv_utf8_str_t             sw_rev_str;                  /**< Software Revision String. */
    // ...
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str,    "serial_num_str");
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,        "hw_rev_str");
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,        "fw_rev_str");
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str,        "sw_rev_str");
    // ...


    //** ble_dis_sys_id_t *             p_sys_id;                    /**< System ID. */
    sys_id.manufacturer_id            = ble_dis_MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ble_dis_ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    //** ble_dis_reg_cert_data_list_t * p_reg_cert_data_list;        /**< IEEE 11073-20601 Regulatory Certification Data List. */
    // ...
    //** ble_dis_pnp_id_t *             p_pnp_id;                    /**< PnP ID. */
    // ...
    //** ble_srv_security_mode_t        dis_attr_md;                 /**< Initial Security Setting for Device Information Characteristics. */
    // ...

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

#endif


//-----------------------------------------------------------------------------------------------------------------------------
#if defined(USE_BAS) //--------------------------------------------------------
//#include "ble_bas.h"
#include "C:\ble_nrf51\nRF51_SDK\components\ble\ble_services\ble_bas\ble_bas.h"

static ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = 42;
    return;
    
    /*TODO
    (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
    TODO*/
}
/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

void services_init_ble_bas(void)
{
    uint32_t         err_code;
    ble_bas_init_t   bas_init;

    
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

}
#endif //----------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    // YOUR_JOB: Add code to initialize the services used by the application.
    
#if defined(USE_BAS)
    services_init_ble_bas();
#endif

#if defined(USE_DIS)
    services_init_ble_dis();
#endif
}



//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
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


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
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


/**@brief Function for handling a Connection Parameters error.
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
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}




//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

/**@brief Function for starting timers.
*/
static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    uint32_t err_code;

    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */

#if defined(USE_BAS)
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for starting application timers.
 */
static void application_timers_start_SAME_AS_timers_start_QQQ(void)
{
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    //karel APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true); // true -> app_timer_evt_schedule
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false); // false -> NULL

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
    */
    
    // Create timers.
#if defined(USE_BAS)
    uint32_t err_code;
    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED,  battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
}




//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

// see ble_gap.h -> Connectable undirected /directed. Scannable undirected, Non connectable undirected.
#define   my_BLE_GAP_ADV_TYPE     BLE_GAP_ADV_TYPE_ADV_IND         // 0x00   /**< Connectable undirected. */
//#define my_BLE_GAP_ADV_TYPE     BLE_GAP_ADV_TYPE_ADV_DIRECT_IND  // 0x01   /**< Connectable directed. */
//#define my_BLE_GAP_ADV_TYPE     BLE_GAP_ADV_TYPE_ADV_SCAN_IND    // 0x02   /**< Scannable undirected. */
//#define my_BLE_GAP_ADV_TYPE     BLE_GAP_ADV_TYPE_ADV_NONCONN_IND // 0x03   /**< Non connectable undirected. */

// see ble_gap.h -> Filter scan/connect requests with whitelist yes/no/both
#define   my_BLE_GAP_ADV_FP       BLE_GAP_ADV_FP_ANY               // 0x00   /**< Allow scan requests and connect requests from any device. */
//#define my_BLE_GAP_ADV_FP       BLE_GAP_ADV_FP_FILTER_SCANREQ    // 0x01   /**< Filter scan requests with whitelist. */
//#define my_BLE_GAP_ADV_FP       BLE_GAP_ADV_FP_FILTER_CONNREQ    // 0x02   /**< Filter connect requests with whitelist. */
//#define my_BLE_GAP_ADV_FP       BLE_GAP_ADV_FP_FILTER_BOTH       // 0x03   /**< Filter both scan and connect requests with whitelist. */


//#define my_APP_ADV_INTERVAL   MSEC_TO_UNITS(1000, UNIT_0_625_MS) /**< Advertising interval (1000 ms). This value can vary between ?100ms? to 10.24s). */
#define   my_APP_ADV_INTERVAL   MSEC_TO_UNITS(2500, UNIT_0_625_MS) /**< Advertising interval (2500 ms). This value can vary between ?100ms? to 10.24s). */
//#define my_APP_ADV_INTERVAL   64     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define   my_APP_ADV_TIMEOUT_IN_SECONDS       0     /**< 0 disables timeout. The advertising timeout (in units of seconds).*/
//#define my_APP_ADV_TIMEOUT_IN_SECONDS     180     /**< The advertising timeout (in units of seconds). */

static ble_gap_adv_params_t m_adv_params;       /**< Parameters to be passed to the stack when starting advertising. */

static void timesaver_set_m_adv_params(void)
{
    m_adv_params.type        = my_BLE_GAP_ADV_TYPE;
    m_adv_params.p_peer_addr = NULL; // Undirected advertisement.
    m_adv_params.fp          = my_BLE_GAP_ADV_FP;
    m_adv_params.interval    = my_APP_ADV_INTERVAL;
    m_adv_params.timeout     = my_APP_ADV_TIMEOUT_IN_SECONDS;
}
/* Notes:
     See /examples/ble_peripheral/ble_app_proximity/main.c(250): Various Advertising Speed Whitelist etc settings
*/
/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    // Start advertising
#if 1 // use pre-stored "m_adv_params" to set
    uint32_t             err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
#else
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = my_BLE_GAP_ADV_TYPE;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = my_BLE_GAP_ADV_FP;
    adv_params.interval    = my_APP_ADV_INTERVAL;
    adv_params.timeout     = my_APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
#endif
    
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    bool                             master_id_matches;
    ble_gap_sec_kdist_t *            p_distributed_keys;
    ble_gap_enc_info_t *             p_enc_info;
    ble_gap_irk_t *                  p_id_info;
    ble_gap_sign_info_t *            p_sign_info;

    static ble_gap_enc_key_t         m_enc_key;           /**< Encryption Key (Encryption Info and Master ID). */
    static ble_gap_id_key_t          m_id_key;            /**< Identity Key (IRK and address). */
    static ble_gap_sign_info_t       m_sign_key;          /**< Signing Key (Connection Signature Resolving Key). */
    static ble_gap_sec_keyset_t      m_keys = {.keys_periph = {&m_enc_key, &m_id_key, &m_sign_key}};

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events (assuming that the button events are only needed in connected
                         state). If this is uncommented out here,
                            1. Make sure that app_button_disable() is called when handling
                               BLE_GAP_EVT_DISCONNECTED below.
                            2. Make sure the app_button module is initialized.
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            */

            // Start detecting button presses.
            err_code = bsp_buttons_enable(1 << 0 /*SEND_MEAS_BUTTON_ID*/);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events. This should be done to save power when not connected
                         to a peer.
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            */
        
            // Stop detecting button presses when not connected.
            err_code = bsp_buttons_enable(BSP_BUTTONS_NONE);
            APP_ERROR_CHECK(err_code);
        
            advertising_start();
            break;

        /* *********************************************** ?????? */
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,
                                                   &m_keys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle,
                                                 NULL,
                                                 0,
                                                 BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            master_id_matches  = memcmp(&p_ble_evt->evt.gap_evt.params.sec_info_request.master_id,
                                        &m_enc_key.master_id,
                                        sizeof(ble_gap_master_id_t)) == 0;
            p_distributed_keys = &m_auth_status.kdist_periph;

            p_enc_info  = (p_distributed_keys->enc  && master_id_matches) ? &m_enc_key.enc_info : NULL;
            p_id_info   = (p_distributed_keys->id   && master_id_matches) ? &m_id_key.id_info   : NULL;
            p_sign_info = (p_distributed_keys->sign && master_id_matches) ? &m_sign_key         : NULL;

            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, p_id_info, p_sign_info);
            APP_ERROR_CHECK(err_code);
            break;
        /* *********************************************** ?????? */

        
        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                APP_ERROR_CHECK(err_code);
                // Configure buttons with sense level low as wakeup source.
                err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
                APP_ERROR_CHECK(err_code);
                // Go to system-off mode (this function will not return; wakeup will cause a reset)                
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        /* karel NOT USED 
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;

        default:
            // No implementation needed.
            break;
        */
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
   
    ble_conn_params_on_ble_evt(p_ble_evt);
    /*
    YOUR_JOB: Add service ble_evt handlers calls here, like, for example:
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    */
#if defined(USE_BAS)
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
#endif
    
    on_ble_evt(p_ble_evt);

}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    //pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

#ifdef S110
    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
/* TODO
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
TODO*/

/**@brief Function for handling a bsp event.
 *
 * @param[in]     evt                        BSP event.
 */
/* YOUR_JOB: Uncomment this function if you need to handle button events.
static void bsp_event_handler(bsp_event_t evt)
{
        switch (evt)
        {
            case BSP_EVENT_KEY_0:
                // Code to handle BSP_EVENT_KEY_0
                break;

            // Handle any other event

            default:
                APP_ERROR_HANDLER(evt);
                break;
        }
    }
}
*/
static void button_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            //not needed blood_pressure_measurement_send();
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{   
        uint32_t err_code;
        // Note: Before start using buttons, assign events to buttons, as shown below.
        //      err_code = bsp_event_to_button_assign(BUTTON_0_ID, BSP_EVENT_KEY_0);
        //      APP_ERROR_CHECK(err_code);
        // Note: Enable buttons which you want to use.
        //      err_code = bsp_buttons_enable((1 << WAKEUP_BUTTON_ID) | (1 << BUTTON_0_ID)); 
        //      APP_ERROR_CHECK(err_code);
        // Note: If the only use of buttons is to wake up, the bsp module can be omitted, and
        // the wakeup button can be configured by
        err_code = bsp_buttons_enable(1 << WAKEUP_BUTTON_ID);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing bsp module.
 */
static void bsp_module_init(void)
{
        uint32_t err_code;
        // Note: If the only use of buttons is to wake up, bsp_event_handler can be NULL.
        //OLD err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
        err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), button_event_handler);
    
        APP_ERROR_CHECK(err_code);
        // Note: If the buttons will be used to do some task, assign bsp_event_handler, as shown below.
        // err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), bsp_event_handler);
        // APP_ERROR_CHECK(err_code);
        // Buttons initialization.
        buttons_init();
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize
    timers_init();
    gpiote_init();
    
    ble_stack_init();
    
    bsp_module_init();
    //TODO scheduler_init();
    // device_manager_init(); - we don't have one???
    gap_params_init();
    advertising_init();
    services_init();
    // sensor_simulator_init(); - we don't have one???
    conn_params_init();
    
    sec_params_init();

    // Start execution
    timers_start();      application_timers_start_SAME_AS_timers_start_QQQ(); 
    advertising_start(); //!!!

    // Enter main loop
    for (;;)
    {
        //TODO app_sched_execute();
        power_manage();
    }
}

/**
 * @}
 */
