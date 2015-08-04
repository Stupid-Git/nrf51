
#ifndef BLE_TD2S_H__
#define BLE_TD2S_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_TD2S_SERVICE 0x0001                      /**< The UUID of the Nordic UART Service. */
#define BLE_TD2S_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/* Forward declaration of the ble_td2s_t type. */
typedef struct ble_td2s_s ble_td2s_t;

/**@brief Nordic UART Service event handler type. */
typedef void (*ble_td2s_data_handler_t) (ble_td2s_t * p_td2s, uint8_t * p_data, uint16_t length);
typedef void (*ble_td2s_cmd_handler_t) (ble_td2s_t * p_td2s, uint8_t * p_data, uint16_t length);
typedef void (*ble_td2s_sts_handler_t) (ble_td2s_t * p_td2s, uint8_t * p_data, uint16_t length);

/**@brief Nordic UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_td2s_init
 *          function.
 */
typedef struct
{
    ble_td2s_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
    ble_td2s_cmd_handler_t  cmd_handler;  /**< Event handler to be called for handling received CMD.  */
    ble_td2s_sts_handler_t  sts_handler;  /**< Event handler to be called for handling received STS.  */
} ble_td2s_init_t;

/**@brief Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_td2s_s
{
    uint8_t                  uuid_type;                 /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                 service_handle;            /**< Handle of Nordic UART Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t tx_handles;                /**< Handles related to the TX characteristic (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t rx_handles;                /**< Handles related to the RX characteristic (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t cmd_handles;               /**< Handles related to the RX characteristic (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t sts_handles;               /**< Handles related to the RX characteristic (as provided by the S110 SoftDevice). */
    uint16_t                 conn_handle;               /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled;   /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_td2s_data_handler_t  data_handler;              /**< Event handler to be called for handling received data. */
    ble_td2s_cmd_handler_t   cmd_handler;               /**< Event handler to be called for handling received CMD. */
    ble_td2s_sts_handler_t   sts_handler;               /**< Event handler to be called for handling received STS. */
};

/**@brief Function for initializing the Nordic UART Service.
 *
 * @param[out] p_td2s      Nordic UART Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_td2s_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_td2s or p_td2s_init is NULL.
 */
uint32_t ble_td2s_init(ble_td2s_t * p_td2s, const ble_td2s_init_t * p_td2s_init);

/**@brief Function for handling the Nordic UART Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this function each time an
 * event is received from the S110 SoftDevice. This function processes the event if it
 * is relevant and calls the Nordic UART Service event handler of the
 * application if necessary.
 *
 * @param[in] p_td2s       Nordic UART Service structure.
 * @param[in] p_ble_evt   Event received from the S110 SoftDevice.
 */
void ble_td2s_on_ble_evt(ble_td2s_t * p_td2s, ble_evt_t * p_ble_evt);

/**@brief Function for sending a string to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in] p_td2s       Pointer to the Nordic UART Service structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_td2s_string_send(ble_td2s_t * p_td2s, uint8_t * p_string, uint16_t length);

#endif // BLE_TD2S_H__
