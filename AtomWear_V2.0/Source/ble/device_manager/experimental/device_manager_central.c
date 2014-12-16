/* Copyright (C) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
 * @file
 */
#include "device_manager.h"
#include "app_trace.h"
#include "ble_advdata.h"
#include "pstorage.h"
#include "ble_hci.h"

#define INVALID_ADDR_TYPE 0xFF


/**
 * @defgroup device_manager_app_states Connection Manager Application States
 * @{
 */
#define CONTROL_PROCEDURE_IN_PROGRESS 0x01 /**< State to know if a security procedure is ongoing or not. */
#define QUEUED_CONTROL_REQUEST        0x02 /**< State to know if there is any queued security request or not. */
/** @} */

/**
 * @defgroup device_manager_conn_inst_states Connection Manager Connection Instances States.
 * @{
 */
#define IDLE             0x01  /**< Indicates connection instance is free. */
#define CONNECTED        0x02  /**< Indicates connection is successfully estblished. */
#define PAIRING          0x04  /**< Indicates pairing procedure is in progress. This state is used for pairing & bonding, as pairing is needed for both. */
#define BONDED           0x08  /**< Indicates device is bonded. */
#define DISCONNECTING    0x10  /**< Indicates disconnection is in progress, app will be notified first, but no further ative procedures on the link. */
#define PAIRING_PENDING  0x20  /**< Indicates pairing request is pending on the link. */
#define BOND_INFO_UPDATE 0x40  /**< Bonding information has been updated, update flash. */
#define LINK_ENCRYPTED   0x80  /**< Indicates link is encrypted. */
/** @} */

/**
 * @defgroup device_manager_peer_id_defines Peer Identification Information Defines.
 *
 * @brief These defines are used to know which of peer identification is applicable for a peer.
 *
 * @details These defines are used to know which of peer identification is applicable for a peer.
 *          Here, bit map is used as it is possible that application has both IRK and address for
 *          identification.
 * @{
 */
#define UNASSIGNED            0xFF  /**< Peer instance is unassigned/unused. */
#define IRK_ENTRY             0x01  /**< Peer instance has IRK as identification information. */
#define ADDR_ENTRY            0x02  /**< Peer instance has address as identification information. */
#define SERVICE_CONTEXT_ENTRY 0x04  /**< Peer instance has service context set. */
#define APP_CONTEXT_ENTRY     0x08  /**< Peer instance has an application context set. */
/** @} */

typedef enum
{
    STORE_ALL_CONTEXT,
    FIRST_BOND_STORE,
    UPDATE_PEER_ADDR
}device_store_state_t;

/**
 * @defgroup device_manager_context_offsets Context Offsets
 * @{
 * @breif Context offsets of each of the context information in persistent memory.
 *
 * @details Below is a layout showing how each of the context information
 *          is stored in persistent memory.
 *
 * All Device context is stored in the flash as follows:
 * +---------+---------+---------+------------------+----------------+--------------------+
 * | Block / Device ID + Layout of stored information in storage block                    |
 * +---------+---------+---------+------------------+----------------+--------------------+
 * | Block 0 | Device 0| Peer Id | Bond Information | Service Context| Application Context|
 * +---------+---------+---------+------------------+----------------+--------------------+
 * | Block 1 | Device 1| Peer Id | Bond Information | Service Context| Application Context|
 * +---------+---------+---------+------------------+----------------+--------------------+
 * |  ...              | ....                                                             |
 * +---------+---------+---------+------------------+----------------+--------------------+
 * | Block N | Device N| Peer Id | Bond Information | Service Context| Application Context|
 * +---------+---------+---------+------------------+----------------+--------------------+
 *
 * Following defines are used to get offset of each of the components with in a block.
 */

#define PEER_ID_STORAGE_OFFSET      0                                                /**< Offset at which peer id  is stored in the block. */
#define BOND_STORAGE_OFFSET         PEER_ID_SIZE                                     /**< Offset at which bond information is stored in the block. */
#define SERVICE_STORAGE_OFFSET      (BOND_STORAGE_OFFSET + BOND_SIZE)                /**< Offset at which service context is stored in the block. */
#define APP_CONTEXT_STORAGE_OFFSET  (SERVICE_STORAGE_OFFSET + SERVICE_CONTEXT_SIZE)  /**< Offset at which application context is stored in the block. */
/** @} */

/**
 * @defgroup device_manager_context_size Context size.
 * @{
 * @breif This group of defines sizes of each of the context information.
 */
#define PEER_ID_SIZE         (sizeof(peer_id_t))                                      /**< Size of peer identification information. */
#define BOND_SIZE            (sizeof(bond_context_t))                                 /**< Size of bond information. */
#define DEVICE_CONTEXT_SIZE  (PEER_ID_SIZE+BOND_SIZE)                                 /**< Size of Device context, include peer identification and bond information. */
#define GATTS_SERVICE_CONTEXT_SIZE (sizeof(dm_gatts_context_t))                       /**< Size of GATTS service context. */
#define GATTC_SERVICE_CONTEXT_SIZE (sizeof(dm_gatt_client_context_t))                 /**< Size of GATTC service context. */
#define SERVICE_CONTEXT_SIZE (GATTS_SERVICE_CONTEXT_SIZE+GATTC_SERVICE_CONTEXT_SIZE)
#if(DEVICE_MANAGER_APP_CONTEXT_SIZE != 0)
#define APP_CONTEXT_SIZE     (sizeof(uint32_t) + DEVICE_MANAGER_APP_CONTEXT_SIZE)     /**< Size of application context. */
#else //DEVICE_MANAGER_APP_CONTEXT_SIZE
#define APP_CONTEXT_SIZE     0                                                        /**< Size of application context. */
#endif // DEVICE_MANAGER_APP_CONTEXT_SIZE
#define ALL_CONTEXT_SIZE     (DEVICE_CONTEXT_SIZE + SERVICE_CONTEXT_SIZE + APP_CONTEXT_SIZE) /**< Size of all contexts. */
/** @} */


/**
 * @defgroup devive_manager_log Module's Log Macros
 * @details Macros used for creating module logs which can be useful in understanding handling
 *          of events or actions on API requests. These are intended for debugging purposes and
 *          can be disabled by defining the DM_DIABLE_LOGS.
 * @note That if ENABLE_DEBUG_LOG_SUPPORT is disabled, having DM_DIABLE_LOGS has no effect.
 * @{
 */
#define DM_DIABLE_LOGS  /**< Enable this macro to disable any logs from this module. */

#ifndef DM_DIABLE_LOGS
#define DM_LOG  app_trace_log   /**< Used for logging details. */
#define DM_ERR  app_trace_log   /**< Used for logging errors in the module. */
#define DM_TRC  app_trace_log   /**< Used for getting trace of execution in the module. */
#define DM_DUMP app_trace_dump  /**< Used for dumping octet information to get details of bond information etc. */
#else // DM_DIABLE_LOGS
#define DM_DUMP(...)            /**< Diasbles dumping of octet streams. */
#define DM_LOG(...)             /**< Diasbles detailed logs. */
#define DM_ERR(...)             /**< Diasbles error logs. */
#define DM_TRC(...)             /**< Diasbles traces. */
#endif // DM_DIABLE_LOGS

/**
 * @defgroup devive_manager_mutex_lock_unlock Module's Mutex Lock/Unlock Macros.
 *
 * @details Macros used to lock and unlock modules. Currently, SDK does not use mutexes but
 *          framework is provided in case need arises to use an alternative architecture.
 * @{
 */
#define DM_MUTEX_LOCK()   SDK_MUTEX_LOCK(m_dm_mutex)    /**< Lock module using mutex */
#define DM_MUTEX_UNLOCK() SDK_MUTEX_UNLOCK(m_dm_mutex)  /**< Unlock module using mutex */
/** @} */


/**
 * @defgroup devive_manager_misc_defines Miscellaneous defines used across the module.
 * @{
 */
#define DM_GATT_ATTR_SIZE             4                                             /**< Size of each GATT attribute to be stored persistently. */
#define DM_GATT_SERVER_ATTR_MAX_SIZE ((DM_GATT_ATTR_SIZE * DM_GATT_CCCD_COUNT) + 2) /**<  Maximum size of GATT attrbutes to be stored.*/
#define DM_SERVICE_CONTEXT_COUNT     (DM_PROTOCOL_CNTXT_ALL+1)                      /**< Maximum number of service contexts. */
#define DM_EVT_DEVICE_CONTEXT_BASE    0x20                                          /**< Base for device context base. */
#define DM_EVT_SERVICE_CONTEXT_BASE   0x30                                          /**< Base for device context base. */
#define DM_EVT_APP_CONTEXT_BASE       0x40                                          /**< Base for device context base. */
#define DM_LOAD_OPERATION_ID          0x01                                          /**< Store operation identifier. */
#define DM_STORE_OPERATION_ID         0x02                                          /**< Store operation identifier. */
#define DM_CLEAR_OPERATION_ID         0x03                                          /**< Clear operation identifier. */
/** @} */
#define DM_GATTS_INVALID_SIZE 0xFFFF

/**
 * @defgroup api_param_check API Parameters check macros.
 *
 * @details Macros that verify parameters passed to the module in the APIs. These macros
 *          could be mapped to nothing in final versions of code to save execution and size.
 *
 * @{
 */
/**
 * @brief If parameter is NULL, returns NRF_ERROR_NULL.
 */
// #define DM_DISABLE_API_PARAM_CHECK /**< Macro to disable API parameters check if need be. */

#ifndef DM_DISABLE_API_PARAM_CHECK

/**
 * @brief Verify NULL parameters are not passed to API.
 */
#define NULL_PARAM_CHECK(PARAM)                                                                   \
        if ((PARAM) == NULL)                                                                      \
        {                                                                                         \
            return (NRF_ERROR_NULL | DEVICE_MANAGER_ERR_BASE);                                    \
        }
/**@} */


/**
 * @brief Verify module's initialization status. Returns NRF_ERROR_INVALID_STATE it is not.
 */
#define VERIFY_MODULE_INITIALIZED()                                                               \
        do                                                                                        \
        {                                                                                         \
            if (!m_module_initialized)                                                            \
            {                                                                                     \
                 return (NRF_ERROR_INVALID_STATE | DEVICE_MANAGER_ERR_BASE);                      \
            }                                                                                     \
        } while(0)


/**
 * @brief Verify module's initialization status. Returns in case it is not.
 */
#define VERIFY_MODULE_INITIALIZED_VOID()                                                          \
        do                                                                                        \
        {                                                                                         \
            if (!m_module_initialized)                                                            \
            {                                                                                     \
                 return;                                                                          \
            }                                                                                     \
        } while(0)


/**
 * @brief Verify application is registered. Returns NRF_ERROR_INVALID_STATE in case a
 *        module API is called without registering an application with the module.
 */
#define VERIFY_APP_REGISTERED(X)                                                                  \
        do                                                                                        \
        {                                                                                         \
            if (((X)>= DEVICE_MANAGER_MAX_APPLICATIONS) ||                                        \
                (m_application_table[(X)].ntf_cb == NULL))                                        \
            {                                                                                     \
                 return (NRF_ERROR_INVALID_STATE | DEVICE_MANAGER_ERR_BASE);                      \
            }                                                                                     \
        } while(0)


/**
 * @brief Verify application is registered. Returns NRF_ERROR_INVALID_STATE in case a
 *        module API is called without registering an application with the module.
 */
#define VERIFY_APP_REGISTERED_VOID(X)                                                             \
        do                                                                                        \
        {                                                                                         \
            if (((X)>= DEVICE_MANAGER_MAX_APPLICATIONS) ||                                        \
                (m_application_table[(X)].ntf_cb == NULL))                                        \
            {                                                                                     \
                 return;                                                                          \
            }                                                                                     \
        } while(0)


/**
 * @brief Verify connection instance is allocated.
 */
#define VERIFY_CONNECTION_INSTANCE(X)                                                             \
        do                                                                                        \
        {                                                                                         \
            if (((X) >= DEVICE_MANAGER_MAX_CONNECTIONS) ||                                        \
                (m_connection_table[(X)].state == IDLE))                                          \
            {                                                                                     \
                 return (NRF_ERROR_INVALID_ADDR | DEVICE_MANAGER_ERR_BASE);                       \
            }                                                                                     \
        } while(0)


/**
 * @brief Verify connection instance is allocated.
 */
#define VERIFY_DEVICE_INSTANCE(X)                                                                 \
        do                                                                                        \
        {                                                                                         \
            if (((X) >= DEVICE_MANAGER_MAX_BONDS) ||                                              \
                (m_peer_table[(X)].id_bitmap == UNASSIGNED))                                      \
            {                                                                                     \
                 return (NRF_ERROR_INVALID_ADDR | DEVICE_MANAGER_ERR_BASE);                       \
            }                                                                                     \
        } while(0)
#else
#define NULL_PARAM_CHECK(X)
#define VERIFY_MODULE_INITIALIZED()
#define VERIFY_MODULE_INITIALIZED_VOID()
#define VERIFY_APP_REGISTERED(X)
#define VERIFY_APP_REGISTERED_VOID(X)
#define VERIFY_CONNECTION_INSTANCE(X)
#define VERIFY_DEVICE_INSTANCE(X)
#endif // DM_DISABLE_API_PARAM_CHECK
/** @} */


/**
 * @defgroup dm_data_types Module's internal data types.
 * @breif This section describes module's internal data structures.
 * @{
 */
/**
 * @brief Peer identification information.
 */
typedef struct
{
    ble_gap_id_key_t peer_id;    /**< IRK and/or address of peer. */
    uint8_t          id_bitmap;  /**< Indicates information if this is assigned or has valid IRK or Address or both. */
} peer_id_t;

STATIC_ASSERT(sizeof(peer_id_t) % 4 == 0);

/**
 * @brief Portion of bonding information exchanged by a device during bond creation that needs to
 *        be stored persistently.
 * @note  An entry is not made in this table unless device is bonded.
 */
typedef struct
{
    ble_gap_enc_key_t peer_enc_key;  /**< Local LTK info, central IRK and address */
} bond_context_t;

STATIC_ASSERT(sizeof(bond_context_t) % 4 == 0); /**< Check to ensure bond information is a multiple of 4. */

/**
 * @brief GATT Server Attributes size and data.
 */
typedef struct
{
    uint16_t size;                                      /**< Size of attributes stored. */
    uint8_t  attributes[DM_GATT_SERVER_ATTR_MAX_SIZE];  /**< Array to hold server attributes. */
} dm_gatts_context_t;
STATIC_ASSERT(sizeof(dm_gatts_context_t) % 4 == 0);

/**
 * @breif GATT Client context information. Place holder for now.
 */
typedef struct
{
    void * dummy;  /**< Place holder, currently unused. */
} dm_gatt_client_context_t;
STATIC_ASSERT(sizeof(dm_gatt_client_context_t) % 4 == 0);
STATIC_ASSERT((DEVICE_MANAGER_APP_CONTEXT_SIZE % 4) == 0);

/**
 * @brief Connection instance definition. Maintains information with respect to an active peer.
 */
typedef struct
{
    ble_gap_addr_t peer_addr;      /**< Peer identification information. This information is retained as long as the connection session exists, once disconnected, for non-bonded devices this information is not stored persistently. */
    uint16_t       conn_handle;    /**< Connection handle for the device. */
    uint8_t        state;          /**< Link status. */
    uint8_t        bonded_dev_id;  /**< In case device is bonded, points to the corresponding bonded device. This index can be used to index service and bond context as well. */
} connection_instance_t;

/**
 * @brief Application instance definition. Maintains information with respect to a registered
 *        application.
 */
typedef struct
{
    dm_event_cb_t        ntf_cb;     /**< Callback registered with the application. */
    ble_gap_sec_params_t sec_param;  /**< Local security parameters registered by the application. */
    uint8_t              state;      /**< Application state. Currently is used only for knowing if any security procedure is in progress and/or a security procedure is pending to be requested. */
    uint8_t              service;    /**< Service registered by the application. */
} application_instance_t;

/**
 * @brief Function pointers used to perform necessary action of storing each of the service context
 *        as registered by the application.
 * @param[in] p_block_handle Storage block identifier.
 * @param[in] p_handle       Device handle identifying device that is being stored.
 */
typedef api_result_t (*service_context_access_t)(pstorage_handle_t const * p_block_handle,
                                                 dm_handle_t const       * p_handle);

/**
 * @brief Function pointers used to perform necessary action of applying context information.
 *
 * @param[in] p_handle Device handle identifying device that is being stored.
 */
typedef api_result_t (*service_context_apply_t)(dm_handle_t * p_handle);

/**
 * @brief Function pointers used to call necessary storage function of store or update.
 *
 * @param[in]  p_dest Destination address where data is to be stored persistently.
 * @param[in]  p_src  Source address containing data to be stored.
 * @param[in]  size   Size of data to be stored expressed in bytes. Should be word aligned.
 * @param[in]  offset Offset in bytes to be applied when writing to the block.
 */
typedef uint32_t (*storage_operation)(pstorage_handle_t * p_dest,
                                      uint8_t           * p_src,
                                      pstorage_size_t     size,
                                      pstorage_size_t     offset);
/** @} */

/**
 * @defgroup dm_tables  Module's internal tables.
 * @breif This section describes module's internal tables and static global variables needed for
 *        its functionality.
 * @{
 */
#if(DEVICE_MANAGER_APP_CONTEXT_SIZE != 0)
static uint8_t                 * m_app_context_table[DEVICE_MANAGER_MAX_BONDS];       /**< Table to remember application contexts of bonded devices. */
#endif // DEVICE_MANAGER_APP_CONTEXT_SIZE
static peer_id_t               m_peer_table[DEVICE_MANAGER_MAX_BONDS];                /**< Table maintaining bonded devices' identification information, an instance is allocated in the table when a device is bonded and freed when bond information is deleted. */
static bond_context_t          m_bond_table[DEVICE_MANAGER_MAX_CONNECTIONS];          /**< Table maintaining bond information for active peers. */
static dm_gatts_context_t      m_gatts_table[DEVICE_MANAGER_MAX_CONNECTIONS];         /**< Table for service information for active connection instances. */
static connection_instance_t   m_connection_table[DEVICE_MANAGER_MAX_CONNECTIONS];    /**< Table maintaining active peer information. An instance is allocated in the table when a new connection is established and freed on disconnection. */
static application_instance_t  m_application_table[DEVICE_MANAGER_MAX_APPLICATIONS];  /**< Table maintaining application instances. */
static pstorage_handle_t       m_storage_handle;                                      /**< Persistent storage handle for blocks requested by the module. */
static uint32_t                m_peer_addr_update;                                    /**< 32-bit bitmap to remember peer device address update */
static bool                    m_module_initialized = false;                          /**< State indicating if module is initialized or not. */
static ble_gap_id_key_t        m_local_id_info;                                       /**< ID information of central in case resolvable address is used. */

SDK_MUTEX_DEFINE(m_dm_mutex) /**< Mutex variable. Currently unused, this declaration does not occupy any space in RAM. */
/** @} */

/**@breif Dummy function, when there is no service registered. */
static __INLINE api_result_t no_service_context_store(pstorage_handle_t const * p_block_handle,
                                                      dm_handle_t const       * p_handle);

/**@breif GATT Server context store function. */
static __INLINE api_result_t gatts_context_store(pstorage_handle_t const * p_block_handle,
                                                 dm_handle_t const       * p_handle);

/**@breif GATT Client context store function. */
static __INLINE api_result_t gattc_context_store(pstorage_handle_t const * p_block_handle,
                                                 dm_handle_t const       * p_handle);

/**@breif  GATT Server & Client context store function. */
static __INLINE api_result_t gattsc_context_store(pstorage_handle_t const * p_block_handle,
                                                  dm_handle_t const       * p_handle);

/**@breif Dummy function, when there is no service registered. */
static __INLINE api_result_t no_service_context_load(pstorage_handle_t const * p_block_handle,
                                                     dm_handle_t const       * p_handle);

/**@breif GATT Server context load function. */
static __INLINE api_result_t gatts_context_load(pstorage_handle_t const * p_block_handle,
                                                dm_handle_t const       * p_handle);

/**@breif GATT Client context load function. */
static __INLINE api_result_t gattc_context_load(pstorage_handle_t const * p_block_handle,
                                                dm_handle_t const       * p_handle);

/**@breif  GATT Server & Client context load function. */
static __INLINE api_result_t gattsc_context_load(pstorage_handle_t const * p_block_handle,
                                                 dm_handle_t const       * p_handle);

/**@breif Dummy function, when there is no service registered. */
static __INLINE api_result_t no_service_context_apply(dm_handle_t * p_handle);

/**@breif GATT Server context apply function. */
static __INLINE api_result_t gatts_context_apply(dm_handle_t * p_handle);

/**@breif GATT Client context apply function. */
static __INLINE api_result_t gattc_context_apply(dm_handle_t * p_handle);

/**@breif  GATT Server & Client context apply function. */
static __INLINE api_result_t gattsc_context_apply(dm_handle_t * p_handle);


/**< Array of function pointers to be able store based on service registered. */
const service_context_access_t m_service_context_store[DM_SERVICE_CONTEXT_COUNT] =
{
    no_service_context_store,  /**< Dummy function, when there is no service context registered. */
    gatts_context_store,       /**< GATT Server context store function. */
    gattc_context_store,       /**< GATT Client context store function. */
    gattsc_context_store       /**< GATT Server & Client context store function. */
};


/**< Array of function pointers to be able store based on service registered. */
const service_context_access_t m_service_context_load[DM_SERVICE_CONTEXT_COUNT] =
{
    no_service_context_load,  /**< Dummy function, when there is no service context registered. */
    gatts_context_load,       /**< GATT Server context load function. */
    gattc_context_load,       /**< GATT Client context load function. */
    gattsc_context_load       /**< GATT Server & Client context load function. */
};


/**< Array of function pointers to be able apply context based on service registered. */
const service_context_apply_t m_service_context_apply[DM_SERVICE_CONTEXT_COUNT] =
{
    no_service_context_apply,  /**< Dummy function, when there is no service context registered. */
    gatts_context_apply,       /**< GATT Server context store function. */
    gattc_context_apply,       /**< GATT Client context store function. */
    gattsc_context_apply       /**< GATT Server & Client context store function. */
};


/**< Constant to update init value for context in flash */
const uint32_t m_context_init_len = 0xFFFFFFFF;

/**< Uinsigned integer constant 1 for bit set and reset. */
const uint32_t m_bit_set = 1;

/**
 * @brief Set update status for device identified by 'index'.
 */
static __INLINE void update_status_bit_set(uint32_t index)
{
    m_peer_addr_update |= (m_bit_set << index);
}


/**
 * @brief Initializes application instance identified by 'index'.
 */
static __INLINE void update_status_bit_reset(uint32_t index)
{
    m_peer_addr_update &= (~(m_bit_set << index));
}


/**
 * @brief Provide status of update status for device identified by 'index'.
 */
static __INLINE bool update_status_bit_is_set(uint32_t index)
{
    return ((m_peer_addr_update & (m_bit_set << index))? true: false);
}


/**
 * @brief Initializes application instance identified by 'index'.
 */
static __INLINE void application_instance_init(uint32_t index)
{
    DM_TRC("[DM]: Initializing Application Instance 0x%08X.\r\n", index);
    m_application_table[index].ntf_cb  = NULL;
    m_application_table[index].state   = 0x00;
    m_application_table[index].service = 0x00;
}


/**
 * @brief Initializes connection instance identified by 'index'.
 */
static __INLINE void connection_instance_init(uint32_t index)
{
    DM_TRC("[DM]: Initializing Connection Instance 0x%08X.\r\n", index);
    m_connection_table[index].state         = IDLE;
    m_connection_table[index].conn_handle   = BLE_CONN_HANDLE_INVALID;
    m_connection_table[index].bonded_dev_id = DM_INVALID_ID;
    memset(&m_connection_table[index].peer_addr, 0, sizeof (ble_gap_addr_t));
}


/**
 * @brief Initializes peer device instance identified by 'index'.
 */
static __INLINE void peer_instance_init(uint32_t index)
{
    DM_TRC("[DM]: Initializing Peer Instance 0x%08X.\r\n", index);
    memset(m_peer_table[index].peer_id.id_addr_info.addr, 0, BLE_GAP_ADDR_LEN);
    memset(m_peer_table[index].peer_id.id_info.irk, 0, BLE_GAP_SEC_KEY_LEN);

    // Initialize address type to invalid.
    m_peer_table[index].peer_id.id_addr_info.addr_type = INVALID_ADDR_TYPE;

    // Initialize identification bit map to unassigned.
    m_peer_table[index].id_bitmap = UNASSIGNED;

    // Reset status bit.
    update_status_bit_reset(index);

#if(DEVICE_MANAGER_APP_CONTEXT_SIZE != 0)
    // Initialize application context for bond device.
    m_app_context_table[index] = NULL;
#endif // DEVICE_MANAGER_APP_CONTEXT_SIZE
}


/**
 * @brief Searches for connection instance matching the connection handle requested or state
 *        requested.
 * @details Connection handle and or state information is used to get a connection instance, it
 *          is possible to ignore connection handle by using BLE_CONN_HANDLE_INVALID.
 */
static api_result_t connection_instance_find(uint16_t   conn_handle,
                                             uint8_t    state,
                                             uint32_t * instance)
{
    api_result_t retval;
    uint32_t     index;

    retval = NRF_ERROR_INVALID_STATE;

    for (index = 0; index < DEVICE_MANAGER_MAX_CONNECTIONS; index++)
    {
        // Search only based on state.
        if (state & m_connection_table[index].state)
        {
            retval = NRF_ERROR_NOT_FOUND;

            // Ignore connection handle!
            if ((conn_handle == BLE_CONN_HANDLE_INVALID) ||
                (conn_handle == m_connection_table[index].conn_handle))
            {
                // Search for matching connection handle.
                (*instance) = index;
                retval      = NRF_SUCCESS;
                break;
            }
        }
    }

    return retval;
}


/**
 * @brief Allocates device instance for a bonded device.
 */
static __INLINE api_result_t device_instance_allocate(uint8_t        * p_device_index,
                                                      ble_gap_addr_t * p_addr)
{
    api_result_t retval;
    uint32_t     index;

    retval = DM_DEVICE_CONTEXT_FULL;

    for (index = 0; index < DEVICE_MANAGER_MAX_BONDS; index++)
    {
        DM_TRC("[DM]:[DI 0x%02X]: Device type 0x%02X.\r\n",
               index, m_peer_table[index].peer_id.id_addr_info.addr_type);
        DM_TRC("[DM]: Device Addr 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X.\r\n",
               m_peer_table[index].peer_id.id_addr_info.addr[0],
               m_peer_table[index].peer_id.id_addr_info.addr[1],
               m_peer_table[index].peer_id.id_addr_info.addr[2],
               m_peer_table[index].peer_id.id_addr_info.addr[3],
               m_peer_table[index].peer_id.id_addr_info.addr[4],
               m_peer_table[index].peer_id.id_addr_info.addr[5]);

        if (m_peer_table[index].id_bitmap == UNASSIGNED)
        {
            if (p_addr->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE)
            {
                m_peer_table[index].id_bitmap           &= (~ADDR_ENTRY);
                m_peer_table[index].peer_id.id_addr_info = (*p_addr);
            }
            else
            {
                m_peer_table[index].id_bitmap &= (~IRK_ENTRY);
            }
            (*p_device_index) = index;
            retval            = NRF_SUCCESS;
            DM_LOG("[DM]: Allocated device instance 0x%02X\r\n", index);
            break;
        }
    }

    return retval;
}


/**
 * @brief Frees a device instance allocated for bonded device.
 */
static __INLINE api_result_t device_instance_free(uint32_t device_index)
{
    api_result_t      retval;
    pstorage_handle_t block_handle;

    // Get block handle.
    retval = pstorage_block_identifier_get(&m_storage_handle, device_index, &block_handle);
    if (retval == NRF_SUCCESS)
    {
        DM_TRC("[DM]:[DI 0x%02X]: Freeing Instance.\r\n", device_index);

        // Request clearing of the block.
        retval = pstorage_clear(&block_handle, ALL_CONTEXT_SIZE);
        if (retval == NRF_SUCCESS)
        {
            // Free instance in RAM.
            peer_instance_init(device_index);
        }
    }

    return retval;
}


/**
 * @brief Searches for the device in the bonded device list.
 */
static api_result_t device_instance_find(ble_gap_addr_t * p_addr, uint32_t * p_device_index)
{
    api_result_t retval;
    uint32_t     index;

    retval = NRF_ERROR_NOT_FOUND;
    DM_TRC("[DM]: Searching for device 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X.\r\n",
           p_addr->addr[0], p_addr->addr[1], p_addr->addr[2], p_addr->addr[3],
           p_addr->addr[4], p_addr->addr[5]);

    for (index = 0; index < DEVICE_MANAGER_MAX_BONDS; index++)
    {
        DM_TRC("[DM]:[DI 0x%02X]: Device type 0x%02X.\r\n",
               index, m_peer_table[index].peer_id.id_addr_info.addr_type);
        DM_TRC("[DM]: Device Addr 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X.\r\n",
               m_peer_table[index].peer_id.id_addr_info.addr[0],
               m_peer_table[index].peer_id.id_addr_info.addr[1],
               m_peer_table[index].peer_id.id_addr_info.addr[2],
               m_peer_table[index].peer_id.id_addr_info.addr[3],
               m_peer_table[index].peer_id.id_addr_info.addr[4],
               m_peer_table[index].peer_id.id_addr_info.addr[5]);

        if (memcmp(&m_peer_table[index].peer_id.id_addr_info, p_addr, sizeof(ble_gap_addr_t)) == 0)
        {
            DM_LOG("[DM]: Found device at instance 0x%02X\r\n",index);
            (*p_device_index) = index;
            retval= NRF_SUCCESS;
            break;
        }
    }

    return retval;
}


/**
 * @breif Notifies connection manager event to application.
 */
static __INLINE void app_evt_notify(dm_handle_t const * const p_handle,
                                    dm_event_t const * const  p_event,
                                    uint32_t const            event_result)
{
    dm_event_cb_t app_cb = m_application_table[0].ntf_cb;

    DM_MUTEX_UNLOCK();

    DM_TRC("[DM]: Notifying application of event 0x%02X\r\n", p_event->event_id);

    UNUSED_VARIABLE(app_cb(p_handle, p_event, event_result));

    DM_MUTEX_LOCK();
}


/**
 * @breif Allocates instance, instance identifier is provided in parameter 'instance' in case
 *        routine succeeds.
 */
static __INLINE uint32_t connection_instance_alloc(uint32_t * instance)
{
    uint32_t retval;

    DM_TRC("[DM]: Request to allocation connection instance\r\n");

    retval = connection_instance_find(BLE_CONN_HANDLE_INVALID, IDLE, instance);
    if (retval == NRF_SUCCESS)
    {
        DM_LOG("[DM]:[%02X]: Connection Instance Allocated.\r\n", (*instance));
        m_connection_table[*instance].state = CONNECTED;
    }
    else
    {
        DM_LOG("[DM]: No free connection instances available\r\n");
        retval = NRF_ERROR_NO_MEM;
    }

    return retval;
}


/**
 * @breif Frees instance, instance identifier is provided in parameter 'instance' in case
 *        routine succeeds.
 */
static __INLINE void connection_instance_free(uint32_t * instance)
{
    DM_TRC("[DM]:[CI 0x%02X]: Freeing connection instance\r\n", (*instance));

    if (m_connection_table[*instance].state != IDLE)
    {
        DM_LOG("[DM]:[%02X]: Freed connection instance.\r\n", (*instance));
        connection_instance_init(*instance);
    }
}


uint32_t dummy_handler(pstorage_handle_t * p_dest,
                       uint8_t           * p_src,
                       pstorage_size_t     size,
                       pstorage_size_t     offset)
{
    return NRF_SUCCESS;
}


/**
 * @breif Routine to save device context persistently.
 */
static __INLINE void device_context_store(dm_handle_t const * p_handle, device_store_state_t state)
{
    pstorage_handle_t block_handle;
    storage_operation store_fn;
    api_result_t      retval;

    DM_LOG("[DM]: --> device_context_store\r\n");

    retval = pstorage_block_identifier_get(&m_storage_handle,
                                           p_handle->device_id,
                                           &block_handle);

    if (retval == NRF_SUCCESS)
    {
        DM_DUMP((uint8_t *)&m_peer_table[p_handle->device_id].peer_id.id_addr_info, sizeof(m_peer_table[p_handle->device_id].peer_id.id_addr_info));
        if ((BOND_INFO_UPDATE ==
            (m_connection_table[p_handle->connection_id].state & BOND_INFO_UPDATE)) ||
            (state == UPDATE_PEER_ADDR))
        {
            DM_LOG("[DM]:[DI %02X]:[CI %02X]: -> Updating bonding information.\r\n",
                   p_handle->device_id, p_handle->connection_id);

            store_fn = pstorage_update;
        }
        else if (state == FIRST_BOND_STORE)
        {
            DM_LOG("[DM]:[DI %02X]:[CI %02X]: -> Storing bonding information.\r\n",
                   p_handle->device_id, p_handle->connection_id);

            store_fn = pstorage_store;
        }
        else
        {
            DM_LOG("[DM]:[DI %02X]:[CI %02X]: -> No update in bonding information.\r\n",
                   p_handle->device_id, p_handle->connection_id);

            // No operation needed.
            store_fn = dummy_handler;
        }

        // Store peer id.
        retval = store_fn(&block_handle,
                          (uint8_t *)&m_peer_table[p_handle->device_id],
                          PEER_ID_SIZE,
                          PEER_ID_STORAGE_OFFSET);

        if ((retval == NRF_SUCCESS) && (state != UPDATE_PEER_ADDR))
        {
            m_connection_table[p_handle->connection_id].state &= (~BOND_INFO_UPDATE);

            // Store bond information.
            retval = store_fn(&block_handle,
                              (uint8_t *)&m_bond_table[p_handle->connection_id],
                              BOND_SIZE,
                              BOND_STORAGE_OFFSET);

            if (retval != NRF_SUCCESS)
            {
                DM_ERR("[DM]:[0x%02X]:Failed to store bond information, reason 0x%08X\r\n",
                       p_handle->device_id, retval);
            }
        }

        if (state != UPDATE_PEER_ADDR)
        {
            // Store service information
            retval = m_service_context_store[m_application_table[p_handle->appl_id].service]
                     (
                         &block_handle,
                         p_handle
                     );
            if (retval != NRF_SUCCESS)
            {
                // Notify application of error event.
                DM_ERR("[DM]: Failed to store service context, reason %08X\r\n", retval);
            }
        }
    }

    if (retval != NRF_SUCCESS)
    {
        // Notify application of error event.
        DM_ERR("[DM]: Failed to store device context, reason %08X\r\n", retval);
    }
}

/**@breif Dummy function, when there is no service registered. */
static __INLINE api_result_t no_service_context_store(pstorage_handle_t const * p_block_handle,
                                                      dm_handle_t const       * p_handle)
{
    DM_LOG("[DM]: --> no_service_context_store\r\n");
    return NRF_SUCCESS;
}

/**@breif GATT Server context store function. */
static __INLINE api_result_t gatts_context_store(pstorage_handle_t const * p_block_handle,
                                                 dm_handle_t const       * p_handle)
{
    storage_operation store_fn;
    uint16_t          attr_len = DM_GATT_SERVER_ATTR_MAX_SIZE;
    uint8_t           sys_data[DM_GATT_SERVER_ATTR_MAX_SIZE];

    DM_LOG("[DM]: --> gatts_context_store\r\n");

    uint32_t retval = sd_ble_gatts_sys_attr_get(
        m_connection_table[p_handle->connection_id].conn_handle,
        sys_data,
        &attr_len);
    if (retval == NRF_SUCCESS)
    {
        if (0 == memcmp(m_gatts_table[p_handle->connection_id].attributes,
                        sys_data, attr_len))
        {
            // No store operation is needed.
            DM_LOG("[DM]:[0x%02X]: No change in GATTS Context information.\r\n",
                   p_handle->device_id);
            if ((m_connection_table[p_handle->connection_id].state & CONNECTED) != CONNECTED)
            {
                DM_LOG("[DM]:[0x%02X]: Resetting GATTS for active instance.\r\n",
                       p_handle->connection_id);
                // Reset GATTS intformation for current context.
                memset(&m_gatts_table[p_handle->connection_id], 0, sizeof(dm_gatts_context_t));
            }
        }
        else
        {
            if (m_gatts_table[p_handle->connection_id].size != 0)
            {
                // There is data already stored in persistent memory, hence update is needed.
                DM_LOG("[DM]:[0x%02X]: Updating stored service context\r\n",p_handle->device_id);
                store_fn = pstorage_update;
            }
            else
            {
                // Fresh write, a store is needed.
                DM_LOG("[DM]:[0x%02X]: Storing service context\r\n",p_handle->device_id);
                store_fn = pstorage_store;
            }
            m_gatts_table[p_handle->connection_id].size = attr_len;
            memcpy(m_gatts_table[p_handle->connection_id].attributes,
                   sys_data,
                   attr_len);
            DM_DUMP((uint8_t *)&m_gatts_table[p_handle->connection_id], sizeof(dm_gatts_context_t));

            DM_LOG("[DM]:[0x%02X]: GATTS Data size 0x%08X\r\n",
                   p_handle->device_id, m_gatts_table[p_handle->connection_id].size);

            // Store GATTS information.
            retval = store_fn((pstorage_handle_t *)p_block_handle,
                              (uint8_t *)&m_gatts_table[p_handle->connection_id],
                              GATTS_SERVICE_CONTEXT_SIZE,
                              SERVICE_STORAGE_OFFSET);
            if (retval != NRF_SUCCESS)
            {
                DM_ERR("[DM]:[0x%02X]:Failed to store service context, reason 0x%08X\r\n",
                       p_handle->device_id, retval);
            }
            else
            {
                DM_LOG("[DM]: Service context successfully stored.\r\n");
            }
        }
    }
    return NRF_SUCCESS;
}


/**@breif Dummy function, when there is no service registered. */
static __INLINE api_result_t gattc_context_store(pstorage_handle_t const * p_block_handle,
                                                 dm_handle_t const       * p_handle)
{
    DM_LOG("[DM]: --> gattc_context_store\r\n");
    return NRF_SUCCESS;
}


/**@breif  GATT Server & Client context store function. */
static __INLINE api_result_t gattsc_context_store(pstorage_handle_t const * p_block_handle,
                                                  dm_handle_t const       * p_handle)
{
    DM_LOG("[DM]: --> gattsc_context_store\r\n");
    api_result_t retval = gatts_context_store(p_block_handle, p_handle);
    if (NRF_SUCCESS == retval)
    {
        retval = gattc_context_store(p_block_handle, p_handle);
    }

    return retval;
}


/**@breif  GATT Server & Client context store function. */
static __INLINE api_result_t no_service_context_load(pstorage_handle_t const * p_block_handle,
                                                     dm_handle_t const       * p_handle)
{
    DM_LOG("[DM]: --> no_service_context_load\r\n");
    return NRF_SUCCESS;
}


/**@breif GATT Server context store function. */
static __INLINE api_result_t gatts_context_load(pstorage_handle_t const * p_block_handle,
                                                dm_handle_t const       * p_handle)
{
    DM_LOG("[DM]:[CI 0x%02X]:[DI 0x%02X]: --> gatts_context_load\r\n",
           p_handle->connection_id,
           p_handle->device_id);
    api_result_t retval = pstorage_load((uint8_t *)&m_gatts_table[p_handle->connection_id],
                                        (pstorage_handle_t *)p_block_handle,
                                        GATTS_SERVICE_CONTEXT_SIZE,
                                        SERVICE_STORAGE_OFFSET);

    if (retval == NRF_SUCCESS)
    {
        DM_LOG("[DM]:[%02X]:[Block ID 0x%08X]: Service context loaded, size 0x%08X\r\n",
               p_handle->connection_id, p_block_handle->block_id,
               m_gatts_table[p_handle->connection_id].size);
        DM_DUMP((uint8_t *)&m_gatts_table[p_handle->connection_id], sizeof(dm_gatts_context_t));
        if (m_gatts_table[p_handle->connection_id].size == DM_GATTS_INVALID_SIZE)
        {
            m_gatts_table[p_handle->connection_id].size = 0;
        }
    }
    else
    {
        DM_ERR("[DM]:[%02X]: Failed to load Service context, reason %08X\r\n",
               p_handle->connection_id, retval);
    }

    return retval;
}


/**@breif  GATT Client context store function. */
static __INLINE api_result_t gattc_context_load(pstorage_handle_t const * p_block_handle,
                                                dm_handle_t const       * p_handle)
{
    DM_LOG("[DM]: --> gattc_context_load\r\n");
    return NRF_SUCCESS;
}


/**@breif  GATT Server & Client context store function. */
static __INLINE api_result_t gattsc_context_load(pstorage_handle_t const * p_block_handle,
                                                 dm_handle_t const       * p_handle)
{
    DM_LOG("[DM]: --> gattsc_context_load\r\n");
    api_result_t retval = gatts_context_load(p_block_handle, p_handle);
    if (NRF_SUCCESS == retval)
    {
        retval = gattc_context_load(p_block_handle, p_handle);
    }

    return retval;
}


/**@breif Dummy function, when there is no service registered. */
static __INLINE api_result_t no_service_context_apply(dm_handle_t * p_handle)
{
    DM_LOG("[DM]: --> no_service_context_apply\r\n");
    DM_LOG("[DM]:[CI 0x%02X]: No Service context\r\n", p_handle->connection_id);
    return NRF_SUCCESS;
}


/**@breif GATT Server context apply function. */
static __INLINE api_result_t gatts_context_apply(dm_handle_t * p_handle)
{
    uint32_t  retval;
    uint8_t * p_gatts_context = NULL;
    uint16_t  context_len     = 0;

    DM_LOG("[DM]: --> gatts_context_apply\r\n");
    DM_LOG("[DM]:[CI 0x%02X]: State 0x%02X, Size 0x%08X\r\n",
           p_handle->connection_id, m_connection_table[p_handle->connection_id].state,
           m_gatts_table[p_handle->connection_id].size);

    if ((m_gatts_table[p_handle->connection_id].size != 0) &&
        ((m_connection_table[p_handle->connection_id].state & LINK_ENCRYPTED) == LINK_ENCRYPTED) &&
        ((m_connection_table[p_handle->connection_id].state & BOND_INFO_UPDATE) !=
         BOND_INFO_UPDATE))
    {
        DM_LOG("[DM]: Setting stored context.\r\n");
        p_gatts_context = &m_gatts_table[p_handle->connection_id].attributes[0];
        context_len     = m_gatts_table[p_handle->connection_id].size;
    }

    retval = sd_ble_gatts_sys_attr_set(m_connection_table[p_handle->connection_id].conn_handle,
                                       p_gatts_context,
                                       context_len);
    if (retval != NRF_SUCCESS)
    {
        DM_LOG("[DM]: Failed to set system attributes, reason 0x%08X.\r\n", retval);
        retval = DM_SERVICE_CONTEXT_NOT_APPLIED;
    }

    return retval;
}


/**@breif GATT Client context apply function. */
static __INLINE api_result_t gattc_context_apply(dm_handle_t * p_handle)
{
    DM_LOG("[DM]: --> gattc_context_apply\r\n");
    return NRF_SUCCESS;
}


/**@breif  GATT Server & Client context apply function. */
static __INLINE api_result_t gattsc_context_apply(dm_handle_t * p_handle)
{
    uint32_t retval;

    DM_LOG("[DM]: --> gattsc_context_apply\r\n");
    retval = gatts_context_apply(p_handle);
    if (retval == NRF_SUCCESS)
    {
        retval = gattc_context_apply(p_handle);
    }

    return retval;
}


/**
 * @brief Routine to request authentication or
 */
uint32_t initiate_security_request(const dm_handle_t * p_handle)
{
    uint32_t retval;

    DM_LOG("[DM]: Initiating authentication request\r\n");
    DM_DUMP((uint8_t *)&m_application_table[p_handle->appl_id].sec_param,
            sizeof (ble_gap_sec_params_t));
    retval = sd_ble_gap_authenticate(m_connection_table[p_handle->connection_id].conn_handle,
                                     &m_application_table[p_handle->appl_id].sec_param);
    if (retval == NRF_SUCCESS)
    {
        m_application_table[p_handle->appl_id].state      |= CONTROL_PROCEDURE_IN_PROGRESS;
        m_connection_table[p_handle->connection_id].state |= PAIRING;
        m_connection_table[p_handle->connection_id].state &= (~PAIRING_PENDING);
    }
    return retval;
}


/**
 * @brief Callback registered with storage module.
 */
static void dm_pstorage_cb_handler(pstorage_handle_t * p_handle,
                                   uint8_t             op_code,
                                   uint32_t            result,
                                   uint8_t           * p_data,
                                   uint32_t            data_len)
{
    VERIFY_APP_REGISTERED_VOID(0);

    if (data_len > ALL_CONTEXT_SIZE)
    {
        // Clearing of all bonds at initialization, no event  is generated.
        return;
    }

    DM_MUTEX_LOCK();

    dm_event_t        dm_event;
    dm_handle_t       dm_handle;
    dm_context_t      context_data;
    pstorage_handle_t block_handle;
    bool              app_notify = true;
    uint32_t          index_count;


    UNUSED_VARIABLE(dm_handle_initialize(&dm_handle));
    dm_handle.appl_id = 0;
    dm_event.event_id = 0x00;

    // Construct the event with which it is related.

    // Initialize context data information and length.
    context_data.p_data = p_data;
    context_data.len    = data_len;


    for (uint32_t index = 0; index < DEVICE_MANAGER_MAX_BONDS; index++)
    {
        uint32_t retval = pstorage_block_identifier_get(&m_storage_handle, index, &block_handle);
        if ((retval == NRF_SUCCESS) &&
            (0 == memcmp(p_handle, &block_handle, sizeof(pstorage_handle_t))))
        {
            dm_handle.device_id = index;
            break;
        }
    }

    if (dm_handle.device_id != DM_INVALID_ID)
    {
        if (op_code == PSTORAGE_CLEAR_OP_CODE)
        {
            if (data_len == ALL_CONTEXT_SIZE)
            {
                dm_event.event_id = DM_EVT_DEVICE_CONTEXT_BASE;
            }
            else
            {
                dm_event.event_id = DM_EVT_APP_CONTEXT_BASE;
            }
        }
        else
        {
            // Update or store operation.
            // Identification of whether it is Device context, application context,
            // of Device, Service or Application Context depends on the range the pointer
            // belongs to.
            index_count = ((uint32_t)(p_data - (uint8_t *)m_peer_table)) / PEER_ID_SIZE;
            if (index_count < DEVICE_MANAGER_MAX_BONDS)
            {
                dm_event.event_param.p_device_context = &context_data;

                // Only peer identification is stored, not bond information. Hence do not notify
                // the application yet, unless, store resulted in a failure.
                if ((result == NRF_SUCCESS) &&
                    (false == update_status_bit_is_set(dm_handle.device_id)))
                {
                    app_notify = false;
                }
                else
                {
                    // Reset update status since update is complete.
                    update_status_bit_reset(dm_handle.device_id);

                    // Notify application of error in storing the context.
                    dm_event.event_id = DM_EVT_DEVICE_CONTEXT_BASE;
                }
            }
            else
            {
                index_count = ((uint32_t)(p_data - (uint8_t *)m_bond_table)) / BOND_SIZE;
                if (index_count < DEVICE_MANAGER_MAX_CONNECTIONS)
                {
                    DM_LOG("[DM]:[0x%02X]:[0x%02X]: Bond context Event\r\n",
                           dm_handle.device_id, dm_handle.connection_id);
                    dm_event.event_param.p_device_context = &context_data;
                    dm_event.event_id                     = DM_EVT_DEVICE_CONTEXT_BASE;
                    dm_handle.connection_id               = index_count;
                    ble_gap_sec_keyset_t keys_exchanged;
                    keys_exchanged.keys_central.p_enc_key = NULL;
                    keys_exchanged.keys_central.p_id_key  = &m_local_id_info;
                    keys_exchanged.keys_periph.p_enc_key  = &m_bond_table[index_count].peer_enc_key;
                    keys_exchanged.keys_periph.p_id_key   =
                        &m_peer_table[dm_handle.device_id].peer_id;

                    // Context information updated to provide the keys.
                    context_data.p_data = (uint8_t *)&keys_exchanged;
                    context_data.len    = sizeof(ble_gap_sec_keyset_t);
                }
                else
                {
                    index_count =
                        ((uint32_t)(p_data -
                                    (uint8_t *)m_gatts_table)) / GATTS_SERVICE_CONTEXT_SIZE;

                    if (index_count < DEVICE_MANAGER_MAX_CONNECTIONS)
                    {
                        DM_LOG("[DM]:[0x%02X]:[0x%02X]: Service context Event\r\n",
                               dm_handle.device_id, dm_handle.connection_id);

                        // Notify application
                        dm_event.event_id       = DM_EVT_SERVICE_CONTEXT_BASE;
                        dm_handle.connection_id = index_count;
                        dm_handle.service_id    = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

                        // Reset the service context now that it has been successfuly
                        // written to the application and the link is disconnected.
                        if ((m_connection_table[index_count].state & CONNECTED) != CONNECTED)
                        {
                            DM_LOG("[DM]:[0x%02X]:[0x%02X]: Resetting bond information for "
                                   "active instance.\r\n",dm_handle.device_id,
                                   dm_handle.connection_id);

                            memset(&m_gatts_table[dm_handle.connection_id], 0,
                                   sizeof (dm_gatts_context_t));
                        }
                    }
                    else
                    {
                        DM_LOG("[DM]:[0x%02X]:[0x%02X]: App context Event\r\n",
                               dm_handle.device_id, dm_handle.connection_id);
                        app_notify        = false;
                        dm_event.event_id = DM_EVT_APP_CONTEXT_BASE;
                        #if(DEVICE_MANAGER_APP_CONTEXT_SIZE != 0)
                        if (p_data == (uint8_t *)(&m_context_init_len))
                        {
                            // Context data is deleted.
                            // This is a hack to get the right event as on delete operation
                            // update operation is used and not clear.
                            op_code    = PSTORAGE_CLEAR_OP_CODE;
                            app_notify = true;
                        }
                        else if (m_app_context_table[dm_handle.device_id] == p_data)
                        {
                            app_notify                         = true;
                            dm_event.event_param.p_app_context = &context_data;
                            // Verify if device is connected, if yes set connection instance.
                            for (uint32_t c_index = 0; c_index < DEVICE_MANAGER_MAX_CONNECTIONS;
                                 c_index++)
                            {
                                if (dm_handle.device_id ==
                                    m_connection_table[c_index].bonded_dev_id)
                                {
                                    dm_handle.connection_id = c_index;
                                    break;
                                }
                            }
                        }
                        #endif // DEVICE_MANAGER_APP_CONTEXT_SIZE
                    }
                }
            }
        }
        if (app_notify == true)
        {
            if (op_code == PSTORAGE_CLEAR_OP_CODE)
            {
                dm_event.event_id |= DM_CLEAR_OPERATION_ID;
            }
            else if (op_code == PSTORAGE_LOAD_OP_CODE)
            {
                dm_event.event_id |= DM_LOAD_OPERATION_ID;
            }
            else
            {
                dm_event.event_id |= DM_STORE_OPERATION_ID;
            }

            dm_event.event_param.p_app_context = &context_data;
            app_evt_notify(&dm_handle, &dm_event, result);
        }
    }

    DM_MUTEX_UNLOCK();
}


/**
 * @brief Module initialization.
 */
api_result_t dm_init(dm_init_param_t const * const p_init_param)
{
    pstorage_module_param_t param;
    pstorage_handle_t       block_handle;
    api_result_t            retval;
    uint32_t                index;

    DM_LOG("[DM]: >> dm_init.\r\n");

    NULL_PARAM_CHECK(p_init_param);

    SDK_MUTEX_INIT(m_dm_mutex);

    DM_MUTEX_LOCK();

    for (index = 0; index < DEVICE_MANAGER_MAX_APPLICATIONS; index++)
    {
        application_instance_init(index);
    }

    for (index = 0; index < DEVICE_MANAGER_MAX_CONNECTIONS; index++)
    {
        connection_instance_init(index);
    }
    memset(m_gatts_table, 0, sizeof(m_gatts_table));

    // Initialization of all device instances.
    for (index = 0; index < DEVICE_MANAGER_MAX_BONDS; index++)
    {
        peer_instance_init(index);
    }

    // All context with respect to a particular device is stored contiguously.
    param.block_size  = ALL_CONTEXT_SIZE;
    param.block_count = DEVICE_MANAGER_MAX_BONDS;
    param.cb          = dm_pstorage_cb_handler;

    retval = pstorage_register(&param, &m_storage_handle);

    if (retval == NRF_SUCCESS)
    {
        m_module_initialized = true;

        if (p_init_param->clear_persistent_data == false)
        {
            DM_LOG("[DM]: Storage handle 0x%08X.\r\n", m_storage_handle.block_id);

            // Copy bonded peer device address and IRK to RAM table.

            // Bonded devices are stored in range (0,DEVICE_MANAGER_MAX_BONDS-1) inclusive.
            // Rest are for active connections that may or may not be bonded.
            for (index = 0; index < DEVICE_MANAGER_MAX_BONDS; index++)
            {
                retval = pstorage_block_identifier_get(&m_storage_handle, index, &block_handle);

                // Issue read request on successfully getting the block identifier.
                if (retval == NRF_SUCCESS)
                {
                    DM_TRC("[DM]:[0x%02X]: Block handle 0x%08X.\r\n", index, block_handle.block_id);

                    retval = pstorage_load((uint8_t *)&m_peer_table[index],
                                           &block_handle,
                                           sizeof (peer_id_t),
                                           0);

                    if (retval != NRF_SUCCESS)
                    {
                        // In case a peer device could not be loaded successfully, rest of the
                        // initialization procedures are abandoned and an error indicated to the
                        // application.
                        DM_ERR(
                            "[DM]: Failed to load peer device %08X from storage, reason %08X.\r\n",
                            index,
                            retval);
                        m_module_initialized = false;
                        break;
                    }
                    else
                    {
                        DM_TRC("[DM]:[DI 0x%02X]: Device type 0x%02X.\r\n",
                               index, m_peer_table[index].peer_id.id_addr_info.addr_type);
                        DM_TRC("[DM]: Device Addr 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X.\r\n",
                               m_peer_table[index].peer_id.id_addr_info.addr[0],
                               m_peer_table[index].peer_id.id_addr_info.addr[1],
                               m_peer_table[index].peer_id.id_addr_info.addr[2],
                               m_peer_table[index].peer_id.id_addr_info.addr[3],
                               m_peer_table[index].peer_id.id_addr_info.addr[4],
                               m_peer_table[index].peer_id.id_addr_info.addr[5]);
                    }

                    DM_DUMP((uint8_t *)&m_peer_table[index].peer_id.id_addr_info, sizeof(m_peer_table[index].peer_id.id_addr_info));
                }
                else
                {
                    // In case a peer device could not be loaded successfully, rest of the
                    // initialization procedures are abandoned and an error indicated to the
                    // application.
                    DM_LOG("[DM]: Failed to get block handle for instance %08X, reason %08X.\r\n",
                           index, retval);
                    m_module_initialized = false;
                    break;
                }
            }
        }
        else
        {
            retval = pstorage_clear(&m_storage_handle, (param.block_size * param.block_count));
            DM_ERR("[DM]: Successfully requested clear of persistent data.\r\n");
        }
    }
    else
    {
        DM_ERR("[DM]: Failed to register with storage module, reason 0x%08X.\r\n", retval);
    }

    DM_MUTEX_UNLOCK();

    DM_TRC("[DM]: << dm_init.\r\n");

    return retval;
}


/**
 * @brief Application registration routine.
 */
api_result_t dm_register
(
    dm_application_instance_t    * p_appl_instance,
    dm_application_param_t const * p_appl_param
)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_appl_instance);
    NULL_PARAM_CHECK(p_appl_param);
    NULL_PARAM_CHECK(p_appl_param->evt_handler);

    DM_MUTEX_LOCK();

    DM_LOG("[DM]: >> dm_register.\r\n");

    uint32_t retval = (NRF_ERROR_NO_MEM | DEVICE_MANAGER_ERR_BASE);

    // Verify if an application instance is available.
    // Currently only one instance is supported.
    if (m_application_table[0].ntf_cb == NULL)
    {
        DM_LOG("[DM]: Application Instance allocated.\r\n");
        // Mark instance as allocated.
        m_application_table[0].ntf_cb    = p_appl_param->evt_handler;
        m_application_table[0].sec_param = p_appl_param->sec_param;
        m_application_table[0].service   = p_appl_param->service_type;

        // Populate application's instance variable with the assigned allication instance.
        *p_appl_instance = 0;
        retval           = NRF_SUCCESS;
    }

    DM_MUTEX_UNLOCK();

    DM_TRC("[DM]: << dm_register.\r\n");

    return retval;
}


/**
 * @breif Request security procedure.
 */
api_result_t dm_security_setup_req(dm_handle_t * p_handle)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_CONNECTION_INSTANCE(p_handle->connection_id);

    DM_MUTEX_LOCK();

    DM_LOG("[DM]: >> dm_security_setup_req\r\n");

    uint32_t retval = (NRF_ERROR_INVALID_STATE | DEVICE_MANAGER_ERR_BASE);

    if ((m_connection_table[p_handle->connection_id].state & CONNECTED) == CONNECTED)
    {
        if ((p_handle->device_id != DM_INVALID_ID) ||
            (m_connection_table[p_handle->connection_id].bonded_dev_id != DM_INVALID_ID))
        {
            p_handle->device_id = m_connection_table[p_handle->connection_id].bonded_dev_id;

            // Already bonded device, encrypt the link.
            retval = sd_ble_gap_encrypt(m_connection_table[p_handle->connection_id].conn_handle,
                                        &m_bond_table[p_handle->connection_id].peer_enc_key.master_id,
                                        &m_bond_table[p_handle->connection_id].peer_enc_key.enc_info);
        }
        else
        {
            uint8_t device_index;

            // Request for pairing, allocate a bonded device instance.
            retval = device_instance_allocate(&device_index,
                                              &m_connection_table[p_handle->connection_id].peer_addr);
            if (retval == NRF_SUCCESS)
            {
                m_connection_table[p_handle->connection_id].bonded_dev_id = device_index;
                p_handle->device_id = device_index;
                if ((m_application_table[p_handle->appl_id].state & CONTROL_PROCEDURE_IN_PROGRESS)
                    == CONTROL_PROCEDURE_IN_PROGRESS)
                {
                    DM_LOG("[DM]: Marked pending\r\n");
                    m_connection_table[p_handle->connection_id].state |= PAIRING_PENDING;
                    m_application_table[p_handle->appl_id].state      |= QUEUED_CONTROL_REQUEST;
                    retval = NRF_SUCCESS;
                }
                else
                {
                    retval = initiate_security_request(p_handle);
                }
            }
        }
    }

    DM_TRC("[DM]: << dm_security_setup_req, 0x%08X\r\n", retval);

    DM_MUTEX_UNLOCK();

    return retval;
}


/**
 * @breif Security status request procedure.
 */
api_result_t dm_security_status_req(dm_handle_t const    * p_handle,
                                    dm_security_status_t * p_status)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_status);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_CONNECTION_INSTANCE(p_handle->connection_id);

    DM_MUTEX_LOCK();

    DM_LOG("[DM]: >> dm_security_status_req\r\n");

    if ((m_connection_table[p_handle->connection_id].state & PAIRING) ||
        (m_connection_table[p_handle->connection_id].state & PAIRING_PENDING))
    {
        (*p_status) = ENCRYPTION_IN_PROGRESS;
    }
    else if (m_connection_table[p_handle->connection_id].state & LINK_ENCRYPTED)
    {
        (*p_status) = ENCRYPTED;
    }
    else
    {
        (*p_status) = NOT_ENCRYPTED;
    }

    DM_TRC("[DM]: << dm_security_status_req\r\n");

    DM_MUTEX_UNLOCK();

    return NRF_SUCCESS;
}


/**
 * @breif Security status request procedure.
 */
api_result_t dm_whitelist_create(dm_application_instance_t const * p_handle,
                                 ble_gap_whitelist_t             * p_whitelist)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_whitelist);
    NULL_PARAM_CHECK(p_whitelist->pp_addrs);
    NULL_PARAM_CHECK(p_whitelist->pp_irks);
    VERIFY_APP_REGISTERED(*p_handle);

    DM_MUTEX_LOCK();

    DM_LOG("[DM]: >> dm_whitelist_create\r\n");

    uint32_t addr_count = 0;
    uint32_t irk_count  = 0;
    bool     connected  = false;

    for (uint32_t index = 0; index < DEVICE_MANAGER_MAX_BONDS; index++)
    {
        connected = false;

        for (uint32_t c_index = 0; c_index < DEVICE_MANAGER_MAX_CONNECTIONS; c_index++)
        {
            if ((index == m_connection_table[c_index].bonded_dev_id) &&
                ((m_connection_table[c_index].state & CONNECTED) == CONNECTED))
            {
                connected = true;
                break;
            }
        }

        if (connected == false)
        {
            if ((irk_count < p_whitelist->irk_count) &&
                ((m_peer_table[index].id_bitmap & IRK_ENTRY) == 0))
            {
                p_whitelist->pp_irks[irk_count] = &m_peer_table[index].peer_id.id_info;
                irk_count++;
            }
            if ((addr_count < p_whitelist->addr_count) &&
                (m_peer_table[index].id_bitmap & ADDR_ENTRY) == 0)
            {
                p_whitelist->pp_addrs[addr_count] = &m_peer_table[index].peer_id.id_addr_info;
                addr_count++;
            }
        }
    }

    p_whitelist->addr_count = addr_count;
    p_whitelist->irk_count  = irk_count;

    DM_LOG("[DM]: Created whitelist, number of IRK = 0x%02X, number of addr = 0x%02X\r\n",
           irk_count, addr_count);

    DM_TRC("[DM]: << dm_whitelist_create\r\n");

    DM_MUTEX_UNLOCK();

    return NRF_SUCCESS;
}


/**
 * @brief Delete peer device context and all related information from database.
 */
api_result_t dm_device_add(dm_handle_t               * p_handle,
                           dm_device_context_t const * p_context)
{
    return (API_NOT_IMPLEMENTED | DEVICE_MANAGER_ERR_BASE);
}


/**
 * @brief Delete peer device context and all related information from database.
 */
api_result_t dm_device_delete(dm_handle_t const * p_handle)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_device_delete\r\n");

    uint32_t retval = device_instance_free(p_handle->device_id);

    DM_TRC("[DM]: << dm_device_delete\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
}


/**
 * @brief Delete peer device context and all related information from database.
 */
api_result_t dm_device_delete_all(dm_application_instance_t const * p_handle)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    VERIFY_APP_REGISTERED((*p_handle));

    DM_MUTEX_LOCK();

    uint32_t retval = NRF_SUCCESS;

    DM_TRC("[DM]: >> dm_device_delete_all\r\n");

    for (uint32_t index = 0; index < DEVICE_MANAGER_MAX_BONDS; index++)
    {
        if (m_peer_table[index].id_bitmap != UNASSIGNED)
        {
            retval = device_instance_free(index);
        }
    }

    DM_TRC("[DM]: << dm_device_delete_all\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
}


/**
 * @brief Set Service Context for a peer device identified by 'p_handle' parameter.
 */
api_result_t dm_service_context_set(dm_handle_t const          * p_handle,
                                    dm_service_context_t const * p_context)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_context);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_CONNECTION_INSTANCE(p_handle->connection_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_service_context_set\r\n");

    if ((p_context->context_data.p_data != NULL) && (p_context->context_data.len != 0) &&
        (p_context->context_data.len < DM_GATT_SERVER_ATTR_MAX_SIZE))
    {
        if (p_context->service_type == DM_PROTOCOL_CNTXT_GATT_SRVR_ID)
        {
            memcpy(m_gatts_table[p_handle->connection_id].attributes,
                   p_context->context_data.p_data,
                   p_context->context_data.len);
        }
    }

    pstorage_handle_t block_handle;
    uint32_t          retval = pstorage_block_identifier_get(&m_storage_handle,
                                                             p_handle->device_id,
                                                             &block_handle);

    retval = m_service_context_store[p_context->service_type](&block_handle, p_handle);

    DM_TRC("[DM]: << dm_service_context_set\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
}


/**
 * @brief Get Service Context for a peer device identified by 'p_handle' parameter.
 */
api_result_t dm_service_context_get(dm_handle_t const    * p_handle,
                                    dm_service_context_t * p_context)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_context);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_service_context_get\r\n");
    if ((p_context->context_data.p_data == NULL) &&
        (p_context->context_data.len < DM_GATT_SERVER_ATTR_MAX_SIZE))
    {
        if (p_context->service_type == DM_PROTOCOL_CNTXT_GATT_SRVR_ID)
        {
            p_context->context_data.p_data = m_gatts_table[p_handle->connection_id].attributes;
            p_context->context_data.len    = m_gatts_table[p_handle->connection_id].size;
        }
    }

    pstorage_handle_t block_handle;
    uint32_t          retval = pstorage_block_identifier_get(&m_storage_handle,
                                                             p_handle->device_id,
                                                             &block_handle);

    retval = m_service_context_load[p_context->service_type](&block_handle, p_handle);

    DM_TRC("[DM]: << dm_service_context_get\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
}


/**
 * @brief Delete Service Context for a peer device identified by 'p_handle' parameter.
 */
api_result_t dm_service_context_delete(dm_handle_t const * p_handle)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    return NRF_SUCCESS;
}


/**
 * @brief Set Application Context for a peer device identified by 'p_handle' parameter.
 */
api_result_t dm_application_context_set(dm_handle_t const              * p_handle,
                                        dm_application_context_t const * p_context)
{
#if(DEVICE_MANAGER_APP_CONTEXT_SIZE != 0)
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_context);
    NULL_PARAM_CHECK(p_context->p_data);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_application_context_set\r\n");

    uint32_t          retval = DM_NO_APP_CONTEXT;
    uint32_t          context_len;
    pstorage_handle_t block_handle;
    storage_operation store_fn = pstorage_store;

    retval = pstorage_block_identifier_get(&m_storage_handle,
                                           p_handle->device_id,
                                           &block_handle);
    if (retval == NRF_SUCCESS)
    {

        retval = pstorage_load((uint8_t *)&context_len,
                               &block_handle,
                               sizeof(uint32_t),
                               APP_CONTEXT_STORAGE_OFFSET);

        if ((retval == NRF_SUCCESS) && (context_len != 0xFFFFFFFF))
        {
            // Data already exists. Need an update.
            store_fn = pstorage_update;
            DM_LOG("[DM]:[DI 0x%02X]: Updating existing application context, existing len 0x%08X, "
                   "new length 0x%08X.\r\n",p_handle->device_id, context_len, p_context->len);
        }
        else
        {
            DM_LOG("[DM]: Storing application context.\r\n");
        }

        // Store/update context length.
        retval = store_fn(&block_handle,
                          (uint8_t *)(&p_context->len),
                          sizeof(uint32_t),
                          APP_CONTEXT_STORAGE_OFFSET);
        if (retval == NRF_SUCCESS)
        {
            // Update context data is used for application context as flash is never
            // cleared if a delete of application context is called.
            retval = pstorage_update(&block_handle,
                                     p_context->p_data,
                                     DEVICE_MANAGER_APP_CONTEXT_SIZE,
                                     (APP_CONTEXT_STORAGE_OFFSET + sizeof(uint32_t)));
            if (retval == NRF_SUCCESS)
            {
                m_app_context_table[p_handle->device_id] = p_context->p_data;
            }
        }
    }

    DM_TRC("[DM]: << dm_application_context_set\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
#else // DEVICE_MANAGER_APP_CONTEXT_SIZE
    return (FEATURE_NOT_ENABLED | DEVICE_MANAGER_ERR_BASE);
#endif // DEVICE_MANAGER_APP_CONTEXT_SIZE
}


/**
 * @brief Get Application Context for a peer device identified by 'p_handle' parameter.
 */
api_result_t dm_application_context_get(dm_handle_t const        * p_handle,
                                        dm_application_context_t * p_context)
{
#if(DEVICE_MANAGER_APP_CONTEXT_SIZE != 0)
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_context);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_application_context_get\r\n");

    uint32_t          retval = DM_NO_APP_CONTEXT;
    uint32_t          context_len;
    pstorage_handle_t block_handle;

    // Check if the context exists.
    if (NULL == p_context->p_data)
    {
        p_context->p_data = m_app_context_table[p_handle->device_id];
    }
    else
    {
        m_app_context_table[p_handle->device_id] = p_context->p_data;
    }


    retval = pstorage_block_identifier_get(&m_storage_handle,
                                           p_handle->device_id,
                                           &block_handle);
    if (retval == NRF_SUCCESS)
    {
        retval = pstorage_load((uint8_t *)&context_len,
                               &block_handle,
                               sizeof(uint32_t),
                               APP_CONTEXT_STORAGE_OFFSET);

        if ((retval == NRF_SUCCESS) && (context_len != 0xFFFFFFFF))
        {
            retval = pstorage_load(p_context->p_data,
                                   &block_handle,
                                   DEVICE_MANAGER_APP_CONTEXT_SIZE,
                                   (APP_CONTEXT_STORAGE_OFFSET + sizeof(uint32_t)));
            if (retval == NRF_SUCCESS)
            {
                p_context->len = context_len;
            }
        }
        else
        {
            retval = DM_NO_APP_CONTEXT;
        }
    }

    DM_TRC("[DM]: << dm_application_context_get\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
#else // DEVICE_MANAGER_APP_CONTEXT_SIZE
    return (FEATURE_NOT_ENABLED | DEVICE_MANAGER_ERR_BASE);
#endif // DEVICE_MANAGER_APP_CONTEXT_SIZE
}


/**
 * @brief Delete Application Context for a peer device identified by 'p_handle' parameter.
 */
api_result_t dm_application_context_delete(const dm_handle_t * p_handle)
{
#if(DEVICE_MANAGER_APP_CONTEXT_SIZE != 0)
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_application_context_delete\r\n");

    uint32_t          retval = DM_NO_APP_CONTEXT;
    uint32_t          context_len;
    pstorage_handle_t block_handle;

    retval = pstorage_block_identifier_get(&m_storage_handle,
                                           p_handle->device_id,
                                           &block_handle);
    if (retval == NRF_SUCCESS)
    {
        retval = pstorage_load((uint8_t *)&context_len,
                               &block_handle,
                               sizeof(uint32_t),
                               APP_CONTEXT_STORAGE_OFFSET);
        if (context_len != m_context_init_len)
        {
            retval = pstorage_update(&block_handle,
                                     (uint8_t *)&m_context_init_len,
                                     sizeof(uint32_t),
                                     APP_CONTEXT_STORAGE_OFFSET);
            if (retval != NRF_SUCCESS)
            {
                DM_ERR("[DM]: Failed to delete application context, reason 0x%08X\r\n", retval);
            }
            else
            {
                m_app_context_table[p_handle->device_id] = NULL;
            }
        }
    }

    DM_TRC("[DM]: << dm_application_context_delete\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
#else // DEVICE_MANAGER_APP_CONTEXT_SIZE
    return (FEATURE_NOT_ENABLED | DEVICE_MANAGER_ERR_BASE);
#endif // DEVICE_MANAGER_APP_CONTEXT_SIZE
}


api_result_t dm_application_instance_set(dm_application_instance_t const * p_appl_instance,
                                         dm_handle_t                     * p_handle)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_appl_instance);
    VERIFY_APP_REGISTERED((*p_appl_instance));

    p_handle->appl_id = (*p_appl_instance);

    return NRF_SUCCESS;
}


/** @brief Initialize device handle. */
uint32_t dm_handle_initialize(dm_handle_t * p_handle)
{
    NULL_PARAM_CHECK(p_handle);

    p_handle->appl_id       = DM_INVALID_ID;
    p_handle->connection_id = DM_INVALID_ID;
    p_handle->device_id     = DM_INVALID_ID;
    p_handle->service_id    = DM_INVALID_ID;

    return NRF_SUCCESS;
}


/**
 * @brief Set device address.
 */
api_result_t dm_peer_addr_set(dm_handle_t const    * p_handle,
                              ble_gap_addr_t const * p_addr)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_addr);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_peer_addr_set\r\n");

    api_result_t retval;

    if ((p_handle->connection_id == DM_INVALID_ID) &&
        (p_addr->addr_type != BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE))
    {
        m_peer_table[p_handle->device_id].peer_id.id_addr_info = (*p_addr);
        update_status_bit_set(p_handle->device_id);
        device_context_store(p_handle, UPDATE_PEER_ADDR);
        retval = NRF_SUCCESS;
    }
    else
    {
        retval = (NRF_ERROR_INVALID_PARAM | DEVICE_MANAGER_ERR_BASE);
    }

    DM_TRC("[DM]: << dm_peer_addr_set\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
}


/**
 * @brief Get device address.
 */
api_result_t dm_peer_addr_get(dm_handle_t const * p_handle,
                              ble_gap_addr_t    * p_addr)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_addr);
    VERIFY_APP_REGISTERED(p_handle->appl_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_peer_addr_get\r\n");

    api_result_t retval;

    retval = (NRF_ERROR_NOT_FOUND | DEVICE_MANAGER_ERR_BASE);

    if (p_handle->device_id == DM_INVALID_ID)
    {
        if ((p_handle->connection_id != DM_INVALID_ID) &&
            ((m_connection_table[p_handle->connection_id].state & CONNECTED) == CONNECTED))
        {
            DM_TRC("[DM]:[CI 0x%02X]: Address get for non bonded active connection.\r\n",
                   p_handle->connection_id);
            (*p_addr) = m_connection_table[p_handle->connection_id].peer_addr;
            retval    = NRF_SUCCESS;
        }
    }
    else
    {
        if ((m_peer_table[p_handle->device_id].id_bitmap & ADDR_ENTRY) == 0)
        {
            DM_TRC("[DM]:[DI 0x%02X]: Address get for bonded device.\r\n",
                   p_handle->device_id);
            (*p_addr) = m_peer_table[p_handle->device_id].peer_id.id_addr_info;
            retval    = NRF_SUCCESS;
        }
    }

    DM_TRC("[DM]: << dm_peer_addr_get\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
}


api_result_t dm_distributed_keys_get(dm_handle_t const    * p_handle,
                                     ble_gap_sec_keyset_t * p_key_dist)
{
    VERIFY_MODULE_INITIALIZED();
    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_key_dist);
    VERIFY_APP_REGISTERED(p_handle->appl_id);
    VERIFY_DEVICE_INSTANCE(p_handle->device_id);

    DM_MUTEX_LOCK();

    DM_TRC("[DM]: >> dm_distributed_keys_get\r\n");

    api_result_t      retval;
    ble_gap_enc_key_t peer_enc_key;
    pstorage_handle_t block_handle;

    retval                              = NRF_ERROR_NOT_FOUND;
    p_key_dist->keys_central.p_enc_key  = NULL;
    p_key_dist->keys_central.p_id_key   = &m_local_id_info;
    p_key_dist->keys_central.p_sign_key = NULL;
    p_key_dist->keys_periph.p_id_key    = &m_peer_table[p_handle->device_id].peer_id;
    p_key_dist->keys_periph.p_sign_key  = NULL;

    retval = pstorage_block_identifier_get(&m_storage_handle, p_handle->device_id, &block_handle);

    if (retval == NRF_SUCCESS)
    {

        retval = pstorage_load((uint8_t *)&peer_enc_key,
                               &block_handle,
                               BOND_SIZE,
                               BOND_STORAGE_OFFSET);

        if (retval == NRF_SUCCESS)
        {
            p_key_dist->keys_central.p_enc_key  = NULL;
            p_key_dist->keys_central.p_id_key   = &m_local_id_info;
            p_key_dist->keys_central.p_sign_key = NULL;
            p_key_dist->keys_periph.p_id_key    = &m_peer_table[p_handle->device_id].peer_id;
            p_key_dist->keys_periph.p_sign_key  = NULL;
            p_key_dist->keys_periph.p_enc_key   = &peer_enc_key;

        }
    }

    DM_TRC("[DM]: << dm_distributed_keys_get\r\n");

    DM_MUTEX_UNLOCK();

    return retval;
}


/**@breif BLE event handler */
void dm_ble_evt_handler(ble_evt_t * p_ble_evt)
{
    uint32_t    retval;
    uint32_t    index;
    uint32_t    device_index;
    bool        notify_app          = false;
    bool        start_sec_procedure = false;
    dm_handle_t handle;
    dm_event_t  event;
    uint32_t    event_result;

    VERIFY_MODULE_INITIALIZED_VOID();
    VERIFY_APP_REGISTERED_VOID(0);
    DM_MUTEX_LOCK();

    UNUSED_VARIABLE(dm_handle_initialize(&handle));

    event_result                  = NRF_SUCCESS;
    retval                        = NRF_SUCCESS;
    event.event_param.p_gap_param = &p_ble_evt->evt.gap_evt;
    event.event_paramlen          = sizeof (ble_gap_evt_t);
    handle.device_id              = DM_INVALID_ID;
    handle.appl_id                = 0;
    index                         = 0x00;

    if (p_ble_evt->header.evt_id != BLE_GAP_EVT_CONNECTED)
    {
        retval = connection_instance_find(p_ble_evt->evt.gap_evt.conn_handle, CONNECTED, &index);
        if (retval == NRF_SUCCESS)
        {
            handle.device_id     = m_connection_table[index].bonded_dev_id;
            handle.connection_id = index;
        }
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            // Allocate connection instance for new connection.
            retval = connection_instance_alloc(&index);

            // Connection instance successfully allocated.
            if (retval == NRF_SUCCESS)
            {
                // Application notification related information.
                notify_app           = true;
                event.event_id       = DM_EVT_CONNECTION;
                handle.connection_id = index;

                m_connection_table[index].conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                m_connection_table[index].state       = CONNECTED;
                m_connection_table[index].peer_addr   =
                    p_ble_evt->evt.gap_evt.params.connected.peer_addr;

                // Find if device exists in bonded device list by address.
                retval = device_instance_find(&p_ble_evt->evt.gap_evt.params.connected.peer_addr,
                                              &device_index);

                if (NRF_SUCCESS == retval)
                {
                    pstorage_handle_t block_handle;

                    m_connection_table[index].bonded_dev_id = device_index;
                    m_connection_table[index].state        |= BONDED;
                    handle.device_id                        = device_index;
                    retval = pstorage_block_identifier_get(&m_storage_handle,
                                                           device_index,
                                                           &block_handle);

                    if (retval == NRF_SUCCESS)
                    {
                        DM_LOG(
                            "[DM]:[%02X]:[Block ID 0x%08X]:Loading bond information at %p, size 0x%08X, offset 0x%08X.\r\n",
                            index,
                            block_handle.block_id,
                            &m_bond_table[index],
                            BOND_SIZE,
                            BOND_STORAGE_OFFSET);

                        retval = pstorage_load((uint8_t *)&m_bond_table[index],
                                               &block_handle,
                                               BOND_SIZE,
                                               BOND_STORAGE_OFFSET);

                        if (retval != NRF_SUCCESS)
                        {
                            DM_ERR("[DM]:[%02X]: Failed to load Bond information, reason %08X\r\n",
                                   index, retval);
                        }

                        DM_LOG("[DM]:[%02X]:Loading service context at %p, size 0x%08X, offset 0x%08X.\r\n",
                               index,
                               &m_gatts_table[index],
                               sizeof(dm_gatts_context_t),
                               SERVICE_STORAGE_OFFSET);
                        retval = m_service_context_load[m_application_table[0].service](&block_handle,
                                                                                        &handle);
                        if (retval != NRF_SUCCESS)
                        {
                            DM_ERR(
                                "[DM]:[%02X]: Failed to load service information, reason %08X\r\n",
                                index,
                                retval);
                        }
                    }
                    else
                    {
                        DM_ERR("[DM]:[%02X]: Failed to get block identifier for "
                               "device %08X, reason %08X.\r\n", index, device_index, retval);
                    }
                }
            }
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            // Disconnection could be peer or self initiated hence disconnecting and connecting
            // both states are permitted, however, connection handle should be known.
            DM_LOG("[DM]: Disconnect Reason 0x%04X\r\n",
                   p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_connection_table[index].state &= (~CONNECTED);
            if ((m_connection_table[index].state & BONDED) == BONDED)
            {
                // Write bond information persistently.
                device_context_store(&handle, STORE_ALL_CONTEXT);
            }
            else
            {
                // Free any allocated instance for device as its not bonded.
                if (handle.device_id != DM_INVALID_ID)
                {
                    peer_instance_init(handle.device_id);
                    handle.device_id = DM_INVALID_ID;
                }
            }
            if ((m_connection_table[index].state & PAIRING) == PAIRING)
            {
                start_sec_procedure           = true;
                m_application_table[0].state &= (~CONTROL_PROCEDURE_IN_PROGRESS);
            }
            m_connection_table[index].state = DISCONNECTING;

            notify_app = true;
            // handle.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            event.event_id = DM_EVT_DISCONNECTION;

            break;
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            DM_LOG("[DM]: >> BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
            notify_app = false;
            ble_gap_sec_keyset_t keys_exchanged;

            DM_LOG("[DM]: 0x%02X, 0x%02X, 0x%02X, 0x%02X\r\n",
                   p_ble_evt->evt.gap_evt.params.sec_params_request.peer_params.kdist_periph.enc,
                   p_ble_evt->evt.gap_evt.params.sec_params_request.peer_params.kdist_periph.id,
                   p_ble_evt->evt.gap_evt.params.sec_params_request.peer_params.kdist_periph.sign,
                   p_ble_evt->evt.gap_evt.params.sec_params_request.peer_params.bond);

            keys_exchanged.keys_central.p_enc_key  = NULL;
            keys_exchanged.keys_central.p_id_key   = &m_local_id_info;
            keys_exchanged.keys_central.p_sign_key = NULL;
            keys_exchanged.keys_periph.p_enc_key   = &m_bond_table[index].peer_enc_key;
            keys_exchanged.keys_periph.p_id_key    =
                &m_peer_table[m_connection_table[index].bonded_dev_id].peer_id;
            keys_exchanged.keys_periph.p_sign_key  = NULL;

            retval = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                                 BLE_GAP_SEC_STATUS_SUCCESS,
                                                 NULL,
                                                 &keys_exchanged);
            if (retval != NRF_SUCCESS)
            {
                DM_LOG("[DM]: Security parameter reply request failed, reason 0x%08X.\r\n", retval);
                event_result = retval;
            }
            else
            {
                m_connection_table[index].state |= PAIRING;
            }
            break;
        case BLE_GAP_EVT_AUTH_STATUS:
        {
            DM_LOG("[DM]: >> BLE_GAP_EVT_AUTH_STATUS, status %08X\r\n",
                   p_ble_evt->evt.gap_evt.params.auth_status.auth_status);

            m_application_table[0].state    &= (~CONTROL_PROCEDURE_IN_PROGRESS);
            m_connection_table[index].state &= (~PAIRING);
            event.event_id                   = DM_EVT_SECURITY_SETUP_COMPLETE;
            notify_app                       = true;
            start_sec_procedure              = true;

            if (p_ble_evt->evt.gap_evt.params.auth_status.auth_status != BLE_GAP_SEC_STATUS_SUCCESS)
            {
                event_result = p_ble_evt->evt.gap_evt.params.auth_status.auth_status;
            }
            else
            {
                DM_DUMP((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status,
                        sizeof(ble_gap_evt_auth_status_t));
                DM_DUMP((uint8_t *)&m_bond_table[index], sizeof (bond_context_t));

                if (p_ble_evt->evt.gap_evt.params.auth_status.bonded == 1)
                {
                    if (handle.device_id != DM_INVALID_ID)
                    {
                        m_connection_table[index].state |= BONDED;

                        // IRK and/or public address is shared, update it.
                        if (p_ble_evt->evt.gap_evt.params.auth_status.kdist_periph.id == 1)
                        {
                            m_peer_table[index].id_bitmap &= (~IRK_ENTRY);
                        }

                        if (m_connection_table[index].bonded_dev_id != DM_INVALID_ID)
                        {
                            DM_LOG("[DM]:[CI 0x%02X]:[DI 0x%02X]: Bonded!\r\n",
                                   index,
                                   handle.device_id);
                            if (m_connection_table[index].peer_addr.addr_type !=
                                BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE)
                            {
                               m_peer_table[handle.device_id].peer_id.id_addr_info =
                                m_connection_table[index].peer_addr;
                               m_peer_table[handle.device_id].id_bitmap &= (~ADDR_ENTRY);

                               DM_DUMP((uint8_t *)&m_peer_table[handle.device_id].peer_id.id_addr_info,
                                       sizeof(m_peer_table[handle.device_id].peer_id.id_addr_info));
                            }

                            device_context_store(&handle, FIRST_BOND_STORE);
                        }
                    }
                }
                else
                {
                    // Pairing request, no need to touch bonding info.
                }
            }
            break;
        }
        case BLE_GAP_EVT_CONN_SEC_UPDATE:
            DM_LOG("[DM]: >> BLE_GAP_EVT_CONN_SEC_UPDATE, Mode 0x%02X, Level 0x%02X\r\n",
                   p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.sm,
                   p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv);
            if ((p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv == 1) &&
                (p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.sm == 1) &&
                ((m_connection_table[index].state & BONDED) == BONDED))
            {
                // Lost bond case, generate a security refresh event!
                memset(m_gatts_table[index].attributes, 0, DM_GATT_SERVER_ATTR_MAX_SIZE);
                event.event_id                   = DM_EVT_SECURITY_SETUP_REFRESH;
                start_sec_procedure              = true;
                m_connection_table[index].state |= PAIRING_PENDING;
                m_connection_table[index].state |= BOND_INFO_UPDATE;
                m_application_table[0].state    |= QUEUED_CONTROL_REQUEST;
            }
            else
            {
                m_connection_table[index].state |= LINK_ENCRYPTED;
                event.event_id                   = DM_EVT_LINK_SECURED;

                // Apply service context.
                retval = m_service_context_apply[m_application_table[0].service](&handle);
                if (retval != NRF_SUCCESS)
                {
                    DM_ERR("[DM]:[CI 0x%02X]:[DI 0x%02X]: Failed to apply service context\r\n",
                           handle.connection_id, handle.device_id);

                    event_result = DM_SERVICE_CONTEXT_NOT_APPLIED;
                }
            }
            event_result = NRF_SUCCESS;
            notify_app   = true;
            break;
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            DM_LOG("[DM]: >> BLE_GATTS_EVT_SYS_ATTR_MISSING\r\n");

            // Apply service context.
            event_result = m_service_context_apply[m_application_table[0].service](&handle);
            break;
        case BLE_GAP_EVT_SEC_REQUEST:
            DM_LOG("[DM]: >> BLE_GAP_EVT_SEC_REQUEST\r\n");
            // Verify if the device is already bonded or not, if bonded, initiate encryption.
            // Else an instance needs to be allocated inorder to initiate bonding. Application
            // to initiate the procedure, module will not do this automatically.
            event.event_id = DM_EVT_SECURITY_SETUP;
            notify_app     = true;
            break;
        default:
            break;
    }

    if (retval != NRF_SUCCESS)
    {
        // TODO: Notify error event to application?
    }

    if (notify_app)
    {
        app_evt_notify(&handle, &event, event_result);
        // Freeing the instance after event is notified so that application can get the context.
        if (event.event_id == DM_EVT_DISCONNECTION)
        {
            // Free instance.
            connection_instance_free(&index);
        }
    }
    if (start_sec_procedure)
    {
        if ((m_application_table[0].state & QUEUED_CONTROL_REQUEST) == QUEUED_CONTROL_REQUEST)
        {
            dm_handle_t pending_handle;
            uint32_t    pending_index;

            DM_LOG("[DM]:Queued request\r\n");
            retval = connection_instance_find(BLE_CONN_HANDLE_INVALID,
                                              PAIRING_PENDING,
                                              &pending_index);
            if (retval == NRF_SUCCESS)
            {
                pending_handle.appl_id       = 0;
                pending_handle.connection_id = pending_index;
                pending_handle.device_id     = m_connection_table[pending_index].bonded_dev_id;
                retval                       = initiate_security_request(&pending_handle);
                DM_LOG("[DM]:[CI 0x%02X]:[DI 0x%02X]: Security request result 0x%08X\r\n",
                       pending_handle.connection_id, pending_handle.device_id, retval);
            }
            else
            {
                DM_LOG("[DM]:Nothing pending!\r\n");
                m_application_table[0].state &= (~QUEUED_CONTROL_REQUEST);
            }
        }
    }

    UNUSED_VARIABLE(retval);

    DM_MUTEX_UNLOCK();
}
