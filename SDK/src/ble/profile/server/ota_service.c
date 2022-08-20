/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      ota_service.h
   * @brief     Source file for using OTA service
   * @author    calvin
   * @date      2017-06-07
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/

#include <string.h>
#include <gatt.h>
#include <bt_types.h>
#include "trace.h"
#include "gap_le.h"
#include "gap_conn_le.h"
#include "gap_msg.h"
#include "gap_adv.h"
#include "app_msg.h"
#include "flash_device.h"

#include "dfu_api.h"
#include "ota_service.h"

/** @addtogroup  SAMPLE_APP Sample App
    * @brief
    * @{
    */

/** @addtogroup  OTA_DEMO OTA Demo App
    * @brief Demo App to demonstrate how to use OTA
    * @{
    */

/** @defgroup  OTA_SERVICE OTA Service
    * @brief LE Service to implement OTA feature
    * @{
    */

/*============================================================================*
 *                              Macros
 *============================================================================*/
/** @defgroup OTA_SERVICE_Exported_Macros OTA service Exported Macros
    * @brief
    * @{
    */

/* Indicate whether support ota image transfer during normal working mode */
#define OTA_NORMAL_MODE 1

#define OTA_RETRY_LIMIT 3

/* Send notification to peer side */
#define ota_service_send_notification(conn_id, p_data, data_len)    \
    server_send_data(conn_id, srv_id_local, BLE_SERVICE_CHAR_DFU_CONTROL_POINT_INDEX, p_data, data_len, GATT_PDU_TYPE_NOTIFICATION)

/** End of OTA_SERVICE_Exported_Macros
    * @}
    */

/*============================================================================*
 *                              Constants
 *============================================================================*/
/** @defgroup OTA_SERVICE_Exported_Constants OTA service Exported Constants
    * @{
    */

/** @brief  OTA service UUID */
static const uint8_t GATT_UUID_OTA_SERVICE[16] = { 0x12, 0xA2, 0x4D, 0x2E, 0xFE, 0x14, 0x48, 0x8e, 0x93, 0xD2, 0x17, 0x3C, 0xFF, 0xD0, 0x00, 0x00};

/** @brief  OTA BLE Service Callbacks */
static const T_FUN_GATT_SERVICE_CBS ota_service_cbs =
{
    ota_service_attr_read_cb,   /**< Read callback function pointer */
    ota_service_attr_write_cb,  /**< Write callback function pointer */
    NULL                        /**< CCCD update callback function pointer */
};

/** @brief  OTA profile/service definition
*   @note   Here is an example of OTA service table including Write
*/
static const T_ATTRIB_APPL gatt_extended_service_table[] =
{
    /*--------------------------OTA Service ---------------------------*/
    /* <<Primary Service>>, .. 0 */
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),        /* flags */
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),     /* type_value */
        },
        UUID_128BIT_SIZE,                           /* bValueLen */
        (void *)GATT_UUID_OTA_SERVICE,              /* p_value_context */
        GATT_PERM_READ                              /* permissions */
    },

    /* <<Characteristic1>>, .. 1 */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP,            /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,     /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*  OTA characteristic value 2 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHAR_OTA),
            HI_WORD(GATT_UUID_CHAR_OTA),
        },
        2,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ | GATT_PERM_WRITE            /* permissions */
    },

    /* <<Characteristic2>>, .. 3, MAC Address */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ,                    /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,     /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*  OTA characteristic value 4 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHAR_MAC),
            HI_WORD(GATT_UUID_CHAR_MAC),
        },
        1,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ                              /* permissions */
    },

    /* <<Characteristic3>>, .. 5, Patch version */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ,                    /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,     /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*  OTA characteristic value 6 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHAR_PATCH),
            HI_WORD(GATT_UUID_CHAR_PATCH),
        },
        1,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ                              /* permissions */
    },

    /* <<Characteristic4>>, .. 7 App version */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ,                    /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,     /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*  OTA characteristic value 8 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHAR_APP_VERSION),
            HI_WORD(GATT_UUID_CHAR_APP_VERSION),
        },
        1,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ                              /* permissions */
    },

    /* <<Characteristic5>>, .. 9, DSP version */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ,                    /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },

    /*  OTA characteristic value 10 */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHAR_DSP_VERSION),
            HI_WORD(GATT_UUID_CHAR_DSP_VERSION),
        },
        1,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ                              /* permissions */
    },

#ifdef OTA_NORMAL_MODE
    /*-------------------------- DFU Service ---------------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),                /* flags */
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),             /* type_value */
        },
        UUID_128BIT_SIZE,                                   /* bValueLen */
        (void *)GATT_UUID128_DFU_SERVICE,                   /* p_value_context */
        GATT_PERM_READ                                      /* permissions  */
    },



    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                             /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP,                    /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                                  /* bValueLen */
        NULL,
        GATT_PERM_READ                                      /* permissions */
    },
    /*--- DFU packet characteristic value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,   /* flags */
        {   /* type_value */
            GATT_UUID128_DFU_PACKET
        },
        0,                                                  /* bValueLen */
        NULL,
        GATT_PERM_WRITE                                     /* permissions */
    },
    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                             /* flags */
        {   /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_WRITE |                         /* characteristic properties */
             GATT_CHAR_PROP_NOTIFY)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                                  /* bValueLen */
        NULL,
        GATT_PERM_READ                                      /* permissions */
    },
    /*--- DFU Control Point value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,   /* flags */
        {   /* type_value */
            GATT_UUID128_DFU_CONTROL_POINT
        },
        0,                                                  /* bValueLen */
        NULL,
        GATT_PERM_WRITE                                     /* permissions */
    },
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                           /* flags */
         ATTRIB_FLAG_CCCD_APPL),
        {   /* type_value */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value.                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT),       /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                                  /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)                  /* permissions */
    }
#endif
};

/** End of OTA_SERVICE_Exported_Constants
    * @}
    */

/*============================================================================*
 *                              Variables
 *============================================================================*/
/** @defgroup OTA_SERVICE_Exported_Variables OTA service Exported Variables
    * @brief
    * @{
    */

/** @brief  Service ID only used in this file */
static T_SERVER_ID srv_id_local;

/** @brief  Function pointer used to send event to application from OTA service
*   @note   It is initiated in ota_add_service()
*/
static P_FUN_SERVER_GENERAL_CB p_ota_extended_cb = NULL;

/** @brief  Array used to temporarily store BD Addr */
static uint8_t mac_addr[6];


bool conn_para_update_in_progress = false;
bool ota_processing_gap = false;
uint8_t retry = 0;
uint16_t ota_signature = 0;
T_START_DFU_PARA *start_dfu_para = NULL;
uint32_t image_total_length = 0;
uint32_t cur_offset = 0;
uint16_t origin_version = 0;

/** End of OTA_SERVICE_Exported_Variables
    * @}
    */

/*============================================================================*
 *                              Private Functions
 *============================================================================*/
/** @defgroup OTA_SERVICE_Exported_Functions OTA service Exported Functions
    * @brief
    * @{
    */

/**
    * @brief  Reset local variables
    * @return void
    */
static void ota_service_clear_local(void)
{
    APP_PRINT_ERROR0("ota_service_clear_local");
    conn_para_update_in_progress = false;
    start_dfu_para = NULL;
    image_total_length = 0;
    ota_signature = 0;
    cur_offset = 0;
}

/**
    * @brief    Reset local variables and disconnect peer for another OTA try
    * @param    conn_id     ID to identify the connection for disconnect
    * @return   void
    */
static void ota_service_reset_local(uint8_t conn_id)
{
    APP_PRINT_ERROR0("ota_service_reset_local");
    le_disconnect(conn_id);
    dfu_reset(ota_signature);
    ota_service_clear_local();
}

/**
    * @brief    Wrapper function to send notification to peer
    * @note
    * @param    conn_id     ID to identify the connection
    * @param    opcode      Notification on the specified opcode
    * @param    len         Notification data length
    * @param    data        Additional notification data
    * @return   void
    */
static void ota_service_prepare_send_notify(uint8_t conn_id, uint8_t opcode, uint8_t len,
                                            uint8_t *data)
{
    uint8_t dfu_notif[DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO] = {0}; /* set to maximun length for notification */
    if (data == NULL || len == 0 || len > DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO - 2)
    {
        return;
    }
//
//      btif_gatt_attr_write_req_cfm(gap_link_table[conn_id].local_mdl_id,
//                                           &srv_id_local,
//                                           BTIF_CAUSE_SUCCESS,
//                                           GATT_SUCCESS,
//                                           BLE_SERVICE_CHAR_DFU_CONTROL_POINT_INDEX
//                                          );
//
    APP_PRINT_INFO3("ota_service_prepare_send_notify, opcode:%x, len:%x, data:%b", opcode, len,
                    TRACE_BINARY(len, data));
    dfu_notif[0] = DFU_OPCODE_NOTIF;
    dfu_notif[1] = opcode;
    memcpy(&dfu_notif[2], data, len);
//      if (opcode == DFU_OPCODE_REPORT_TARGET_INFO)
//      {
//          if (len < DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO - DFU_NOTIFY_LENGTH_ARV)
//          {
//                  APP_PRINT_ERROR1("ota_service_prepare_send_notify length error:%x/(6)", len);
//                  return;
//          }
//          LE_ARRAY_TO_UINT16(dfu_notif.p.NotifyTargetImageInfo.nOrigFwVersion, &data[1]);
//          //TODO: add support to OTA continue at last lost position.
//          LE_ARRAY_TO_UINT32(dfu_notif.p.NotifyTargetImageInfo.nImageUpdateOffset, &data[3]);
//          length = DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO;
//      }
    ota_service_send_notification(conn_id, (uint8_t *)&dfu_notif, len + 2);
}

/**
    * @brief    All the connection parameter update change events are handled in this function
    * @param    conn_id     ID to identify the connection
    * @param    status      Connection parameter result, 0 indicates success, otherwise fail
    * @return   void
    */
static void ota_handle_bt_gap_conn_para_change_evt(uint8_t conn_id, uint8_t status)
{
    switch (status)
    {
    case GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS:
        {
            uint16_t conn_interval;
            uint16_t conn_slave_latency;
            uint16_t conn_supervision_timeout;
            conn_para_update_in_progress = false;
            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);

            APP_PRINT_INFO3("GAP_MSG_LE_CONN_PARAM_UPDATE update success, conn_interval = 0x%x, conn_slave_latency = 0x%x, conn_supervision_timeout = 0x%x",
                            conn_interval, conn_slave_latency, conn_supervision_timeout);
        }
        break;
    case GAP_CONN_PARAM_UPDATE_STATUS_FAIL:
        {
            APP_PRINT_INFO0("GAP_MSG_LE_CONN_PARAM_UPDATE failed.");
            conn_para_update_in_progress = false;
        }
        break;
    case GAP_CONN_PARAM_UPDATE_STATUS_PENDING:
        {
            APP_PRINT_INFO0("GAP_MSG_LE_CONN_PARAM_UPDATE param request success.");
        }
        break;
    default:
        break;
    }
}

/**
    * @brief    Only for debug to see any dev states change
    * @param    new_state   New gap state
    * @return   void
    */
static void ota_handle_bt_dev_state_change_evt(T_GAP_DEV_STATE new_state)
{
    APP_PRINT_INFO3("ota_handle_bt_dev_state_change_evt: init state = %d adv state = %d conn state = %d",
                    new_state.gap_init_state,
                    new_state.gap_adv_state, new_state.gap_conn_state);
}

/**
    * @brief    All the connection status change  events are handled in this function
    * @param    conn_id     Optional conn_id for further process
    * @param    new_state   New connection states
    * @param    disc_cause  Only valid when new_state is disconnect
    * @return   void
    */
static void ota_handle_bt_new_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state,
                                             uint16_t disc_cause)
{
    APP_PRINT_INFO2("ota_handle_bt_new_conn_state_evt: conn_id = %d new_state = %d",
                    conn_id, new_state);
    switch (new_state)
    {
    /* Connection is disconnected or idle with no advertising */
    case GAP_CONN_STATE_DISCONNECTED:
        {
            APP_PRINT_INFO1("peripheral_HandleBtConnStateChangeEvt: discon reason:%d", disc_cause);
            if (retry++ < OTA_RETRY_LIMIT)
            {
                le_adv_start();
            }
        }
        break;
    /* Device is connected */
    case GAP_CONN_STATE_CONNECTED:
        retry = 0;
        break;
    default:
        break;

    }
}

/**
    * @brief    Handle written request on DFU control point characteristic
    * @param    conn_id     ID to identify the connection
    * @param    length      Length of value to be written
    * @param    p_value     Value to be written
    * @return   T_APP_RESULT
    * @retval   Handle result of this request
    */
static T_APP_RESULT ota_service_handle_cp_req(uint8_t conn_id, uint16_t length, uint8_t *p_value)
{
    T_APP_RESULT cause = APP_RESULT_INVALID_PDU;
    uint8_t results = DFU_ARV_SUCCESS;
    uint8_t opcode = *p_value;
    uint8_t *p = p_value + 1;
    APP_PRINT_INFO2("===>OTA_Service_HandleCPReq, opcode:%x, length:%x\n", opcode, length);
    if (opcode >= DFU_OPCODE_MAX || opcode <= DFU_OPCODE_MIN)
    {
        cause = APP_RESULT_INVALID_PDU;
        APP_PRINT_ERROR1("OTA_Service_HandleCPReq, opcode not expected", opcode);
        return cause;
    }

    if (!ota_processing_gap)
    {
        ota_processing_gap = true;
    }
    switch (opcode)
    {
    case DFU_OPCODE_START_DFU:
        if (length == DFU_LENGTH_START_DFU + 4)    /* 4 bytes is pending for encrypt */
        {
            start_dfu_para = (T_START_DFU_PARA *)p;
            image_total_length = start_dfu_para->image_length * 4 + DFU_HEADER_SIZE;
            ota_signature = start_dfu_para->signature;
            APP_PRINT_TRACE6("DFU_OPCODE_START_DFU: nICType=0x%x, nOTAFlag=0x%x, nSignature=0x%x, nVersion=0x%x,nCRC16=0x%x*4Bytes, nImageLength=0x%x",
                             start_dfu_para->ic_type,
                             start_dfu_para->ota_flag,
                             start_dfu_para->signature,
                             start_dfu_para->version,
                             start_dfu_para->crc16,
                             start_dfu_para->image_length
                            );
            if (dfu_update(start_dfu_para->signature, 0, DFU_HEADER_SIZE,
                           (uint8_t *)&start_dfu_para->ic_type) == 0)
            {
                cause = APP_RESULT_SUCCESS;
                cur_offset += DFU_HEADER_SIZE;
            }
            else
            {
                results = DFU_ARV_FAIL_OPERATION;
                ota_service_reset_local(conn_id);
            }
        }
        else
        {
            APP_PRINT_ERROR0("DFU_OPCODE_START_DFU: invalid length");
            results = DFU_ARV_FAIL_OPERATION;
        }
        ota_service_prepare_send_notify(conn_id, DFU_OPCODE_START_DFU, 1, &results);
        break;
    case DFU_OPCODE_RECEIVE_FW_IMAGE_INFO:
        if (length == DFU_LENGTH_RECEIVE_FW_IMAGE_INFO)
        {
            uint16_t sign;

            LE_ARRAY_TO_UINT16(sign, p);

            if (sign == ota_signature)
            {
                cause = APP_RESULT_SUCCESS;
                LE_ARRAY_TO_UINT32(cur_offset, p + 2);
                APP_PRINT_TRACE2("DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: nSignature = 0x%x, nCurOffSet = %d",
                                 sign, cur_offset);
            }
            else
            {
                APP_PRINT_ERROR2("Receive FW Image Info with signature unexpected(%4x/%4x)", sign,
                                 ota_signature);
            }
        }
        else
        {

            APP_PRINT_TRACE0("DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: invalid length");
        }
        break;
    case DFU_OPCODE_VALID_FW:
        if (length == DFU_LENGTH_VALID_FW)
        {
            uint16_t sign;

            LE_ARRAY_TO_UINT16(sign, p);

            if (sign == ota_signature)
            {
                if (!dfu_check_checksum(sign))
                {
                    APP_PRINT_TRACE0("DFU_OPCODE_VALID_FW, CRC error");
                    results = DFU_ARV_FAIL_CRC_ERROR;
                    /* Reset to start OTA again */
                    ota_service_reset_local(conn_id);
                }
                else
                {
                    /* As OTA already done, stop process GAP message now */
                    APP_PRINT_TRACE0("DFU_OPCODE_VALID_FW, CRC Sccuess");
                    dfu_set_ota_bank_flag(true);
                    ota_processing_gap = false;
                    cause = APP_RESULT_SUCCESS;
                }
            }
            else
            {
                results = DFU_ARV_FAIL_INVALID_PARAMETER;
                APP_PRINT_ERROR2("Receive FW Image Info with signature unexpected(%4x/%4x)", sign,
                                 ota_signature);
            }
        }
        else
        {
            results = DFU_ARV_FAIL_INVALID_PARAMETER;
            APP_PRINT_TRACE0("DFU_OPCODE_VALID_FW: invalid length");
        }
        ota_service_prepare_send_notify(conn_id, DFU_OPCODE_VALID_FW, 1, &results);
        break;
    case DFU_OPCODE_ACTIVE_IMAGE_RESET:
        cause = APP_RESULT_SUCCESS;
        /* notify bootloader to reset and use new image */
        APP_PRINT_TRACE0("DFU_OPCODE_ACTIVE_IMAGE_RESET:");
        if (1)
        {
            dfu_fw_active_reset();
        }
        ota_service_clear_local();
        break;
    case DFU_OPCODE_SYSTEM_RESET:
        {
            APP_PRINT_TRACE0("DFU_OPCODE_SYSTEM_RESET:");
            cause = APP_RESULT_SUCCESS;
            ota_service_clear_local();
            break;
        }
    case DFU_OPCODE_REPORT_TARGET_INFO:
        if (length == DFU_LENGTH_REPORT_TARGET_INFO)
        {
            uint16_t verion = 0;
            uint16_t sign;
            uint8_t data[DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO - DFU_NOTIFY_LENGTH_ARV + 1] = {0};
            APP_PRINT_TRACE0("DFU_OPCODE_REPORT_TARGET_INFO:");

            LE_ARRAY_TO_UINT16(sign, p);

            if (!dfu_get_fw_version(sign, &verion))
            {
                results = DFU_ARV_FAIL_INVALID_PARAMETER;
            }
            else
            {
                cause = APP_RESULT_SUCCESS;
            }
            data[0] = results;
            LE_UINT16_TO_ARRAY(&data[1], verion);
            //LE_UINT32_TO_ARRAY(&data[3], 0);
            LE_UINT32_TO_ARRAY(&data[3], cur_offset);   /* To support continuous OTA */
            APP_PRINT_INFO3("DFU_OPCODE_REPORT_TARGET_INFO results=0x%x, cause=0x%x, cur_offset=0x%x",
                            results, cause, cur_offset);
            ota_service_prepare_send_notify(conn_id, DFU_OPCODE_REPORT_TARGET_INFO,
                                            DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO - DFU_NOTIFY_LENGTH_ARV + 1, data);
        }
        else
        {
            APP_PRINT_TRACE0("DFU_OPCODE_REPORT_TARGET_INFO: invalid length");
        }
        break;
    case DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ:
        {
            uint8_t notif_data[3] = {0};
            notif_data[0] = DFU_OPCODE_NOTIF;
            notif_data[1] = DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ;


            if (length == DFU_LENGTH_CONN_PARA_TO_UPDATE_REQ)
            {
                if (conn_para_update_in_progress)
                {
                    APP_PRINT_ERROR0("bOTA_ConnParaUpdInProgress already!!");
                    notif_data[2] = DFU_ARV_FAIL_OPERATION;
                    ota_service_send_notification(conn_id, notif_data, sizeof(notif_data));
                }
                else
                {
                    uint16_t conn_interval_min;
                    uint16_t conn_interval_max;
                    uint16_t conn_latency;
                    uint16_t superv_tout;

                    LE_ARRAY_TO_UINT16(conn_interval_min, p_value + 1);
                    LE_ARRAY_TO_UINT16(conn_interval_max, p_value + 3);
                    LE_ARRAY_TO_UINT16(conn_latency, p_value + 5);
                    LE_ARRAY_TO_UINT16(superv_tout, p_value + 7);


                    if (GAP_CAUSE_SUCCESS == le_update_conn_param(conn_id, conn_interval_min,
                                                                  conn_interval_max, conn_latency, superv_tout,
                                                                  2 * (conn_interval_min - 1), 2 * (conn_interval_max - 1)))
                    {
                        /* Connection Parameter Update Request sent successfully, means this procedure is in progress. */
                        conn_para_update_in_progress = true;
                        cause = APP_RESULT_SUCCESS;
                        APP_PRINT_INFO4("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ intMin=0x%x, intMax=0x%x, lat=0x%x, supto=0x%x.",
                                        conn_interval_min, conn_interval_max, conn_latency, superv_tout);
                    }
                    else
                    {
                        notif_data[2] = DFU_ARV_FAIL_OPERATION;
                        ota_service_send_notification(conn_id, notif_data, sizeof(notif_data));
                        APP_PRINT_ERROR0("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ le_update_conn_param failed.");
                    }
                }
            }
            else
            {
                /* TODO: to be masked */
                APP_PRINT_ERROR1("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ length = %d Error!", length);
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                ota_service_send_notification(conn_id, notif_data, sizeof(notif_data));

            }
            break;
        }
    }

    return cause;
}

/**
    * @brief    Handle written request on DFU packet characteristic
    * @param    conn_id     ID to identify the connection
    * @param    length      Length of value to be written
    * @param    p_value     Value to be written
    * @return   T_APP_RESULT
    * @retval   Handle result of this request
    */
static T_APP_RESULT ota_service_handle_packet(uint8_t conn_id, uint16_t length, uint8_t *p_value)
{

    APP_PRINT_INFO3("ota_service_handle_packet: length=%d, nCurOffSet =%d, nImageTotalLength= %d",
                    length,
                    cur_offset,
                    image_total_length
                   );

    if (cur_offset + length > image_total_length)
    {
        /*  Image transfer error. ignore this OTA process, and may send notification to otherside to resend again.
            TODO: send notification to application for restart OTA process. Need implementation on application side.
            Reset local OTA parameter */
        ota_service_reset_local(conn_id);
        return APP_RESULT_INVALID_PDU;
    }

    if (dfu_update(ota_signature, cur_offset, length, p_value) != 0)
    {
        /*  TODO: if write to flash error, need to restart OTA flow from the beginning or last posistion?
            Currently, only reset local parameter, and disconnect the link. It will start OTA again */
        ota_service_reset_local(conn_id);
        return APP_RESULT_APP_ERR;

    }
    cur_offset += length;
    return APP_RESULT_SUCCESS;
}

/**
    * @brief    Write characteristic data from service
    * @param    conn_id     ID to identify the connection
    * @param    service_id   Service ID to be written
    * @param    attr_index  Attribute index of characteristic
    * @param    write_type  Write type of data to be written
    * @param    length      Length of value to be written
    * @param    p_value     Value to be written
    * @param    p_write_ind_post_proc   Write indicate post procedure
    * @return   T_APP_RESULT
    * @retval   Profile procedure result
    */
static T_APP_RESULT ota_service_attr_write_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                              uint16_t attr_index, T_WRITE_TYPE write_type, uint16_t length,
                                              uint8_t *p_value, P_FUN_WRITE_IND_POST_PROC *p_write_ind_post_proc)
{
    T_OTA_CALLBACK_DATA callback_data;
    T_APP_RESULT  cause = APP_RESULT_SUCCESS;
    APP_PRINT_INFO2("--> ota_service_attr_write_cb, index=%d(%d)", attr_index,
                    BLE_SERVICE_CHAR_OTA_INDEX);
    if (BLE_SERVICE_CHAR_OTA_INDEX == attr_index)
    {
        /* Make sure written value size is valid. */
        if ((length != sizeof(uint8_t)) || (p_value == NULL))
        {
            cause  = APP_RESULT_INVALID_VALUE_SIZE;
        }
        else
        {
            /* Notify Application. */
            callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
            callback_data.msg_data.write.opcode = OTA_WRITE_CHAR_VAL;
            callback_data.conn_id = conn_id;
            callback_data.msg_data.write.value = p_value[0];

            if (p_ota_extended_cb)
            {
                p_ota_extended_cb(service_id, (void *)&callback_data);
            }
        }
    }
    else if (BLE_SERVICE_CHAR_DFU_PACKET_INDEX == attr_index)
    {
        return ota_service_handle_packet(conn_id, length, p_value);
    }
    else if (BLE_SERVICE_CHAR_DFU_CONTROL_POINT_INDEX == attr_index)
    {
        return ota_service_handle_cp_req(conn_id, length, p_value);
    }
    else
    {
        APP_PRINT_ERROR2("--> OTA_AttrWrite Error  attr_index = 0x%x length=%d",
                         attr_index, length);
        cause = APP_RESULT_ATTR_NOT_FOUND;
    }
    return cause;

}

/**
    * @brief    Read characteristic data from service
    * @param    conn_id     ID to identify the connection
    * @param    service_id  ServiceID of characteristic data
    * @param    attr_index  Attribute index of getting characteristic data
    * @param    offset      Used for Blob Read
    * @param    p_length    Length of getting characteristic data
    * @param    pp_value    Data got from service
    * @return   T_APP_RESULT
    * @retval   Profile procedure result
    */
static T_APP_RESULT ota_service_attr_read_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                             uint16_t attr_index,
                                             uint16_t offset, uint16_t *p_length, uint8_t **pp_value)
{
    T_APP_RESULT  cause = APP_RESULT_SUCCESS;
    APP_PRINT_INFO1("<-- OTA_AttrRead, index=%d", attr_index);
    switch (attr_index)
    {
    default:
        APP_PRINT_ERROR1("<-- OTA_AttrRead, Attr not found, index=%d", attr_index);
        cause  = APP_RESULT_ATTR_NOT_FOUND;
        break;
    case BLE_SERVICE_CHAR_MAC_ADDRESS_INDEX:
        {
            gap_get_param(GAP_PARAM_BD_ADDR, mac_addr);
            uint8_t addr[6];
            for (int i = 0; i < 6; i++)
            {
                addr[i] = mac_addr[5 - i];
            }
            memcpy(mac_addr, addr, 6);
            *pp_value  = (uint8_t *)mac_addr;
            *p_length = sizeof(mac_addr);
        }
        break;
    case BLE_SERVICE_CHAR_PATCH_INDEX:
        {
            dfu_get_fw_version(SIGNATURE_PATCH_FLASH, &origin_version);
            *pp_value  = (uint8_t *)&origin_version;
            *p_length = sizeof(origin_version);
            APP_PRINT_TRACE1("OTA_AttrRead patch version: %d", origin_version);
        }
        break;

    case BLE_SERVICE_CHAR_APP_VERSION_INDEX:
        {
            dfu_get_fw_version(SIGNATURE_APP, &origin_version);
            *pp_value  = (uint8_t *)&origin_version;
            *p_length = sizeof(origin_version);
            APP_PRINT_TRACE1("OTA_AttrRead app version: %d", origin_version);
        }
        break;
    case BLE_SERVICE_CHAR_DSP_VERSION_INDEX:
        {
            dfu_get_fw_version(SIGNATURE_DSP_PATCH, &origin_version);
            *pp_value  = (uint8_t *)&origin_version;
            *p_length = sizeof(origin_version);
            APP_PRINT_TRACE1("OTA_AttrRead DSP version: %d", origin_version);
        }
        break;
        /* TODO: add other version later */

    }
    return (cause);
}

/** End of OTA_SERVICE_Exported_Functions
    * @}
    */


/** End of OTA_SERVICE
    * @}
    */

/** End of OTA_DEMO
    * @}
    */

/** End of SAMPLE_APP
    * @}
    */


/*============================================================================*
 *                              Public Functions
 *============================================================================*/
/**
    * @brief    Add OTA BLE service to application
    * @param    p_func  Pointer of APP callback function called by profile
    * @return   Service ID auto generated by profile layer
    * @retval   A T_SERVER_ID type value
    */
T_SERVER_ID ota_add_service(void *p_func)
{
    T_SERVER_ID service_id;
    if (false == server_add_service(&service_id,
                                    (uint8_t *)gatt_extended_service_table,
                                    sizeof(gatt_extended_service_table),
                                    ota_service_cbs))
    {
        APP_PRINT_ERROR1("ota_add_service: service_id %d", service_id);
        service_id = 0xff;
        return service_id;
    }
    p_ota_extended_cb = (P_FUN_SERVER_GENERAL_CB)p_func;
    srv_id_local = service_id;
    return service_id;
}

/**
    * @brief    Used to handle BT GAP messages
    * @note     OTA module will handle and only handle BT GAP conn parameter update/disc,
    *           and will leverage LE module to handle other BT GAP messages. Add this to
    *           application if you want to handle connection related IO notification.
    * @param    p_io_msg    Pointer to message to be handled
    * @return   void
    */
void ota_handle_gap_message(T_IO_MSG *p_io_msg)
{
    T_LE_GAP_MSG gap_msg;
    /* To confirm if it is update message */
    if (p_io_msg->type != IO_MSG_TYPE_BT_STATUS)
    {
        return;
    }

    memcpy(&gap_msg, &p_io_msg->u.param, sizeof(p_io_msg->u.param));
    switch (p_io_msg->subtype)
    {
    case GAP_MSG_LE_CONN_PARAM_UPDATE:
        ota_handle_bt_gap_conn_para_change_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
                                               gap_msg.msg_data.gap_conn_param_update.status);
        break;
    case GAP_MSG_LE_CONN_STATE_CHANGE:
        ota_handle_bt_new_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
                                         (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
                                         gap_msg.msg_data.gap_conn_state_change.disc_cause);
        break;
    case GAP_MSG_LE_DEV_STATE_CHANGE:
        {
            ota_handle_bt_dev_state_change_evt(gap_msg.msg_data.gap_dev_state_change.new_state);
        }
        break;
    default:
        break;
    }
}

