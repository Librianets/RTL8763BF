/**
*********************************************************************************************************
*               Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     privacy_mgnt.c
* @brief    simple BLE profile source file.
* @details  Demonstration of how to implement a self-definition profile.
* @author
* @date     2016-02-18
* @version  v0.1
*********************************************************************************************************
*/
#include <trace.h>
#include <string.h>
#include <gap.h>
#include <privacy_mgnt.h>

bool privacy_whitelist = false;
T_LE_PRIVACY_STATE app_privacy_resolution_state = LE_PRIVACY_RESOLUTION_DISABLED;


void privacy_modify_resolving_list(T_GAP_RESOLV_LIST_OP op, T_GAP_IDENT_ADDR_TYPE addr_type,
                                   uint8_t *addr, bool device_mode)
{
    T_GAP_CAUSE cause;
    APP_PRINT_INFO3("T_GAP_RESOLV_LIST_OP %d, BD %s, type %d\n", op, TRACE_BDADDR(addr), addr_type);
    switch (op)
    {
    case GAP_RESOLV_LIST_OP_CLEAR:
        {
            APP_PRINT_INFO0("resolving list clear");
            cause = le_privacy_modify_resolv_list(GAP_RESOLV_LIST_OP_CLEAR, GAP_IDENT_ADDR_PUBLIC, NULL);
            if (cause == GAP_CAUSE_SUCCESS)
            {
                if (privacy_whitelist)
                {
                    le_modify_white_list(GAP_WHITE_LIST_OP_CLEAR, NULL, GAP_REMOTE_ADDR_LE_PUBLIC);
                }
            }
            else
            {
                APP_PRINT_ERROR1("clear failed: cause %d", cause);
            }
        }
        break;
    case GAP_RESOLV_LIST_OP_ADD:
        {
            APP_PRINT_INFO0("resolving list add");
            cause = le_privacy_modify_resolv_list(GAP_RESOLV_LIST_OP_ADD, addr_type, addr);
            if (cause == GAP_CAUSE_SUCCESS)
            {
                if (privacy_whitelist)
                {
                    le_modify_white_list(GAP_WHITE_LIST_OP_ADD, addr,
                                         (T_GAP_REMOTE_ADDR_TYPE)addr_type);
                }
                if (device_mode)
                {
                    le_privacy_set_mode(addr_type, addr, GAP_PRIVACY_MODE_DEVICE);
                }
            }
            else if (cause != GAP_CAUSE_INVALID_STATE)
            {
                APP_PRINT_ERROR1("Add failed: cause %d", cause);
            }
            else
            {
                APP_PRINT_ERROR0("Add failed: invalid state");
            }
        }
        break;
    case GAP_RESOLV_LIST_OP_REMOVE:
        {
            APP_PRINT_INFO0("resolving list remove");
            cause = le_privacy_modify_resolv_list(GAP_RESOLV_LIST_OP_REMOVE, addr_type, addr);
            if (cause == GAP_CAUSE_SUCCESS)
            {
                if (privacy_whitelist)
                {
                    le_modify_white_list(GAP_WHITE_LIST_OP_REMOVE, addr, (T_GAP_REMOTE_ADDR_TYPE)addr_type);
                }
            }
            else if (cause != GAP_CAUSE_INVALID_STATE)
            {
                APP_PRINT_ERROR1("Remove failed: cause %d", cause);
            }
            else
            {
                APP_PRINT_ERROR0("Remove failed: invalid state");
            }
        }
        break;
    default:
        break;
    }

}


/**
* @brief   add privacy device to resolving list
* @return  void
*/
bool privacy_add_device(T_LE_KEY_ENTRY *p_entry)
{
    if (p_entry != NULL && p_entry->is_used)
    {

        APP_PRINT_INFO1("privacy_add_device: p_entry->is_used = %d", p_entry->is_used);
        bool device_mode = true;
        T_LE_PRIVACY_INFO privacy_info;
        if (le_get_privacy_info(p_entry, &privacy_info))
        {
            if (privacy_info.is_discov && privacy_info.resolv_addr_only)
            {
                device_mode = false;
            }
        }
        if (p_entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT)
        {
            APP_PRINT_INFO0("privacy_add_device: LE_KEY_STORE_REMOTE_IRK_BIT");
            privacy_modify_resolving_list(GAP_RESOLV_LIST_OP_ADD,
                                          (T_GAP_IDENT_ADDR_TYPE)p_entry->resolved_remote_bd.remote_bd_type,
                                          p_entry->resolved_remote_bd.addr, device_mode);
            return true;
        }
        else
        {
            APP_PRINT_INFO0("privacy_add_device: GAP_REMOTE_ADDR_LE_PUBLIC");
            if ((p_entry->remote_bd.remote_bd_type == GAP_REMOTE_ADDR_LE_PUBLIC) ||
                ((p_entry->remote_bd.remote_bd_type == GAP_REMOTE_ADDR_LE_RANDOM) &&
                 ((p_entry->remote_bd.addr[5] & RANDOM_ADDR_MASK) == RANDOM_ADDR_MASK_STATIC))
               )
            {
                privacy_modify_resolving_list(GAP_RESOLV_LIST_OP_ADD,
                                              (T_GAP_IDENT_ADDR_TYPE)p_entry->remote_bd.remote_bd_type,
                                              p_entry->remote_bd.addr, device_mode);
                return true;
            }
            else
            {
                APP_PRINT_ERROR1("privacy_add_device: failed, idx %d", p_entry->idx);
            }
        }
    }
    return false;
}

/**
* @brief   privacy init
* @return  void
*/
void privacy_init(bool whitelist)
{
    T_LE_KEY_ENTRY *p_entry;

    APP_PRINT_INFO1("privacy_init: whitelist %d", whitelist);

    privacy_whitelist = whitelist;
    le_privacy_register_cb(privacy_msg_callback);

    p_entry = le_get_high_priority_bond();
    if ((p_entry != NULL) && (p_entry->is_used))
    {
        privacy_add_device(p_entry);
    }
    return;
}


/**
* @brief   handle privacy resolution status
* @return  void
*/
void privacy_handle_le_privacy_resolution_status_info(T_LE_PRIVACY_RESOLUTION_STATUS_INFO
                                                      resolv_status)
{
    APP_PRINT_INFO1("privacy_handle_le_privacy_resolution_status_info: status 0x%x\n",
                    resolv_status.status);
    app_privacy_resolution_state = resolv_status.status;
}

/**
* @brief   modify resolving list response
* @return  void
*/
void privacy_handle_le_privacy_modify_resolv_list_rsp(T_LE_PRIVACY_MODIFY_RESOLV_LIST_RSP *p_rsp)
{
    APP_PRINT_INFO2("privacy_handle_le_privacy_modify_resolv_list_rsp: operation  0x%x, casue 0x%x",
                    p_rsp->operation, p_rsp->cause);
    if (p_rsp->cause == GAP_SUCCESS)
    {
        extern bool resolv_list_exist;
        if (p_rsp->operation == GAP_RESOLV_LIST_OP_ADD)
        {
            resolv_list_exist = true;
        }
        else
        {
            resolv_list_exist = false;
        }
    }
    else
    {
    }
}


/**
* @brief   privacy message callback
* @return  void
*/
T_APP_RESULT privacy_msg_callback(uint8_t msg_type, T_LE_PRIVACY_CB_DATA msg_data)
{
    T_APP_RESULT result = APP_RESULT_SUCCESS;
    APP_PRINT_INFO1("privacy_msg_callback: msg_type  %d", msg_type);

    switch (msg_type)
    {
    case GAP_MSG_LE_PRIVACY_RESOLUTION_STATUS_INFO:
        privacy_handle_le_privacy_resolution_status_info(msg_data.le_privacy_resolution_status_info);
        break;

    case GAP_MSG_LE_PRIVACY_MODIFY_RESOLV_LIST:
        privacy_handle_le_privacy_modify_resolv_list_rsp(msg_data.p_le_privacy_modify_resolv_list_rsp);
        break;

    default:
        break;
    }
    return result;
}

/**
* @brief   privacy handle bond modify message, used in app_gap_callback
* @return  void
*/
void privacy_handle_bond_modify_msg(T_LE_BOND_MODIFY_TYPE type, T_LE_KEY_ENTRY *p_entry)
{
    APP_PRINT_INFO1("privacy_handle_bond_modify_msg: type 0x%x", type);

    if (type == LE_BOND_CLEAR)
    {
        privacy_modify_resolving_list(GAP_RESOLV_LIST_OP_CLEAR, GAP_IDENT_ADDR_PUBLIC, NULL, false);
    }
    else if (type == LE_BOND_DELETE)
    {
        if (p_entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT)
        {
            privacy_modify_resolving_list(GAP_RESOLV_LIST_OP_REMOVE,
                                          (T_GAP_IDENT_ADDR_TYPE)p_entry->resolved_remote_bd.remote_bd_type,
                                          p_entry->resolved_remote_bd.addr, false);
        }
        else
        {
            privacy_modify_resolving_list(GAP_RESOLV_LIST_OP_REMOVE,
                                          (T_GAP_IDENT_ADDR_TYPE)p_entry->remote_bd.remote_bd_type,
                                          p_entry->remote_bd.addr, false);
        }
    }
}

