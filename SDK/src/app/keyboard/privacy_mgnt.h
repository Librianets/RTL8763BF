/**
*****************************************************************************************
*     Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     privacy_mgnt.h
  * @brief    privacy managerment.
  * @details  privacy managerment.
  * @author   jane
  * @date     2016-02-18
  * @version  v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _PRIVACY_MGNT_H_
#define _PRIVACY_MGNT_H_

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/* Add Includes here */
#include <gap_bond_le.h>
#include <gap_privacy.h>



extern T_LE_PRIVACY_STATE app_privacy_resolution_state;

bool privacy_add_device(T_LE_KEY_ENTRY *p_entry);
void privacy_init(bool whitelist);
T_APP_RESULT privacy_msg_callback(uint8_t msg_type, T_LE_PRIVACY_CB_DATA msg_data);
void privacy_handle_bond_modify_msg(T_LE_BOND_MODIFY_TYPE type, T_LE_KEY_ENTRY *p_entry);

#ifdef __cplusplus
}
#endif

#endif /* _PRIVACY_MGNT_H_ */
