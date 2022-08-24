/**
****************************************************************************************************
*               Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
****************************************************************************************************
* @file      flash_adv_cfg.h
* @brief
* @note      flash advanced functions
* @author    Grace
* @date      2018-04-19
* @version   v0.1
* **************************************************************************************************
*/

#ifndef _FLASH_ADV_CFG_H_
#define _FLASH_ADV_CFG_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef  __cplusplus
extern  "C" {
#endif  // __cplusplus

/** @defgroup  FLASH_DEVICE    Flash Device
    * @{
    */

/*============================================================================*
  *                                   Types
  *============================================================================*/
/** @defgroup FLASH_DEVICE_Exported_Types Flash Device Exported Types
    * @brief
    * @{
    */

/** End of FLASH_DEVICE_Exported_Types
  * @}
  */
/*************************************************************************************************
  *                                   Functions
*************************************************************************************************/
/** @defgroup FLASH_DEVICE_Exported_Functions Flash Device Exported Functions
    * @brief
    * @{
    */

/**
* @brief    get block protect level
* @param    *bp_lv  a set of BPx ~ BP0
* @return   success or not
*/
bool flash_get_block_protect_locked(uint8_t *bp_lv);

/**
* @brief    set block protect by map
* @param    bp_lv a set of BPx ~ BP0
* @return   success or not
*/
bool flash_set_block_protect_locked(uint8_t bp_lv);

/**
* @brief    only unlock prefer section by addres
* @param    unlock_addr address section to be unlocked
* @param    *old_bp_lv before unlock
* @return   success or not
*/
bool flash_sw_protect_unlock_by_addr_locked(uint32_t unlock_addr, uint8_t *old_bp_lv);


/** @} */ /* End of group FLASH_DEVICE_Exported_Functions */


/** @} */ /* End of group FLASH_DEVICE */

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // _FLASH_ADV_CFG_H_
