/**
************************************************************************************************************
*               Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     rtl876x_lib_platform.h
* @brief
* @details
* @author
* @date     2018-07-17
* @version
*************************************************************************************************************
*/

#ifndef _RTL876X_LIB_PLATFORM_H_
#define _RTL876X_LIB_PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief Write MAC address to config, this is mainly used on production line.
  * @param[in] p_mac_addr         The buffer hold MAC address (48 bits).
  * @return Write MAC to config fail or success.
  *     @retval true              Write MAC to config success.
  *     @retval false             Write MAC to config fails or not write existed MAC.
  */
bool UpdateMAC(uint8_t *p_mac_addr);

/**
  * @brief Write 40M XTAL calibration data to config, this is mainly used on production line.
  * @param[in] xtal               The value of 40M XTAL calibration data
  * @return Write calibration data to config fail or success.
  *     @retval true              Success.
  *     @retval false             Fail.
  */
bool WriteXtalToConfig(uint8_t xtal);

#ifdef __cplusplus
}
#endif

#endif /* _RTL876X_LIB_PLATFORM_H_ */
