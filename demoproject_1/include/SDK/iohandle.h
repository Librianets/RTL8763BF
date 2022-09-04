#ifndef _IOHANDLE_H
#define _IOHANDLE_H

/*============================================================================*
  *                               Header Files
  *============================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <app_msg.h>
#ifdef __cplusplus
extern "C" {
#endif
/**  @brief IO sub types definition for @ref IO_MSG_TYPE_KEYSCAN type */
typedef void *TimerHandle_t;
typedef enum
{
    MSG_SHORT_PRESS     = 1,
    MSG_SHORT_RELEASE   = 2,
    MSG_LONG_PRESS      = 3,
    MSG_LONG_RELEASE    = 4,
} T_MSG_TYPE_PXP_GPIO;
#define ALERT_HIGH_PERIOD 100
#define ALERT_LOW_PERIOD 900

#define LED_BLINK 0x01
#define BEEP_ALERT 0x02
extern TimerHandle_t xTimerPxpIO;
extern bool allowedPxpEnterDlps;
extern uint8_t keystatus;
void PINMUX_Configuration(void);
void PAD_Configuration(void);
void RCC_Configuration(void);
void driver_init(void);
void KEY_INT_Handle(void);
void Pxp_HandleButtonEvent(T_IO_MSG io_msg);
void StartPxpIO(uint32_t lowPeroid, uint32_t HighPeroid, uint8_t mode, uint32_t cnt);
void StopPxpIO(void);
void ChangeConnectionParameter(uint16_t interval, uint16_t latency, uint16_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* _IOHANDLE_H */
