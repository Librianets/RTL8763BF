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

typedef void *TimerHandle_t;
typedef enum
{
    MSG_KEYS_SHORT_PRESS     = 1,
    MSG_KEYS_SHORT_RELEASE   = 2,
    MSG_KEYS_LONG_PRESS      = 3,
    MSG_KEYS_LONG_RELEASE    = 4,
    MSG_KEYM_SHORT_PRESS     = 5,
    MSG_KEYM_SHORT_RELEASE   = 6,
    MSG_KEYM_LONG_PRESS      = 7,
    MSG_KEYM_LONG_RELEASE    = 8,
} T_MSG_TYPE_PXP_GPIO;
#define ALERT_HIGH_PERIOD 100
#define ALERT_LOW_PERIOD 900

#define LED_BLINK  0x01
#define BEEP_ALERT 0x02

extern uint8_t keyMstatus;
extern uint8_t keySstatus;

extern bool gMBeepflg;
extern bool gSBeepflg;
extern bool allowedPxpEnterDlps;
void PINMUX_Configuration(void);
void PAD_Configuration(void);
void RCC_Configuration(void);
void KEY_M_INT_Handle(void);
void KEY_S_INT_Handle(void);
void driver_init(void);
void Pxp_HandleButtonEvent(T_IO_MSG io_msg);
void StartPxpMIO(uint32_t lowPeroid, uint32_t HighPeroid, uint8_t mode, uint32_t cnt);
void StopPxpMIO(void);
void StartPxpSIO(uint32_t lowPeroid, uint32_t HighPeroid, uint8_t mode, uint32_t cnt);
void StopPxpSIO(void);
#ifdef __cplusplus
}
#endif

#endif /* _IOHANDLE_H */
