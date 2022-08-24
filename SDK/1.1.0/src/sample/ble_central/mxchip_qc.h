#ifndef __MXCHIP_QC_H__
#define __MXCHIP_QC_H__


#define QC_TEST_PRINT_STRING(NAME, CONTENT) do{ mf_printf(NAME); mf_printf(" "); mf_printf(CONTENT); mf_printf("\r\n");}while(0)
#define QC_TEST_PRINT_STRING_FUN(NAME, CONTENT_FUNC) do{    mf_printf(NAME); \
                                                            mf_printf(" ");  \
                                                            memset (str, 0, sizeof (str));  \
                                                            CONTENT_FUNC (str, sizeof (str));  \
                                                            mf_printf(str);  \
                                                            mf_printf("\r\n");}while(0)
#define EMB1082
																														
#define SDK_VERSION      "0.8.1"
#define SERIAL_NUMBER    "0660.XF01.LY01"
																														
#if defined EMB1082
#define MODULE					 "EMB1082"			
#elif defined EMB1087
#define MODULE					 "EMB1087"			
#else
#error "module undefined"
#endif
																														
#define APP_VERSION_STR	 "APP_v1.0.1"//@"MODULE

#if defined EMB1082																														
#define MXCHIP_QC_CHECK_PIN			P0_1
#define MXCHIP_ATE_CHECK_PIN    P0_0
#elif defined EMB1087
#define MXCHIP_QC_CHECK_PIN			P0_0
#define MXCHIP_ATE_CHECK_PIN    P2_7
#else
#define MXCHIP_QC_CHECK_PIN			P0_1
#define MXCHIP_ATE_CHECK_PIN    P0_0
#endif																														
																														
#define UART_TX          P3_0
#define UART_RX          P3_1
																														
void mxchip_qc_init( void );

#endif
