#include <string.h>
#include <gatt.h>
#include <bt_types.h>
#include "trace.h"
#include "gap_le.h"
#include "gap_conn_le.h"
#include "gap_msg.h"
#include "app_msg.h"
#include "flash_device.h"
#include "rtl876x_wdg.h"
#include "rcu_ota_service.h"
#include "rcu_dfu_service.h"
#include "dfu_api.h"
#include "patch_header_check.h"
#include "board.h"
#include "flash_adv_cfg.h"
#include "os_sched.h"
#include "mem_config.h"
#include "otp.h"
#include "rtl876x_hw_aes.h"
#include "patch_header_check.h"
#include "user_flash_driver.h"
#include "rcu_application.h"
#include "rcu_gap.h"
#include "os_timer.h"
#include "swtimer.h"
#include "os_sched.h"
#include "app_section.h"
#include "os_sync.h"
#include "platform_utils.h"

#if SUPPORT_SILENT_OTA
/*============================================================================*
 *                              Micro
 *============================================================================*/
/* image header length */
#define IMG_HEADER_SIZE          1024
#define DFU_HEADER_SIZE          12 /*currently, first 12 byte only will be treated as image header*/

/*support flash suspend config: 0 not support, 1 support*/
#define  SUPPORT_ERASE_SUSPEND    0
#if  SUPPORT_ERASE_SUSPEND
#define  BIT_STATUS_SUSPEND       BIT(7)
#define  BIT_STATUS_WEL           BIT(1)
#define  CMD_SECTOR_ERASE         0x20
#endif

/*user data copy config*/
#define  USER_DATA_COPY_EN        0  /*0 disable, 1 enable*/

#define  RCU_STATE_CHECK()        (app_global_data.rcu_status == RCU_STATUS_PAIRED)
#define  RESET_TIME_OUT_TIMER()   os_timer_restart(&dfu_time_out_timer, DFU_PROC_TIME_OUT)
#define  gDfuServiceId            app_global_data.dfu_srv_id
#define  SECTOR_SIZE              0x1000

#define  BIT_STATUS_WIP           BIT(0)

#define  DFU_PROC_TIME_OUT        2000  /* 2s */
/*============================================================================*
 *                              Local Functions
 *============================================================================*/

void silence_BufferCheckProc(uint16_t _mBufferSize, uint16_t _mCrc);
bool sil_dfu_copy_img(uint16_t signature, uint32_t dlAddress, uint32_t dlSize);
void dfu_timer_init(void);
void dfu_process_timeout_timer_callback(TimerHandle_t p_timer);
void dfu_timeout_handle(void);

/*============================================================================*
 *                              External Functions
 *============================================================================*/

bool hw_aes_decrypt_16byte(uint8_t *input, uint8_t *output);

/*============================================================================*
 *                              External Variables
 *============================================================================*/

extern   T_APP_GLOBAL_DATA app_global_data;

/*============================================================================*
 *                              Local Variables
 *============================================================================*/

P_FUN_SERVER_GENERAL_CB pfnDfuExtendedCB = NULL;
const uint8_t   SILENCE_GATT_UUID128_DFU_SERVICE[16] = {0x12, 0xA2, 0x4D, 0x2E, 0xFE, 0x14, 0x48, 0x8e, 0x93, 0xD2, 0x17, 0x3C, 0x87, 0x62, 0x00, 0x00};

bool gOtaValidFlg = false;
uint8_t prev_bp_lv = 0xfe;
uint16_t ota_tmp_buf_used_size;
uint16_t mBufSize;
uint16_t mCrcVal;
uint32_t gSilDfuResendOffset = 0;
uint8_t *g_pOtaTempBufferHead;
uint8_t OtaTempBufferHead[DFU_TEMP_BUFFER_SIZE];

TDFU_CB dfuPara;

/*time out timer*/
TimerHandle_t dfu_time_out_timer;

T_ATTRIB_APPL silence_dfu_service[] =
{

    /*-------------------------- DFU Service ---------------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),           /* flags     */
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),              /* type_value */
        },
        UUID_128BIT_SIZE,                                    /* bValueLen     */
        (void *)SILENCE_GATT_UUID128_DFU_SERVICE,           /* p_value_context */
        GATT_PERM_READ                                      /* permissions  */
    },



    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {                                           /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP/* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*--- DFU packet characteristic value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,                              /* flags */
        {                                                           /* type_value */
            GATT_UUID128_DFU_PACKET
        },
        0,                                                 /* bValueLen */
        NULL,
        GATT_PERM_WRITE                 /* permissions */
    },
    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {                                           /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_WRITE |                   /* characteristic properties */
             GATT_CHAR_PROP_NOTIFY)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*--- DFU Control Point value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,                              /* flags */
        {                                                           /* type_value */
            GATT_UUID128_DFU_CONTROL_POINT
        },
        0,                                                 /* bValueLen */
        NULL,
        GATT_PERM_WRITE                 /* permissions */
    },
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* flags */
         ATTRIB_FLAG_CCCD_APPL),
        {                                           /* type_value */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* permissions */
    }

};

/**
 * @brief dfu process time out timer callback
 *
 * @param p_timer - timer handler
 * @return none
 * @retval void
 */
void dfu_process_timeout_timer_callback(TimerHandle_t p_timer)
{
    /*check dfu time out*/
    if (dfuPara.is_dfu_progressing)
    {
        dfu_timeout_handle();
    }
}

/**
 * @brief dfu process time init
 *
 * @param none
 * @return none
 * @retval void
 */
void dfu_timer_init(void)
{
    /* pair_fail_disconn_timer is used to disconnect for pair failed */
    if (false == os_timer_create(&dfu_time_out_timer, "dfu_time_out_timer",  1, \
                                 DFU_PROC_TIME_OUT, false, dfu_process_timeout_timer_callback))
    {
        APP_PRINT_INFO0("[dfu_timer_init] init dfu_timer_init failed");
    }
}

/**
*  @brief: unlock flash is need when erase or write flash.
*/
DATA_RAM_FUNCTION bool unlock_flash_all(void)
{
    prev_bp_lv = 0;
    DFU_PRINT_TRACE0("**********[Flash Set] Flash unlock ***********");
    if (FLASH_SUCCESS == flash_sw_protect_unlock_by_addr_locked((0x00800000), &prev_bp_lv))
    {
        DFU_PRINT_TRACE1("[Flash Set] Flash unlock address = 0x800000, prev_bp_lv = %d", prev_bp_lv);
        return true;
    }
    return false;
}
/**
*  @brief: lock flash after erase or write flash.
*/
DATA_RAM_FUNCTION void lock_flash(void)
{
    if (prev_bp_lv != 0xfe)
    {
        flash_set_block_protect_locked(prev_bp_lv);
    }
}

/**
*  @brief: get image size accord to the image id.
*/
uint32_t get_temp_ota_bank_size_by_img_id(T_IMG_ID image_id)
{
    uint32_t image_size = 0;

    bool enable_old_ota = !is_ota_support_bank_switch();
    if (enable_old_ota)
    {
#if (PATCH_SUPPORT_OTA_APP_DATA_EXTENSION == 1)
        if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
            || image_id == AppData1 || image_id == AppData2 || image_id == AppData3
            || image_id == AppData4 || image_id == AppData5 || image_id == AppData6)
#else
        if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
            || image_id == AppData1 || image_id == AppData2)
#endif
        {
            image_size = flash_get_bank_size(FLASH_OTA_TMP);
        }
        //others will return 0
    }
    else
    {
        uint32_t ota_bank0_addr = flash_get_bank_addr(FLASH_OTA_BANK_0);
        uint32_t temp_bank_addr;
        if (ota_bank0_addr == get_active_ota_bank_addr())
        {
            temp_bank_addr = flash_get_bank_addr(FLASH_OTA_BANK_1);
        }
        else
        {
            temp_bank_addr = ota_bank0_addr;
        }

        if (image_id == OTA)
        {
            image_size = OTA_HEADER_SIZE;
        }
#if (PATCH_SUPPORT_OTA_APP_DATA_EXTENSION == 1)
        else if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
                 || image_id == AppData1 || image_id == AppData2 || image_id == AppData3
                 || image_id == AppData4 || image_id == AppData5 || image_id == AppData6)
#else
        else if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
                 || image_id == AppData1 || image_id == AppData2)
#endif
        {
            // auth ota temp bank and get address
            // image_authencation will fail after secure boot, so remove it
            if (!check_header_valid(temp_bank_addr, OTA))
            {
                image_size = 0;
            }
            else
            {
                image_size = HAL_READ32((uint32_t) & ((T_OTA_HEADER_FORMAT *)temp_bank_addr)->secure_boot_size,
                                        (image_id - SecureBoot) * 8);

                //attention: if use old ota header generate tool, app data3-6 addr will be default value 0xffffffff
                if (OTA_HEADER_DEFAULT_VALUE == image_size)
                {
                    image_size = 0;
                }
            }
        }
        else //others will return 0
        {
        }
    }

    return image_size;
}

/**
* @brief check ota image size whether exceed flash layout address.
*/
bool check_dfu_update_image_length(uint16_t signature, uint32_t offset, uint32_t length,
                                   void *p_void, uint32_t *ret)
{
    uint32_t temp_bank_size = 0;
    *ret = 0;

    if (p_void == NULL)
    {
        *ret = __LINE__;
        return false;
    }

    //temp_bank_size = flash_ioctl(flash_ioctl_get_temp_bank_size_by_image_id, signature, 0); //if patch support
    temp_bank_size = get_temp_ota_bank_size_by_img_id((T_IMG_ID)signature);

    if (offset == 0)
    {
        T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *) p_void;
        uint32_t total_length = p_header->payload_len + 1024;

        if (total_length > temp_bank_size)
        {
            DFU_PRINT_ERROR2("New Image too large! total_length = %d, temp_bank_size = %d", total_length,
                             temp_bank_size);
            *ret = __LINE__;
            return false;
        }
    }

    if (offset + length > temp_bank_size)
    {
        DFU_PRINT_ERROR3("New Image single packet too large! offset = %d, length = %d, temp_bank_size = %d",
                         offset, length, temp_bank_size);
        *ret = __LINE__;
        return false;
    }

    //check pass
    return true;
}


/**
*   flash suspend support
*/
#if SUPPORT_ERASE_SUSPEND
bool erase_in_progress = false;
bool is_suspend = false;
uint32_t signal;
DATA_RAM_FUNCTION void app_flash_erase_suspend()
{
    if (erase_in_progress && !is_suspend)
    {
        bool ret;
        uint8_t status1, status2;
        uint32_t ctrlr0 = SPIC->ctrlr0;
        uint32_t addr_len = SPIC->addr_length;

        signal = os_lock();

        spic_enable(DISABLE);
        spic_clr_multi_ch();
        spic_set_tx_mode();

        SPIC->addr_length = 0;
        spic_set_dr(DATA_BYTE, 0x75);

        spic_enable(ENABLE);
        spic_wait_busy();
        spic_enable(DISABLE);

        while (1)
        {
            ret = flash_cmd_rx(0x35, 1, &status1);
            ret &= flash_cmd_rx(0x05, 1, &status2);

            DFU_PRINT_INFO3("ret is %x,status1 is %x,status2 is %x", ret, status1, status2);

            if ((!(status1 & BIT_STATUS_WIP) && (status2 & BIT_STATUS_SUSPEND)) ||
                (!(status1 & BIT_STATUS_WIP) && !(status1 & BIT_STATUS_WEL)))
            {
                DFU_PRINT_INFO0("SUSPEND OK");
                platform_delay_us(1);
                is_suspend = true;
                break;
            }
        }
        //restore ctrl0 and addr_len register
        SPIC->ctrlr0 = ctrlr0;
        SPIC->addr_length = addr_len;
        os_unlock(signal);
    }
}
DATA_RAM_FUNCTION void app_flash_erase_resume()
{
    if (erase_in_progress && is_suspend)
    {
        uint32_t ctrlr0 = SPIC->ctrlr0;
        uint32_t addr_len = SPIC->addr_length;

        signal = os_lock();
        spic_enable(DISABLE);
        spic_clr_multi_ch();
        spic_set_tx_mode();

        SPIC->addr_length = 0;

        spic_set_dr(DATA_BYTE, 0x7A);
        spic_enable(ENABLE);
        spic_wait_busy();
        spic_enable(DISABLE);
        is_suspend = false;
        SPIC->ctrlr0 = ctrlr0;
        SPIC->addr_length = addr_len;
        os_unlock(signal);
        DFU_PRINT_INFO0("RESUME OK");
    }
}

DATA_RAM_FUNCTION bool app_flash_wait_busy(void)
{
    uint8_t status1 = 0, status2 = 0;
    uint32_t ctr = 0;
    bool ret;

    while (ctr++ <= 0x10000)
    {
        signal = os_lock();
        ret = flash_cmd_rx(0x05, 1, &status1);
        ret &= flash_cmd_rx(0x35, 1, &status2);

        DFU_PRINT_INFO3("ret is %x,status1 is %x,status2 is %x", ret, status1, status2);
        if (!ret)
        {
            os_unlock(signal);
            goto wait_busy_fail;
        }

        if (!(status1 & BIT_STATUS_WIP) && !(status2 & BIT_STATUS_SUSPEND))
        {
            erase_in_progress = false;
            os_unlock(signal);
            return true;
        }
        os_unlock(signal);
        DFU_PRINT_INFO1("CNT is %x", ctr);
        os_delay(1);
    }

wait_busy_fail:
    return false;
}

//app_flash_wait_busy()
DATA_RAM_FUNCTION bool app_flash_cmd_tx(uint8_t cmd, uint8_t data_len, uint8_t *data_buf)
{
    bool retval = true;
    DFU_PRINT_INFO0("app_flash_cmd_tx");
    uint32_t ctrlr0 = SPIC->ctrlr0;
    uint32_t addr_len = SPIC->addr_length;

    spic_enable(DISABLE);
    spic_clr_multi_ch();
    spic_set_tx_mode();

    SPIC->addr_length = data_len;

    spic_set_dr(DATA_BYTE, cmd);

    while (data_len--)
    {
        spic_set_dr(DATA_BYTE, *data_buf++);
    }

    spic_enable(ENABLE);

    if (!spic_wait_busy())
    {
        retval = false;
    }

    spic_enable(DISABLE);

    os_unlock(signal);

    if (retval == true && !app_flash_wait_busy())
    {
        retval = false;
    }
    //restore ctrl0 and addr_len register
    SPIC->ctrlr0 = ctrlr0;
    SPIC->addr_length = addr_len;

    return retval;
}
//extern bool allowedDfuEnterDlps;
DATA_RAM_FUNCTION uint32_t flash_erase_sector(uint32_t addr)
{

    static uint8_t address[3];
    DFU_PRINT_INFO1("==> flash_erase_sector :%x \r\n", addr);
    address[0] = (addr >> 16) & 0xff;
    address[1] = (addr >> 8) & 0xff;
    address[2] = addr & 0xff;

    //allowedDfuEnterDlps = false;//dlps io driver store/restore in xip
    signal = os_lock();
    erase_in_progress = true;
    app_flash_erase_resume();
    flash_write_enable();
    app_flash_cmd_tx(CMD_SECTOR_ERASE, 3, address);
    //allowedDfuEnterDlps = true;
    //os_unlock(signal);
    return 0;
}
#else
DATA_RAM_FUNCTION bool app_flash_wait_busy(void)
{
    uint8_t status = 0;
    uint32_t ctr = 0;
    bool ret;

    while (ctr++ <= 0x10000)
    {
        ret = flash_cmd_rx(0x05, 1, &status);
        spic_enable(DISABLE);
        DBG_DIRECT("ret is %x,status is %x", ret, status);
        if (!ret)
        {
            goto wait_busy_fail;
        }

        if (!(status & BIT_STATUS_WIP))
        {
            return true;
        }
        DBG_DIRECT("CNT is %x", ctr);
        os_delay(1);
    }

wait_busy_fail:
    return false;
}

//app_flash_wait_busy()
DATA_RAM_FUNCTION bool app_flash_cmd_tx(uint8_t cmd, uint8_t data_len, uint8_t *data_buf)
{
    bool retval = true;
    DBG_DIRECT("app_flash_cmd_tx");
    uint32_t ctrlr0 = SPIC->ctrlr0;
    uint32_t addr_len = SPIC->addr_length;

    spic_enable(DISABLE);
    spic_clr_multi_ch();
    spic_set_tx_mode();

    SPIC->addr_length = data_len;

    spic_set_dr(DATA_BYTE, cmd);

    while (data_len--)
    {
        spic_set_dr(DATA_BYTE, *data_buf++);
    }

    spic_enable(ENABLE);

    if (!spic_wait_busy())
    {
        retval = false;
    }

    spic_enable(DISABLE);
    DBG_DIRECT("spic_wait_busy ..%x", retval);

    if (retval == true && !app_flash_wait_busy())
    {
        retval = false;
    }
    DBG_DIRECT("app_flash_wait_busy ..%x", retval);

    //restore ctrl0 and addr_len register
    SPIC->ctrlr0 = ctrlr0;
    SPIC->addr_length = addr_len;

    return retval;
}

DATA_RAM_FUNCTION uint32_t flash_erase_sector(uint32_t addr)
{

    static uint8_t address[3];
    DFU_PRINT_INFO1("==> flash_erase_sector :%x \r\n", addr);
    address[0] = (addr >> 16) & 0xff;
    address[1] = (addr >> 8) & 0xff;
    address[2] = addr & 0xff;

    flash_lock(FLASH_LOCK_USER_MODE_ERASE);
    flash_write_enable();
    app_flash_cmd_tx(0x20, 3, address);
    flash_unlock(FLASH_LOCK_USER_MODE_ERASE);
    return 0;
}
#endif //#if SUPPORT_ERASE_SUSPEND

/**
*  @brief: write specific image data feceived from host into flash.
*/
DATA_RAM_FUNCTION uint32_t sil_dfu_update(uint16_t signature, uint32_t offset, uint32_t length,
                                          uint32_t/*void*/ *p_void)
{
    uint32_t result = 0;
    uint32_t dfu_base_addr;
    uint32_t start_addr;
    uint32_t s_val;

    DFU_PRINT_INFO1("==> dfu_update length:%d \r\n", length);
    /*ASSERT((length % 4) == 0);*/
    /*ASSERT(p_void);*/

    if (length % 4)
    {
        result = __LINE__;
        goto L_Return;
    }

    if (p_void == 0)
    {
        result = __LINE__;
        goto L_Return;
    }
    /*get back up area address*/
    dfu_base_addr = get_temp_ota_bank_addr_by_img_id((T_IMG_ID)signature);
    if (dfu_base_addr == 0)
    {
        result = __LINE__;
        goto L_Return;
    }

    /* before erase temp image or write image to flash temp, check access length depend flash layout */
    if (!check_dfu_update_image_length(signature, offset, length, p_void, &result))
    {
        result = __LINE__;
        goto L_Return;
    }

    /*if it's start_packet*/
    if (offset == 0)
    {
        /*ASSERT(length>=sizeof(image_header_t));*/
        T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *) p_void;
        p_header->ctrl_flag.flag_value.not_ready = 0x1; /*make sure image is not ready, will use it later*/
        DFU_PRINT_TRACE3("dfu_update New Image Header:0x%08x, Signature:0x%08x, dfu_base_addr:0x%08x",
                         length, signature, dfu_base_addr);
    }

    if ((offset % FMC_SEC_SECTION_LEN) == 0)   //new page starts
    {
        flash_erase_sector(dfu_base_addr + offset);
    }
    else  // cross page
    {
        if ((offset / FMC_SEC_SECTION_LEN) != ((offset + length) / FMC_SEC_SECTION_LEN))
        {
            flash_erase_sector((dfu_base_addr + offset + length) & ~(FMC_SEC_SECTION_LEN - 1));
        }
    }
    start_addr = dfu_base_addr + offset;
    DFU_PRINT_TRACE1("*p_void:0x%08x", *p_void);
    s_val = flash_auto_read(start_addr | FLASH_OFFSET_TO_NO_CACHE);
    DFU_PRINT_TRACE1("*s_val:0x%08x", s_val);
    for (int i = 0; i < length; i = i + 4)
    {
        flash_auto_write(start_addr + i, *(uint32_t *)p_void);

        s_val = flash_auto_read(start_addr + i | FLASH_OFFSET_TO_NO_CACHE);
        DFU_PRINT_TRACE1("*s_val:0x%08x", s_val);
        if (s_val != *(uint32_t *)p_void)
        {
            DFU_PRINT_TRACE3("s_val:0x%08x, *p_void:0x%08x, i:0x%08x",
                             s_val, *(uint32_t *)p_void, i);
            result = __LINE__;
            os_delay(1000);
            goto L_Return;
        }
        else
        {
            p_void++;
        }

    }

//   ota_offset = offset + length; //for re-ota

L_Return:

    DFU_PRINT_INFO1("<==dfu_update result:%d \r\n", result);
    return result;
}

DATA_RAM_FUNCTION uint32_t sil_dfu_flash_erase(uint16_t signature, uint32_t offset)
{
    uint32_t result = 0;
    uint32_t dfu_base_addr;

    dfu_base_addr = get_temp_ota_bank_addr_by_img_id((T_IMG_ID)signature);
    if (dfu_base_addr == 0)
    {
        result = __LINE__;
        goto L_Return;
    }

    flash_erase_sector(dfu_base_addr + offset);
L_Return:
    DFU_PRINT_INFO1("<==sil_dfu_flash_erase result:%d \r\n", result);
    return result;
}

/**
*  @brief  time out handle for dfu exception stop
*/
void dfu_timeout_handle(void)
{
    DFU_PRINT_INFO0("[DFU] dfu time out handle.");
    dfuPara.is_dfu_progressing = false;
#ifdef NEW_SET_LATENCY_FUNC
    app_set_latency_status(LATENCY_DFU_PROC_BIT, LANTENCY_ON);
#else
    app_set_latency_status(LANTENCY_ON);
#endif
    /*param updata*/

}
/**
*  @brief  check dfu if working
*/
bool  dfu_check_working(void)
{
    return  dfuPara.is_dfu_progressing;
}
/**
 * @brief silence_dfu_service_handle_control_point_req
 *
 * @param length     control point cmd length.
 * @param p_value    control point cmd address..
 * @return None
*/
DATA_RAM_FUNCTION void  silence_dfu_service_handle_control_point_req(uint16_t length,
                                                                     uint8_t *p_value)
{
    T_DFU_CTRL_POINT dfu_control_point;
    uint8_t notif_data[9] = {0};

    dfu_control_point.opcode = * p_value;
    uint8_t *p = p_value + 1;

    DFU_PRINT_TRACE2("dfu_service_handle_control_point_req: opcode=0x%x, length=%d",
                     dfu_control_point.opcode, length);

    if (dfu_control_point.opcode >= DFU_OPCODE_MAX || dfu_control_point.opcode <= DFU_OPCODE_MIN)
    {
        notif_data[0] = DFU_OPCODE_NOTIF;
        notif_data[1] = dfu_control_point.opcode;
        notif_data[2] = 0xff;
        server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                         notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
        return;
    }

    /*check rcu state*/
    if (!RCU_STATE_CHECK())
    {
        DFU_PRINT_TRACE0("[DFU] err: rcu is not paired.");

        notif_data[0] = DFU_OPCODE_NOTIFICATION;
        notif_data[1] = dfu_control_point.opcode;
        notif_data[2] = DFU_ARV_FAIL_SYS_STATE_ERROR;
        server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                         notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
        return;
    }

    switch (dfu_control_point.opcode)
    {
    case DFU_OPCODE_START_DFU: //0x01
        if (length == DFU_LENGTH_START_DFU + 4)/*4 bytes is pending for encrypt*/
        {
            if (OTP->ota_with_encryption_data)
            {
                DFU_PRINT_TRACE1("Data before decryped: %b", TRACE_BINARY(16, p));
                hw_aes_init(OTP->aes_key, NULL, AES_MODE_ECB, OTP->ota_with_encryption_use_aes256);
                hw_aes_decrypt_16byte(p, p);
                DFU_PRINT_TRACE1("Data after decryped: %b", TRACE_BINARY(16, p));
            }

            dfu_control_point.p.start_dfu.ic_type = (*p);
            p += 1;
            dfu_control_point.p.start_dfu.secure_version = (*p);
            p += 1;
            LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.ctrl_flag.value, p);
            p += 2;
            LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.signature, p);
            p += 2;
            LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.crc16, p);
            p += 2;
            LE_ARRAY_TO_UINT32(dfu_control_point.p.start_dfu.image_length, p);

            DFU_PRINT_TRACE6("DFU_OPCODE_START_DFU: ic_type=0x%x, secure_version=0x%x, ctrl_flag.value=0x%x, signature=0x%x,crc16=0x%x*4Bytes, image_length=0x%x",
                             dfu_control_point.p.start_dfu.ic_type,
                             dfu_control_point.p.start_dfu.secure_version,
                             dfu_control_point.p.start_dfu.ctrl_flag.value,
                             dfu_control_point.p.start_dfu.signature,
                             dfu_control_point.p.start_dfu.crc16,
                             dfu_control_point.p.start_dfu.image_length
                            );
            dfuPara.ic_type = dfu_control_point.p.start_dfu.ic_type;
            dfuPara.ctrl_flag.value = dfu_control_point.p.start_dfu.ctrl_flag.value;
            dfuPara.signature = dfu_control_point.p.start_dfu.signature;
            // p_dfu->version = dfu_control_point.p.start_dfu.version;
            dfuPara.crc16 = dfu_control_point.p.start_dfu.crc16;
            dfuPara.image_length = dfu_control_point.p.start_dfu.image_length;

            dfuPara.image_total_length = dfuPara.image_length + IMG_HEADER_SIZE;

            /*check if start dfu fileds are vaild*/
            if ((dfuPara.ic_type != 0x05) || (dfuPara.signature < OTA) || (dfuPara.signature > AppData2)
                || (dfuPara.image_total_length > OTP->ota_bank0_size))
            {
                DFU_PRINT_TRACE0("[DFU] err: start dfu fileds are invalid.");

                notif_data[0] = DFU_OPCODE_NOTIFICATION;
                notif_data[1] = DFU_OPCODE_START_DFU;
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                 notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
                return;
            }

            unlock_flash_all();
            if (sil_dfu_update(dfuPara.signature, 0, DFU_HEADER_SIZE, (uint32_t *)&dfuPara.ic_type) == 0)
            {
                lock_flash();
                dfuPara.nCurOffSet += DFU_HEADER_SIZE;
            }
            else
            {
                lock_flash();
                dfu_reset(dfuPara.signature);
                dfu_fw_active_reset();
            }

            PROFILE_PRINT_INFO0("dfu_act_notify_start_dfu");

            notif_data[0] = DFU_OPCODE_NOTIFICATION;
            notif_data[1] = DFU_OPCODE_START_DFU;
            notif_data[2] = DFU_ARV_SUCCESS;
            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);

            /*ready for dfu*/
#ifdef NEW_SET_LATENCY_FUNC
            app_set_latency_status(LATENCY_DFU_PROC_BIT, LANTENCY_OFF);
#else
            app_set_latency_status(LANTENCY_OFF);
#endif
            dfuPara.is_dfu_progressing = true;
        }
        break;

    case DFU_OPCODE_RECEIVE_FW_IMAGE_INFO://0x02
        if (length == DFU_LENGTH_RECEIVE_FW_IMAGE_INFO)
        {
            LE_ARRAY_TO_UINT16(dfuPara.signature, p);
            p += 2;
            LE_ARRAY_TO_UINT32(dfuPara.nCurOffSet, p);
            if (dfuPara.nCurOffSet == 0)
            {
                ota_tmp_buf_used_size = 0;
                gSilDfuResendOffset = 0;
            }
            DFU_PRINT_TRACE2("DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: signature = 0x%x, cur_offset = %d",
                             dfuPara.signature, dfuPara.nCurOffSet);

        }
        else
        {
            DFU_PRINT_TRACE0("DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: invalid length");
        }

        break;

    case DFU_OPCODE_VALID_FW://0x03

        if (length == DFU_LENGTH_VALID_FW)
        {
            bool check_result;
            LE_ARRAY_TO_UINT16(dfuPara.signature, p);
            DFU_PRINT_TRACE1("DFU_OPCODE_VALID_FW: signature = 0x%x", dfuPara.signature);

            unlock_flash_all();
            flash_lock(FLASH_LOCK_USER_MODE_READ);
            check_result = dfu_check_checksum(dfuPara.signature);
            flash_unlock(FLASH_LOCK_USER_MODE_READ);
            lock_flash();
            DFU_PRINT_INFO1("dfu_act_notify_valid, check_result:%d (1: Success, 0: Fail)", check_result);

            if (check_result)
            {
                notif_data[2] = DFU_ARV_SUCCESS;
            }
            else
            {
                notif_data[2] = DFU_ARV_FAIL_CRC_ERROR;
            }
            notif_data[0] = DFU_OPCODE_NOTIF;
            notif_data[1] = DFU_OPCODE_VALID_FW;
            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
        }
        else
        {
            DFU_PRINT_TRACE0("DFU_OPCODE_VALID_FW: invalid length");
        }
        break;

    case DFU_OPCODE_ACTIVE_IMAGE_RESET://0x04
        /*notify bootloader to reset and use new image*/

        DFU_PRINT_TRACE0("DFU_OPCODE_ACTIVE_IMAGE_RESET:");

        if (is_ota_support_bank_switch())
        {
            uint32_t ota_addr;
            unlock_flash_all();
            ota_addr = get_header_addr_by_img_id(OTA);
            DFU_PRINT_INFO1("ota_addr:%x", ota_addr);
            flash_erase_locked(FLASH_ERASE_SECTOR, ota_addr & 0xffffff);
            lock_flash();
        }
        os_delay(app_global_data.conn_interval * (app_global_data.conn_latency + 1) * 4);
        unlock_flash_all();
        rcu_terminate_connection(DISCONN_REASON_SILENT_OTA);
        break;

    case DFU_OPCODE_SYSTEM_RESET://0x05
        DFU_PRINT_ERROR0("DFU_OPCODE_SYSTEM_RESET");
        os_delay(2000);
        WDG_SystemReset(RESET_ALL, DFU_SYSTEM_RESET);
        break;

    case DFU_OPCODE_REPORT_TARGET_INFO://0x06
        if (length == DFU_LENGTH_REPORT_TARGET_INFO)
        {
            LE_ARRAY_TO_UINT16(dfuPara.signature, p);
            PROFILE_PRINT_INFO1("dfuPara.signature is %x\r\n", dfuPara.signature);

            dfu_report_target_fw_info(dfuPara.signature, &dfuPara.origin_image_version,
                                      (uint32_t *)&dfuPara.nCurOffSet);

            notif_data[0] = DFU_OPCODE_NOTIFICATION;
            notif_data[1] = DFU_OPCODE_REPORT_TARGET_INFO;
            notif_data[2] = DFU_ARV_SUCCESS;

            LE_UINT16_TO_ARRAY(&notif_data[3], dfuPara.origin_image_version);
            LE_UINT32_TO_ARRAY(&notif_data[5], dfuPara.nCurOffSet);

            PROFILE_PRINT_INFO1("dfuPara.nCurOffSet is %x\r\n", dfuPara.nCurOffSet);

            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO, GATT_PDU_TYPE_NOTIFICATION);
        }
        else
        {
            DFU_PRINT_TRACE0("DFU_OPCODE_REPORT_TARGET_INFO: invalid length");
        }
        break;
    case DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ://0x07
        {
            notif_data[0] = DFU_OPCODE_NOTIF;
            notif_data[1] = DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ;


            if (length  == DFU_LENGTH_CONN_PARA_TO_UPDATE_REQ)
            {
                if (dfuPara.ota_conn_para_upd_in_progress)
                {
                    DFU_PRINT_ERROR0("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ bOTA_ConnParaUpdInProgress!!");
                    notif_data[2] = DFU_ARV_FAIL_OPERATION;
                    server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                     notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
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

                    if (le_update_conn_param(0, conn_interval_min, conn_interval_max, conn_latency,
                                             2000 / 10, conn_interval_min * 2 - 2, conn_interval_max * 2 - 2) == GAP_CAUSE_SUCCESS)
                    {
                        /* Connection Parameter Update Request sent successfully, means this procedure is in progress. */
                        dfuPara.ota_conn_para_upd_in_progress = true;
                        DFU_PRINT_INFO4("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ intMin=0x%x, intMax=0x%x, lat=0x%x, supto=0x%x.",
                                        conn_interval_min, conn_interval_max, conn_latency, superv_tout);
                    }
                    else
                    {
                        notif_data[2] = DFU_ARV_FAIL_OPERATION;
                        server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                         notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
                    }
                }
            }
            else
            {
                /*TODO: to be masked.*/
                DFU_PRINT_ERROR1("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ length = %d Error!", length);
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                 notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
            }
        }
        break;
    case DFU_OPCODE_BUFFER_CHECK_EN: //0x09
        {
            DFU_PRINT_TRACE1("DFU_OPCODE_BUFFER_CHECK_EN, MTUSIZE is %d", app_global_data.mtu_size);
            notif_data[0] = DFU_OPCODE_NOTIF;
            notif_data[1] = DFU_OPCODE_BUFFER_CHECK_EN;
            notif_data[2] = 0x01;
            LE_UINT16_TO_ARRAY(&notif_data[3], DFU_TEMP_BUFFER_SIZE);
            LE_UINT16_TO_ARRAY(&notif_data[5], app_global_data.mtu_size);
            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
        }
        break;
    case DFU_OPCODE_REPORT_BUFFER_CRC:        //0x0a
        {
            LE_ARRAY_TO_UINT16(mBufSize, p);
            p += 2;
            LE_ARRAY_TO_UINT16(mCrcVal, p);
            DFU_PRINT_TRACE2("DFU_OPCODE_REPORT_BUFFER_CRC mBufferSize is 0x%x,mCrc is 0x%x", mBufSize,
                             mCrcVal);
            silence_BufferCheckProc(mBufSize, mCrcVal);
        }
        break;
    case DFU_OPCODE_RECEIVE_IC_TYPE://0x0b
        {
            uint8_t ic_type;
            notif_data[0] = DFU_OPCODE_NOTIF;
            notif_data[1] = DFU_OPCODE_RECEIVE_IC_TYPE;
            if (dfu_report_target_ic_type(OTA, &ic_type))
            {
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                notif_data[3] =  0xff;
            }
            else
            {
                notif_data[2] = DFU_ARV_SUCCESS;
                notif_data[3] =  ic_type;
            }
            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, 4, GATT_PDU_TYPE_NOTIFICATION);
        }
        break;
#if USER_DATA_COPY_EN
    case DFU_OPCODE_COPY_IMG://0x0c
        {
            uint32_t dlAddress, dlSize;
            LE_ARRAY_TO_UINT16(dfuPara.signature, p);
            p += 2;
            LE_ARRAY_TO_UINT32(dlAddress, p);
            p += 4;
            LE_ARRAY_TO_UINT32(dlSize, p);
            DFU_PRINT_TRACE2("DFU_OPCODE_COPY_IMG dlAddress is 0x%x,dlSize is 0x%x", dlAddress,
                             dlSize);

            notif_data[0] = DFU_OPCODE_NOTIF;
            notif_data[1] = DFU_OPCODE_COPY_IMG;

            if (sil_dfu_copy_img(dfuPara.signature, dlAddress, dlSize))
            {
                notif_data[2] = DFU_ARV_SUCCESS;
            }
            else
            {
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
            }
            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, 3, GATT_PDU_TYPE_NOTIFICATION);
        }
        break;
#endif//#if USER_DATA_COPY_EN
    default:
        {
            DFU_PRINT_TRACE1("dfu_service_handle_control_point_req: Unknown Opcode=0x%x",
                             dfu_control_point.opcode
                            );
        }

        break;

    }
}

#if USER_DATA_COPY_EN
/**
*  @brief  copy user data from inactive bank data area to active one.
*/
bool sil_dfu_copy_img(uint16_t signature, uint32_t dlAddress, uint32_t dlSize)
{
    uint32_t error_code = 0;
    uint32_t source_base_addr;
    uint32_t dest_base_addr;
    int remain_size = dlSize;
    uint32_t s_val;
    uint32_t dlOffset, tmp_offset;

    if ((signature != AppData1) && (signature != AppData2))
    {
        error_code = __LINE__;
        goto L_Return;
    }
    if (dlAddress % 0x1000)
    {
        error_code = __LINE__;
        goto L_Return;
    }

    source_base_addr = get_active_ota_bank_addr() & 0xffffff;

    if (flash_get_bank_addr(FLASH_OTA_BANK_0) == get_active_ota_bank_addr())
    {
        dest_base_addr = flash_get_bank_addr(FLASH_OTA_BANK_1) & 0xffffff;
    }
    else
    {
        dest_base_addr = flash_get_bank_addr(FLASH_OTA_BANK_0) & 0xffffff;
    }
    if ((source_base_addr % 4096) || (dest_base_addr % 4096))
    {
        error_code = __LINE__;
        goto L_Return;
    }
    if (dest_base_addr >= dlAddress)
    {
        error_code = __LINE__;
        goto L_Return;
    }
    dlOffset = dlAddress - dest_base_addr;
    tmp_offset = dlOffset;
    if (dlOffset % 4096)
    {
        error_code = __LINE__;
        goto L_Return;
    }
    T_IMG_HEADER_FORMAT *p_data_header;
    p_data_header = (T_IMG_HEADER_FORMAT *)(source_base_addr + dlOffset);
    if (p_data_header->ctrl_header.image_id != signature)
    {
        error_code = __LINE__;
        goto L_Return;
    }

    while (remain_size > 0)
    {
        if (!((dest_base_addr + tmp_offset) % 4096)) //must 4k align
        {
            flash_erase_sector(dest_base_addr + tmp_offset);
        }
        if (remain_size > 2048)
        {
            memcpy(g_pOtaTempBufferHead, (uint8_t *)(source_base_addr + tmp_offset), 2048);
            if (remain_size ==  dlSize)
            {
                T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *) g_pOtaTempBufferHead;
                p_header->ctrl_flag.flag_value.not_ready = 0x1; /*make sure image is not ready, will use it later*/
            }
            for (int i = 0; i < 2048; i = i + 4)
            {
                flash_auto_write(dest_base_addr + tmp_offset + i, *(uint32_t *)g_pOtaTempBufferHead);

                s_val = flash_auto_read(dest_base_addr + tmp_offset + i | FLASH_OFFSET_TO_NO_CACHE);
                if (s_val != *(uint32_t *)g_pOtaTempBufferHead)
                {
                    DFU_PRINT_TRACE3("s_val:0x%08x, *p_void:0x%08x, i:0x%08x",
                                     s_val, *(uint32_t *)g_pOtaTempBufferHead, i);
                    error_code = __LINE__;
                    goto L_Return;
                }
                else
                {
                    g_pOtaTempBufferHead += 4;
                }
            }


            remain_size -= 2048;
        }
        else
        {
            memcpy(g_pOtaTempBufferHead, (uint8_t *)(source_base_addr + tmp_offset), remain_size);
            for (int i = 0; i < remain_size; i = i + 4)
            {
                flash_auto_write(dest_base_addr + tmp_offset + i, *(uint32_t *)g_pOtaTempBufferHead);

                s_val = flash_auto_read(dest_base_addr + tmp_offset + i | FLASH_OFFSET_TO_NO_CACHE);
                if (s_val != *(uint32_t *)g_pOtaTempBufferHead)
                {
                    DFU_PRINT_TRACE3("s_val:0x%08x, *p_void:0x%08x, i:0x%08x",
                                     s_val, *(uint32_t *)g_pOtaTempBufferHead, i);
                    error_code = __LINE__;
                    goto L_Return;
                }
                else
                {
                    g_pOtaTempBufferHead += 4;
                }
            }
            remain_size = 0;
        }
        tmp_offset += 2048;
    }

L_Return:
    DFU_PRINT_INFO1("<====dfu_copy_img  error_code:%d", error_code);
    if (error_code)
    {
        return false;
    }
    return true;
}
#endif//#if USER_DATA_COPY_EN
/**
 * @brief silence_BufferCheckProc
 *
 * @param _mBufferSize     size for buffer check.
 * @param _mCrc            calced buffer crc value.
 * @return None
*/
DATA_RAM_FUNCTION void silence_BufferCheckProc(uint16_t _mBufferSize, uint16_t _mCrc)
{
    uint16_t offset = 0;
    uint8_t notif_data[7] = {0};
    notif_data[0] = DFU_OPCODE_NOTIF;
    notif_data[1] = DFU_OPCODE_REPORT_BUFFER_CRC;

    if (mBufSize > DFU_TEMP_BUFFER_SIZE)
    {
        //invalid para
        ota_tmp_buf_used_size = 0;
        notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
        LE_UINT32_TO_ARRAY(&notif_data[3], dfuPara.nCurOffSet);
        server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                         notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
        return;
    }
    if (ota_tmp_buf_used_size == mBufSize ||
        dfuPara.nCurOffSet + ota_tmp_buf_used_size == dfuPara.image_total_length
       )
    {
        if (dfu_checkbufcrc(g_pOtaTempBufferHead, ota_tmp_buf_used_size, _mCrc))     //crc error

        {
            ota_tmp_buf_used_size = 0;
            notif_data[2] = DFU_ARV_FAIL_CRC_ERROR;
            LE_UINT32_TO_ARRAY(&notif_data[3], dfuPara.nCurOffSet);
            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
            return;
        }

        else //crc ok
        {
            if (OTP->ota_with_encryption_data)
            {
                //aes
                do
                {
                    if ((ota_tmp_buf_used_size - offset) >= 16)
                    {
                        hw_aes_init(OTP->aes_key, NULL, AES_MODE_ECB, OTP->ota_with_encryption_use_aes256);
                        hw_aes_decrypt_16byte(g_pOtaTempBufferHead + offset, g_pOtaTempBufferHead + offset);
                        offset += 16;
                    }
                    else
                    {
                        break;
                    }
                }
                while (1);
            }
            unlock_flash_all();
            DFU_PRINT_TRACE1(" p_dfu->cur_offset, %x", dfuPara.nCurOffSet);

            uint32_t result = sil_dfu_update(dfuPara.signature, dfuPara.nCurOffSet, ota_tmp_buf_used_size,
                                             (uint32_t *)g_pOtaTempBufferHead);

            if (result == 0)
            {
                dfuPara.nCurOffSet += ota_tmp_buf_used_size;

                if ((dfuPara.nCurOffSet - gSilDfuResendOffset) >= SECTOR_SIZE)
                {
                    gSilDfuResendOffset += SECTOR_SIZE;
                }
                ota_tmp_buf_used_size = 0;
                notif_data[2] = DFU_ARV_SUCCESS; //valid
                LE_UINT32_TO_ARRAY(&notif_data[3], dfuPara.nCurOffSet);
                server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                 notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
                lock_flash();
                return;
            }
            else
            {
                uint32_t ret = 0;
                uint32_t cnt = 0;
                do
                {
                    dfu_flash_erase(dfuPara.signature, gSilDfuResendOffset);
                    //delay_ms_asm(200);
                    ret = dfu_flash_check_blank(dfuPara.signature, gSilDfuResendOffset, SECTOR_SIZE);
                    if (ret)
                    {
                        cnt++;
                    }
                    else
                    {
                        break;
                    }
                    if (cnt >= 3)     //check 0xff failed,erase failed
                    {
                        //erase error
                        ota_tmp_buf_used_size = 0;
                        dfuPara.nCurOffSet =  gSilDfuResendOffset;
                        notif_data[2] = DFU_ARV_FAIL_ERASE_ERROR;
                        LE_UINT32_TO_ARRAY(&notif_data[3], dfuPara.nCurOffSet);
                        server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                         notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
                        lock_flash();
                        return;
                    }
                }
                while (1);

                if ((dfuPara.nCurOffSet - gSilDfuResendOffset) > SECTOR_SIZE) //need erase two sector
                {
                    cnt = 0;
                    do
                    {
                        //erase sector :addr ~ gDfuResendOffset+SECTOR_SIZE;
                        dfu_flash_erase(dfuPara.signature, gSilDfuResendOffset + SECTOR_SIZE);
                        ret = dfu_flash_check_blank(dfuPara.signature, gSilDfuResendOffset + SECTOR_SIZE, SECTOR_SIZE);
                        if (ret)
                        {
                            cnt++;
                        }
                        else
                        {
                            break;
                        }
                        if (cnt >= 3)     //check 0xff failed,erase failed
                        {
                            //erase error
                            ota_tmp_buf_used_size = 0;
                            dfuPara.nCurOffSet =  gSilDfuResendOffset;
                            notif_data[2] = DFU_ARV_FAIL_ERASE_ERROR;
                            LE_UINT32_TO_ARRAY(&notif_data[3], dfuPara.nCurOffSet);
                            server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                             notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
                            lock_flash();
                            return;
                        }
                    }
                    while (1);
                }
                //erase ok
                ota_tmp_buf_used_size = 0;
                dfuPara.nCurOffSet =  gSilDfuResendOffset;
                notif_data[2] = DFU_ARV_FAIL_PROG_ERROR;
                LE_UINT32_TO_ARRAY(&notif_data[3], dfuPara.nCurOffSet);
                server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                 notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
                lock_flash();
                return;
            }
        }
    }
    else
    {
        DFU_PRINT_TRACE2("ota_temp_buf_used_size is %d,_mBufferSize=%d", ota_tmp_buf_used_size,
                         _mBufferSize);
        //flush buffer.
        ota_tmp_buf_used_size = 0;
        notif_data[2] = DFU_ARV_FAIL_LENGTH_ERROR;
        LE_UINT32_TO_ARRAY(&notif_data[3], dfuPara.nCurOffSet);
        server_send_data(0, gDfuServiceId, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                         notif_data, 7, GATT_PDU_TYPE_NOTIFICATION);
        return;
    }
}


/**
 * @brief silence_dfu_service_handle_packet_req
 *
 * @param length     data reviewed length.
 * @param p_value    data receive point address.
 * @return None
*/
DATA_RAM_FUNCTION void silence_dfu_service_handle_packet_req(uint16_t length, uint8_t *p_value)
{
    DFU_PRINT_TRACE4("dfu_service_handle_packet_req: length=%d, cur_offset =%d, ota_temp_buf_used_size = %d,image_total_length= %d",
                     length,
                     dfuPara.nCurOffSet,
                     ota_tmp_buf_used_size,
                     dfuPara.image_total_length
                    );

    if (dfuPara.nCurOffSet + ota_tmp_buf_used_size + length > dfuPara.image_total_length)
    {
        DFU_PRINT_TRACE4("dfu_service_handle_packet_req: p_dfu->cur_offset=%d, ota_temp_buf_used_size =%d, length= %d, image_total_length = %d ",
                         dfuPara.nCurOffSet,
                         ota_tmp_buf_used_size,
                         length,
                         dfuPara.image_total_length
                        );
        PROFILE_PRINT_INFO0("dfu_act_reset_and_activate");
    }
    else
    {
#if OTA_BUF_CHECK_EN
        {
            memcpy(g_pOtaTempBufferHead + ota_tmp_buf_used_size, p_value, length);
            ota_tmp_buf_used_size += length;
        }
#else
        if (length >= 16)
        {
            if (OTP->ota_with_encryption_data)
            {
                uint16_t offset = 0;
                do
                {
                    if ((length - offset) >= 16)
                    {
                        hw_aes_init(OTP->aes_key, NULL, AES_MODE_ECB, OTP->ota_with_encryption_use_aes256);
                        hw_aes_decrypt_16byte(p_value + offset, p_value + offset);
                        offset += 16;
                    }
                    else
                    {
                        break;
                    }
                }
                while (1);
            }
        }

        memcpy(g_pOtaTempBufferHead + ota_tmp_buf_used_size, p_value, length);
        ota_tmp_buf_used_size += length;

        if (ota_tmp_buf_used_size == (DFU_TEMP_BUFFER_SIZE - DFU_TEMP_BUFFER_SIZE % 20) ||
            dfuPara.nCurOffSet + ota_tmp_buf_used_size == dfuPara.image_total_length
           )
        {
            unlock_flash_all();
            if (sil_dfu_update(dfuPara.signature, dfuPara.nCurOffSet, ota_tmp_buf_used_size,
                               (uint32_t *)g_pOtaTempBufferHead) == 0)
            {
                lock_flash();
            }
            else
            {
                /*eflash write fail, we should restart ota procedure.*/
                lock_flash();
                dfu_reset(dfuPara.signature);
                dfu_fw_active_reset();
            }
            dfuPara.nCurOffSet += ota_tmp_buf_used_size;
            ota_tmp_buf_used_size = 0;
        }
#endif
    }
}

/**
 * @brief write characteristic data from service.
 *
 * @param ServiceID          ServiceID to be written.
 * @param iAttribIndex       Attribute index of characteristic.
 * @param wLength            length of value to be written.
 * @param pValue             value to be written.
 * @return Profile procedure result
*/
DATA_RAM_FUNCTION T_APP_RESULT dfu_attr_write_cb(uint8_t conn_id, uint8_t service_id,
                                                 uint16_t attrib_index,
                                                 T_WRITE_TYPE write_type,
                                                 uint16_t length, uint8_t *p_value, P_FUN_WRITE_IND_POST_PROC *p_write_ind_post_proc)
{
    T_APP_RESULT  wCause = APP_RESULT_SUCCESS;

    RESET_TIME_OUT_TIMER();

    if (attrib_index == INDEX_DFU_CONTROL_POINT_CHAR_VALUE)
    {
        silence_dfu_service_handle_control_point_req(/*(T_DFU *)p_dfu, */length, p_value);
    }
    else if (attrib_index == INDEX_DFU_PACKET_VALUE)
    {
        silence_dfu_service_handle_packet_req(/*(T_DFU *)p_dfu, */length, p_value);
    }
    else
    {
        PROFILE_PRINT_INFO1("!!!dfuServiceAttribPut Fail: iAttribIndex=%d.", attrib_index);

        return APP_RESULT_ATTR_NOT_FOUND;

    }
    return wCause;
}

/**
 * @brief update CCCD bits from stack.
 *
 * @param ServiceId          Service ID.
 * @param Index          Attribute index of characteristic data.
 * @param wCCCBits         CCCD bits from stack.
 * @return None
*/
void dfu_cccd_update_cb(uint8_t conn_id, T_SERVER_ID service_id, uint16_t index, uint16_t ccc_bits)
{
    TDFU_CALLBACK_DATA callback_data;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
    callback_data.conn_id = conn_id;
    bool bHandle = true;
    PROFILE_PRINT_INFO2("DfuCccdUpdateCb  Index = %d wCCCDBits %x", index, ccc_bits);
    switch (index)
    {
    case INDEX_DFU_CHAR_CCCD_INDEX:
        {
            if (ccc_bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY)
            {
                // Enable Notification
                callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
                callback_data.msg_data.notification_indification_index = DFU_NOTIFY_ENABLE;
            }
            else
            {
                callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
                callback_data.msg_data.notification_indification_index = DFU_NOTIFY_DISABLE;
            }
            break;
        }
    default:
        {
            bHandle = false;
            break;
        }

    }
    /* Notify Application. */
    if (pfnDfuExtendedCB && (bHandle == true))
    {
        pfnDfuExtendedCB(service_id, (void *)&callback_data);
    }

    return;
}

/**
 * @brief OTA ble Service Callbacks.
*/
const T_FUN_GATT_SERVICE_CBS DfuServiceCBs =
{
    NULL,   // Read callback function pointer
    dfu_attr_write_cb,  // Write callback function pointer
    dfu_cccd_update_cb                    // CCCD update callback function pointer
};

/**
 * @brief  add OTA ble service to application.
 *
 * @param  pFunc          pointer of app callback function called by profile.
 * @return service ID auto generated by profile layer.
 * @retval ServiceId
*/
uint8_t dfu_add_service(void *pFunc)
{
    uint8_t ServiceId;

    if (false == server_add_service(&ServiceId,
                                    (uint8_t *)silence_dfu_service,
                                    sizeof(silence_dfu_service),
                                    DfuServiceCBs))
    {
        PROFILE_PRINT_ERROR1("DFUService_AddService: ServiceId %d", ServiceId);
        ServiceId = 0xff;
        return ServiceId;
    }
    pfnDfuExtendedCB = (P_FUN_SERVER_GENERAL_CB)pFunc;
    /*dfu timer init*/
    dfu_timer_init();
    g_pOtaTempBufferHead = OtaTempBufferHead;
    return ServiceId;
}


#endif  // #if SUPPORT_SILENT_OTA
