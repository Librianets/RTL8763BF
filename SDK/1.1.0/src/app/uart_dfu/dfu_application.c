/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      dfu_application.c
* @brief     dfu application implementation
* @details   dfu application implementation
* @author    ken_mei
* @date      2017-07-03
* @version   v0.1
* *********************************************************************************************************
*/
#include <trace.h>
#include <string.h>
#include <gap_msg.h>
#include <board.h>
#include <app_msg.h>
#include "os_timer.h"
#include "dfu_application.h"
#include "rtl876x_wdg.h"
#include "rtl876x_uart.h"
#include "uart_application.h"
#include "trace.h"
#include "dfu_service.h"
#include "gap_msg.h"
#include "app_msg.h"
#include "flash_device.h"
#include "dfu_flash.h"
#include "patch_header_check.h"
#include "board.h"
#include "flash_adv_cfg.h"
#include "os_sched.h"
#include "mem_config.h"
#include "otp.h"
#include "rtl876x_hw_aes.h"
#include "patch_header_check.h"
#include "user_flash_driver.h"
#include "os_sched.h"
#include "app_section.h"
#include "os_sync.h"
#include "platform_utils.h"
#include "flash_adv_cfg.h"


extern ReceiveBufStruct IO_Receive;

#define DFU_TEMP_BUFFER_SIZE 2048

extern bool OtaActRstPending;//for device active disconnect flg,then reset.

uint8_t *g_pOtaTempBufferHead;
uint8_t TempBufferHead[DFU_TEMP_BUFFER_SIZE];
uint16_t g_OtaTempBufferUsedSize;
uint8_t erase_error = 0;
TDFU_CB dfuPara;
uint16_t ota_tmp_buf_used_size = 0;
uint8_t prev_bp_lv = 0xfe;
bool flstatus = false;//flash lock status
uint32_t signal;//os lock signal
bool hw_aes_decrypt_16byte(uint8_t *input, uint8_t *output);
void silence_BufferCheckProc(uint16_t _mBufferSize, uint16_t _mCrc);
uint32_t dfu_flash_check_blank(uint16_t signature, uint32_t offset, uint16_t nSize);
void peripheral_handle_gap_msg(T_IO_MSG  *p_gap_msg);
uint8_t silence_dfu_service_handle_packet_req(uint16_t length, uint8_t *p_value);
void dfu_service_handle(void);
/******************************************************************
 * @fn          app_handle_io_msg
 * @brief      All the application events are pre-handled in this function.
 *                All the IO MSGs are sent to this function, Then the event handling function
 *                shall be called according to the MSG type.
 *
 * @param    io_driver_msg_recv  - bee io msg data
 * @return     void
 */
void app_handle_io_msg(T_IO_MSG io_driver_msg_recv)
{
    uint16_t msg_type = io_driver_msg_recv.type;

    switch (msg_type)
    {
    case IO_MSG_TYPE_BT_STATUS:
        {
        }
        break;
    case IO_MSG_TYPE_UART:
        {
            dfu_service_handle();
            APP_PRINT_INFO0("IO_MSG_TYPE_UART");
        }
        break;
    default:
        break;
    }
}

void getpacket(uint8_t *packet_buffer, uint16_t bufferlen)
{
    uint32_t flags;
    flags = os_lock();//enter critical section

    if (IO_Receive.ReadOffset + bufferlen <= RECEIVE_BUF_MAX_LENGTH)
    {
        memcpy(packet_buffer, IO_Receive.buf + IO_Receive.ReadOffset, bufferlen);
        IO_Receive.ReadOffset += bufferlen;
        if (IO_Receive.ReadOffset == RECEIVE_BUF_MAX_LENGTH)
        {
            IO_Receive.ReadOffset = 0;
        }
    }
    else
    {
        uint16_t len1 = RECEIVE_BUF_MAX_LENGTH - IO_Receive.ReadOffset;
        memcpy(packet_buffer, IO_Receive.buf + IO_Receive.ReadOffset, len1);
        IO_Receive.ReadOffset = 0;
        memcpy(packet_buffer + len1, IO_Receive.buf + IO_Receive.ReadOffset, bufferlen - len1);
        IO_Receive.ReadOffset += bufferlen - len1;
    }

    IO_Receive.datalen -= bufferlen;
    os_unlock(flags); //exit critical section
}
void uart_send_rsp(uint8_t *buf, uint32_t length)
{
    uint32_t i;
    uint32_t count = length / 16;
    uint32_t remainder = length % 16;
    for (i = 0; i < count; i++)
    {
        UART_SendData(UART, &buf[16 * i], 16);
        while (UART_GetFlagState(UART, UART_FLAG_THR_TSR_EMPTY) != SET);
    }
    /* send left bytes */
    UART_SendData(UART, &buf[16 * i], remainder);
    while (UART_GetFlagState(UART, UART_FLAG_THR_TSR_EMPTY) != SET);
}
uint16_t crc16_check(uint8_t *buf, uint16_t len, uint16_t value)
{

    uint16_t b = 0xA001;
    bool reminder = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        value ^= buf[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            reminder = value % 2;
            value >>= 1;
            if (reminder == 1)
            {
                value ^= b;
            }
        }
    }
    return value;
}
void send_rsp(uint8_t opcode, uint16_t len, uint8_t *payload)
{
    uint8_t rsp_event[20] = {};
    rsp_event[0] = DFU_OPCODE_EVENT;
    rsp_event[1] = opcode;
    rsp_event[2] = DFU_OPCODE_EVENT_HEADER;
    rsp_event[3] = len & 0xff;
    rsp_event[4] = len >> 8;
    memcpy(rsp_event + 5, (uint8_t *)payload, len);
    uint16_t crc_value = crc16_check(rsp_event, len + 5, 0xFFFF);
    rsp_event[len + 5] = crc_value & 0xff;
    rsp_event[len + 6] = crc_value >> 8;
    uart_send_rsp(rsp_event, len + 7);
}
void dfu_service_handle()
{
    uint8_t opcode = 0;
    uint16_t length = 0;
    uint16_t    crc_calc_value = 0;
    uint8_t header_err[3] = {0x01, 0x12, 0x01};
    uint8_t packet_header[5];
    uint8_t crc_buffer[2];
    if (IO_Receive.datalen == 0)
    {
        return;
    }
    getpacket(packet_header, 5);
    opcode = packet_header[1];
    header_err[0] = opcode;
    length = packet_header[4] << 8 | packet_header[3];
    APP_PRINT_INFO2("opcode = 0x%x,Receive length = 0x%x", opcode, length);
    if (length + 2 > IO_Receive.datalen)
    {
        APP_PRINT_INFO1("Waiting data IO_Receive.datalen = %d", IO_Receive.datalen);
        return;
    }

    if (packet_header[0] != 0x03)
    {

        header_err[2] = 0x02;
        send_rsp(opcode, 3, header_err);
        DFU_PRINT_INFO1("start byte err =0x%x \r\n", packet_header[0]);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;
    }
    else if (length + 2 != IO_Receive.datalen)
    {
        header_err[2] = 0x03;
        send_rsp(opcode, 3, header_err);
        DFU_PRINT_INFO2("payload length err! length+2=%d ,IO_Receive.datalen=%d\r\n", length + 2,
                        IO_Receive.datalen);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;

    }
    else if ((opcode >= DFU_OPCODE_MAX || opcode <= DFU_OPCODE_MIN) && packet_header[2] != 0x12)
    {
        header_err[2] = 0x04;
        send_rsp(opcode, 3, header_err);
        DFU_PRINT_INFO2("opcode err opcode=0x%x%x \r\n", packet_header[2], opcode);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;
    }
    getpacket(TempBufferHead + ota_tmp_buf_used_size, length);
    crc_calc_value = crc16_check(packet_header, 5, 0xFFFF);
    crc_calc_value = crc16_check(TempBufferHead + ota_tmp_buf_used_size, length, crc_calc_value);
    crc_calc_value = (crc_calc_value >> 8) | (crc_calc_value & 0xff) << 8;
    APP_PRINT_INFO1("crc_calc_value = 0x%x", crc_calc_value);
    getpacket(crc_buffer, 2);
    uint16_t crc16_value = crc_buffer[0] << 8 | crc_buffer[1];
    if (crc16_value != crc_calc_value)
    {
        APP_PRINT_INFO1("crc16_rev err value = 0x%x", crc16_value);
        header_err[2] = 0x01;
        send_rsp(opcode, 3, header_err);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;
    }

    dfu_service_handle_control_point_req(opcode, length, TempBufferHead);
}

bool unlock_flash_all(void)
{
    prev_bp_lv = 0;
    APP_PRINT_INFO0("**********[Flash Set] Flash unlock ***********");
    if (FLASH_SUCCESS == flash_sw_protect_unlock_by_addr_locked((0x00800000), &prev_bp_lv))
    {
        APP_PRINT_INFO1("[Flash Set] Flash unlock address = 0x800000, prev_bp_lv = %d", prev_bp_lv);
        return true;
    }
    return false;
}
void lock_flash(void)
{
    if (prev_bp_lv != 0xfe)
    {
        flash_set_block_protect_locked(prev_bp_lv);
    }
}
DATA_RAM_FUNCTION bool app_flash_wait_busy(void)
{
    uint8_t status = 0;
    uint32_t ctr = 0;
    bool ret;

    while (ctr++ <= 0x100000)
    {
        ret = flash_cmd_rx(0x05, 1, &status);

//        DFU_PRINT_INFO2("ret is %x,status is %x", ret, status);
        if (!ret)
        {
            goto wait_busy_fail;
        }

        if (!(status & BIT_STATUS_WIP))
        {
            return true;
        }
//        DFU_PRINT_INFO1("CNT is %x", ctr);
    }

wait_busy_fail:
    return false;
}

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
    DFU_PRINT_INFO1("app_flash_wait_busy ..%x", retval);
    if (retval == true && !app_flash_wait_busy())
    {
        retval = false;
    }


    //restore ctrl0 and addr_len register
    SPIC->ctrlr0 = ctrlr0;
    SPIC->addr_length = addr_len;

    return retval;
}
/**
 * @brief erase a sector of the flash.
 *
 * @param  addr          flash addr in sector to be erase.
 * @return  0 if erase successfully, error line number otherwise
*/
DATA_RAM_FUNCTION uint32_t flash_erase_sector(uint32_t addr)
{

    static uint8_t address[3];
    DFU_PRINT_INFO1("==> flash_erase_sector :%x \r\n", addr);
    address[0] = (addr >> 16) & 0xff;
    address[1] = (addr >> 8) & 0xff;
    address[2] = addr & 0xff;

    flstatus = flash_lock(FLASH_LOCK_USER_MODE_ERASE);//signal = os_lock();
    flash_write_enable();
    app_flash_cmd_tx(0x20, 3, address);
    flash_unlock(FLASH_LOCK_USER_MODE_ERASE);//os_unlock(signal);
    return 0;
}
/**
 * @brief  write specified image data with specified length to flash
 * @param  signature          signature to identify FW.
 * @param  offset             offset of the image.
 * @param  length             length of data.
 * @param  p_void             pointer to data.
 * @return 0 if write FW image successfully, error line number otherwise
*/
uint32_t sil_dfu_update(uint16_t signature, uint32_t offset, uint32_t length,
                        uint32_t/*void*/ *p_void)
{
    uint32_t result = 0;
    uint32_t dfu_base_addr;
    uint32_t start_addr;
    uint32_t s_val;

    DFU_PRINT_INFO1("==> dfu_update length:%d \r\n", length);

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
    DFU_PRINT_INFO1("==> dfu_update dfu_base_addr:0x%x \r\n", dfu_base_addr);
    if (dfu_base_addr == 0)
    {
        result = __LINE__;
        goto L_Return;
    }
//      dfu_base_addr&= 0xffffff;
    /*if it's start_packet*/
    if (offset == 0)
    {
        /*ASSERT(length>=sizeof(image_header_t));*/
        T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *) p_void;
        p_header->ctrl_flag.flag_value.not_ready = 0x1; /*make sure image is not ready, will use it later*/
        APP_PRINT_INFO3("dfu_update New Image Header:0x%08x, Signature:0x%08x, dfu_base_addr:0x%08x",
                        length, signature, dfu_base_addr);
    }

    if ((offset % FMC_SEC_SECTION_LEN) == 0)   //new page starts
    {
        erase_error = flash_erase_sector(dfu_base_addr + offset);
    }
    else  // cross page
    {
        if ((offset / FMC_SEC_SECTION_LEN) != ((offset + length) / FMC_SEC_SECTION_LEN))
        {
            erase_error = flash_erase_sector((dfu_base_addr + offset + length) & ~(FMC_SEC_SECTION_LEN - 1));
        }
    }
    start_addr = dfu_base_addr + offset;
    APP_PRINT_INFO3("start_addr:0x%08x, *p_void:0x%08x, *p_void:0x%08x",
                    start_addr, *p_void, *(p_void + 1));
    for (int i = 0; i < length; i = i + 4)
    {
        flash_auto_write(start_addr + i, *(uint32_t *)p_void);

        s_val = flash_auto_read(start_addr + i | FLASH_OFFSET_TO_NO_CACHE);

        if (s_val != *(uint32_t *)p_void)
        {
            APP_PRINT_INFO3("s_val:0x%08x, *p_void:0x%08x, i:0x%08x",
                            s_val, *(uint32_t *)p_void, i);
            result = __LINE__;
            goto L_Return;
        }
        else
        {
            p_void++;
        }
    }

L_Return:

    DFU_PRINT_INFO1("<==dfu_update result:%d \r\n", result);
    return result;
}

DATA_RAM_FUNCTION void dfu_service_handle_control_point_req(uint8_t opcode, uint16_t length,
                                                            uint8_t *p_value)
{
    T_DFU_CTRL_POINT dfu_control_point;
    uint8_t notif_data[20] = {0};
    uint8_t ic_type;
    uint8_t result = 0;
    uint8_t old_bp_lv = 0x02;
    bool check_result;
    uint8_t status = DFU_SUCCESS;
    APP_PRINT_INFO2("dfu_service_handle_control_point_req: opcode=0x%x, length=%d",
                    opcode, length);

    switch (opcode)
    {
    case DFU_OPCODE_START_DFU:
#if HW_AES_KEY
        if (OTP->ota_with_encryption_data)
        {
            DFU_PRINT_TRACE1("Data before decryped: %b", TRACE_BINARY(16, p_value));
            hw_aes_init(OTP->aes_key, NULL, AES_MODE_ECB, OTP->ota_with_encryption_use_aes256);
            hw_aes_decrypt_16byte(p_value, p_value);
            DFU_PRINT_TRACE1("Data after decryped: %b", TRACE_BINARY(16, p_value));
        }
#endif
        dfu_control_point.p.start_dfu.ic_type = (*p_value);
        p_value += 1;
        dfu_control_point.p.start_dfu.secure_version = (*p_value);
        p_value += 1;
        LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.ctrl_flag.value, p_value);
        p_value += 2;
        LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.signature, p_value);
        p_value += 2;
        LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.crc16, p_value);
        p_value += 2;

        LE_ARRAY_TO_UINT32(dfu_control_point.p.start_dfu.image_length, p_value);

        APP_PRINT_INFO6("DFU_OPCODE_START_DFU: ic_type=0x%x, secure_version=0x%x, ctrl_flag.value=0x%x, signature=0x%x,crc16=0x%x*4Bytes, image_length=0x%x",
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
        if (dfuPara.ic_type == 0x05 && dfuPara.signature >= OTA && dfuPara.signature <= AppData2 \
            && dfuPara.image_total_length < OTP->ota_bank0_size)
        {

            status = DFU_SUCCESS;
            send_rsp(opcode, 1, &status);
        }
        else
        {
            status = DFU_INVALID_PARAMETER;
            send_rsp(opcode, 1, &status);
        }

        break;

    case DFU_WRITE_IMAGE:
        result = silence_dfu_service_handle_packet_req(length, p_value);

        notif_data[0] = (dfuPara.nCurOffSet + ota_tmp_buf_used_size) & 0xff;
        notif_data[1] = ((dfuPara.nCurOffSet + ota_tmp_buf_used_size) >> 8) & 0xff;
        notif_data[2] = ((dfuPara.nCurOffSet + ota_tmp_buf_used_size) >> 16) & 0xff;
        notif_data[3] = (dfuPara.nCurOffSet + ota_tmp_buf_used_size) >> 24;
        notif_data[4] = result;
        send_rsp(DFU_WRITE_IMAGE, 5, notif_data);

        break;

    case DFU_VALID_IMAGE:

        if (dfuPara.signature < 0x2790 || dfuPara.signature > 0x2796)
        {
            APP_PRINT_INFO1("signature err= 0x%x", dfuPara.signature);
            status = DFU_INVALID_PARAMETER;
            send_rsp(DFU_VALID_IMAGE, 1, &status);
            return;
        }
        APP_PRINT_INFO1("DFU_OPCODE_VALID_FW: signature = 0x%x", dfuPara.signature);

        flstatus = flash_lock(FLASH_LOCK_USER_MODE_READ);//signal = os_lock();
        check_result = dfu_check_checksum(dfuPara.signature);
        flash_unlock(FLASH_LOCK_USER_MODE_READ);//os_unlock(signal);

        DFU_PRINT_INFO1("dfu_act_notify_valid, check_result:%d (1: Success, 0: Fail)", check_result);

        if (check_result)
        {
            status = DFU_SUCCESS;
            send_rsp(DFU_VALID_IMAGE, 1, &status);
        }
        else
        {
            status = DFU_ERR;
            send_rsp(DFU_VALID_IMAGE, 1, &status);
        }
        break;

    case DFU_OPCODE_ACTIVE_IMAGE_RESET:
        /*notify bootloader to reset and use new image*/

        APP_PRINT_INFO0("DFU_OPCODE_ACTIVE_IMAGE_RESET:");
        status = DFU_SUCCESS;
        send_rsp(DFU_OPCODE_ACTIVE_IMAGE_RESET, 1, &status);

        if (flash_sw_protect_unlock_by_addr_locked(0x800000, &old_bp_lv))
        {
            APP_PRINT_INFO1("Unlock success! old_bp_lv=%d", old_bp_lv);
        }
        WDG_SystemReset(RESET_ALL, DFU_SWITCH_TO_OTA);
        break;


    case DFU_OPCODE_SYSTEM_RESET:
        DFU_PRINT_ERROR0("DFU_OPCODE_SYSTEM_RESET");
        status = DFU_SUCCESS;
        send_rsp(DFU_OPCODE_SYSTEM_RESET, 1, &status);
        //os_delay(2000);
        WDG_SystemReset(RESET_ALL, DFU_SYSTEM_RESET);
        break;

    case DFU_OPCODE_REPORT_TARGET_INFO:

        LE_ARRAY_TO_UINT16(dfuPara.signature, p_value);
        PROFILE_PRINT_INFO1("dfuPara.signature is %x\r\n", dfuPara.signature);

        dfu_report_target_fw_info(dfuPara.signature, &dfuPara.origin_image_version,
                                  (uint32_t *)&dfuPara.nCurOffSet);
        dfu_report_target_ic_type(dfuPara.signature, &ic_type);

        notif_data[0] = ic_type;
        notif_data[1] = APP_BANK;
        notif_data[2] = DFU_TEMP_BUFFER_SIZE & 0xff;
        notif_data[3] = DFU_TEMP_BUFFER_SIZE >> 8;
        notif_data[4] = dfuPara.origin_image_version & 0xff;
        notif_data[5] = dfuPara.origin_image_version >> 8;
        notif_data[6] = 0x00;
        notif_data[7] = 0x00;
        notif_data[8] = DFU_SUCCESS;
        send_rsp(DFU_OPCODE_REPORT_TARGET_INFO, 9, notif_data);
        break;
    default:
        {
            APP_PRINT_INFO1("dfu_service_handle_control_point_req: Unknown Opcode=0x%x",
                            dfu_control_point.opcode
                           );
        }

        break;

    }
}

/**
 * @brief silence_dfu_service_handle_packet_req
 *
 * @param length     data reviewed length.
 * @param p_value    data receive point address.
 * @return None
*/
uint8_t silence_dfu_service_handle_packet_req(uint16_t length, uint8_t *p_value)
{
    uint8_t result = 0;
    APP_PRINT_INFO4("dfu_service_handle_packet_req: length=%d, cur_offset =%d, ota_temp_buf_used_size = %d,image_total_length= %d",
                    length,
                    dfuPara.nCurOffSet,
                    ota_tmp_buf_used_size,
                    dfuPara.image_total_length
                   );

    if (dfuPara.nCurOffSet + ota_tmp_buf_used_size + length > dfuPara.image_total_length)
    {
        APP_PRINT_INFO4("dfu_service_handle_packet_req: p_dfu->cur_offset=%d, ota_temp_buf_used_size =%d, length= %d, image_total_length = %d ",
                        dfuPara.nCurOffSet,
                        ota_tmp_buf_used_size,
                        length,
                        dfuPara.image_total_length
                       );
        result = DFU_LENGTH_ERROR;
        PROFILE_PRINT_INFO0("DFU_LENGTH_ERROR");
    }
    else
    {
#if HW_AES_KEY
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
#endif
        ota_tmp_buf_used_size += length;

        if (ota_tmp_buf_used_size == 2048/*DFU_TEMP_BUFFER_SIZE*/ ||
            dfuPara.nCurOffSet + ota_tmp_buf_used_size == dfuPara.image_total_length
           )
        {
            unlock_flash_all();
            if (sil_dfu_update(dfuPara.signature, dfuPara.nCurOffSet, ota_tmp_buf_used_size,
                               (uint32_t *)TempBufferHead) == 0)
            {
                lock_flash();
                dfuPara.nCurOffSet += ota_tmp_buf_used_size;
                ota_tmp_buf_used_size = 0;
            }
            else
            {
                result = DFU_PROG_ERROR;
                ota_tmp_buf_used_size -= length;
            }

        }
    }
    return result;
}

