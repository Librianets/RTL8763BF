/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     key_handler.c
* @brief    .
* @details
* @author   Parker
* @date     2018-04-23
* @version  v1.0
*********************************************************************************************************
*/
#include <rtl876x.h>
#include <trace.h>
#include <board.h>
#include <key_handler.h>
#include <profile_server.h>
#include <keyboard_app.h>
#include <hids_kb.h>
#include <gap_bond_le.h>


const uint16_t key_map[KEYPAD_ROW_SIZE][KEYPAD_COLUMN_SIZE] =
{
    {KB_0,       KB_1,         KB_2,                KB_3},
    {KB_4,       KB_5,         KB_6,                KB_7},
    {KB_8,       KB_9,         KB_DELETE_Backspace, KB_Return_ENTER},
    {KB_a_A,     KB_CapsLock,       KB_LeftArrow,        KB_RightArrow}
};

PASSKEY_DEF passkey = {0, 0, false};

/******************************************************************
 * @brief    Set passkey
 */
void set_passkey(uint8_t key_num, PASSKEY_DEF *p_passkey)
{
    if (key_num < 10)
    {
        if (p_passkey->count < 6)
        {
            p_passkey->value = p_passkey->value * 10;
            p_passkey->value += key_num;
            p_passkey->count++;
        }
    }
    else if (key_num == 10)
    {
        //delete the last input key value
        p_passkey->value = p_passkey->value / 10;
        if (p_passkey->count > 0)
        {
            p_passkey->count--;
        }
    }
}

/******************************************************************
 * @brief    handle pair event
 */
void keyboard_handle_pair_event(T_KEYSCAN_FIFIO_DATA *p_key_data)
{
    uint8_t usage_id;

    for (uint8_t i = 0; i < p_key_data->len; i++)
    {
        APP_PRINT_INFO4("No.%d,Total len=%d: (row, column) = (%d, %d)", i + 1, p_key_data->len,
                        p_key_data->key[i].row, p_key_data->key[i].column);
        usage_id = key_map[p_key_data->key[i].row][p_key_data->key[i].column];

        switch (usage_id)
        {
        case KB_0:
            {
                APP_PRINT_INFO0("passKey = KB_0");
                set_passkey(0, &passkey);
                break;
            }
        case KB_1:
            {
                APP_PRINT_INFO0("passKey = KB_1");
                set_passkey(1, &passkey);
                break;
            }
        case KB_2:
            {
                APP_PRINT_INFO0("passKey = KB_2");
                set_passkey(2, &passkey);
                break;
            }
        case KB_3:
            {
                APP_PRINT_INFO0("passKey = KB_3");
                set_passkey(3, &passkey);
                break;
            }
        case KB_4:
            {
                APP_PRINT_INFO0("passKey = KB_4");
                set_passkey(4, &passkey);
                break;
            }
        case KB_5:
            {
                APP_PRINT_INFO0("passKey = KB_5");
                set_passkey(5, &passkey);
                break;
            }

        case KB_6:
            {
                APP_PRINT_INFO0("passKey = KB_6");
                set_passkey(6, &passkey);
                break;
            }
        case KB_7:
            {
                APP_PRINT_INFO0("passKey = KB_7");
                set_passkey(7, &passkey);
                break;
            }
        case KB_8:
            {
                APP_PRINT_INFO0("passKey = KB_8");
                set_passkey(8, &passkey);
                break;
            }
        case KB_9:
            {
                APP_PRINT_INFO0("passKey = KB_9");
                set_passkey(9, &passkey);
                break;
            }
        case KB_Return_ENTER:
            {
                APP_PRINT_INFO0("passKey = KB_Return_ENTER\n\n");
                if (passkey.count == 6)
                {
                    APP_PRINT_INFO1("passKey = %d", passkey.value);
                    le_bond_passkey_input_confirm(keyboard_con_id, passkey.value, GAP_CFM_CAUSE_ACCEPT);
                    passkey.enable_input = false;
                }
                else
                {
                    APP_PRINT_ERROR0("Error: The passkey length is not equal to six, please input again.\n");
                }
                // clear passkey data
                passkey.count = 0;
                passkey.value = 0;
                break;
            }

        case KB_DELETE_Backspace:
            {
                set_passkey(10, &passkey);
                APP_PRINT_INFO0("Delete the last input key value");
                break;
            }

        default:
            {
                APP_PRINT_ERROR1("Invalid Passkey keyCode: 0x%x\n\n", usage_id);
                break;
            }
        }
    }
}

/******************************************************************
 * @brief    handle keyscan press event
 */
void keyboard_handle_press_event(T_KEYSCAN_FIFIO_DATA *p_key_data)
{
    uint8_t keycount = 0;
    uint16_t usage_id;
    uint8_t keyboard_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


    for (uint8_t i = 0; i < p_key_data->len; i++)
    {
        APP_PRINT_INFO4("No.%d,Total len=%d: (row, column) = (%d, %d)", i + 1, p_key_data->len,
                        p_key_data->key[i].row, p_key_data->key[i].column);
        usage_id = key_map[p_key_data->key[i].row][p_key_data->key[i].column];

        /********Normal Keys**********/
        if (usage_id <= 0xA4)
        {
            if (keycount < 6)
            {
                keyboard_data[2 + keycount] = usage_id;
                keycount++;
            }
        }
        /*********Modifier Keys********/
        else if ((usage_id >= 0xE0) && (usage_id <= 0xE7))
        {
            keyboard_data[0] |= 1 << (usage_id & 0x07);
        }
    }

    server_send_data(keyboard_con_id, hid_srv_id, GATT_SVC_HID_REPORT_INPUT_INDEX,
                     keyboard_data, sizeof(keyboard_data), GATT_PDU_TYPE_NOTIFICATION);
}

/******************************************************************
 * @brief    handle keyscan release event
 */
void keyboard_handle_release_event(void)
{
    uint8_t keyboard_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    server_send_data(keyboard_con_id, hid_srv_id, GATT_SVC_HID_REPORT_INPUT_INDEX,
                     keyboard_data, sizeof(keyboard_data), GATT_PDU_TYPE_NOTIFICATION);
}

