/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CRSF_WRITE_H  /* Guard against multiple inclusion */
#define _CRSF_WRITE_H

#include "crsf.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef LIBCRSF_ENABLE_COMMAND
/* LIBCRSF_CMD_FRAME********************************************************* */
typedef enum {
  LIBCRSF_FC_CMD                        = 0x01,
  LIBCRSF_BT_CMD                        = 0x03,
  LIBCRSF_OSD_CMD                       = 0x05,
  LIBCRSF_VTX_CMD                       = 0x08,
  LIBCRSF_LED_CMD                       = 0x09,
  LIBCRSF_GENERAL_CMD                   = 0x0A,
  LIBCRSF_RC_RX_CMD                     = 0x10,
  LIBCRSF_WIFI_MODULE                   = 0x12,
  LIBCRSF_ACK                           = 0xFF,
} libCrsf_Commands;

typedef enum {
  LIBCRSF_FC_FORCE_DISARM_SUBCMD        = 0x01,
  LIBCRSF_FC_SCALE_CHANNEL_SUBCMD       = 0x02,
} libCrsf_FC_Subcommands;

typedef enum {
  LIBCRSF_BT_RESET_SUBCMD               = 0x01,
  LIBCRSF_BT_ENABLE_SUBCMD              = 0x02,
  LIBCRSF_BT_ECHO_SUBCMD                = 0x64,
} libCrsf_BT_Subcommands;

typedef enum {
  LIBCRSF_OSD_SEND_BUTTON_SUBCMD        = 0x01,
} libCrsf_OSD_Subcommands;

typedef enum {
  LIBCRSF_VTX_CHANGE_CHANNEL_SUBCMD         = 0x01,
  LIBCRSF_VTX_CHANGE_FREQ_SUBCMD            = 0x02,
  LIBCRSF_VTX_CHANGE_POWER_SUBCMD           = 0x03,
  LIBCRSF_VTX_CHANGE_PITMODE_SUBCMD         = 0x04,
  LIBCRSF_VTX_POWER_UP_FROM_PITMODE_SUBCMD  = 0x05,
} libCrsf_VTX_Subcommands;

typedef enum {
  LIBCRSF_LED_SET_DEFAULT_SUBCMD        = 0x01,
  LIBCRSF_LED_OVERRIDE_COLOR_SUBCMD     = 0x02,
  LIBCRSF_LED_OVERRIDE_PULSE_SUBCMD     = 0x03,
  LIBCRSF_LED_OVERRIDE_BLINK_SUBCMD     = 0x04,
  LIBCRSF_LED_OVERRIDE_SHIFT_SUBCMD     = 0x05,
} libCrsf_LED_Subcommands;

typedef enum {
  LIBCRSF_RC_RX_SET_TO_BIND_MODE_SUBCMD         = 0x01,
  LIBCRSF_RC_RX_CANCEL_BIND_MODE_SUBCMD         = 0x02,
  LIBCRSF_RC_RX_MODEL_SELECTION_SUBCMD          = 0x05,
  LIBCRSF_RC_RX_CURRENT_MODEL_SELECTION_SUBCMD  = 0x06,
  LIBCRSF_RC_RX_REPLY_CURRENT_MODEL_SUBCMD      = 0x07,
} libCrsf_RC_RX_Subcommands;

typedef enum {
  LIBCRSF_GENERAL_START_BOOTLOADER_SUBCMD     = 0x0A,
  LIBCRSF_GENERAL_ERASE_MEMORY_SUBCMD         = 0x0B,
  LIBCRSF_GENERALSOFTWARE_PRODUCT_KEY_SUBCMD  = 0x60,
  LIBCRSF_GENERALPRODUCT_FEEDBACK_SUBCMD      = 0x61,
} libCrsf_GENERAL_Subcommands;

typedef enum {
  LIBCRSF_WIFI_FIRMWARE_FILE_URL_SUBCMD       = 0x01,
} libCrsf_WIFI_Subcommands;

typedef struct {
  libCrsf_Commands command_id;
  uint8_t sub_command_id;
  uint8_t *payload;
} libCrsf_command_s;
#endif

#ifdef LIBCRSF_ENABLE_OPENTX_RELATED
#define LIBCRSF_REMOTE_RELATED_SUBCOMMAND_START 5
#define LIBCRSF_SD_DATA_START                   6
#define LIBCRSF_MAX_SD_PATH_SIZE                53
#define LIBCRSF_MAX_SD_PAYLOAD_START            19
#define LIBCRSF_MAX_SD_PAYLOAD_SIZE             44

typedef enum {
  LIBCRSF_REMOTE_SD_OPEN                    = 0x01,
  LIBCRSF_REMOTE_SD_CLOSE                   = 0x02,
  LIBCRSF_REMOTE_SD_READ_ACCESS             = 0x03,
  LIBCRSF_REMOTE_SD_WRITE_ACCESS            = 0x04,
  LIBCRSF_REMOTE_SD_WRITE_ACK               = 0x05,
  LIBCRSF_REMOTE_SD_ERASE_FILE              = 0x06,
  LIBCRSF_REMOTE_SD_MOUNT_STATUS            = 0x07,
  LIBCRSF_REMOTE_CRSF_TIMING_CORRECTION     = 0x10,
} libCrsf_Remote_Frame;

typedef union {
  struct {
    char path[ LIBCRSF_MAX_SD_PATH_SIZE ];
    uint32_t size;
  } info;
  struct {
    uint32_t addr;
    uint32_t size;
    uint32_t chunk_addr;
    uint8_t is_reply;
    uint8_t payload[ LIBCRSF_MAX_SD_PAYLOAD_SIZE ];
  } data;
  struct {
    uint32_t chunk_addr;
  } ack;
  struct {
    uint8_t is_mounted;
  } mount_status;
  struct {
    uint32_t interval;
    int32_t offset;
  } sync_time_Data;
} libCrsf_Remote_Data_u;
#endif

/* Checking and Setup Function*********************************************** */
bool libCrsf_checkif_devicecalled( uint8_t *p_arr, bool General_Call );

/* Write Command************************************************************* */
void libCrsf_crsfwrite( uint8_t frameType, uint8_t *p_arr, ... );

/* Extended Header Frames**************************************************** */
void libCrsf_packpingcommand( uint8_t *p_arr, uint32_t *i );
void libCrsf_packdeviceinfo( uint8_t *p_arr, uint32_t *i );

#ifdef LIBCRSF_ENABLE_COMMAND
/* CRSF Command************************************************************** */
void libCrsf_packcommandframe( uint8_t *p_arr, uint32_t *i
    , uint8_t target_address, libCrsf_command_s *command );

void libCrsf_pack_fc_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_FC_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_bt_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_BT_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_osd_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_OSD_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_vtx_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_VTX_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_led_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_LED_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_general_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_GENERAL_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_rc_rx_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_RC_RX_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_wifi_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_WIFI_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_ack_sub_command( uint8_t *p_arr, uint32_t *i, uint8_t *payload );
#endif

#ifdef LIBCRSF_ENABLE_OPENTX_RELATED
void libCrsf_packremote( uint8_t *p_arr, uint32_t *i
        , uint8_t target_device, uint8_t remote_command_id, libCrsf_Remote_Data_u *remote_data );
#endif

/* ************************************************************************** */

  /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _CRSF_WRITE_H */

/* *****************************************************************************
 End of File
 */
