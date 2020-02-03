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

#include "crsf.h"
#include "crsf_write.h"
#include "crsf_utilities.h"

/* EXTERNAL USE************************************************************** */
extern uint8_t libCrsf_MySlaveAddress;
extern char *libCrsf_MyDeviceName;
extern uint32_t libCrsf_MySerialNo;
extern uint32_t libCrsf_MyHwID;
extern uint32_t libCrsf_MyFwID;

uint8_t libCrsf_MyparamListSize = 0;
uint8_t libCrsf_senderaddress;

/* Checking and Setup Function*********************************************** */
bool libCrsf_checkif_devicecalled( uint8_t *p_arr, bool General_Call ) {
  bool Feedback = false;

  if ( ( General_Call
     && *( p_arr + LIBCRSF_EXT_HEAD_DST_ADD ) == LIBCRSF_BROADCAST_ADD )
     || *( p_arr + LIBCRSF_EXT_HEAD_DST_ADD ) == libCrsf_MySlaveAddress ) {
    Feedback = true;
    libCrsf_senderaddress = *( p_arr + LIBCRSF_EXT_HEAD_ORG_ADD );
  }
  return Feedback;
}

/* Write Command************************************************************* */
void libCrsf_crsfwrite( uint8_t frameType, uint8_t *p_arr, ... ) {
  uint32_t i = 1; /* one byte space for length */

  va_list argp;
  va_start( argp, p_arr );

  libUtil_Write8( p_arr, &i, frameType );
  switch ( frameType ) {
    case LIBCRSF_EX_PARAM_PING_DEVICE:
      libCrsf_packpingcommand( p_arr, &i );
      break;
    case LIBCRSF_EX_PARAM_DEVICE_INFO:
      libCrsf_packdeviceinfo( p_arr, &i );
      break;
#ifdef LIBCRSF_ENABLE_COMMAND
    case LIBCRSF_CMD_FRAME:
      libCrsf_packcommandframe( p_arr, &i
          , ( uint8_t ) va_arg( argp, int ), ( libCrsf_command_s *) va_arg( argp, int * ) );
      break;
#endif
#ifdef LIBCRSF_ENABLE_OPENTX_RELATED
    case LIBCRSF_OPENTX_RELATED:
      libCrsf_packremote( p_arr, &i
              , ( uint8_t ) va_arg( argp, int ), ( libCrsf_Remote_Frame ) va_arg( argp, int ), ( libCrsf_Remote_Data_u * ) va_arg( argp, int * ) );
      break;
#endif
    default:
      break;
  }
  va_end( argp );
  libUtil_WriteEnd_8( p_arr, 0, i, POLYNOM_1 );
}


/* Extended Header Frames**************************************************** */
void libCrsf_packpingcommand( uint8_t *p_arr, uint32_t *i ) {
  libUtil_Write8( p_arr, i, LIBCRSF_BROADCAST_ADD );
  libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
}

void libCrsf_packdeviceinfo( uint8_t *p_arr, uint32_t *i ) {
  libUtil_Write8( p_arr, i, libCrsf_senderaddress );
  libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
  libUtil_WriteString( p_arr, i, libCrsf_MyDeviceName, true );
  libUtil_Write32( p_arr, i, libCrsf_MySerialNo );
  libUtil_Write32( p_arr, i, libCrsf_MyHwID );
  libUtil_Write32( p_arr, i, libCrsf_MyFwID );
  if( libCrsf_MyparamListSize <= 2 ) {
    libUtil_Write8( p_arr, i, 0x00 );
  } else {
    libUtil_Write8( p_arr, i, libCrsf_MyparamListSize - 2 );
  }
  libUtil_Write8( p_arr, i, LIBCRSF_PARAM_VERSION_NUMBER );
}

#ifdef LIBCRSF_ENABLE_COMMAND
/* CRSF Command************************************************************** */
void libCrsf_packcommandframe( uint8_t *p_arr, uint32_t *i
    , uint8_t target_address, libCrsf_command_s *command ) {
  libUtil_Write8( p_arr, i, target_address );
  libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
  libUtil_Write8( p_arr, i, command->command_id );
  libUtil_Write8( p_arr, i, command->sub_command_id );
  if( command->payload != NULL ) {
    switch( command->command_id ) {
      case LIBCRSF_ACK:
        libCrsf_pack_ack_sub_command( p_arr, i, command->payload );
        break;
      default:
        break;
    }
  }
  libUtil_WriteEnd_8( p_arr, 0, *i, POLYNOM_2 );
  ( *i )++;
}

void libCrsf_pack_ack_sub_command( uint8_t *p_arr, uint32_t *i, uint8_t *payload ) {

}

/* ************************************************************************** */
#endif

#ifdef LIBCRSF_ENABLE_OPENTX_RELATED
/* CRSF OPENTX RELATED************************************************************** */
void libCrsf_packremote( uint8_t *p_arr, uint32_t *i
        , uint8_t target_device, uint8_t remote_command_id, libCrsf_Remote_Data_u *remote_data ) {

  libUtil_Write8( p_arr, i, target_device );
  libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
  libUtil_Write8( p_arr, i, remote_command_id );

  switch( remote_command_id ) {
#ifdef LIBCRSF_ENABLE_SD
    case LIBCRSF_REMOTE_SD_OPEN:
      libUtil_WriteBytes( p_arr, i, (uint8_t*)remote_data->info.path, LIBCRSF_MAX_SD_PATH_SIZE );
      libUtil_Write32( p_arr, i, remote_data->info.size );
      break;
    case LIBCRSF_REMOTE_SD_CLOSE:
      break;
    case LIBCRSF_REMOTE_SD_READ_ACCESS:
      libUtil_Write32( p_arr, i, remote_data->data.addr );
      libUtil_Write32( p_arr, i, remote_data->data.size );
      libUtil_Write32( p_arr, i, remote_data->data.chunk_addr );
      libUtil_Write8( p_arr, i, remote_data->data.is_reply );
      libUtil_WriteBytes( p_arr, i, remote_data->data.payload, LIBCRSF_MAX_SD_PAYLOAD_SIZE );
      break;
    case LIBCRSF_REMOTE_SD_WRITE_ACCESS:
      break;
    case LIBCRSF_REMOTE_SD_WRITE_ACK:
      libUtil_Write32( p_arr, i, remote_data->ack.chunk_addr);
      break;
    case LIBCRSF_REMOTE_SD_ERASE_FILE:
      libUtil_WriteBytes( p_arr, i, (uint8_t*)remote_data->info.path, LIBCRSF_MAX_SD_PATH_SIZE );
      break;
    case LIBCRSF_REMOTE_SD_MOUNT_STATUS:
      libUtil_Write8( p_arr, i, remote_data->mount_status.is_mounted );
      break;
#endif
    default:
      break;
  }
}
#endif

/* *****************************************************************************
 End of File
 */
