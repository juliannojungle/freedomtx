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

#include "opentx.h"

#define XPOT_DELTA 10
#define XPOT_DELAY 10 /* cycles */

enum CalibrationState {
  CALIB_START = 0,
#if defined(PCBTANGO)
  CALIB_SET_P0,
  CALIB_SET_P1,
  CALIB_SET_P2,
  CALIB_SET_P3,
  CALIB_SET_P4,
  CALIB_SET_P5,
  CALIB_SET_P6,
  CALIB_SET_P7,
  CALIB_SET_P8,
  CALIB_CAL_POINTS,
#endif  // #if defined(PCBTANGO)
  CALIB_MOVE_STICKS,
  CALIB_STORE,
  CALIB_FINISHED
};

#if defined(PCBTANGO)
enum{
  GIMBAL_LEFT = 0,
  GIMBAL_RIGHT,
  GIMBAL_COUNT
};

enum{
  GIMBAL_LEFT_SEL = 0,
  GIMBAL_BOTH_SEL,
  GIMBAL_RIGHT_SEL,
  GIMBAL_SEL_COUNT
};
#define CALIB_POINT_COUNT   (CALIB_SET_P8 - CALIB_SET_P0 + 1)
const int16_t point_pos[CALIB_POINT_COUNT][2] = {{0,0}, {1024,0}, {1024,1024}, {0,1024}, {-1024,1024}, {-1024,0}, {-1024,-1024}, {0,-1024}, {1024,-1024}};
#define LLABEL_CENTERX            (15)
#define BOTHLABEL_CENTERX         (53)
#define RLABEL_CENTERX            (90)
#define POINT_CAL_COUNTDOWN       (3)
#endif  // #if defined(PCBTANGO)

void menuCommonCalib(event_t event)
{
#if defined(SIMU)
  reusableBuffer.calib.state = CALIB_FINISHED;
  return;
#else

#if defined(PCBTANGO)
  uint8_t gim_select = crossfireSharedData.gim_select;
  int16_t force_point_pos[4];
  int16_t *countdown_timer = &reusableBuffer.calib.midVals[0];
  int16_t *count = &reusableBuffer.calib.midVals[1];
  int16_t curr_time;

  if( reusableBuffer.calib.state > CALIB_START && reusableBuffer.calib.state < CALIB_FINISHED && crossfireSharedData.stick_state > reusableBuffer.calib.state ) {
    // to sync the state from crossfire
    reusableBuffer.calib.state = crossfireSharedData.stick_state;
  }
#endif  //#if defined(PCBTANGO)

  menuCalibrationState = reusableBuffer.calib.state; // make sure we don't scroll while calibrating

  switch (event) {
    case EVT_ENTRY:
    case EVT_KEY_BREAK(KEY_EXIT):
      reusableBuffer.calib.state = CALIB_START;
      crossfireSharedData.stick_state = CALIB_FINISHED;
      break;

    case EVT_KEY_BREAK(KEY_ENTER):
      reusableBuffer.calib.state++;
      break;

    case EVT_KEY_BREAK(KEY_MENU):
      *count = POINT_CAL_COUNTDOWN;
      *countdown_timer = get_tmr10ms() / 100;
      STOP_PLAY(0);
      reusableBuffer.calib.state = CALIB_SET_P0;
      break;

#if defined(PCBTANGO)
    case EVT_ROTARY_LEFT:
      if( reusableBuffer.calib.state == CALIB_START ){
        if( gim_select == GIMBAL_RIGHT_SEL )
          gim_select = GIMBAL_LEFT_SEL;
        else
          gim_select = ( gim_select + 1 % 3 );
      }
      break;

    case EVT_ROTARY_RIGHT:
      if( reusableBuffer.calib.state == CALIB_START ){
        if( gim_select )
          gim_select = ( gim_select - 1 % 3 );
        else
          gim_select = GIMBAL_RIGHT_SEL;
      }
      break;
#endif
  }

  switch (reusableBuffer.calib.state) {
    case CALIB_START:
      // START CALIBRATION
      if (!READ_ONLY()) {
        lcdDrawTextAlignedLeft(MENU_HEADER_HEIGHT+2*FH, STR_MENUTOSTART);
        for (uint8_t j = 0; j < GIMBAL_SEL_COUNT; j++) {
          switch (j) {
            case 0:
              lcdDrawTextAtIndex(LLABEL_CENTERX, MENU_HEADER_HEIGHT+4*FH, STR_LEFT, 0, (gim_select == GIMBAL_LEFT_SEL ? INVERS : 0));
              break;
            case 1:
              lcdDrawTextAtIndex(BOTHLABEL_CENTERX, MENU_HEADER_HEIGHT+4*FH, STR_BOTH, 0, (gim_select == GIMBAL_BOTH_SEL ? INVERS : 0));
              break;
            case 2:
              lcdDrawTextAtIndex(RLABEL_CENTERX, MENU_HEADER_HEIGHT+4*FH, STR_RIGHT, 0, (gim_select == GIMBAL_RIGHT_SEL ? INVERS : 0));
              break;
          }
        }
      }
#if defined(PCBTANGO)
      crossfireSharedData.gim_select = gim_select;
      *countdown_timer = get_tmr10ms() / 100;
      *count = POINT_CAL_COUNTDOWN;
#endif
      break;

#if defined(PCBTANGO)
    case CALIB_SET_P0 ... CALIB_SET_P8:
      crossfireSharedData.stick_state = reusableBuffer.calib.state;
      if( gim_select == GIMBAL_BOTH_SEL ){
        force_point_pos[CONVERT_MODE(1)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][1];
        force_point_pos[CONVERT_MODE(0)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][0];
        force_point_pos[CONVERT_MODE(2)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][1];
        force_point_pos[CONVERT_MODE(3)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][0];
      }
      else if( gim_select == GIMBAL_LEFT_SEL ){
        force_point_pos[CONVERT_MODE(1)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][1];
        force_point_pos[CONVERT_MODE(0)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][0];
        force_point_pos[CONVERT_MODE(2)] = 0;
        force_point_pos[CONVERT_MODE(3)] = 0;
      }
      else if( gim_select == GIMBAL_RIGHT_SEL ){
        force_point_pos[CONVERT_MODE(1)] = 0;
        force_point_pos[CONVERT_MODE(0)] = 0;
        force_point_pos[CONVERT_MODE(2)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][1];
        force_point_pos[CONVERT_MODE(3)] = point_pos[reusableBuffer.calib.state - CALIB_SET_P0][0];
      }
      if( reusableBuffer.calib.state == CALIB_SET_P0 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P0, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P1 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P1, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P2 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P2, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P3 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P3, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P4 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P4, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P5 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P5, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P6 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P6, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P7 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P7, INVERS);
      else if( reusableBuffer.calib.state == CALIB_SET_P8 )
        lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVESTICK_P8, INVERS);
      lcdDrawTextAlignedLeft(MENU_HEADER_HEIGHT+2*FH, STR_MENUWHENDONE);
      curr_time = get_tmr10ms() / 100;
      if( gim_select == GIMBAL_BOTH_SEL ){
        if( *count == -1 ){
          AUDIO_KEY_ERROR();
          *countdown_timer = curr_time;
          *count = POINT_CAL_COUNTDOWN;
          reusableBuffer.calib.state++;
        }
        else if( curr_time  - *countdown_timer >= 1 ){
          if( *count != 0 )
            playNumber( *count, 0, 0, 0 );
          (*count)--;
          *countdown_timer = curr_time;
        }
      }
      break;

    case CALIB_CAL_POINTS:
      crossfireSharedData.stick_state = reusableBuffer.calib.state;
      lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_CAL_POINTS, INVERS);
      lcdDrawTextAlignedLeft(MENU_HEADER_HEIGHT+2*FH, STR_RELEASE_STICK);
      for (uint8_t i=0; i<NUM_STICKS+NUM_POTS+NUM_SLIDERS; i++) {
        reusableBuffer.calib.loVals[i] = 15000;
        reusableBuffer.calib.hiVals[i] = -15000;
        reusableBuffer.calib.midVals[i] = 0;
      }
      break;
      
    case CALIB_MOVE_STICKS:
      // MOVE STICKS/POTS
      crossfireSharedData.stick_state = reusableBuffer.calib.state;
      lcdDrawText(0*FW, MENU_HEADER_HEIGHT+FH, STR_MOVE_PLUS, INVERS);
      lcdDrawTextAlignedLeft(MENU_HEADER_HEIGHT+2*FH, STR_MENUWHENDONE);

      for (uint8_t i=0; i<NUM_STICKS+NUM_POTS+NUM_SLIDERS; i++) {
        // skip the unselected gimbal
        if( gim_select == GIMBAL_RIGHT_SEL && ( i == 0 || i == 1 ))
            continue;
        else if( gim_select == GIMBAL_LEFT_SEL && ( i == 2 || i == 3))
            continue;
        int16_t vt = anaIn(i);
        reusableBuffer.calib.loVals[i] = min(vt, reusableBuffer.calib.loVals[i]);
        reusableBuffer.calib.hiVals[i] = max(vt, reusableBuffer.calib.hiVals[i]);
        if (abs(reusableBuffer.calib.loVals[i]-reusableBuffer.calib.hiVals[i]) > 50) {
          g_eeGeneral.calib[i].mid = reusableBuffer.calib.midVals[i];
          int16_t v = reusableBuffer.calib.midVals[i] - reusableBuffer.calib.loVals[i];
          g_eeGeneral.calib[i].spanNeg = v - v/STICK_TOLERANCE;
          v = reusableBuffer.calib.hiVals[i] - reusableBuffer.calib.midVals[i];
          g_eeGeneral.calib[i].spanPos = v - v/STICK_TOLERANCE;
        }
      }
      break;
#endif  //#if defined(PCBTANGO)

    case CALIB_STORE:
#if defined(PCBTANGO)
      crossfireSharedData.stick_state = reusableBuffer.calib.state;
#endif  //#if defined(PCBTANGO)
      g_eeGeneral.chkSum = evalChkSum();
      storageDirty(EE_GENERAL);
      reusableBuffer.calib.state = CALIB_FINISHED;
      break;

    default:
      reusableBuffer.calib.state = CALIB_START;
      break;
  }

#if defined(PCBTANGO) && !defined(SIMU)
  if( reusableBuffer.calib.state >= CALIB_SET_P0 && reusableBuffer.calib.state <= CALIB_SET_P8 )
    doMainScreenGraphics( (uint32_t)force_point_pos );
  else
#endif
    doMainScreenGraphics( 0 );
#endif
}

void menuRadioCalibration(event_t event)
{
  check_simple(STR_MENUCALIBRATION, event, MENU_RADIO_CALIBRATION, menuTabGeneral, DIM(menuTabGeneral), 0);
  TITLE(STR_MENUCALIBRATION);
  menuCommonCalib(READ_ONLY() ? 0 : event);
  if (menuEvent) {
    menuCalibrationState = CALIB_START;
  }
}

void menuFirstCalib(event_t event)
{
  if (event == EVT_KEY_BREAK(KEY_EXIT) || reusableBuffer.calib.state == CALIB_FINISHED) {
    menuCalibrationState = CALIB_START;
    chainMenu(menuMainView);
  }
  else {
    lcdDrawTextAlignedCenter(0*FH, MENUCALIBRATION);
    lcdInvertLine(0);
    menuCommonCalib(event);
  }
}
