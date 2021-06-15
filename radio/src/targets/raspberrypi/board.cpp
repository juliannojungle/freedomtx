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

// #include "opentx.h"
#include "raylib-cpp.hpp"

// void boardInit()
int main(void)
{
  int screenWidth = 480;
  int screenHeight = 272;
  raylib::Color textColor = raylib::Color::LightGray();
  raylib::Window window(screenWidth, screenHeight, "OpenTX - Raspberry Pi");

  SetTargetFPS(60);

  while (!window.ShouldClose()) { // MAIN LOOP?? Detect window close button or ESC key
    BeginDrawing();
    {
      window.ClearBackground(RAYWHITE);
      textColor.DrawText("FreedomTX - Raspberry Pi port", 190, 200, 20);
    }
    EndDrawing();
  }
  return 0;
}