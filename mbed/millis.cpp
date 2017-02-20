/*
 uip_millis.cpp
 Copyright (c) 2015 Zoltan Hudak <hudakz@inbox.com>
 All rights reserved.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
  */
#if !defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_ARCH_SAM)

#if defined(__MBED__)
#include "millis.h"
#include <mbed.h>

volatile unsigned long  _millis;

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void millis_start(void) {
    SysTick_Config(SystemCoreClock / 1000);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
extern "C" void SysTick_Handler(void) {
    _millis += 1;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
extern "C" unsigned long millis(void) {
    return _millis;
}
#endif
#endif
