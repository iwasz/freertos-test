/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#pragma once
#include <FreeRTOS.h>
#include <cstdint>
#include <cstdlib>

namespace uart {

constexpr TickType_t DEFAULT_SEND_TIMEOUT = pdMS_TO_TICKS (100);

void usart2Init ();
bool send (uint8_t *data, size_t len, TickType_t timeout = DEFAULT_SEND_TIMEOUT);

} // namespace uart