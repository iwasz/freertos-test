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

constexpr TickType_t DEFAULT_UART_TIMEOUT = pdMS_TO_TICKS (100);

void init ();

/**
 * Send len bytes from the buffer data, and block on a semapthore until it was sent. This
 * means, that while to the user the function appears to be blocking, in reality it waits
 * on a semaphore (with timeout @timeout) letting other tasks to run (other than the task
 * which called this function).
 *
 * This function is not thread safe meaning that it should not be used simultaneously from
 * many threads. Mutual exclusion has to be added in such scenarios.
 */
bool send (uint8_t const *data, size_t len, TickType_t timeout = DEFAULT_UART_TIMEOUT);

/**
 * Sends all teh contents of the container.
 *
 * Assumptions : Con have to have data () member, and size () member. For example:
 * std::array <uint8_t, 16>, std::vector <uint8_t> etc.
 */
template <typename Con> bool send (Con const &container, TickType_t timeout = DEFAULT_UART_TIMEOUT)
{
        return send (reinterpret_cast<uint8_t const *> (container.data ()), container.size (), timeout);
}

bool receive (uint8_t *data, size_t len, TickType_t timeout = DEFAULT_UART_TIMEOUT);

/**
 * Receives exactly conteinr.size() bytes and stores them in the container.
 *
 * Assumptions : Con have to have data () member, and size () member. For example:
 * std::array <uint8_t, 16>, std::vector <uint8_t> etc.
 */
template <typename Con, typename = int> bool receive (Con &container, TickType_t timeout = DEFAULT_UART_TIMEOUT)
{
        return receive (reinterpret_cast<uint8_t *> (container.data ()), container.size (), timeout);
}

} // namespace uart