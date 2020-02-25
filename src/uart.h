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

// TODO Remove
#include <etl/vector.h>
#include <stdint.h>

/*
Variables:

* DEFAULT_UART_TIMEOUT
* line buffer size
GPIO ports
GPIO pins
UART number
* urat irq priority
urat irq isr

TX dma number
TX dma channel number
* TX dma irq priority
TX dma irq ISR

RX dma number
RX dma channel number
* RX dma irq priority
RX dma irq ISR
*/

/**
 * Simple 8N1 uart implementation blocking on a RTOS primitive (like a semaphore, but
 * plans are to switch to a task notification).
 */
namespace uart {

constexpr TickType_t DEFAULT_UART_TIMEOUT = pdMS_TO_TICKS (100);

using Vector = etl::vector<uint8_t, 128>;

/**
 * UART status.
 */
enum class Status {
        OK,
        TIMEOUT, // Timeout waiting on semaphore, or other OS primitive
        PARITY_ERROR,
        FRAMING_ERROR,
        NOISE_ERROR,
        OVERRUN_ERROR // Occurs when new byte arrived while there's still unread one in the RDR register.
};

/// Line ends when receiving.
enum class LineEnd { CR_LF, CR, LF, EITHER };

/// Must be nitialized. No RAII for now.
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
Status send (uint8_t const *data, size_t len, TickType_t timeout = DEFAULT_UART_TIMEOUT);
Status send (const char *str, TickType_t timeout = DEFAULT_UART_TIMEOUT);

/**
 * Sends all teh contents of the container.
 *
 * Assumptions : Con have to have data () member, and size () member. For example:
 * std::array <uint8_t, 16>, std::vector <uint8_t> etc.
 */
template <typename Con> Status send (Con const &container, TickType_t timeout = DEFAULT_UART_TIMEOUT)
{
        return send (reinterpret_cast<uint8_t const *> (container.data ()), container.size (), timeout);
}

Status receive (uint8_t *data, size_t len, TickType_t timeout);

/**
 * Receives exactly conteinr.size() bytes and stores them in the container.
 *
 * Assumptions : Con have to have data () member, and size () member. For example:
 * std::array <uint8_t, 16>, std::vector <uint8_t> etc.
 */
template <typename Con, typename = int> Status receive (Con &container, TickType_t timeout)
{
        return receive (reinterpret_cast<uint8_t *> (container.data ()), container.size (), timeout);
}

/**
 *
 */
Vector receiveLine (uint8_t *data, size_t maxLen, TickType_t timeout, LineEnd lineEnd = LineEnd::CR_LF);

/**
 *
 */
bool clearLineBuffer ();

// template <typename Con>
// Con receiveLine (TickType_t timeout = DEFAULT_UART_TIMEOUT)

} // namespace uart