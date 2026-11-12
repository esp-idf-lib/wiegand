/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file wiegand.h
 * @defgroup wiegand wiegand
 * @{
 *
 * ESP-IDF Wiegand protocol receiver
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __WIEGAND_H__
#define __WIEGAND_H__

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct wiegand_reader wiegand_reader_t;

/**
 * @brief Data reception callback
 *
 * Called from a task, not from an interrupt, when a frame is complete, that is
 * when no pulse arrived for 50 ms. The frame is in @ref wiegand_reader_t::buf,
 * its length in bits in @ref wiegand_reader_t::bits.
 *
 * Data received while the callback runs is lost, reception is disabled for the
 * duration of the call. The callback may call @ref wiegand_reader_disable() to
 * stop reception, but it must not call @ref wiegand_reader_done().
 *
 * @param reader Reader descriptor
 */
typedef void (*wiegand_callback_t)(wiegand_reader_t *reader);

/**
 * Bit and byte order of data
 */
typedef enum
{
    WIEGAND_MSB_FIRST = 0,
    WIEGAND_LSB_FIRST
} wiegand_order_t;

/**
 * @brief Wiegand reader descriptor
 *
 * Fields marked as internal are maintained by the driver. The descriptor must
 * stay alive until @ref wiegand_reader_done() is called.
 */
struct wiegand_reader
{
    gpio_num_t gpio_d0;                    //!< D0 line, set by wiegand_reader_init()
    gpio_num_t gpio_d1;                    //!< D1 line, set by wiegand_reader_init()
    wiegand_callback_t callback;           //!< Frame callback, set by wiegand_reader_init()
    wiegand_order_t bit_order;             //!< Bit order, set by wiegand_reader_init()
    wiegand_order_t byte_order;            //!< Byte order, set by wiegand_reader_init()

    uint8_t *buf;                          //!< Frame buffer, allocated by wiegand_reader_init()
    size_t size;                           //!< Frame buffer size in bytes, set by wiegand_reader_init()
    volatile size_t bits;                  //!< Bits in the current frame, internal
    esp_timer_handle_t timer;              //!< Frame timeout timer, internal
    volatile bool enabled;                 //!< Reception state, internal
};

/**
 * @brief Create and initialize reader instance.
 *
 * Both lines must be usable and not used by another reader of this driver; the
 * driver keeps track of D0/D1 it has taken and returns `ESP_ERR_INVALID_STATE`
 * if the lines are busy. Lines occupied by other drivers are not detected.
 *
 * A frame ends when no pulse arrives for 50 ms, then @ref wiegand_callback_t is
 * called with the frame in @ref wiegand_reader_t::buf and its length in bits in
 * @ref wiegand_reader_t::bits.
 *
 * Buffer layout:
 *
 * - `WIEGAND_MSB_FIRST` byte order fills the buffer from its end, so the frame
 *   is right-aligned: `buf[buf_size - 1]` is the last byte of the frame;
 * - `WIEGAND_LSB_FIRST` byte order fills the buffer from its beginning:
 *   `buf[0]` is the first byte of the frame;
 * - `WIEGAND_MSB_FIRST` bit order packs the first received bit into the most
 *   significant bit of a byte, `WIEGAND_LSB_FIRST` into the least significant.
 *
 * @param reader           Reader descriptor
 * @param gpio_d0          GPIO pin for D0
 * @param gpio_d1          GPIO pin for D0
 * @param internal_pullups Enable internal pull-up resistors for D0 and D1 GPIO
 * @param buf_size         Reader buffer size in bytes, must be large enough to
 *                         contain entire Wiegand key
 * @param callback         Callback function for processing received codes
 * @param bit_order        Bit order of data
 * @param byte_order       Byte order of data
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if a pointer is NULL, a pin is not usable or both
 *        pins are the same
 *      - `ESP_ERR_INVALID_STATE` if D0 or D1 is already used by another reader
 *      - `ESP_ERR_NO_MEM` if the frame buffer could not be allocated
 *      - error code from the GPIO or esp_timer driver otherwise
 */
esp_err_t wiegand_reader_init(wiegand_reader_t *reader, gpio_num_t gpio_d0, gpio_num_t gpio_d1,
                              bool internal_pullups, size_t buf_size, wiegand_callback_t callback, wiegand_order_t bit_order,
                              wiegand_order_t byte_order);

/**
 * @brief Disable reader
 *
 * While reader is disabled, it will not receive new data. A frame that is being
 * received is dropped.
 *
 * May be called from the callback.
 *
 * @param reader Reader descriptor
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p reader is NULL
 *      - `ESP_ERR_INVALID_STATE` if the reader is not initialized
 */
esp_err_t wiegand_reader_disable(wiegand_reader_t *reader);

/**
 * @brief Enable reader
 *
 * Clears the frame buffer and the bit counter, so the frame received before is
 * lost.
 *
 * @param reader Reader descriptor
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p reader is NULL
 *      - `ESP_ERR_INVALID_STATE` if the reader is not initialized
 */
esp_err_t wiegand_reader_enable(wiegand_reader_t *reader);

/**
 * @brief Delete reader instance.
 *
 * Releases the GPIO lines, so they can be used again. The descriptor is left in
 * the uninitialized state: any call except @ref wiegand_reader_init() returns
 * `ESP_ERR_INVALID_STATE`.
 *
 * Must not be called from the callback.
 *
 * @param reader Reader descriptor
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p reader is NULL
 *      - `ESP_ERR_INVALID_STATE` if the reader is not initialized
 *      - error code from the GPIO or esp_timer driver otherwise
 */
esp_err_t wiegand_reader_done(wiegand_reader_t *reader);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __WIEGAND_H__ */
