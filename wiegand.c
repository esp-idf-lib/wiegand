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
 * @file wiegand.c
 *
 * ESP-IDF Wiegand protocol receiver
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "wiegand.h"

static const char *TAG = "wiegand";

#define TIMER_INTERVAL_US 50000 // 50ms
// esp_timer_delete() must not be called while the callback may use the reader,
// so the reader asks the timer task to delete the timer: the request is a
// one-shot timer that fires almost immediately
#define RELEASE_DELAY_US 100
#define RELEASE_TIMEOUT_MS 100

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

// D0 and D1 lines are tracked here: the GPIO driver silently replaces the ISR
// handler of a pin, so a second reader on the same line would steal interrupts
// instead of reporting an error
static portMUX_TYPE pins_lock = portMUX_INITIALIZER_UNLOCKED;
static bool pins_busy[GPIO_NUM_MAX];

// The ISR service is global, so it is installed once and never removed
static bool isr_service_ready;

static esp_err_t isr_service_install(void)
{
    if (isr_service_ready)
        return ESP_OK;

    esp_err_t res = gpio_install_isr_service(0);
    // Already installed, e.g. by the application
    if (res == ESP_ERR_INVALID_STATE)
        res = ESP_OK;

    isr_service_ready = res == ESP_OK;

    return res;
}

static bool pins_acquire(gpio_num_t d0, gpio_num_t d1)
{
    bool acquired = false;

    portENTER_CRITICAL(&pins_lock);
    if (!pins_busy[d0] && !pins_busy[d1])
    {
        pins_busy[d0] = true;
        pins_busy[d1] = true;
        acquired = true;
    }
    portEXIT_CRITICAL(&pins_lock);

    return acquired;
}

static void pins_release(gpio_num_t d0, gpio_num_t d1)
{
    portENTER_CRITICAL(&pins_lock);
    pins_busy[d0] = false;
    pins_busy[d1] = false;
    portEXIT_CRITICAL(&pins_lock);
}

static void isr_disable(wiegand_reader_t *reader)
{
    gpio_set_intr_type(reader->gpio_d0, GPIO_INTR_DISABLE);
    gpio_set_intr_type(reader->gpio_d1, GPIO_INTR_DISABLE);
}

static void isr_enable(wiegand_reader_t *reader)
{
    gpio_set_intr_type(reader->gpio_d0, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(reader->gpio_d1, GPIO_INTR_NEGEDGE);
}

// Not IRAM_ATTR: the handler calls gpio_get_level(), which is not in IRAM
static void isr_handler(void *arg)
{
    wiegand_reader_t *reader = (wiegand_reader_t *)arg;
    if (!reader->enabled)
        return;

    int d0 = gpio_get_level(reader->gpio_d0);
    int d1 = gpio_get_level(reader->gpio_d1);

    // ignore equal
    if (d0 == d1)
        return;
    // overflow
    if (reader->bits >= reader->size * 8)
        return;

    esp_timer_stop(reader->timer);

    uint8_t value;
    if (reader->bit_order == WIEGAND_MSB_FIRST)
        value = (d0 ? 0x80 : 0) >> (reader->bits % 8);
    else
        value = (d0 ? 1 : 0) << (reader->bits % 8);

    if (reader->byte_order == WIEGAND_MSB_FIRST)
        reader->buf[reader->size - reader->bits / 8 - 1] |= value;
    else
        reader->buf[reader->bits / 8] |= value;

    reader->bits++;

    esp_timer_start_once(reader->timer, TIMER_INTERVAL_US);
}

static void timer_handler(void *arg)
{
    wiegand_reader_t *reader = (wiegand_reader_t *)arg;

    // Called to delete the timer: done() waits for us
    if (reader->releasing)
    {
        esp_timer_stop(reader->timer);
        esp_err_t res = esp_timer_delete(reader->timer);
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Could not delete the timer: %s", esp_err_to_name(res));
        else
            reader->timer = NULL;

        xSemaphoreGive(reader->released);

        return;
    }

    // The reader could be released already
    if (!reader->buf)
        return;

    ESP_LOGD(TAG, "Got %zu bits of data", reader->bits);

    isr_disable(reader);
    esp_timer_stop(reader->timer);

    if (reader->callback)
        reader->callback(reader);

    // The callback may have stopped or released the reader
    if (!reader->enabled || !reader->buf)
        return;

    reader->bits = 0;
    memset(reader->buf, 0, reader->size);
    isr_enable(reader);
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t wiegand_reader_init(wiegand_reader_t *reader, gpio_num_t gpio_d0, gpio_num_t gpio_d1,
                              bool internal_pullups, size_t buf_size, wiegand_callback_t callback, wiegand_order_t bit_order,
                              wiegand_order_t byte_order)
{
    CHECK_ARG(reader && buf_size && callback);
    CHECK_ARG(gpio_d0 != gpio_d1);
    CHECK_ARG(GPIO_IS_VALID_GPIO(gpio_d0) && GPIO_IS_VALID_GPIO(gpio_d1));

    memset(reader, 0, sizeof(wiegand_reader_t));
    reader->gpio_d0 = gpio_d0;
    reader->gpio_d1 = gpio_d1;
    reader->size = buf_size;
    reader->bit_order = bit_order;
    reader->byte_order = byte_order;
    reader->callback = callback;

    if (!pins_acquire(gpio_d0, gpio_d1))
    {
        ESP_LOGE(TAG, "GPIO%d or GPIO%d is already used by another reader", gpio_d0, gpio_d1);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t res = isr_service_install();
    if (res != ESP_OK)
        goto fail_release;

    reader->buf = calloc(buf_size, 1);
    if (!reader->buf)
    {
        res = ESP_ERR_NO_MEM;
        goto fail_release;
    }

    reader->released = xSemaphoreCreateBinary();
    if (!reader->released)
    {
        res = ESP_ERR_NO_MEM;
        goto fail_buf;
    }

    esp_timer_create_args_t timer_args =
    {
        .name = TAG,
        .arg = reader,
        .callback = timer_handler,
        .dispatch_method = ESP_TIMER_TASK
    };
    res = esp_timer_create(&timer_args, &reader->timer);
    if (res != ESP_OK)
        goto fail_sem;

    res = gpio_set_direction(gpio_d0, GPIO_MODE_INPUT);
    if (res != ESP_OK)
        goto fail_timer;
    res = gpio_set_direction(gpio_d1, GPIO_MODE_INPUT);
    if (res != ESP_OK)
        goto fail_timer;
    res = gpio_set_pull_mode(gpio_d0, internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
    if (res != ESP_OK)
        goto fail_timer;
    res = gpio_set_pull_mode(gpio_d1, internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
    if (res != ESP_OK)
        goto fail_timer;

    isr_disable(reader);
    res = gpio_isr_handler_add(gpio_d0, isr_handler, reader);
    if (res != ESP_OK)
        goto fail_timer;
    res = gpio_isr_handler_add(gpio_d1, isr_handler, reader);
    if (res != ESP_OK)
        goto fail_d0;

    isr_enable(reader);
    reader->enabled = true;

    ESP_LOGD(TAG, "Reader initialized on D0=%d, D1=%d", gpio_d0, gpio_d1);

    return ESP_OK;

fail_d0:
    gpio_isr_handler_remove(gpio_d0);
fail_timer:
    esp_timer_delete(reader->timer);
    reader->timer = NULL;
fail_sem:
    vSemaphoreDelete(reader->released);
    reader->released = NULL;
fail_buf:
    free(reader->buf);
    reader->buf = NULL;
fail_release:
    pins_release(gpio_d0, gpio_d1);
    memset(reader, 0, sizeof(wiegand_reader_t));

    return res;
}

esp_err_t wiegand_reader_disable(wiegand_reader_t *reader)
{
    CHECK_ARG(reader);
    if (!reader->timer || !reader->buf)
        return ESP_ERR_INVALID_STATE;

    isr_disable(reader);
    esp_timer_stop(reader->timer);
    reader->enabled = false;

    ESP_LOGD(TAG, "Reader on D0=%d, D1=%d disabled", reader->gpio_d0, reader->gpio_d1);

    return ESP_OK;
}

esp_err_t wiegand_reader_enable(wiegand_reader_t *reader)
{
    CHECK_ARG(reader);
    if (!reader->timer || !reader->buf)
        return ESP_ERR_INVALID_STATE;

    reader->bits = 0;
    memset(reader->buf, 0, reader->size);

    isr_enable(reader);
    reader->enabled = true;

    ESP_LOGD(TAG, "Reader on D0=%d, D1=%d enabled", reader->gpio_d0, reader->gpio_d1);

    return ESP_OK;
}

esp_err_t wiegand_reader_done(wiegand_reader_t *reader)
{
    CHECK_ARG(reader);
    if (!reader->timer || !reader->buf)
        return ESP_ERR_INVALID_STATE;

    gpio_num_t d0 = reader->gpio_d0;
    gpio_num_t d1 = reader->gpio_d1;

    isr_disable(reader);
    CHECK(gpio_isr_handler_remove(d0));
    CHECK(gpio_isr_handler_remove(d1));
    esp_timer_stop(reader->timer);

    // Let the timer task delete the timer: esp_timer_delete() fails while the
    // timer is armed, and the handler must not run after the reader is freed
    reader->releasing = true;
    esp_err_t res = esp_timer_start_once(reader->timer, RELEASE_DELAY_US);
    if (res == ESP_OK)
        xSemaphoreTake(reader->released, pdMS_TO_TICKS(RELEASE_TIMEOUT_MS));

    reader->releasing = false;
    if (reader->timer)
    {
        ESP_LOGE(TAG, "Could not release the frame timer of the reader on D0=%d, D1=%d", d0, d1);
        return ESP_ERR_INVALID_STATE;
    }

    reader->enabled = false;
    free(reader->buf);
    reader->buf = NULL;
    reader->bits = 0;
    vSemaphoreDelete(reader->released);
    reader->released = NULL;
    pins_release(d0, d1);

    ESP_LOGD(TAG, "Reader removed");

    return ESP_OK;
}
