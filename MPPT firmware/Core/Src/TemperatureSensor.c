/*
 * TemperatureSensor.c
 *
 * Continuous, non-blocking DS18B20 temperature acquisition.
 */

#include "TemperatureSensor.h"

#include "SerialConsole.h"
#include "Timebase.h"
#include "ds18b20.h"
#include "main.h"
#include "ow.h"
#include "tim.h"

#include <stdint.h>
#include <stdio.h>

#define TEMPERATURE_SENSOR_MAX_COUNT       (5U)
#define TEMPERATURE_TRANSFER_TIMEOUT_MS    (25U)
#define TEMPERATURE_SEARCH_TIMEOUT_MS      (200U)
#define TEMPERATURE_RETRY_DELAY_MS         (1000U)

typedef enum
{
    TEMPERATURE_START_CONVERSION = 0,
    TEMPERATURE_WAIT_CONVERT_TX,
    TEMPERATURE_WAIT_CONVERSION,
    TEMPERATURE_WAIT_SENSOR_READ
} temperature_state_t;

static ds18b20_t ds18;
static temperature_state_t temperature_state = TEMPERATURE_START_CONVERSION;
static uint32_t temperature_state_started_ms = 0U;
static uint32_t temperature_start_delay_ms = 0U;
static uint8_t temperature_sensor_count = 0U;
static uint8_t temperature_sensor_index = 0U;
static volatile uint32_t temperature_tim_callback_count = 0U;
static uint32_t temperature_transfer_callback_start = 0U;
static bool temperature_failure_reported = false;

/* A complete set is staged here, then published together in temp_c. */
static int16_t pending_temp_c[TEMPERATURE_SENSOR_MAX_COUNT];
static int16_t temp_c[TEMPERATURE_SENSOR_MAX_COUNT] =
{
    DS18B20_ERROR,
    DS18B20_ERROR,
    DS18B20_ERROR,
    DS18B20_ERROR,
    DS18B20_ERROR
};

static void ds18_tim_cb(TIM_HandleTypeDef *htim);
static void TemperatureSensor_PrintBusDebug(const char *phase);

/**
  * @brief  Advances continuous temperature acquisition without waiting.
  * @note   Call this on every pass through the main loop. TIM3 interrupts
  *         perform the microsecond 1-Wire timing between calls.
  */
void TemperatureSensor_Task(void)
{
    const uint32_t now_ms = HAL_GetTick();
    ow_err_t error;

    if (temperature_sensor_count == 0U)
    {
        return;
    }

    switch (temperature_state)
    {
        case TEMPERATURE_START_CONVERSION:
            /* A failed bus gets a short rest; successful cycles restart immediately. */
            if (!Timebase_HasElapsed(now_ms,
                                     temperature_state_started_ms,
                                     temperature_start_delay_ms))
            {
                return;
            }

            temperature_transfer_callback_start = temperature_tim_callback_count;
            error = ds18b20_cnv(&ds18);
            if (error != OW_ERR_NONE)
            {
                break;
            }

            temperature_state_started_ms = now_ms;
            temperature_state = TEMPERATURE_WAIT_CONVERT_TX;
            return;

        case TEMPERATURE_WAIT_CONVERT_TX:
            /* Convert-T is sent by the TIM3 ISR; this task only checks progress. */
            if (ds18b20_is_busy(&ds18))
            {
                if (Timebase_HasElapsed(now_ms,
                                        temperature_state_started_ms,
                                        TEMPERATURE_TRANSFER_TIMEOUT_MS))
                {
                    TemperatureSensor_PrintBusDebug("convert-timeout");
                    ow_abort(&ds18.ow);
                    break;
                }
                return;
            }

            if (ds18b20_last_error(&ds18) != OW_ERR_NONE)
            {
                break;
            }

            /* The conversion time begins after the command is fully transmitted. */
            temperature_state_started_ms = now_ms;
            temperature_state = TEMPERATURE_WAIT_CONVERSION;
            return;

        case TEMPERATURE_WAIT_CONVERSION:
            if (!Timebase_HasElapsed(now_ms,
                                     temperature_state_started_ms,
                                     (uint32_t)ds18.cnv_time))
            {
                return;
            }

            for (uint8_t i = 0U; i < temperature_sensor_count; i++)
            {
                pending_temp_c[i] = DS18B20_ERROR;
            }

            temperature_sensor_index = 0U;
            temperature_transfer_callback_start = temperature_tim_callback_count;
            error = ds18b20_req_read(&ds18, temperature_sensor_index);
            if (error != OW_ERR_NONE)
            {
                break;
            }

            temperature_state_started_ms = now_ms;
            temperature_state = TEMPERATURE_WAIT_SENSOR_READ;
            return;

        case TEMPERATURE_WAIT_SENSOR_READ:
            /* Each scratchpad read progresses entirely in TIM3 interrupts. */
            if (ds18b20_is_busy(&ds18))
            {
                if (Timebase_HasElapsed(now_ms,
                                        temperature_state_started_ms,
                                        TEMPERATURE_TRANSFER_TIMEOUT_MS))
                {
                    TemperatureSensor_PrintBusDebug("read-timeout");
                    ow_abort(&ds18.ow);
                    break;
                }
                return;
            }

            if (ds18b20_last_error(&ds18) != OW_ERR_NONE)
            {
                break;
            }

            /* ds18b20_read_c() also checks the scratchpad CRC. */
            pending_temp_c[temperature_sensor_index] = ds18b20_read_c(&ds18);
            temperature_sensor_index++;

            if (temperature_sensor_index >= temperature_sensor_count)
            {
                for (uint8_t i = 0U; i < temperature_sensor_count; i++)
                {
                    temp_c[i] = pending_temp_c[i];
                }

                /* No normal-cycle delay: keep the sensors converting continuously. */
                temperature_state = TEMPERATURE_START_CONVERSION;
                temperature_state_started_ms = now_ms;
                temperature_start_delay_ms = 0U;
                temperature_failure_reported = false;
                return;
            }

            /* Start the next sensor directly; no extra state is needed. */
            temperature_transfer_callback_start = temperature_tim_callback_count;
            error = ds18b20_req_read(&ds18, temperature_sensor_index);
            if (error != OW_ERR_NONE)
            {
                break;
            }

            temperature_state_started_ms = now_ms;
            return;

        default:
            break;
    }

    /* Any path reaching here failed: show invalid data and retry after a pause. */
    TemperatureSensor_PrintBusDebug("runtime-error");
    for (uint8_t i = 0U; i < temperature_sensor_count; i++)
    {
        temp_c[i] = DS18B20_ERROR;
    }

    temperature_state = TEMPERATURE_START_CONVERSION;
    temperature_state_started_ms = now_ms;
    temperature_start_delay_ms = TEMPERATURE_RETRY_DELAY_MS;
}

void TemperatureSensor_Init(void)
{
    ds18b20_config_t ds18_config =
    {
        .alarm_high = 50,
        .alarm_low = -50,
        .cnv_bit = DS18B20_CNV_BIT_9
    };
    ow_init_t ow_init_struct = {0};
    uint32_t operation_started_ms;

    temperature_sensor_count = 0U;
    temperature_tim_callback_count = 0U;
    temperature_transfer_callback_start = 0U;
    temperature_failure_reported = false;

    ow_init_struct.tim_handle = &htim3;
    ow_init_struct.gpio = DQ__Temp_GPIO_Port;
    ow_init_struct.pin = DQ__Temp_Pin;
    ow_init_struct.tim_cb = ds18_tim_cb;
    ow_init_struct.done_cb = NULL;
    ow_init_struct.rom_id_filter = DS18B20_ID;

    ds18b20_init(&ds18, &ow_init_struct);

    /* Discovery is a one-time, bounded startup operation. Runtime reads do not block. */
    if (ds18b20_update_rom_id(&ds18) != OW_ERR_NONE)
    {
        return;
    }

    operation_started_ms = HAL_GetTick();
    while (ds18b20_is_busy(&ds18))
    {
        if (Timebase_HasElapsed(HAL_GetTick(),
                                operation_started_ms,
                                TEMPERATURE_SEARCH_TIMEOUT_MS))
        {
            ow_abort(&ds18.ow);
            break;
        }
    }

    /* A later search branch can fail even after one or more valid ROM IDs
     * have been collected. Keep those sensors usable instead of treating
     * discovery as all-or-nothing. */
    if (ds18.ow.rom_id_found == 0U)
    {
        return;
    }

    temperature_sensor_count = ds18.ow.rom_id_found;
    if (temperature_sensor_count > TEMPERATURE_SENSOR_MAX_COUNT)
    {
        temperature_sensor_count = TEMPERATURE_SENSOR_MAX_COUNT;
    }

    /* Configuration is optional for acquisition. If it fails, retain the
     * discovered sensors and use the safe power-up 12-bit conversion delay. */
    if (ds18b20_conf(&ds18, &ds18_config) == OW_ERR_NONE)
    {
        operation_started_ms = HAL_GetTick();
        while (ds18b20_is_busy(&ds18))
        {
            if (Timebase_HasElapsed(HAL_GetTick(),
                                    operation_started_ms,
                                    TEMPERATURE_TRANSFER_TIMEOUT_MS))
            {
                ow_abort(&ds18.ow);
                break;
            }
        }
    }

    if (ds18b20_last_error(&ds18) != OW_ERR_NONE)
    {
        ds18.cnv_time = DS18B20_CNV_TIM_12;
    }

    temperature_state = TEMPERATURE_START_CONVERSION;
    temperature_state_started_ms = HAL_GetTick();
    temperature_start_delay_ms = 0U;
}

float TemperatureSensor_GetTemp(void)
{
    int32_t sum_temps = 0;
    uint8_t valid_count = 0U;

    for (uint8_t i = 0U; i < temperature_sensor_count; i++)
    {
        if (temp_c[i] != DS18B20_ERROR)
        {
            sum_temps += temp_c[i];
            valid_count++;
        }
    }

    if (valid_count == 0U)
    {
        return 0.0f;
    }

    return ((float)sum_temps / (float)valid_count) / 100.0f;
}

int16_t TemperatureSensor_GetCentiC(uint8_t sensor_index)
{
    if (sensor_index >= temperature_sensor_count)
    {
        return DS18B20_ERROR;
    }

    return temp_c[sensor_index];
}

static void ds18_tim_cb(TIM_HandleTypeDef *htim)
{
    (void)htim;
    temperature_tim_callback_count++;
    ow_callback(&ds18.ow);
}

static void TemperatureSensor_PrintBusDebug(const char *phase)
{
    char line[240];
    int length;

    if (temperature_failure_reported)
    {
        return;
    }
    temperature_failure_reported = true;

    length = snprintf(line,
                      sizeof(line),
                      "TEMPDBG phase=%s n=%u/%u ts=%u err=%u ow=%u ph=%u b=%u.%u cb=%lu tim=%lX/%lX/%lX/%lu/%lu hs=%u irq=%lu/%lu pin=%u\r\n",
                      phase,
                      (unsigned int)temperature_sensor_index,
                      (unsigned int)temperature_sensor_count,
                      (unsigned int)temperature_state,
                      (unsigned int)ds18b20_last_error(&ds18),
                      (unsigned int)ds18.ow.state,
                      (unsigned int)ds18.ow.buf.bit_ph,
                      (unsigned int)ds18.ow.buf.byte_idx,
                      (unsigned int)ds18.ow.buf.bit_idx,
                      (unsigned long)(temperature_tim_callback_count -
                                      temperature_transfer_callback_start),
                      (unsigned long)TIM3->CR1,
                      (unsigned long)TIM3->DIER,
                      (unsigned long)TIM3->SR,
                      (unsigned long)TIM3->CNT,
                      (unsigned long)TIM3->ARR,
                      (unsigned int)htim3.State,
                      (unsigned long)NVIC_GetEnableIRQ(TIM3_IRQn),
                      (unsigned long)NVIC_GetPendingIRQ(TIM3_IRQn),
                      (unsigned int)HAL_GPIO_ReadPin(DQ__Temp_GPIO_Port,
                                                    DQ__Temp_Pin));

    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        SerialConsole_Send(line);
    }
}
