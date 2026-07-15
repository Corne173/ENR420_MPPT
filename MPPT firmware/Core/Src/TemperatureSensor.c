/*
 * Telemetry.c
 *
 * Periodic diagnostic telemetry output implementation.
 */

#include "TemperatureSensor.h"

#include "main.h"
#include "tim.h"
#include "ds18b20.h"
#include "ow.h"
#include <stdint.h>

ds18b20_t ds18;

#define TEMPERATURE_SENSOR_MAX_COUNT (5U)

static int16_t temp_c[TEMPERATURE_SENSOR_MAX_COUNT] =
{
    DS18B20_ERROR,
    DS18B20_ERROR,
    DS18B20_ERROR,
    DS18B20_ERROR,
    DS18B20_ERROR
};
void ds18_tim_cb(TIM_HandleTypeDef *htim);
void ds18_done_cb(ow_err_t error);

/**
  * @brief  Temperature sensor
  *
  * 
  * 
  */
void TemperatureSensor_Task(void)
{
    ds18b20_cnv(&ds18);
    while(ds18b20_is_busy(&ds18));
    while(!ds18b20_is_cnv_done(&ds18));

    for (uint8_t i = 0; (i < ds18.ow.rom_id_found) && (i < TEMPERATURE_SENSOR_MAX_COUNT); i++) {
        ds18b20_req_read(&ds18, i);
        while(ds18b20_is_busy(&ds18));
        temp_c[i] = ds18b20_read_c(&ds18);
    }
     
}

void TemperatureSensor_Init(void)
{
    ow_init_t ow_init_struct;
    ow_init_struct.tim_handle = &htim3;
    ow_init_struct.gpio = GPIOA;
    ow_init_struct.pin = GPIO_PIN_10;
    ow_init_struct.tim_cb = ds18_tim_cb;
    ow_init_struct.done_cb = NULL;   // Optional
    ow_init_struct.rom_id_filter = DS18B20_ID;

    ds18b20_init(&ds18, &ow_init_struct);

    // Update ROM IDs for all devices
    ds18b20_update_rom_id(&ds18);
    while(ds18b20_is_busy(&ds18));

    // Configure alarm thresholds and resolution
    ds18b20_config_t ds18_conf = {
        .alarm_high = 50,
        .alarm_low = -50,
        .cnv_bit = DS18B20_CNV_BIT_9
    };
    ds18b20_conf(&ds18, &ds18_conf);
    while(ds18b20_is_busy(&ds18));
    
  
}

float TemperatureSensor_GetTemp(void)
{
    int32_t sum_temps = 0;
    uint8_t count = ds18.ow.rom_id_found;

    if (count > TEMPERATURE_SENSOR_MAX_COUNT)
    {
        count = TEMPERATURE_SENSOR_MAX_COUNT;
    }

    if (count == 0U)
    {
        return 0.0f;
    }

    for (uint8_t i = 0; i < count; i++) {
        if (temp_c[i] != DS18B20_ERROR)
        {
            sum_temps = sum_temps + temp_c[i];
        }
    }
    
    return ((float)sum_temps / (float)count) / 100.0f;
}

int16_t TemperatureSensor_GetCentiC(uint8_t sensor_index)
{
    if ((sensor_index >= ds18.ow.rom_id_found) ||
        (sensor_index >= TEMPERATURE_SENSOR_MAX_COUNT))
    {
        return DS18B20_ERROR;
    }

    return temp_c[sensor_index];
}
void ds18_tim_cb(TIM_HandleTypeDef *htim)
{
    ow_callback(&ds18.ow);
}

void ds18_done_cb(ow_err_t error)
{
}