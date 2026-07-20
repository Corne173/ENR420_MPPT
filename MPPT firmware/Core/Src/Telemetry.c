/*
 * Telemetry.c
 *
 * Periodic diagnostic telemetry output implementation.
 */

#include "Telemetry.h"

#include "Measurements.h"
#include "IrradianceSensor.h"
#include "MpptController.h"
#include "SerialConsole.h"
#include "TemperatureSensor.h"
#include "main.h"

#include <stdio.h>

#define TELEMETRY_TEMPERATURE_ERROR_C_X100 (-10000)

static void Telemetry_FormatTemperatureC(int16_t temperature_centi_c, char *buffer, size_t buffer_size);
static long Telemetry_RoundFloatToLong(float value);

/**
  * @brief  Sends one telemetry packet over the USART2 PC console.
  *
  * Packet format:
  * i_in_raw,i_out_raw,v_out_raw,v_in_raw,valid,temp0_c,temp1_c,irr_w_m2,state,fault
  *
  * The receiving PC adds its Unix timestamp when the packet arrives.
  */
void Telemetry_Task(void)
{
  char line[160];
  char temp0_c[12];
  char temp1_c[12];
  const Measurements_t *measurements = Measurements_GetLatest();
  const long irradiance_w_m2 = Telemetry_RoundFloatToLong(IrradianceSensor_GetWPerM2());

  Telemetry_FormatTemperatureC(TemperatureSensor_GetCentiC(0U), temp0_c, sizeof(temp0_c));
  Telemetry_FormatTemperatureC(TemperatureSensor_GetCentiC(1U), temp1_c, sizeof(temp1_c));

  /* Keep raw ADC counts, then append final sensor values. */
  const int length = snprintf(line,
                              sizeof(line),
                              "%u,%u,%u,%u,%u,%s,%s,%ld,%s,%s\r\n",
                              measurements->i_in_raw,
                              measurements->i_out_raw,
                              measurements->v_out_raw,
                              measurements->v_in_raw,
                              measurements->valid ? 1U : 0U,
                              temp0_c,
                              temp1_c,
                              irradiance_w_m2,
                              getStateName(getState()),
                              getFaultName(getFault()));

  /* Send only complete packets that fit in the local buffer. */
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    SerialConsole_Send(line);
  }
}

static void Telemetry_FormatTemperatureC(int16_t temperature_centi_c, char *buffer, size_t buffer_size)
{
  int temperature_abs;
  const char *sign = "";

  if (temperature_centi_c == TELEMETRY_TEMPERATURE_ERROR_C_X100)
  {
    (void)snprintf(buffer, buffer_size, "NA");
    return;
  }

  temperature_abs = (int)temperature_centi_c;
  if (temperature_abs < 0)
  {
    sign = "-";
    temperature_abs = -temperature_abs;
  }

  (void)snprintf(buffer,
                 buffer_size,
                 "%s%d.%02d",
                 sign,
                 temperature_abs / 100,
                 temperature_abs % 100);
}

static long Telemetry_RoundFloatToLong(float value)
{
  return (long)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}
