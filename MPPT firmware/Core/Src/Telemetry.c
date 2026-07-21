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
static void Telemetry_FormatFixed(float value,
                                  uint32_t scale,
                                  uint8_t decimal_places,
                                  char *buffer,
                                  size_t buffer_size);
static long Telemetry_RoundFloatToLong(float value);

/**
  * @brief  Sends one telemetry packet over the USART2 PC console.
  *
  * Packet format:
  * i_in_raw,i_out_raw,v_out_raw,v_in_raw,valid,temp0_c,temp1_c,irr_w_m2,state,fault,
  * mppt_phase,mppt_gain,mppt_step,buck_duty,boost_duty,reference_power_w,sampled_power_w
  *
  * The receiving PC adds its Unix timestamp when the packet arrives.
  */
void Telemetry_Task(void)
{
  char line[256];
  char temp0_c[12];
  char temp1_c[12];
  char gain[16];
  char step[16];
  char buck_duty[16];
  char boost_duty[16];
  char reference_power_w[20];
  char sampled_power_w[20];
  const Measurements_t *measurements = Measurements_GetLatest();
  const MpptControllerStatus_t *mppt_status = MpptController_GetStatus();
  const long irradiance_w_m2 = Telemetry_RoundFloatToLong(IrradianceSensor_GetWPerM2());

  Telemetry_FormatTemperatureC(TemperatureSensor_GetCentiC(0U), temp0_c, sizeof(temp0_c));
  Telemetry_FormatTemperatureC(TemperatureSensor_GetCentiC(1U), temp1_c, sizeof(temp1_c));
  Telemetry_FormatFixed(mppt_status->gain, 10000U, 4U, gain, sizeof(gain));
  Telemetry_FormatFixed(mppt_status->step, 10000U, 4U, step, sizeof(step));
  Telemetry_FormatFixed(mppt_status->buck_duty, 10000U, 4U, buck_duty, sizeof(buck_duty));
  Telemetry_FormatFixed(mppt_status->boost_duty, 10000U, 4U, boost_duty, sizeof(boost_duty));
  Telemetry_FormatFixed(mppt_status->reference_power_w,
                        100U,
                        2U,
                        reference_power_w,
                        sizeof(reference_power_w));
  Telemetry_FormatFixed(mppt_status->sampled_power_w,
                        100U,
                        2U,
                        sampled_power_w,
                        sizeof(sampled_power_w));

  /* Keep raw ADC counts, then append final sensor values. */
  const int length = snprintf(line,
                              sizeof(line),
                              "%u,%u,%u,%u,%u,%s,%s,%ld,%s,%s,"
                              "%s,%s,%s,%s,%s,%s,%s\r\n",
                              measurements->i_in_raw,
                              measurements->i_out_raw,
                              measurements->v_out_raw,
                              measurements->v_in_raw,
                              measurements->valid ? 1U : 0U,
                              temp0_c,
                              temp1_c,
                              irradiance_w_m2,
                              getStateName(getState()),
                              getFaultName(getFault()),
                              MpptController_GetPhaseName(mppt_status->phase),
                              gain,
                              step,
                              buck_duty,
                              boost_duty,
                              reference_power_w,
                              sampled_power_w);

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

static void Telemetry_FormatFixed(float value,
                                  uint32_t scale,
                                  uint8_t decimal_places,
                                  char *buffer,
                                  size_t buffer_size)
{
  const long scaled_value = Telemetry_RoundFloatToLong(value * (float)scale);
  const char *sign = (scaled_value < 0L) ? "-" : "";
  const unsigned long magnitude = (scaled_value < 0L)
                                      ? (unsigned long)(-scaled_value)
                                      : (unsigned long)scaled_value;

  (void)snprintf(buffer,
                 buffer_size,
                 "%s%lu.%0*lu",
                 sign,
                 magnitude / scale,
                 (int)decimal_places,
                 magnitude % scale);
}

static long Telemetry_RoundFloatToLong(float value)
{
  return (long)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}
