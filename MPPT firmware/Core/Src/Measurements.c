/*
 * Measurements.c
 *
 * ADC measurement acquisition and conversion for the MPPT converter.
 */

#include "Measurements.h"

#include "adc.h"

#include <stddef.h>

/* ADC conversion and basic plausibility limits. */
#define ADC_REFERENCE_VOLTAGE       (3.3f)
#define ADC_MAX_COUNTS              (4095.0f)
#define ADC_POLL_TIMEOUT_MS         (2U)
#define ADC_RAW_SATURATION_COUNT    (4090U)
#define ADC_AVERAGE_SAMPLE_COUNT    (16U)

/* Sensor scale factors from the voltage/current sensing equations in the guide. */
#define INPUT_VOLTAGE_SENSOR_SCALE    (46.45f)
#define OUTPUT_VOLTAGE_SENSOR_SCALE   (28.78f)
#define INPUT_CURRENT_SENSOR_V_PER_A  (0.100f)
#define OUTPUT_CURRENT_SENSOR_V_PER_A (0.0333f)

static float Measurements_InputVoltageFromAdc(float adc_voltage);
static float Measurements_OutputVoltageFromAdc(float adc_voltage);
static float Measurements_InputCurrentFromAdc(float adc_voltage);
static float Measurements_OutputCurrentFromAdc(float adc_voltage);
static bool Adc_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint16_t *raw_count);
static float Adc_RawToVolts(uint16_t raw_count);

static Measurements_t measurements_latest = {0};
static float input_current_offset_v = 0.0f;
static float output_current_offset_v = 0.0f;

/**
  * @brief  Polls all ADC feedback channels and converts them to engineering units.
  * @retval true when all ADC conversions completed successfully.
  */
bool Measurements_Update(void)
{
  bool ok = true;

  /* Read the four analogue feedback channels listed in the guide's STM32 pin map.
   * Each stored raw value is the rounded average of ADC_AVERAGE_SAMPLE_COUNT
   * conversions, which reduces random ADC noise before scaling to volts/amps. */
  ok = ok && Adc_ReadChannel(&hadc1, ADC_CHANNEL_1, &measurements_latest.i_in_raw);
  ok = ok && Adc_ReadChannel(&hadc1, ADC_CHANNEL_2, &measurements_latest.i_out_raw);
  ok = ok && Adc_ReadChannel(&hadc2, ADC_CHANNEL_1, &measurements_latest.v_out_raw);
  ok = ok && Adc_ReadChannel(&hadc2, ADC_CHANNEL_2, &measurements_latest.v_in_raw);

  /* First convert raw ADC counts to voltages at the STM32 pins. */
  measurements_latest.i_in_adc_v = Adc_RawToVolts(measurements_latest.i_in_raw);
  measurements_latest.i_out_adc_v = Adc_RawToVolts(measurements_latest.i_out_raw);
  measurements_latest.v_out_adc_v = Adc_RawToVolts(measurements_latest.v_out_raw);
  measurements_latest.v_in_adc_v = Adc_RawToVolts(measurements_latest.v_in_raw);

  /* Then convert sensor output voltages to physical converter quantities. */
  measurements_latest.i_in_a = Measurements_InputCurrentFromAdc(measurements_latest.i_in_adc_v);
  measurements_latest.i_out_a = Measurements_OutputCurrentFromAdc(measurements_latest.i_out_adc_v);
  measurements_latest.v_out_v = Measurements_OutputVoltageFromAdc(measurements_latest.v_out_adc_v);
  measurements_latest.v_in_v = Measurements_InputVoltageFromAdc(measurements_latest.v_in_adc_v);

  /* Store PV input power for telemetry/debugging. The MPPT hook calculates the
   * same quantity from its input arguments. */
  measurements_latest.pv_power_w = measurements_latest.v_in_v * measurements_latest.i_in_a;
  measurements_latest.valid = ok;
  measurements_latest.updated_ms = HAL_GetTick();

  return ok;
}

/**
  * @brief  Returns the latest measurement snapshot.
  * @retval Pointer to the module-owned latest measurement packet.
  */
const Measurements_t *Measurements_GetLatest(void)
{
  return &measurements_latest;
}

/**
  * @brief  Checks for saturated ADC readings that indicate invalid feedback.
  * @param  measurements Latest converted measurement packet.
  * @retval true when all ADC channels are inside the usable ADC range.
  */
bool Measurements_AreInAdcRange(const Measurements_t *measurements)
{
  if (measurements == NULL)
  {
    return false;
  }

  /* This check only catches obviously unusable readings close to full-scale ADC
   * saturation. Practical over-voltage and over-current limits should be added
   * separately once the approved laboratory limits are known. */
  return (measurements->valid &&
          (measurements->i_in_raw < ADC_RAW_SATURATION_COUNT) &&
          (measurements->i_out_raw < ADC_RAW_SATURATION_COUNT) &&
          (measurements->v_out_raw < ADC_RAW_SATURATION_COUNT) &&
          (measurements->v_in_raw < ADC_RAW_SATURATION_COUNT));
}

/**
  * @brief  Stores the present current-sensor ADC voltages as zero-current offsets.
  */
void Measurements_CaptureZeroCurrentOffsets(void)
{
  /* Current sensors usually do not output exactly 0 V at 0 A. Store their present
   * no-load voltages and subtract them from later current measurements. */
  input_current_offset_v = measurements_latest.i_in_adc_v;
  output_current_offset_v = measurements_latest.i_out_adc_v;
}

/**
  * @brief  Converts input-voltage sensor ADC voltage to panel/input voltage.
  * @param  adc_voltage ADC pin voltage from ADC2_IN2.
  * @retval Estimated panel/input voltage in volts.
  */
static float Measurements_InputVoltageFromAdc(float adc_voltage)
{
  /* V_in = V_ADC multiplied by the input-voltage divider/sensor scale factor. */
  return adc_voltage * INPUT_VOLTAGE_SENSOR_SCALE;
}

/**
  * @brief  Converts output-voltage sensor ADC voltage to converter output voltage.
  * @param  adc_voltage ADC pin voltage from ADC2_IN1.
  * @retval Estimated converter output voltage in volts.
  */
static float Measurements_OutputVoltageFromAdc(float adc_voltage)
{
  /* V_out = V_ADC multiplied by the output-voltage divider/sensor scale factor. */
  return adc_voltage * OUTPUT_VOLTAGE_SENSOR_SCALE;
}

/**
  * @brief  Converts input-current sensor ADC voltage to panel/input current.
  * @param  adc_voltage ADC pin voltage from ADC1_IN1.
  * @retval Estimated panel/input current in amps.
  */
static float Measurements_InputCurrentFromAdc(float adc_voltage)
{
  /* I_in = (sensor voltage - zero-current offset) / sensor sensitivity. */
  return (adc_voltage - input_current_offset_v) / INPUT_CURRENT_SENSOR_V_PER_A;
}

/**
  * @brief  Converts output-current sensor ADC voltage to converter output current.
  * @param  adc_voltage ADC pin voltage from ADC1_IN2.
  * @retval Estimated converter output current in amps.
  */
static float Measurements_OutputCurrentFromAdc(float adc_voltage)
{
  /* I_out uses the output sensor's sensitivity, which differs from the input sensor. */
  return (adc_voltage - output_current_offset_v) / OUTPUT_CURRENT_SENSOR_V_PER_A;
}

/**
  * @brief  Selects one ADC channel, averages blocking conversions, and returns
  *         the averaged raw ADC count.
  * @param  hadc ADC instance to use.
  * @param  channel ADC channel number.
  * @param  raw_count Destination for the 12-bit ADC result.
  * @retval true when the conversion completed successfully.
  */
static bool Adc_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint16_t *raw_count)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Select one ADC input channel for this averaged conversion. This project polls
   * channels one at a time to keep the scaffold explicit for students. */
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    return false;
  }

  uint32_t raw_sum = 0U;

  for (uint32_t sample = 0U; sample < ADC_AVERAGE_SAMPLE_COUNT; sample++)
  {
    if (HAL_ADC_Start(hadc) != HAL_OK)
    {
      return false;
    }

    /* This is a blocking wait with a short timeout. It is acceptable for a
     * teaching scaffold, but higher-performance versions usually use DMA or
     * interrupts. */
    if (HAL_ADC_PollForConversion(hadc, ADC_POLL_TIMEOUT_MS) != HAL_OK)
    {
      (void)HAL_ADC_Stop(hadc);
      return false;
    }

    raw_sum += HAL_ADC_GetValue(hadc);
    (void)HAL_ADC_Stop(hadc);
  }

  *raw_count = (uint16_t)((raw_sum + (ADC_AVERAGE_SAMPLE_COUNT / 2U)) /
                          ADC_AVERAGE_SAMPLE_COUNT);
  return true;
}

/**
  * @brief  Converts a 12-bit ADC count to ADC input-pin voltage.
  * @param  raw_count Raw ADC count.
  * @retval ADC input-pin voltage in volts.
  */
static float Adc_RawToVolts(uint16_t raw_count)
{
  /* For a 12-bit ADC, 4095 counts corresponds approximately to V_REF. */
  return ((float)raw_count * ADC_REFERENCE_VOLTAGE) / ADC_MAX_COUNTS;
}