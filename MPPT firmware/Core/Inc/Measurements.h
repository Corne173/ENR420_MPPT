/*
 * Measurements.h
 *
 * ADC measurement acquisition and conversion for the MPPT converter.
 */

#ifndef INC_MEASUREMENTS_H_
#define INC_MEASUREMENTS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * One snapshot of the converter measurements.
 *
 * Each physical signal appears in three forms:
 *   1. raw ADC counts, useful for debugging ADC configuration;
 *   2. ADC pin voltage, useful for checking sensor interface circuits;
 *   3. engineering units, used by the MPPT and fault logic.
 */
typedef struct
{
  /* Averaged raw 12-bit ADC results from the STM32 ADC peripherals. */
  uint16_t i_in_raw;
  uint16_t i_out_raw;
  uint16_t v_out_raw;
  uint16_t v_in_raw;

  /* Voltages actually present at the STM32 ADC pins, in the 0 V to 3.3 V range. */
  float i_in_adc_v;
  float i_out_adc_v;
  float v_out_adc_v;
  float v_in_adc_v;

  /* Converted quantities at the converter terminals. */
  float i_in_a;
  float i_out_a;
  float v_out_v;
  float v_in_v;
  float pv_power_w;

  /* Validity and timestamp help the control code reject stale or failed reads. */
  bool valid;
  uint32_t updated_ms;
} Measurements_t;

bool Measurements_Update(void);
const Measurements_t *Measurements_GetLatest(void);
void Measurements_CaptureZeroCurrentOffsets(void);
bool Measurements_AreInAdcRange(const Measurements_t *measurements);

#endif /* INC_MEASUREMENTS_H_ */