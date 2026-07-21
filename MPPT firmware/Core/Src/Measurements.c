/*
 * Measurements.c
 *
 * PWM-synchronised ADC acquisition and conversion for the MPPT converter.
 */

#include "Measurements.h"

#include "adc.h"
#include "SerialConsole.h"
#include "tim.h"

#include <stddef.h>
#include <stdio.h>

/* ADC conversion and basic plausibility limits. */
#define ADC_REFERENCE_VOLTAGE          (3.3f)
#define ADC_MAX_COUNTS                 (4095.0f)
#define ADC_RAW_SATURATION_COUNT       (4090U)
#define ADC_AVERAGE_SAMPLE_COUNT       (16U)

/* Each ADC ring contains 32 PWM frames. The newest 16 complete frames are
 * averaged, leaving another 16-frame guard region while the snapshot is copied. */
#define ADC_DMA_WORDS_PER_FRAME        (2U)
#define ADC_DMA_FRAME_COUNT            (32U)
#define ADC_DMA_BUFFER_LENGTH          (ADC_DMA_FRAME_COUNT * ADC_DMA_WORDS_PER_FRAME)
#define ADC_AVERAGE_WORD_COUNT         (ADC_AVERAGE_SAMPLE_COUNT * ADC_DMA_WORDS_PER_FRAME)
#define ADC_FIRST_BUFFER_TIMEOUT_MS    (5U)
#define ADC_START_ATTEMPT_COUNT        (2U)
#define ADC_STALE_TIMEOUT_MS           (3U)

/* Rank order in both DMA rings. Input voltage and input current are rank 2, so
 * they are sampled together at the latest sample-and-hold point in the period. */
#define ADC1_I_OUT_WORD                (0U)
#define ADC1_I_IN_WORD                 (1U)
#define ADC2_V_OUT_WORD                (0U)
#define ADC2_V_IN_WORD                 (1U)

/* Sensor scale factors from the voltage/current sensing equations in the guide. */
#define INPUT_VOLTAGE_SENSOR_SCALE     (46.45f)
#define OUTPUT_VOLTAGE_SENSOR_SCALE    (28.78f)
#define INPUT_CURRENT_SENSOR_V_PER_A   (0.100f)
#define OUTPUT_CURRENT_SENSOR_V_PER_A  (0.0333f)

static float Measurements_InputVoltageFromAdc(float adc_voltage);
static float Measurements_OutputVoltageFromAdc(float adc_voltage);
static float Measurements_InputCurrentFromAdc(float adc_voltage);
static float Measurements_OutputCurrentFromAdc(float adc_voltage);
static float Adc_RawToVolts(uint16_t raw_count);
static Fault_t Measurements_StartSynchronizedAttempt(uint32_t attempt_number);
static void Measurements_PrintStartupDiagnostics(uint32_t attempt_number,
                                                 uint32_t adc1_words_received,
                                                 uint32_t adc2_words_received,
                                                 bool include_registers);
static bool Measurements_DmaIsArmed(ADC_HandleTypeDef *hadc,
                                    DMA_HandleTypeDef *hdma);
static bool Measurements_DmaCycleComplete(DMA_HandleTypeDef *hdma);
static void Measurements_ClearDmaCycleFlag(DMA_HandleTypeDef *hdma);
static bool Measurements_CopyLatestSamples(uint16_t *adc1_snapshot,
                                           uint16_t *adc2_snapshot);
static void Measurements_StopAcquisition(void);
static void Measurements_LatchAcquisitionFault(Fault_t fault);

static Measurements_t measurements_latest = {0};
static float input_current_offset_v = 0.0f;
static float output_current_offset_v = 0.0f;

/* These uninitialised arrays are placed in normal 0x20000000 SRAM by the linker.
 * DMA cannot access the STM32F303's 0x10000000 CCM RAM. */
static volatile uint16_t adc1_dma_buffer[ADC_DMA_BUFFER_LENGTH]
    __attribute__((aligned(4)));
static volatile uint16_t adc2_dma_buffer[ADC_DMA_BUFFER_LENGTH]
    __attribute__((aligned(4)));

static volatile Fault_t acquisition_fault = FAULT_NONE;
static volatile bool acquisition_active = false;
static uint16_t adc1_last_remaining = ADC_DMA_BUFFER_LENGTH;
static uint16_t adc2_last_remaining = ADC_DMA_BUFFER_LENGTH;
static uint32_t adc1_last_progress_ms = 0U;
static uint32_t adc2_last_progress_ms = 0U;

/**
  * @brief  Arms both ADC DMA rings, then starts their channel-3-derived TIM1 TRGO.
  * @retval FAULT_NONE on success, otherwise the exact startup stage that failed.
  */
Fault_t Measurements_StartSynchronized(void)
{
  Fault_t fault = FAULT_ADC_BOTH_FIRST_BUFFER;

  for (uint32_t attempt = 0U; attempt < ADC_START_ATTEMPT_COUNT; attempt++)
  {
    fault = Measurements_StartSynchronizedAttempt(attempt + 1U);
    if (fault == FAULT_NONE)
    {
      return FAULT_NONE;
    }

    /* A missing first buffer can be caused by a stale post-flash ADC/timer
     * handshake. The failed attempt has already stopped both ADCs and TIM1 TRGO,
     * so one complete re-arm is safe. Configuration/start failures are returned
     * immediately because repeating them cannot repair an invalid setup. */
    if ((fault != FAULT_ADC1_FIRST_BUFFER) &&
        (fault != FAULT_ADC2_FIRST_BUFFER) &&
        (fault != FAULT_ADC_BOTH_FIRST_BUFFER))
    {
      return fault;
    }
  }

  return fault;
}

/**
  * @brief  Performs one complete ADC DMA and TIM1 TRGO acquisition start attempt.
  */
static Fault_t Measurements_StartSynchronizedAttempt(uint32_t attempt_number)
{
  const uint32_t adc1_tc_flag = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_adc1);
  const uint32_t adc2_tc_flag = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_adc2);
  uint32_t adc1_words_received = 0U;
  uint32_t adc2_words_received = 0U;
  uint32_t start_ms;
  uint16_t adc1_previous_remaining;
  uint16_t adc2_previous_remaining;
  bool adc1_ready = false;
  bool adc2_ready = false;

  acquisition_active = false;
  acquisition_fault = FAULT_NONE;
  measurements_latest.valid = false;

  /* Keep both gate drivers inhibited while the timer begins producing internal
   * ADC trigger events. Channel 3 has no GPIO output. */
  HAL_GPIO_WritePin(BST_DIS_GPIO_Port, BST_DIS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BCK_DIS_GPIO_Port, BCK_DIS_Pin, GPIO_PIN_SET);

  /* Establish a deterministic first period. PWM channels 1/2 are already off. */
  (void)HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_CC3);

  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)(void *)adc1_dma_buffer,
                        ADC_DMA_BUFFER_LENGTH) != HAL_OK)
  {
    acquisition_fault = FAULT_ADC1_DMA_START;
    Measurements_StopAcquisition();
    return acquisition_fault;
  }

  /* HAL enables half/full callbacks by default. They are unnecessary at 250 kHz;
   * retain only the DMA transfer-error interrupt and poll the first full flag. */
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);
  if (!Measurements_DmaIsArmed(&hadc1, &hdma_adc1))
  {
    acquisition_fault = FAULT_ADC1_DMA_START;
    Measurements_StopAcquisition();
    return acquisition_fault;
  }

  if (HAL_ADC_Start_DMA(&hadc2,
                        (uint32_t *)(void *)adc2_dma_buffer,
                        ADC_DMA_BUFFER_LENGTH) != HAL_OK)
  {
    acquisition_fault = FAULT_ADC2_DMA_START;
    Measurements_StopAcquisition();
    return acquisition_fault;
  }

  __HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT | DMA_IT_TC);
  if (!Measurements_DmaIsArmed(&hadc2, &hdma_adc2))
  {
    acquisition_fault = FAULT_ADC2_DMA_START;
    Measurements_StopAcquisition();
    return acquisition_fault;
  }

  /* Start the timer last so neither ADC can miss the first shared trigger. */
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_CC3);
  if ((HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
      ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U) ||
      ((htim1.Instance->CCER & TIM_CCER_CC3E) == 0U))
  {
    acquisition_fault = FAULT_ADC_TRIGGER_START;
    Measurements_StopAcquisition();
    return acquisition_fault;
  }

  /* Only 16 complete frames are needed for the first published average. Track
   * DMA counter progress as the primary evidence instead of depending solely on
   * transfer-complete flags, which can be consumed or stale around a debugger
   * reset. The TC flags remain a valid shortcut when a complete ring arrives. */
  adc1_previous_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc1);
  adc2_previous_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc2);
  start_ms = HAL_GetTick();
  do
  {
    uint16_t adc1_remaining;
    uint16_t adc2_remaining;

    if (acquisition_fault != FAULT_NONE)
    {
      Measurements_StopAcquisition();
      return acquisition_fault;
    }

    adc1_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc1);
    adc2_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc2);

    adc1_words_received +=
        (adc1_previous_remaining + ADC_DMA_BUFFER_LENGTH - adc1_remaining) %
        ADC_DMA_BUFFER_LENGTH;
    adc2_words_received +=
        (adc2_previous_remaining + ADC_DMA_BUFFER_LENGTH - adc2_remaining) %
        ADC_DMA_BUFFER_LENGTH;
    adc1_previous_remaining = adc1_remaining;
    adc2_previous_remaining = adc2_remaining;

    adc1_ready =
        (adc1_words_received >= ADC_AVERAGE_WORD_COUNT) ||
        ((DMA1->ISR & adc1_tc_flag) != 0U);
    adc2_ready =
        (adc2_words_received >= ADC_AVERAGE_WORD_COUNT) ||
        ((DMA1->ISR & adc2_tc_flag) != 0U);
    if (adc1_ready && adc2_ready)
    {
      break;
    }
  }
  while ((HAL_GetTick() - start_ms) < ADC_FIRST_BUFFER_TIMEOUT_MS);

  if (!adc1_ready || !adc2_ready)
  {
    Measurements_PrintStartupDiagnostics(attempt_number,
                                         adc1_words_received,
                                         adc2_words_received,
                                         attempt_number >= ADC_START_ATTEMPT_COUNT);

    if (!adc1_ready && !adc2_ready)
    {
      acquisition_fault = FAULT_ADC_BOTH_FIRST_BUFFER;
    }
    else
    {
      acquisition_fault = !adc1_ready ? FAULT_ADC1_FIRST_BUFFER
                                      : FAULT_ADC2_FIRST_BUFFER;
    }
    Measurements_StopAcquisition();
    return acquisition_fault;
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_adc1, adc1_tc_flag);
  __HAL_DMA_CLEAR_FLAG(&hdma_adc2, adc2_tc_flag);

  adc1_last_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc1);
  adc2_last_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc2);
  adc1_last_progress_ms = HAL_GetTick();
  adc2_last_progress_ms = adc1_last_progress_ms;
  acquisition_active = true;

  /* Publish a real measurement before the controller captures zero-current offsets. */
  if (!Measurements_Update())
  {
    if (acquisition_fault == FAULT_NONE)
    {
      acquisition_fault = FAULT_ADC_READ;
    }
    Measurements_StopAcquisition();
  }

  return acquisition_fault;
}

/**
  * @brief  Prints targeted state only when the first ADC samples do not arrive.
  */
static void Measurements_PrintStartupDiagnostics(uint32_t attempt_number,
                                                 uint32_t adc1_words_received,
                                                 uint32_t adc2_words_received,
                                                 bool include_registers)
{
  char line[220];
  int length;

  length = snprintf(line,
                    sizeof(line),
                    "ADCDBG attempt=%lu words=%lu/%lu remaining=%lu/%lu%s\r\n",
                    (unsigned long)attempt_number,
                    (unsigned long)adc1_words_received,
                    (unsigned long)adc2_words_received,
                    (unsigned long)__HAL_DMA_GET_COUNTER(&hdma_adc1),
                    (unsigned long)__HAL_DMA_GET_COUNTER(&hdma_adc2),
                    include_registers ? " final" : " retry");
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    SerialConsole_Send(line);
  }

  if (!include_registers)
  {
    return;
  }

  length = snprintf(line,
                    sizeof(line),
                    "ADCDBG TIM cr1=%08lX cr2=%08lX ccmr2=%08lX ccer=%08lX cnt=%lu ccr3=%lu sr=%08lX\r\n",
                    (unsigned long)htim1.Instance->CR1,
                    (unsigned long)htim1.Instance->CR2,
                    (unsigned long)htim1.Instance->CCMR2,
                    (unsigned long)htim1.Instance->CCER,
                    (unsigned long)htim1.Instance->CNT,
                    (unsigned long)htim1.Instance->CCR3,
                    (unsigned long)htim1.Instance->SR);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    SerialConsole_Send(line);
  }

  length = snprintf(line,
                    sizeof(line),
                    "ADCDBG ADC cr=%08lX/%08lX cfgr=%08lX/%08lX isr=%08lX/%08lX DMA ccr=%08lX/%08lX isr=%08lX\r\n",
                    (unsigned long)hadc1.Instance->CR,
                    (unsigned long)hadc2.Instance->CR,
                    (unsigned long)hadc1.Instance->CFGR,
                    (unsigned long)hadc2.Instance->CFGR,
                    (unsigned long)hadc1.Instance->ISR,
                    (unsigned long)hadc2.Instance->ISR,
                    (unsigned long)hdma_adc1.Instance->CCR,
                    (unsigned long)hdma_adc2.Instance->CCR,
                    (unsigned long)DMA1->ISR);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    SerialConsole_Send(line);
  }
}

/**
  * @brief  Returns the first latched acquisition failure.
  */
Fault_t Measurements_GetAcquisitionFault(void)
{
  return (Fault_t)acquisition_fault;
}

/**
  * @brief  Safely stops both ADC DMA streams and their shared trigger.
  * @note   Used before calibration and deliberate fault recovery.
  */
void Measurements_StopSynchronized(void)
{
  Measurements_StopAcquisition();
  measurements_latest.valid = false;
}

/**
  * @brief  Averages the newest 16 PWM-synchronised frames and converts them.
  * @retval true when both DMA streams are active, fresh, and copied coherently.
  */
bool Measurements_Update(void)
{
  uint16_t adc1_snapshot[ADC_AVERAGE_WORD_COUNT];
  uint16_t adc2_snapshot[ADC_AVERAGE_WORD_COUNT];
  uint32_t i_in_sum = 0U;
  uint32_t i_out_sum = 0U;
  uint32_t v_in_sum = 0U;
  uint32_t v_out_sum = 0U;
  uint32_t now_ms;
  uint16_t adc1_remaining;
  uint16_t adc2_remaining;
  bool adc1_wrapped;
  bool adc2_wrapped;

  if (!acquisition_active || (acquisition_fault != FAULT_NONE))
  {
    measurements_latest.valid = false;
    return false;
  }

  /* Catch a DMA/ADC error even if its interrupt has not yet been dispatched. */
  if (((HAL_DMA_GetError(&hdma_adc1) & HAL_DMA_ERROR_TE) != 0U) ||
      ((HAL_ADC_GetError(&hadc1) & HAL_ADC_ERROR_DMA) != 0U))
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC1_DMA_TRANSFER);
    measurements_latest.valid = false;
    return false;
  }
  if (((HAL_DMA_GetError(&hdma_adc2) & HAL_DMA_ERROR_TE) != 0U) ||
      ((HAL_ADC_GetError(&hadc2) & HAL_ADC_ERROR_DMA) != 0U))
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC2_DMA_TRANSFER);
    measurements_latest.valid = false;
    return false;
  }
  if ((HAL_ADC_GetError(&hadc1) & HAL_ADC_ERROR_OVR) != 0U)
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC1_OVERRUN);
    measurements_latest.valid = false;
    return false;
  }
  if ((HAL_ADC_GetError(&hadc2) & HAL_ADC_ERROR_OVR) != 0U)
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC2_OVERRUN);
    measurements_latest.valid = false;
    return false;
  }

  if (((hdma_adc1.Instance->CCR & DMA_CCR_EN) == 0U) ||
      ((hadc1.Instance->CR & ADC_CR_ADSTART) == 0U))
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC1_STALE);
    measurements_latest.valid = false;
    return false;
  }
  if (((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U) ||
      ((htim1.Instance->CCER & TIM_CCER_CC3E) == 0U))
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC_TRIGGER_LOST);
    measurements_latest.valid = false;
    return false;
  }
  if (((hdma_adc2.Instance->CCR & DMA_CCR_EN) == 0U) ||
      ((hadc2.Instance->CR & ADC_CR_ADSTART) == 0U))
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC2_STALE);
    measurements_latest.valid = false;
    return false;
  }

  now_ms = HAL_GetTick();
  adc1_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc1);
  adc2_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc2);
  adc1_wrapped = Measurements_DmaCycleComplete(&hdma_adc1);
  adc2_wrapped = Measurements_DmaCycleComplete(&hdma_adc2);

  if ((adc1_remaining != adc1_last_remaining) || adc1_wrapped)
  {
    adc1_last_remaining = adc1_remaining;
    adc1_last_progress_ms = now_ms;
    if (adc1_wrapped)
    {
      Measurements_ClearDmaCycleFlag(&hdma_adc1);
    }
  }
  else if ((now_ms - adc1_last_progress_ms) > ADC_STALE_TIMEOUT_MS)
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC1_STALE);
    measurements_latest.valid = false;
    return false;
  }

  if ((adc2_remaining != adc2_last_remaining) || adc2_wrapped)
  {
    adc2_last_remaining = adc2_remaining;
    adc2_last_progress_ms = now_ms;
    if (adc2_wrapped)
    {
      Measurements_ClearDmaCycleFlag(&hdma_adc2);
    }
  }
  else if ((now_ms - adc2_last_progress_ms) > ADC_STALE_TIMEOUT_MS)
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC2_STALE);
    measurements_latest.valid = false;
    return false;
  }

  if (!Measurements_CopyLatestSamples(adc1_snapshot, adc2_snapshot))
  {
    Measurements_LatchAcquisitionFault(FAULT_ADC_SNAPSHOT);
    measurements_latest.valid = false;
    return false;
  }

  for (uint32_t frame = 0U; frame < ADC_AVERAGE_SAMPLE_COUNT; frame++)
  {
    const uint32_t word = frame * ADC_DMA_WORDS_PER_FRAME;
    i_out_sum += adc1_snapshot[word + ADC1_I_OUT_WORD];
    i_in_sum += adc1_snapshot[word + ADC1_I_IN_WORD];
    v_out_sum += adc2_snapshot[word + ADC2_V_OUT_WORD];
    v_in_sum += adc2_snapshot[word + ADC2_V_IN_WORD];
  }

  measurements_latest.i_in_raw =
      (uint16_t)((i_in_sum + (ADC_AVERAGE_SAMPLE_COUNT / 2U)) /
                 ADC_AVERAGE_SAMPLE_COUNT);
  measurements_latest.i_out_raw =
      (uint16_t)((i_out_sum + (ADC_AVERAGE_SAMPLE_COUNT / 2U)) /
                 ADC_AVERAGE_SAMPLE_COUNT);
  measurements_latest.v_out_raw =
      (uint16_t)((v_out_sum + (ADC_AVERAGE_SAMPLE_COUNT / 2U)) /
                 ADC_AVERAGE_SAMPLE_COUNT);
  measurements_latest.v_in_raw =
      (uint16_t)((v_in_sum + (ADC_AVERAGE_SAMPLE_COUNT / 2U)) /
                 ADC_AVERAGE_SAMPLE_COUNT);

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
  measurements_latest.pv_power_w = measurements_latest.v_in_v * measurements_latest.i_in_a;
  measurements_latest.valid = true;
  measurements_latest.updated_ms = now_ms;

  return true;
}

/**
  * @brief  Returns the latest measurement snapshot.
  */
const Measurements_t *Measurements_GetLatest(void)
{
  return &measurements_latest;
}

/**
  * @brief  Checks for saturated ADC readings that indicate invalid feedback.
  */
bool Measurements_AreInAdcRange(const Measurements_t *measurements)
{
  if (measurements == NULL)
  {
    return false;
  }

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
  input_current_offset_v = measurements_latest.i_in_adc_v;
  output_current_offset_v = measurements_latest.i_out_adc_v;
}

/**
  * @brief  HAL callback for ADC overrun and DMA transfer failures.
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  Fault_t fault;
  const uint32_t error = HAL_ADC_GetError(hadc);

  if (hadc == &hadc1)
  {
    fault = ((error & HAL_ADC_ERROR_DMA) != 0U) ? FAULT_ADC1_DMA_TRANSFER
          : ((error & HAL_ADC_ERROR_OVR) != 0U) ? FAULT_ADC1_OVERRUN
                                                : FAULT_ADC1_INTERNAL;
  }
  else if (hadc == &hadc2)
  {
    fault = ((error & HAL_ADC_ERROR_DMA) != 0U) ? FAULT_ADC2_DMA_TRANSFER
          : ((error & HAL_ADC_ERROR_OVR) != 0U) ? FAULT_ADC2_OVERRUN
                                                : FAULT_ADC2_INTERNAL;
  }
  else
  {
    return;
  }

  Measurements_LatchAcquisitionFault(fault);
}

/**
  * @brief  Verifies the state that the local HAL should establish after Start_DMA.
  * @note   This check is required because this HAL ignores HAL_DMA_Start_IT's return.
  */
static bool Measurements_DmaIsArmed(ADC_HandleTypeDef *hadc,
                                    DMA_HandleTypeDef *hdma)
{
  return ((HAL_DMA_GetState(hdma) == HAL_DMA_STATE_BUSY) &&
          ((hdma->Instance->CCR & DMA_CCR_EN) != 0U) &&
          (__HAL_DMA_GET_COUNTER(hdma) == ADC_DMA_BUFFER_LENGTH) &&
          ((hadc->Instance->CFGR & ADC_CFGR_DMAEN) != 0U) &&
          ((hadc->Instance->CR & ADC_CR_ADSTART) != 0U));
}

static bool Measurements_DmaCycleComplete(DMA_HandleTypeDef *hdma)
{
  return (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma)) != 0U);
}

static void Measurements_ClearDmaCycleFlag(DMA_HandleTypeDef *hdma)
{
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
}

/**
  * @brief  Copies the newest 16 complete frames from each 32-frame DMA ring.
  * @retval true unless DMA advanced into the selected guard region while copying.
  */
static bool Measurements_CopyLatestSamples(uint16_t *adc1_snapshot,
                                           uint16_t *adc2_snapshot)
{
  for (uint32_t attempt = 0U; attempt < 4U; attempt++)
  {
    uint32_t adc1_write_before;
    uint32_t adc2_write_before;
    uint32_t adc1_boundary;
    uint32_t adc2_boundary;
    uint32_t adc1_start;
    uint32_t adc2_start;
    uint32_t adc1_write_after;
    uint32_t adc2_write_after;
    uint32_t adc1_advance;
    uint32_t adc2_advance;
    uint32_t adc1_index;
    uint32_t adc2_index;

    adc1_write_before =
        (ADC_DMA_BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(&hdma_adc1)) %
        ADC_DMA_BUFFER_LENGTH;
    adc2_write_before =
        (ADC_DMA_BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(&hdma_adc2)) %
        ADC_DMA_BUFFER_LENGTH;

    /* If rank 1 has arrived but rank 2 has not, exclude that partial frame. */
    adc1_boundary = adc1_write_before & ~(ADC_DMA_WORDS_PER_FRAME - 1U);
    adc2_boundary = adc2_write_before & ~(ADC_DMA_WORDS_PER_FRAME - 1U);

    /* DMA1 services the two simultaneous ADC requests one after the other.
     * During that very short arbitration window one ring can be one complete
     * frame ahead. Retry so voltage/current snapshots always end at the same
     * PWM event. */
    if (adc1_boundary != adc2_boundary)
    {
      continue;
    }

    adc1_start = (adc1_boundary + ADC_DMA_BUFFER_LENGTH - ADC_AVERAGE_WORD_COUNT) %
                 ADC_DMA_BUFFER_LENGTH;
    adc2_start = (adc2_boundary + ADC_DMA_BUFFER_LENGTH - ADC_AVERAGE_WORD_COUNT) %
                 ADC_DMA_BUFFER_LENGTH;

    adc1_index = adc1_start;
    adc2_index = adc2_start;
    for (uint32_t word = 0U; word < ADC_AVERAGE_WORD_COUNT; word++)
    {
      adc1_snapshot[word] = adc1_dma_buffer[adc1_index];
      adc2_snapshot[word] = adc2_dma_buffer[adc2_index];

      adc1_index++;
      adc2_index++;
      if (adc1_index == ADC_DMA_BUFFER_LENGTH)
      {
        adc1_index = 0U;
      }
      if (adc2_index == ADC_DMA_BUFFER_LENGTH)
      {
        adc2_index = 0U;
      }
    }

    __DMB();
    adc1_write_after =
        (ADC_DMA_BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(&hdma_adc1)) %
        ADC_DMA_BUFFER_LENGTH;
    adc2_write_after =
        (ADC_DMA_BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(&hdma_adc2)) %
        ADC_DMA_BUFFER_LENGTH;
    adc1_advance = (adc1_write_after + ADC_DMA_BUFFER_LENGTH - adc1_write_before) %
                   ADC_DMA_BUFFER_LENGTH;
    adc2_advance = (adc2_write_after + ADC_DMA_BUFFER_LENGTH - adc2_write_before) %
                   ADC_DMA_BUFFER_LENGTH;

    /* The unused half-ring gives 32 DMA writes of protection. */
    if ((adc1_advance < ADC_AVERAGE_WORD_COUNT) &&
        (adc2_advance < ADC_AVERAGE_WORD_COUNT))
    {
      return true;
    }
  }

  return false;
}

/**
  * @brief  Stops trigger generation and both ADC DMA streams during startup cleanup.
  */
static void Measurements_StopAcquisition(void)
{
  acquisition_active = false;
  (void)HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_ADC_Stop_DMA(&hadc1);
  (void)HAL_ADC_Stop_DMA(&hadc2);
}

/**
  * @brief  Latches a runtime acquisition failure and makes the power stage safe.
  * @note   This function is safe to call from an ADC/DMA interrupt callback.
  */
static void Measurements_LatchAcquisitionFault(Fault_t fault)
{
  if (acquisition_fault == FAULT_NONE)
  {
    acquisition_fault = fault;
  }

  acquisition_active = false;

  /* Inhibit the external drivers before freezing the PWM/trigger timer. */
  HAL_GPIO_WritePin(BST_DIS_GPIO_Port, BST_DIS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BCK_DIS_GPIO_Port, BCK_DIS_Pin, GPIO_PIN_SET);
  CLEAR_BIT(htim1.Instance->CCER, TIM_CCER_CC3E);
  CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_CEN);
}

static float Measurements_InputVoltageFromAdc(float adc_voltage)
{
  return adc_voltage * INPUT_VOLTAGE_SENSOR_SCALE;
}

static float Measurements_OutputVoltageFromAdc(float adc_voltage)
{
  return adc_voltage * OUTPUT_VOLTAGE_SENSOR_SCALE;
}

static float Measurements_InputCurrentFromAdc(float adc_voltage)
{
  return (adc_voltage - input_current_offset_v) / INPUT_CURRENT_SENSOR_V_PER_A;
}

static float Measurements_OutputCurrentFromAdc(float adc_voltage)
{
  return (adc_voltage - output_current_offset_v) / OUTPUT_CURRENT_SENSOR_V_PER_A;
}

static float Adc_RawToVolts(uint16_t raw_count)
{
  return ((float)raw_count * ADC_REFERENCE_VOLTAGE) / ADC_MAX_COUNTS;
}
