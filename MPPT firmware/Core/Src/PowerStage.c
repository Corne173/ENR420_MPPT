/*
 * PowerStage.c
 *
 *  Created on: 11 Jun 2026
 *      Author: Lauren
 */
#include "PowerStage.h"
#include "tim.h"

#define POWER_STAGE_DUTY_MIN (0.0f)
#define POWER_STAGE_DUTY_MAX (0.95f)

static uint32_t Pwm_DutyToCompare(float duty);
static float PowerStage_ClampFloat(float value, float min_value, float max_value);

static bool power_stage_pwm_running = false;

void PowerStage_Disable(void)
{
  /* The gate-driver disable pins are active high. Set them first so the driver
   * outputs are disabled before PWM registers or timer channels are changed. */
  HAL_GPIO_WritePin(BST_DIS_GPIO_Port, BST_DIS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BCK_DIS_GPIO_Port, BCK_DIS_Pin, GPIO_PIN_SET);

  /* A compare value of zero requests 0% duty on both PWM channels. */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);

  /* Stop both the main and complementary outputs for the two TIM1 channels. */
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  power_stage_pwm_running = false;
}

/**
  * @brief  Starts TIM1 main/complementary PWM outputs and releases the gate-driver
  *         disable pins.
  * @retval true when PWM outputs are running and drivers are enabled.
  */
bool PowerStage_Enable(void)
{
  if (!power_stage_pwm_running)
  {
    /* Start the high-side/low-side complementary outputs before releasing the
     * gate-driver disable pins. If any start call fails, return to the safe state. */
    if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK))
    {
      PowerStage_Disable();
      return false;
    }

    power_stage_pwm_running = true;
  }

  /* RESET clears the active-high disable signal, allowing the drivers to switch. */
  HAL_GPIO_WritePin(BST_DIS_GPIO_Port, BST_DIS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BCK_DIS_GPIO_Port, BCK_DIS_Pin, GPIO_PIN_RESET);
  return true;
}

/**
  * @brief  Applies buck and boost duty commands to TIM1 compare registers.
  * @param  buck_duty Duty command for the buck bridge.
  * @param  boost_duty Duty command for the boost bridge.
  */
void PowerStage_SetDuty(float buck_duty, float boost_duty)
{
  /* Clamp first so no algorithm can request an unsafe 100% duty or a negative duty. */
  const float clamped_buck_duty =
      PowerStage_ClampFloat(buck_duty, POWER_STAGE_DUTY_MIN, POWER_STAGE_DUTY_MAX);
  const float clamped_boost_duty =
      PowerStage_ClampFloat(boost_duty, POWER_STAGE_DUTY_MIN, POWER_STAGE_DUTY_MAX);

  /* TIM1 channel 1 drives the buck bridge command; TIM1 channel 2 drives the
   * boost bridge command. CubeMX configures the complementary outputs/dead time. */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Pwm_DutyToCompare(clamped_buck_duty));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Pwm_DutyToCompare(clamped_boost_duty));
}

/**
  * @brief  Converts a normalized duty value into a TIM1 compare value.
  * @param  duty Duty command from 0.0 to 1.0 before clamping.
  * @retval Timer compare value corresponding to the requested duty.
  */
static uint32_t Pwm_DutyToCompare(float duty)
{
  /* The auto-reload register is the timer period. The compare register selects
   * where within that period the PWM output changes state. */
  const uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t compare =
      (uint32_t)(PowerStage_ClampFloat(duty,
                                       POWER_STAGE_DUTY_MIN,
                                       POWER_STAGE_DUTY_MAX) *
                 (float)(auto_reload + 1U));

  /* Prevent a compare value larger than the timer period. */
  if (compare > auto_reload)
  {
    compare = auto_reload;
  }

  return compare;
}

/**
  * @brief  Limits a floating-point value to a minimum and maximum.
  * @param  value Value to clamp.
  * @param  min_value Lower allowed value.
  * @param  max_value Upper allowed value.
  * @retval Clamped value.
  */
static float PowerStage_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}
