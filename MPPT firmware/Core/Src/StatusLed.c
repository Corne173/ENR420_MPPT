/*
 * StatusLed.c
 *
 * Test-only LD3/PB3 status indication implementation.
 */

#include "StatusLed.h"


#include "gpio.h"
#include "main.h"

/*
 * LD3 test pattern configuration.
 * PB3 is shared with the PCB signal BST_STP, so this LED feature is intended
 * for MCU-only testing, not for an energised converter.
 */
#define LD3_STATUS_TEST_ENABLE     (1U)
#define LD3_IDLE_PERIOD_MS         (4000U)
#define LD3_IDLE_ON_TIME_MS        (1000U)
#define LD3_RUN_PERIOD_MS          (200U)
#define LD3_RUN_ON_TIME_MS         (100U)

static void StatusLed_ApplyPattern(uint32_t period_ms, uint32_t on_time_ms);
static void StatusLed_Set(bool led_on);

static State_t status_led_previous_state = STATE_INIT;
static uint32_t status_led_pattern_started_ms = 0U;

/**
  * @brief  Test-only LD3 status LED task.
  * @param  state Current MPPT state to display on LD3.
  *
  * LD3 is on D13/PB3. In this project PB3 is also BST_STP, so this must only be
  * used while testing without an energised converter.
  */
void StatusLed_Task(State_t state)
{
#if (LD3_STATUS_TEST_ENABLE != 0U)
  /* Restart the blink pattern whenever the displayed state changes. */
  if (state != status_led_previous_state)
  {
    status_led_previous_state = state;
    status_led_pattern_started_ms = HAL_GetTick();
  }

  switch (state)
  {
    case STATE_IDLE:
      /* Slow blink: controller is safe and waiting for a start command. */
      StatusLed_ApplyPattern(LD3_IDLE_PERIOD_MS, LD3_IDLE_ON_TIME_MS);
      break;

    case STATE_MPPT_RUN:
      /* Faster blink: MPPT updates are active. Use only during non-energised tests. */
      StatusLed_ApplyPattern(LD3_RUN_PERIOD_MS, LD3_RUN_ON_TIME_MS);
      break;

    case STATE_INIT:
    case STATE_MPPT_STARTUP:
    case STATE_FAULT:
    default:
      StatusLed_Set(false);
      break;
  }
#else
  (void)state;
  StatusLed_Set(false);
#endif
}

/**
  * @brief  Applies a repeating on/off LED pattern.
  * @param  period_ms Total pattern period in milliseconds.
  * @param  on_time_ms Time within the period for which LD3 is on.
  */
static void StatusLed_ApplyPattern(uint32_t period_ms, uint32_t on_time_ms)
{
  const uint32_t phase_ms =
      (uint32_t)(HAL_GetTick() - status_led_pattern_started_ms) % period_ms;

  StatusLed_Set(phase_ms < on_time_ms);
}

/**
  * @brief  Drives LD3 on the Nucleo board through the shared D13/PB3 pin.
  * @param  led_on true turns LD3 on, false turns LD3 off.
  */
static void StatusLed_Set(bool led_on)
{
  HAL_GPIO_WritePin(BST_STP_GPIO_Port, BST_STP_Pin,
                    led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}