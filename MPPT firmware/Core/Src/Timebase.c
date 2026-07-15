/*
 * Timebase.c
 *
 * Wrap-safe timing helpers for cooperative firmware tasks.
 */

#include "Timebase.h"

/**
  * @brief  Checks elapsed time using unsigned tick arithmetic.
  * @param  now_ms Current HAL tick in milliseconds.
  * @param  last_ms Previous task execution tick.
  * @param  period_ms Required task period in milliseconds.
  * @retval true when the requested period has elapsed.
  */
bool Timebase_HasElapsed(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms)
{
  /* Unsigned subtraction still works correctly when HAL_GetTick() wraps around. */
  return ((uint32_t)(now_ms - last_ms) >= period_ms);
}