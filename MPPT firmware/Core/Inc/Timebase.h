/*
 * Timebase.h
 *
 * Wrap-safe timing helpers for cooperative firmware tasks.
 */

#ifndef INC_TIMEBASE_H_
#define INC_TIMEBASE_H_

#include <stdbool.h>
#include <stdint.h>

bool Timebase_HasElapsed(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms);

#endif /* INC_TIMEBASE_H_ */