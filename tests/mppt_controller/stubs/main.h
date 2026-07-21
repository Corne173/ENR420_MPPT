#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  FAULT_NONE = 0,
  FAULT_PWM_START
} Fault_t;

uint32_t HAL_GetTick(void);
void enterFault(Fault_t fault);

#endif
