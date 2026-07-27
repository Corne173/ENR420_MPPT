#ifndef TEST_MEASUREMENTS_H
#define TEST_MEASUREMENTS_H

#include <main.h>

typedef struct
{
  float v_in_v;
  float i_in_a;
  bool valid;
  uint32_t updated_ms;
} Measurements_t;

const Measurements_t *Measurements_GetLatest(void);

#endif
