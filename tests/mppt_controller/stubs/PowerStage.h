#ifndef TEST_POWER_STAGE_H
#define TEST_POWER_STAGE_H

#include <main.h>

#define POWER_STAGE_DUTY_MIN (0.0f)
#define POWER_STAGE_DUTY_MAX (0.95f)

bool PowerStage_Enable(void);
void PowerStage_SetDuty(float buck_duty, float boost_duty);

#endif
