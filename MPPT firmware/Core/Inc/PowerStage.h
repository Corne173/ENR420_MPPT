/*
 * PowerStage.h
 *
 *  Created on: 11 Jun 2026
 *      Author: Lauren
 */

#ifndef INC_POWERSTAGE_H_
#define INC_POWERSTAGE_H_

#include <stdbool.h>

#include "main.h"

/* Public command limits let control algorithms avoid requesting duties that
 * would be silently clipped by the power-stage layer. */
#define POWER_STAGE_DUTY_MIN (0.0f)
#define POWER_STAGE_DUTY_MAX (0.95f)

void PowerStage_Disable(void);
bool PowerStage_Enable(void);
void PowerStage_SetDuty(float buck_duty, float boost_duty);

#endif /* INC_POWERSTAGE_H_ */
