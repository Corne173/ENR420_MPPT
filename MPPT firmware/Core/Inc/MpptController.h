/*
 * MpptController.h
 *
 * Converter state machine and MPPT control interface.
 */

#ifndef INC_MPPTCONTROLLER_H_
#define INC_MPPTCONTROLLER_H_

#include <stdbool.h>

typedef enum
{
  MPPT_CONTROLLER_PHASE_SETTLING = 0,
  MPPT_CONTROLLER_PHASE_SAMPLING
} MpptControllerPhase_t;

typedef struct
{
  MpptControllerPhase_t phase;
  float gain;
  float step;
  float buck_duty;
  float boost_duty;
  float reference_power_w;
  float sampled_power_w;
} MpptControllerStatus_t;

void MpptController_Update(void);
void MpptController_Startup(void);
const MpptControllerStatus_t *MpptController_GetStatus(void);
const char *MpptController_GetPhaseName(MpptControllerPhase_t phase);

#endif /* INC_MPPTCONTROLLER_H_ */
