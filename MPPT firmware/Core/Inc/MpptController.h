/*
 * MpptController.h
 *
 * Converter state machine and MPPT control interface.
 */

#ifndef INC_MPPTCONTROLLER_H_
#define INC_MPPTCONTROLLER_H_

#include <stdbool.h>

void MpptController_Update(void);
void MpptController_Startup(void);

#endif /* INC_MPPTCONTROLLER_H_ */