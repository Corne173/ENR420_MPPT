/*
 * TemperatureSensor.h
 *
 * Temperature sensor interface
 */

#ifndef INC_TEMPERATURESENSOR_H_
#define INC_TEMPERATURESENSOR_H_

#include <stdint.h>

void TemperatureSensor_Task(void);
void TemperatureSensor_Init(void);
float TemperatureSensor_GetTemp(void);
int16_t TemperatureSensor_GetCentiC(uint8_t sensor_index);

#endif /* INC_STATUSLED_H_ */