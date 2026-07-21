/*
 * IrradianceSensor.h
 *
 * RS485/Modbus irradiance sensor polling interface.
 */

#ifndef INC_IRRADIANCESENSOR_H_
#define INC_IRRADIANCESENSOR_H_

#include <usart.h>

#include <stdbool.h>

void IrradianceSensor_Init(void);
void IrradianceSensor_Task(void);
bool IrradianceSensor_IsValid(void);
float IrradianceSensor_GetWPerM2(void);

void IrradianceSensor_TxCpltCallback(UART_HandleTypeDef *huart);
void IrradianceSensor_RxCpltCallback(UART_HandleTypeDef *huart);
void IrradianceSensor_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* INC_IRRADIANCESENSOR_H_ */
