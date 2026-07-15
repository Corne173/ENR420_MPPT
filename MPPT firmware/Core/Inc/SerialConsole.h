/*
 * SerialConsole.h
 *
 * USART2 PC-console command and status interface.
 */

#ifndef INC_SERIALCONSOLE_H_
#define INC_SERIALCONSOLE_H_

#include "usart.h"

void SerialConsole_Init(void);
void SerialConsole_Task(void);

void SerialConsole_PrintBootMessage(void);
void SerialConsole_PrintStatus(void);

void SerialConsole_Send(const char *text);
void SerialConsole_SendLine(const char *text);

void SerialConsole_RxCpltCallback(UART_HandleTypeDef *huart);
void SerialConsole_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* INC_SERIALCONSOLE_H_ */