/*
 * ModbusRtu.h
 *
 * Reusable Modbus RTU frame helpers.
 */

#ifndef INC_MODBUSRTU_H_
#define INC_MODBUSRTU_H_

#include <stddef.h>
#include <stdint.h>

uint16_t ModbusRtu_Crc16(const uint8_t *data, size_t length);
void ModbusRtu_StoreU16BigEndian(uint8_t *destination, uint16_t value);
uint16_t ModbusRtu_ReadU16BigEndian(const uint8_t *source);

#endif /* INC_MODBUSRTU_H_ */