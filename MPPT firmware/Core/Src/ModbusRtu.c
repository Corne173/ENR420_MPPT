/*
 * ModbusRtu.c
 *
 * Reusable Modbus RTU frame helpers.
 */

#include "ModbusRtu.h"

/**
  * @brief  Calculates the Modbus RTU CRC-16 over a frame.
  * @param  data Frame bytes excluding the CRC field.
  * @param  length Number of bytes to process.
  * @retval Modbus CRC value before low-byte/high-byte frame ordering.
  */
uint16_t ModbusRtu_Crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;

  for (size_t byte_index = 0U; byte_index < length; byte_index++)
  {
    crc ^= data[byte_index];

    for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/**
  * @brief  Stores a 16-bit value in Modbus big-endian register order.
  */
void ModbusRtu_StoreU16BigEndian(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)((value >> 8) & 0x00FFU);
  destination[1] = (uint8_t)(value & 0x00FFU);
}

/**
  * @brief  Reads a 16-bit value from Modbus big-endian register order.
  */
uint16_t ModbusRtu_ReadU16BigEndian(const uint8_t *source)
{
  return (uint16_t)(((uint16_t)source[0] << 8) | source[1]);
}