/*
 * IrradianceSensor.c
 *
 * RS485/Modbus irradiance sensor polling implementation.
 */

#include "IrradianceSensor.h"

#include "ModbusRtu.h"
#include "Timebase.h"
#include "gpio.h"
#include "usart.h"

#include <stddef.h>
#include <stdint.h>

/*
 * Irradiance sensor Modbus transaction configuration.
 *
 * One Modbus RTU poll works as follows:
 *   1. The task waits for IRRADIANCE_TASK_PERIOD_MS so the sensor is not queried
 *      on every pass through the main loop.
 *   2. IrradianceSensor_BuildReadRequest() creates this request frame:
 *        [slave ID][function][start register][register count][CRC low][CRC high]
 *      The start register and register count are sent most-significant byte
 *      first. The CRC is calculated by the firmware and appended automatically.
 *   3. The RS485 transceiver is placed in transmit mode and HAL_UART_Transmit()
 *      sends the request on USART1.
 *   4. The transceiver is returned to receive mode and HAL_UART_Receive() waits
 *      for the sensor response:
 *        [slave ID][function][byte count][data bytes][CRC low][CRC high]
 *   5. IrradianceSensor_ParseReadResponse() checks the echoed slave ID and
 *      function code, confirms the byte count and CRC, then converts the returned
 *      register data to W/m^2.
 *
 * Treat the numeric values below as scaffold values until they have been checked
 * against the guide and the specific sensor's Modbus table.
 */
#define IRRADIANCE_SENSOR_ENABLE                    (1U)
#define IRRADIANCE_MODBUS_SLAVE_ID                  (0x01U)       /* Check: sensor Modbus address / slave ID. */
#define IRRADIANCE_MODBUS_FUNCTION_CODE             (0x03U)    /* Check: read function code for this register type. */
#define IRRADIANCE_MODBUS_START_REGISTER            (0x0000U)  /* Check: first irradiance register address. */
#define IRRADIANCE_MODBUS_REGISTER_COUNT            (1U)       /* Check: number of 16-bit registers returned. */
#define IRRADIANCE_MODBUS_REQUEST_LENGTH            (8U)
#define IRRADIANCE_MODBUS_RESPONSE_MAX_LENGTH       (32U)
#define IRRADIANCE_MODBUS_TIMEOUT_MS                (100U)     /* Check/test: maximum wait for one reply. */
#define IRRADIANCE_RAW_TO_W_PER_M2_SCALE            (1.0f)     /* Check: raw register to W/m^2 scale factor. */

static size_t IrradianceSensor_BuildReadRequest(uint8_t *request, size_t request_size);
static bool IrradianceSensor_ParseReadResponse(const uint8_t *response,
                                               size_t response_length,
                                               float *irradiance_out_w_m2);
static void Rs485_SetTransmitMode(void);
static void Rs485_SetReceiveMode(void);

static float irradiance_w_m2 = 0.0f;
static bool irradiance_valid = false;

/**
  * @brief  Places the RS485 transceiver in receive mode and clears the local
  *         irradiance measurement status.
  */
void IrradianceSensor_Init(void)
{
  Rs485_SetReceiveMode();
  irradiance_w_m2 = 0.0f;
  irradiance_valid = false;
}

/**
  * @brief  Student scaffold for polling an irradiance sensor over USART1/RS485
  *         using Modbus RTU.
  *
  * This task is called from the main loop. It is disabled by default; enable it
  * only after confirming the constants and USART1/RS485 settings documented in
  * the irradiance configuration block near the top of this file.
  *
  * When enabling it, keep the blocking UART calls short. A production controller
  * should normally move Modbus receive handling to interrupts, DMA, or a small
  * non-blocking state machine so that MPPT timing is not disturbed.
  */
void IrradianceSensor_Task(void)
{
  uint8_t request[IRRADIANCE_MODBUS_REQUEST_LENGTH] = {0};
  uint8_t response[IRRADIANCE_MODBUS_RESPONSE_MAX_LENGTH] = {0};
  size_t request_length;
  size_t expected_response_length;

  /* Keep the scaffold compiled but inactive until the sensor constants and UART
   * settings have been checked. Set IRRADIANCE_SENSOR_ENABLE to 1 only after that. */
  if (IRRADIANCE_SENSOR_ENABLE == 0U)
  {
    return;
  }

  

  /* A normal Modbus read response contains:
   *   slave ID, function code, byte count, data bytes, CRC low, CRC high.
   * Each requested register contributes two data bytes. For example, reading one
   * 16-bit register gives 5 + 2*1 = 7 response bytes. */
  expected_response_length =
      5U + ((size_t)IRRADIANCE_MODBUS_REGISTER_COUNT * 2U);

  if (expected_response_length > sizeof(response))
  {
    irradiance_valid = false;
    return;
  }

  /* Build request[] from the constants near the top of this file. This helper
   * also inserts the correct Modbus byte order and appends the CRC. */
  request_length = IrradianceSensor_BuildReadRequest(request, sizeof(request));
  if (request_length == 0U)
  {
    irradiance_valid = false;
    return;
  }

  /* RS485 is half-duplex on this board. Enable transmit mode before driving the bus. */
  Rs485_SetTransmitMode();

  /* Send the complete Modbus request frame on USART1. request_length is normally
   * 8 bytes for a standard single-register read request. */
  if (HAL_UART_Transmit(&huart1,
                        request,
                        (uint16_t)request_length,
                        IRRADIANCE_MODBUS_TIMEOUT_MS) != HAL_OK)
  {
    Rs485_SetReceiveMode();
    irradiance_valid = false;
    return;
  }

  /* After the request has been sent, release the bus driver and enable receiving
   * so the sensor can reply on the same RS485 pair. */
  Rs485_SetReceiveMode();

  /* Read the expected number of response bytes into response[]. For a one-register
   * read this is usually 7 bytes: address, function, byte count, 2 data bytes, CRC. */
  if (HAL_UART_Receive(&huart1,
                       response,
                       (uint16_t)expected_response_length,
                       IRRADIANCE_MODBUS_TIMEOUT_MS) != HAL_OK)
  {
    irradiance_valid = false;
    return;
  }

  /* Validate the reply, check its CRC, and convert the returned data register to
   * the engineering value irradiance_w_m2. */
  if (!IrradianceSensor_ParseReadResponse(response,
                                          expected_response_length,
                                          &irradiance_w_m2))
  {
    irradiance_valid = false;
    return;
  }

  /* Reaching this point means the complete request/response transaction succeeded. */
  irradiance_valid = true;
}

bool IrradianceSensor_IsValid(void)
{
  return irradiance_valid;
}

float IrradianceSensor_GetWPerM2(void)
{
  return irradiance_w_m2;
}

/**
  * @brief  Builds a Modbus RTU read-register request frame.
  *
  * Students normally do not need to type raw Modbus request bytes by hand. This
  * helper constructs the request from the constants at the top of the file and
  * appends the CRC automatically.
  *
  * @param  request Destination buffer.
  * @param  request_size Size of destination buffer in bytes.
  * @retval Number of bytes written, or 0 if the buffer is too small.
  */
static size_t IrradianceSensor_BuildReadRequest(uint8_t *request, size_t request_size)
{
  uint16_t crc;

  if (request_size < IRRADIANCE_MODBUS_REQUEST_LENGTH)
  {
    return 0U;
  }

  /* Modbus RTU request byte layout for a register read:
   *   request[0]    = sensor slave address;
   *   request[1]    = Modbus function code from the sensor documentation;
   *   request[2..3] = first register address, most-significant byte first;
   *   request[4..5] = number of 16-bit registers to read;
   *   request[6..7] = CRC-16, transmitted low byte first in Modbus RTU.
   */
  request[0] = IRRADIANCE_MODBUS_SLAVE_ID;
  request[1] = IRRADIANCE_MODBUS_FUNCTION_CODE;
  ModbusRtu_StoreU16BigEndian(&request[2], IRRADIANCE_MODBUS_START_REGISTER);
  ModbusRtu_StoreU16BigEndian(&request[4], IRRADIANCE_MODBUS_REGISTER_COUNT);

  crc = ModbusRtu_Crc16(request, 6U);
  request[6] = (uint8_t)(crc & 0x00FFU);
  request[7] = (uint8_t)((crc >> 8) & 0x00FFU);

  return IRRADIANCE_MODBUS_REQUEST_LENGTH;
}

/**
  * @brief  Parses a Modbus RTU response from the irradiance sensor.
  * @param  response Received response frame.
  * @param  response_length Number of received bytes.
  * @param  irradiance_out_w_m2 Destination for converted irradiance.
  * @retval true when the response is structurally valid and converted.
  */
static bool IrradianceSensor_ParseReadResponse(const uint8_t *response,
                                               size_t response_length,
                                               float *irradiance_out_w_m2)
{
  const size_t minimum_response_length = 7U;
  uint16_t received_crc;
  uint16_t calculated_crc;
  uint16_t raw_irradiance;

  if ((response_length < minimum_response_length) || (irradiance_out_w_m2 == NULL))
  {
    return false;
  }

  /* The first two reply bytes must echo the addressed sensor and function code.
   * If this check fails, confirm the sensor address and read function in the
   * datasheet, then confirm that the STM32 is receiving the correct RS485 bus. */
  if ((response[0] != IRRADIANCE_MODBUS_SLAVE_ID) ||
      (response[1] != IRRADIANCE_MODBUS_FUNCTION_CODE))
  {
    return false;
  }

  /* response[2] is the Modbus byte count. A single 16-bit register requires at
   * least two data bytes at response[3] and response[4]. */
  if (response[2] < 2U)
  {
    return false;
  }

  received_crc = (uint16_t)response[response_length - 2U] |
                 ((uint16_t)response[response_length - 1U] << 8);
  calculated_crc = ModbusRtu_Crc16(response, response_length - 2U);
  if (received_crc != calculated_crc)
  {
    return false;
  }

  /*
   * TODO: confirm the sensor data format.
   *
   * This scaffold assumes the first returned register is an unsigned 16-bit
   * irradiance value. If the sensor returns a signed value, 32-bit value, float,
   * or multiple channels, replace this conversion with the datasheet format.
   */
  raw_irradiance = ModbusRtu_ReadU16BigEndian(&response[3]);
  *irradiance_out_w_m2 =
      (float)raw_irradiance * IRRADIANCE_RAW_TO_W_PER_M2_SCALE;

  return true;
}

/**
  * @brief  Enables RS485 transmit mode.
  */
static void Rs485_SetTransmitMode(void)
{
  /* Most RS485 transceivers use DE high to transmit and active-low /RE high to
   * disable receiving. Confirm the board/transceiver before live testing. */
  HAL_GPIO_WritePin(DE___RS485_GPIO_Port, DE___RS485_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RE___RS485_GPIO_Port, RE___RS485_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Enables RS485 receive mode.
  */
static void Rs485_SetReceiveMode(void)
{
  HAL_GPIO_WritePin(DE___RS485_GPIO_Port, DE___RS485_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RE___RS485_GPIO_Port, RE___RS485_Pin, GPIO_PIN_RESET);
}