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

/* Treat these values as scaffold values until they have been checked against
 * the guide and the specific sensor's Modbus table. */
#define IRRADIANCE_SENSOR_ENABLE                    (1U)
#define IRRADIANCE_MODBUS_SLAVE_ID                  (0x01U)       /* Check: sensor Modbus address / slave ID. */
#define IRRADIANCE_MODBUS_FUNCTION_CODE             (0x03U)    /* Check: read function code for this register type. */
#define IRRADIANCE_MODBUS_START_REGISTER            (0x0000U)  /* Check: first irradiance register address. */
#define IRRADIANCE_MODBUS_REGISTER_COUNT            (1U)       /* Check: number of 16-bit registers returned. */
#define IRRADIANCE_MODBUS_REQUEST_LENGTH            (8U)
#define IRRADIANCE_MODBUS_RESPONSE_MAX_LENGTH       (32U)
#define IRRADIANCE_MODBUS_RESPONSE_LENGTH           (5U + (IRRADIANCE_MODBUS_REGISTER_COUNT * 2U))
#define IRRADIANCE_TASK_PERIOD_MS                   (200U)      /* Poll at 5 Hz. */
#define IRRADIANCE_TX_TIMEOUT_MS                    (30U)
#define IRRADIANCE_RX_TIMEOUT_MS                    (100U)
#define IRRADIANCE_RAW_TO_W_PER_M2_SCALE            (1.0f)     /* Check: raw register to W/m^2 scale factor. */

typedef enum
{
  IRRADIANCE_STATE_WAIT = 0,
  IRRADIANCE_STATE_TX,
  IRRADIANCE_STATE_RX,
  IRRADIANCE_STATE_READY
} IrradianceSensor_State;

static size_t IrradianceSensor_BuildReadRequest(uint8_t *request, size_t request_size);
static bool IrradianceSensor_ParseReadResponse(const uint8_t *response,
                                               size_t response_length,
                                               float *irradiance_out_w_m2);
static void Rs485_SetTransmitMode(void);
static void Rs485_SetReceiveMode(void);

static uint8_t request[IRRADIANCE_MODBUS_REQUEST_LENGTH];
static uint8_t response[IRRADIANCE_MODBUS_RESPONSE_MAX_LENGTH];
static size_t request_length = 0U;
static volatile IrradianceSensor_State state = IRRADIANCE_STATE_WAIT;
static volatile uint32_t state_started_ms = 0U;
static volatile bool uart_error = false;
static uint32_t last_poll_ms = 0U;
static float irradiance_w_m2 = 0.0f;
static bool irradiance_valid = false;

/**
  * @brief  Places the RS485 transceiver in receive mode and clears the local
  *         irradiance measurement status.
  */
void IrradianceSensor_Init(void)
{
  Rs485_SetReceiveMode();

  request_length = IrradianceSensor_BuildReadRequest(request, sizeof(request));
  state = IRRADIANCE_STATE_WAIT;
  state_started_ms = 0U;
  uart_error = false;
  last_poll_ms = HAL_GetTick();
  irradiance_w_m2 = 0.0f;
  irradiance_valid = false;
}

/**
  * @brief  Advances one non-blocking USART1/RS485 Modbus transaction.
  */
void IrradianceSensor_Task(void)
{
  uint32_t now_ms;

  if (IRRADIANCE_SENSOR_ENABLE == 0U)
  {
    return;
  }

  if ((request_length == 0U) ||
      (IRRADIANCE_MODBUS_RESPONSE_LENGTH > sizeof(response)))
  {
    irradiance_valid = false;
    return;
  }

  now_ms = HAL_GetTick();

  /* UART errors and timeouts are recovered in the main loop, not in the ISR. */
  if (uart_error)
  {
    uart_error = false;
    (void)HAL_UART_Abort(&huart1);
    Rs485_SetReceiveMode();
    state = IRRADIANCE_STATE_WAIT;
    last_poll_ms = now_ms;
    irradiance_valid = false;
    return;
  }

  switch (state)
  {
    case IRRADIANCE_STATE_WAIT:
      if (Timebase_HasElapsed(now_ms, last_poll_ms, IRRADIANCE_TASK_PERIOD_MS))
      {
        last_poll_ms = now_ms;
        state_started_ms = now_ms;
        state = IRRADIANCE_STATE_TX;
        Rs485_SetTransmitMode();

        if (HAL_UART_Transmit_IT(&huart1,
                                request,
                                (uint16_t)request_length) != HAL_OK)
        {
          uart_error = true;
        }
      }
      break;

    case IRRADIANCE_STATE_TX:
      if (Timebase_HasElapsed(now_ms, state_started_ms, IRRADIANCE_TX_TIMEOUT_MS))
      {
        uart_error = true;
      }
      break;

    case IRRADIANCE_STATE_RX:
      if (Timebase_HasElapsed(now_ms, state_started_ms, IRRADIANCE_RX_TIMEOUT_MS))
      {
        uart_error = true;
      }
      break;

    case IRRADIANCE_STATE_READY:
      /* CRC checking and conversion stay out of the interrupt handler. */
      irradiance_valid = IrradianceSensor_ParseReadResponse(
          response,
          IRRADIANCE_MODBUS_RESPONSE_LENGTH,
          &irradiance_w_m2);
      state = IRRADIANCE_STATE_WAIT;
      break;

    default:
      uart_error = true;
      break;
  }
}

/**
  * @brief  Releases the RS485 bus after the last stop bit and starts reception.
  */
void IrradianceSensor_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart->Instance != USART1) || (state != IRRADIANCE_STATE_TX))
  {
    return;
  }

  Rs485_SetReceiveMode();
  state_started_ms = HAL_GetTick();
  state = IRRADIANCE_STATE_RX;

  if (HAL_UART_Receive_IT(&huart1,
                          response,
                          (uint16_t)IRRADIANCE_MODBUS_RESPONSE_LENGTH) != HAL_OK)
  {
    uart_error = true;
  }
}

/**
  * @brief  Marks the fixed-length Modbus reply for main-loop parsing.
  */
void IrradianceSensor_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart->Instance == USART1) && (state == IRRADIANCE_STATE_RX))
  {
    state = IRRADIANCE_STATE_READY;
  }
}

/**
  * @brief  Defers USART1 error recovery to IrradianceSensor_Task().
  */
void IrradianceSensor_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_error = true;
  }
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
