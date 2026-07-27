/*
 * SerialConsole.c
 *
 * USART2 PC-console command and status implementation.
 */

#include "SerialConsole.h"

#include "MpptController.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static void SerialConsole_StartReceive(void);
static void SerialConsole_StartTransmit(void);
static void SerialConsole_ProcessCommand(uint8_t command);
static void SerialConsole_PrintHelp(void);

#define SERIAL_CONSOLE_TX_BUFFER_SIZE 512U

static uint8_t serial_console_rx_byte = 0U;
static volatile uint8_t serial_console_pending_command = 0U;
static volatile bool serial_console_command_pending = false;

/* Main-loop code writes at head; the USART2 TX interrupt consumes from tail. */
static uint8_t serial_console_tx_buffer[SERIAL_CONSOLE_TX_BUFFER_SIZE];
static volatile uint16_t serial_console_tx_head = 0U;
static volatile uint16_t serial_console_tx_tail = 0U;
static volatile uint16_t serial_console_tx_in_flight = 0U;
static volatile bool serial_console_tx_active = false;

/**
  * @brief  Starts the USART2 PC-console receive interrupt.
  */
void SerialConsole_Init(void)
{
  /* USART2 is used as the PC console in this scaffold. Reception is interrupt
   * driven one byte at a time so the main loop does not have to wait for input. */
  SerialConsole_StartReceive();
}

/**
  * @brief  Handles pending PC-console commands outside the UART interrupt.
  */
void SerialConsole_Task(void)
{
  uint8_t command;

  /* Retry queued output if HAL was temporarily busy when it was enqueued. */
  SerialConsole_StartTransmit();

  /* If no UART callback has supplied a new command byte, this task does nothing. */
  if (!serial_console_command_pending)
  {
    return;
  }

  /* Copy and clear the pending command before processing it. This keeps the UART
   * callback short and leaves command effects to the main-loop context. */
  command = serial_console_pending_command;
  serial_console_command_pending = false;
  SerialConsole_ProcessCommand(command);
}

/**
  * @brief  Prints the startup banner and command summary.
  */
void SerialConsole_PrintBootMessage(void)
{
  SerialConsole_SendLine("");
  SerialConsole_SendLine("ENR MPPT boot");
  SerialConsole_PrintHelp();
}

/**
  * @brief  Prints current state and latched fault over USART2.
  */
void SerialConsole_PrintStatus(void)
{
  SerialConsole_Send("State: ");
  SerialConsole_SendLine(getStateName(getState()));
  SerialConsole_Send("Fault: ");
  SerialConsole_SendLine(getFaultName(getFault()));
}

/**
  * @brief  Sends a null-terminated string over the USART2 PC console.
  * @param  text String to transmit.
  */
void SerialConsole_Send(const char *text)
{
  size_t length;
  size_t first_length;
  uint16_t head;
  uint16_t tail;
  uint16_t free_space;
  uint16_t new_head;

  length = strlen(text);
  if (length == 0U)
  {
    return;
  }

  head = serial_console_tx_head;
  tail = serial_console_tx_tail;
  free_space = (head >= tail)
             ? (uint16_t)(SERIAL_CONSOLE_TX_BUFFER_SIZE - (head - tail) - 1U)
             : (uint16_t)(tail - head - 1U);

  /* Never queue a partial string: incomplete CSV lines are harder to recover. */
  if (length > free_space)
  {
    return;
  }

  first_length = length;
  if (first_length > (SERIAL_CONSOLE_TX_BUFFER_SIZE - head))
  {
    first_length = SERIAL_CONSOLE_TX_BUFFER_SIZE - head;
  }

  memcpy(&serial_console_tx_buffer[head], text, first_length);
  if (length > first_length)
  {
    memcpy(serial_console_tx_buffer,
           &text[first_length],
           length - first_length);
  }

  new_head = (uint16_t)(head + length);
  if (new_head >= SERIAL_CONSOLE_TX_BUFFER_SIZE)
  {
    new_head = (uint16_t)(new_head - SERIAL_CONSOLE_TX_BUFFER_SIZE);
  }

  /* Publish only after the complete string is safely in the ring buffer. */
  serial_console_tx_head = new_head;
  SerialConsole_StartTransmit();
}

/**
  * @brief  Sends a null-terminated string followed by CRLF over USART2.
  * @param  text String to transmit.
  */
void SerialConsole_SendLine(const char *text)
{
  SerialConsole_Send(text);
  SerialConsole_Send("\r\n");
}

/**
  * @brief  UART receive-complete callback used by the USART2 PC console.
  * @param  huart UART handle that completed reception.
  */
void SerialConsole_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* Interrupt callbacks should be short. Store the byte and let
     * SerialConsole_Task() process it from the main loop. */
    serial_console_pending_command = serial_console_rx_byte;
    serial_console_command_pending = true;
    SerialConsole_StartReceive();
  }
}

/**
  * @brief  USART2 transmit-complete callback advances the queued output.
  * @param  huart UART handle that completed transmission.
  */
void SerialConsole_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t new_tail;

  if ((huart->Instance == USART2) && serial_console_tx_active)
  {
    new_tail = (uint16_t)(serial_console_tx_tail +
                          serial_console_tx_in_flight);
    if (new_tail >= SERIAL_CONSOLE_TX_BUFFER_SIZE)
    {
      new_tail = (uint16_t)(new_tail - SERIAL_CONSOLE_TX_BUFFER_SIZE);
    }

    serial_console_tx_tail = new_tail;
    serial_console_tx_in_flight = 0U;
    serial_console_tx_active = false;
    SerialConsole_StartTransmit();
  }
}

/**
  * @brief  UART error callback; restarts the USART2 PC-console receiver.
  * @param  huart UART handle that reported an error.
  */
void SerialConsole_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* If a UART framing/noise/overrun error occurs, re-arm reception so the
     * console can recover without resetting the MCU. */
    SerialConsole_StartReceive();
  }
}

/**
  * @brief  Arms one-byte interrupt receive on the USART2 PC console.
  */
static void SerialConsole_StartReceive(void)
{
  /* Arm reception for exactly one byte. The receive-complete callback below
   * stores that byte and then calls this function again. */
  (void)HAL_UART_Receive_IT(&huart2, &serial_console_rx_byte, 1U);
}

/**
  * @brief  Starts the next contiguous section of queued USART2 output.
  */
static void SerialConsole_StartTransmit(void)
{
  uint16_t head;
  uint16_t tail;
  uint16_t length;

  if (serial_console_tx_active)
  {
    return;
  }

  head = serial_console_tx_head;
  tail = serial_console_tx_tail;
  if (head == tail)
  {
    return;
  }

  length = (head > tail)
         ? (uint16_t)(head - tail)
         : (uint16_t)(SERIAL_CONSOLE_TX_BUFFER_SIZE - tail);

  serial_console_tx_in_flight = length;
  serial_console_tx_active = true;
  if (HAL_UART_Transmit_IT(&huart2,
                           &serial_console_tx_buffer[tail],
                           length) != HAL_OK)
  {
    /* Keep the data queued; SerialConsole_Task() will retry later. */
    serial_console_tx_in_flight = 0U;
    serial_console_tx_active = false;
  }
}

/**
  * @brief  Converts received serial characters into MPPT state requests.
  * @param  command Received ASCII command.
  */
static void SerialConsole_ProcessCommand(uint8_t command)
{
  switch (command)
  {
    case 's':
    case 'S':
      /* Request a transition from IDLE to STARTUP. The state machine checks
       * measurements before it actually enables switching. */
      requestMpptStart();
      SerialConsole_SendLine("Start requested");
      break;

    case 'x':
    case 'X':
      /* Request a controlled stop. The state machine will disable the power stage. */
      requestStop();
      SerialConsole_SendLine("Stop requested");
      break;

    case 'r':
    case 'R':
      /* Fault reset is deliberate. It clears the latched fault only in FAULT state. */
      requestFaultReset();
      SerialConsole_SendLine("Fault reset requested");
      break;

    case '?':
      /* Print the present state and fault code without changing control state. */
      SerialConsole_PrintStatus();
      break;

    case 'h':
    case 'H':
      SerialConsole_PrintHelp();
      break;

    case '\r':
    case '\n':
      break;

    default:
      SerialConsole_SendLine("Unknown command");
      SerialConsole_PrintHelp();
      break;
  }
}

/**
  * @brief  Prints the available serial-console commands.
  */
static void SerialConsole_PrintHelp(void)
{
  SerialConsole_SendLine("Commands: s=start, x=stop, r=reset fault, ?=status, h=help");
}
