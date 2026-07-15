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
static void SerialConsole_ProcessCommand(uint8_t command);
static void SerialConsole_PrintHelp(void);

static uint8_t serial_console_rx_byte = 0U;
static volatile uint8_t serial_console_pending_command = 0U;
static volatile bool serial_console_command_pending = false;

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
  (void)HAL_UART_Transmit(&huart2,
                          (uint8_t *)text,
                          (uint16_t)strlen(text),
                          100U);
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