#include "gpio.h"

/* Initializes GPIO clocks and configures control outputs for RS485, converter gating, and temperature sensing. */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOF, DE___RS485_Pin|GPIO_PIN_1, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, RE___RS485_Pin|BST_STP_Pin|BCK_STP_Pin|BCK_DIS_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DQ__Temp_GPIO_Port, DQ__Temp_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DE___RS485_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RE___RS485_Pin|BST_STP_Pin|BCK_STP_Pin|BCK_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DQ__Temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DQ__Temp_GPIO_Port, &GPIO_InitStruct);

}
