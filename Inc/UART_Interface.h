#ifndef __UART_INTERFACE_H__
#define __UART_INTERFACE_H__

#include "main.h"
#include "UART_Command_Parser.h"
#include "ssd1306.h"
#include "Joystick_2_Axis.h"

#include <stdio.h>
#include <ctype.h>


#define UART_LINE          UART4
#define BUFFER_RX_SIZE     (0x20) // actually, the buffer is 4 times larger
#define BUFFER_TX_SIZE     (0x20) // actually, the buffer is 4 times larger


extern  UART_HandleTypeDef  huart4;
extern  ADC_HandleTypeDef   hadc1;
extern  I2C_HandleTypeDef   hi2c1;
extern  SSD1306_t           SSD1306;

extern  float               gyro_xyz[3];

extern  volatile float      magneto_xyz[3];
extern  volatile float      acc_xyz[3];
extern           float      HeadingValue;

extern  uint8_t             rx_buff[4 * BUFFER_RX_SIZE];
extern  uint8_t             tx_buff[4 * BUFFER_TX_SIZE];


void      UART_InputPromt();

void      UART_TRANSMIT_IT_BLOCK_IF_BUSY(UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size);
void      UART_RECEIVE_IT_BLOCK_IF_BUSY (UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size);

uint32_t  GetN_UART_TxError();
uint32_t  GetN_UART_RxError();

void      HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
