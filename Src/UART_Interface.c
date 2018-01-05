#include "UART_Interface.h"

static volatile uint32_t  _i                      = 0; // rx_buff position index

uint8_t                   rx_buff[4 * BUFFER_RX_SIZE] = {0};
uint8_t                   tx_buff[4 * BUFFER_TX_SIZE] = {0};

static uint8_t            rx_data[2]              = {0};

static volatile uint32_t  number_rx_error         = 0,
                          number_tx_error         = 0;


void UART_TRANSMIT_IT_BLOCK_IF_BUSY(UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size){

  HAL_StatusTypeDef TX_status;

  do{
    HAL_UART_AbortTransmit_IT(huart);
    TX_status = HAL_UART_Transmit_IT(huart, pData, Size);
    if(TX_status == HAL_ERROR){
      ++number_tx_error;
    }
  }while(TX_status == HAL_BUSY);

  ssd1306_WriteString( (char *) pData, Font_7x10, White);
  ssd1306_UpdateScreen();
}

void UART_RECEIVE_IT_BLOCK_IF_BUSY(UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size){

  HAL_StatusTypeDef RX_status;

  do{
    //HAL_UART_AbortReceive_IT(huart);
    RX_status = HAL_UART_Receive_IT(huart, pData, Size);
    if(RX_status == HAL_ERROR){
      ++number_rx_error;
    }
  }while(RX_status == HAL_BUSY);
}

uint32_t GetN_UART_RxError(){
  return number_rx_error;
}

uint32_t GetN_UART_TxError(){
  return number_tx_error;
}

typedef struct{
  uint32_t        ErrorCode;
  uint8_t const * ErrorStr;
} UART_errno_str;

UART_errno_str errno_descriptions[] =
    {
      { HAL_UART_ERROR_NONE , (uint8_t const * ) "No error"           },
      { HAL_UART_ERROR_PE   , (uint8_t const * ) "Parity error"       },
      { HAL_UART_ERROR_NE   , (uint8_t const * ) "Noise error"        },
      { HAL_UART_ERROR_FE   , (uint8_t const * ) "frame error"        },
      { HAL_UART_ERROR_ORE  , (uint8_t const * ) "Overrun error"      },
      { HAL_UART_ERROR_DMA  , (uint8_t const * ) "DMA transfer error" },
      { HAL_UART_ERROR_BUSY , (uint8_t const * ) "Busy Error"         },
    };

#define number_strerrno ( sizeof(errno_descriptions) / sizeof(errno_descriptions[0]) )

uint8_t const * UART_strerrno(uint32_t ErrorCode){
  uint8_t const * str_ptr = 0;
  for(uint32_t i = 0; i < number_strerrno; ++i){
    if(ErrorCode == errno_descriptions[i].ErrorCode){
      str_ptr = errno_descriptions[i].ErrorStr;
      break;
    }
  }
  return str_ptr;
}

void UART_InputPromt(){
  sprintf((char*)tx_buff, "\r\n>");
  UART_TRANSMIT_IT_BLOCK_IF_BUSY(&huart4, tx_buff, sizeof("\r\n>"));
  UART_RECEIVE_IT_BLOCK_IF_BUSY(&huart4, rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

  if(huart->Instance == UART_LINE){
    uint8_t sym = rx_data[0];

    if(isalnum((int)sym) || sym == '\n' || sym == '\r' || sym == '=' || sym == '\b' || sym == ' '){

      if(sym == '\b'){
        if(_i > 0){
          UART_TRANSMIT_IT_BLOCK_IF_BUSY(huart, (uint8_t * )"\b \b", sizeof("\b \b"));
          --_i;
        }
        goto _exit;
      }

      if(_i == sizeof(rx_buff) - 2 && sym != '\r'){
        goto _exit;
      }

      rx_buff[_i] = sym;

      if(rx_buff[_i] == '\r'){
        rx_buff[_i]     = 0;
        //UART_TRANSMIT_IT_BLOCK_IF_BUSY(huart, (uint8_t * ) "\r\n", sizeof "\r\n");
        if(!_i){
          UART_TRANSMIT_IT_BLOCK_IF_BUSY(huart, (uint8_t * ) "\r\n>", sizeof "\r\n>");
          goto _exit;
        }

        sprintf((char*)tx_buff, "%s", rx_buff);
        ParseCommand();
        _i  = 0;
      }
      else{
        UART_TRANSMIT_IT_BLOCK_IF_BUSY(huart, rx_data, 1);
        ++_i;
      }
    }

_exit:
    UART_RECEIVE_IT_BLOCK_IF_BUSY(huart,  rx_data, 1);
  }
}
