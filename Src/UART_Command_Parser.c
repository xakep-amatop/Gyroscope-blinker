#include "UART_Command_Parser.h"

extern uint32_t const leds[];

typedef uint32_t (*pfnCommands)(uint32_t, uint8_t *);

static uart_str commands[] =
  {
    { (uint8_t *) "help",                   sizeof("help")                  },
    { (uint8_t *) "get mode",               sizeof("get mode")              },
    { (uint8_t *) "get lastval=compass",    sizeof("get lastval=compass")   },
    { (uint8_t *) "get lastval=gyroscope",  sizeof("get lastval=gyroscope") },
    { (uint8_t *) "get lastval=joystick",   sizeof("get lastval=joystick")  },
    { (uint8_t *) "set mode=compass",       sizeof("set mode=compass")      },
    { (uint8_t *) "set mode=gyroscope",     sizeof("set mode=gyroscope")    },
    { (uint8_t *) "set mode=joystick",      sizeof("set mode=joystick")     },
  };

#define number_commands  (sizeof(commands) / sizeof(commands[0]))

static const uint8_t msg_help[] =
  "\r\nInstructions for use!"
  " OPTIONS:\r\n"
  " get mode - get current mode\r\n"
  " get last_val=[gyroscope|compass|joystick]\r\n"
  " set mode=[gyroscope|compass|joystick]\r\n"
  " help - show this text\r\n>";

#define msg_help_length (sizeof(                      \
  "\r\nInstructions for use!"                         \
  " OPTIONS:\r\n"                          \
  " get mode - get current mode\r\n"               \
  " get last_val=[gyroscope|compass|joystick]\r\n" \
  " set mode=[gyroscope|compass|joystick]\r\n" \
  " help - show this text\r\n>" ))

uart_str modes[] =
  {
    { (uint8_t *) "\r\nCurrent mode: GYROSCOPE\r\n>",  sizeof("\r\nCurrent mode: GYROSCOPE\r\n>") },
    { (uint8_t *) "\r\nCurrent mode: COMPASS\r\n>",    sizeof("\r\nCurrent mode: COMPASS\r\n>")   },
    { (uint8_t *) "\r\nCurrent mode: JOYSTICK\r\n>",   sizeof("\r\nCurrent mode: JOYSTICK\r\n>")  },
  };

static inline uint32_t NotLastSymbol(uint32_t position, uint8_t * _str){
  return ( _str[commands[position].length - 1] != 0 );
}

static const uint8_t * info[] =
  {
    (uint8_t *) "\r\nUnknown command or incorrect argument!!!\r\n>",
    (uint8_t *) "\r\nOK!\r\n>"
  };

static uint32_t ShowHelp(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }

  //HAL_UART_Transmit_IT(&huart4, (uint8_t * ) msg_help, strlen((char*)msg_help));
  UART_TRANSMIT_IT_BLOCK_IF_BUSY(&huart4, (uint8_t * ) msg_help, msg_help_length);
  return 1;
}

static uint32_t GetCurrentMode(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }
  UART_TRANSMIT_IT_BLOCK_IF_BUSY(&huart4, modes[current_mode].name, modes[current_mode].length);
  return 1;
}

static uint32_t GetLastValGyroscope(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }

  sprintf( (char * ) tx_buff, "\r\nGYRO:\r\nx = %06li.%03lu\r\ny = %06li.%03lu\r\nz = %06li.%03lu\r\n\r\n>",
                              (int32_t)  gyro_xyz[0],
                              (uint32_t) fabs(gyro_xyz[0] * 1000) % 1000,
                              (int32_t)  gyro_xyz[1],
                              (uint32_t) fabs(gyro_xyz[1] * 1000) % 1000,
                              (int32_t)  gyro_xyz[2],
                              (uint32_t) fabs(gyro_xyz[2] * 1000) % 1000);

  UART_TRANSMIT_IT_BLOCK_IF_BUSY(& huart4, tx_buff, (uint16_t) strlen( (char *) tx_buff));

  return 1;
}

static uint32_t GetLastValCompass(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }

  sprintf( (char * ) tx_buff, "\r\nMAGNETO:\r\nx = %06li.%03lu\r\ny = %06li.%03lu\r\nz = %06li.%03lu\r\na = %03lu.%02lu\r\n\r\n>",
                              (int32_t)  magneto_xyz[0],
                              (uint32_t) fabs(magneto_xyz[0] * 1000) % 1000,
                              (int32_t)  magneto_xyz[1],
                              (uint32_t) fabs(magneto_xyz[1] * 1000) % 1000,
                              (int32_t)  magneto_xyz[2],
                              (uint32_t) fabs(magneto_xyz[2] * 1000) % 1000,
                              (uint32_t) HeadingValue,
                              (uint32_t) (HeadingValue * 100) % 100);

  UART_TRANSMIT_IT_BLOCK_IF_BUSY(& huart4, tx_buff, (uint16_t) strlen( (char *) tx_buff));

  return 1;
}

static uint32_t GetLastValJoystick(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }

  sprintf( (char * )  tx_buff, "\r\nJOYSTICK:\r\nx = %05li\r\ny = %05li\r\n\r\na = %04li.%03lu\r\ni = %02li.%03lu\r\nd = %04li.%03lu\r\n\r\n",
                      _joystick.relative_x,
                      _joystick.relative_y,

                      (int32_t) _joystick.angle,
                      (uint32_t) fabs(_joystick.angle * 1000) % 1000,

                      (int32_t) _joystick.intensity,
                      (uint32_t) fabs(_joystick.intensity * 1000) % 1000 ,

                      (int32_t) _joystick.distance,
                      (uint32_t) fabs(_joystick.distance  * 1000) % 1000 );

  UART_TRANSMIT_IT_BLOCK_IF_BUSY(& huart4, tx_buff, (uint16_t) strlen( (char *) tx_buff));

  return 1;
}

static uint32_t SetModeGyroscope(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }

  if (current_mode == GYROSCOPE_MODE)
    return 1;

  for(uint32_t i = 0; i < LED_NUM; ++i){
    BSP_LED_Off(leds[i]);
  }

  HAL_ADC_Stop_IT(&hadc1);
  HAL_NVIC_DisableIRQ(EXTI2_TSC_IRQn);
  current_mode = GYROSCOPE_MODE;

  return 1;
}

static uint32_t SetModeCompass(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }

  if (current_mode == MAGNETOMETER_MODE)
    return 1;

  for(uint32_t i = 0; i < LED_NUM; ++i){
    BSP_LED_Off(leds[i]);
  }

  HAL_ADC_Stop_IT(&hadc1);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
  current_mode = MAGNETOMETER_MODE;

  return 1;
}

static uint32_t SetModeJoystick(uint32_t position, uint8_t * str){
  if( NotLastSymbol(position, str) ){
    return 0;
  }

  if (current_mode == JOYSTICK_MODE)
    return 1;

  for(uint32_t i = 0; i < LED_NUM; ++i){
    BSP_LED_Off(leds[i]);
  }

  HAL_ADC_Start_IT(&hadc1);
  HAL_NVIC_DisableIRQ(EXTI2_TSC_IRQn);
  current_mode = JOYSTICK_MODE;

  return 1;
}

pfnCommands hCommands[] =
  {
    ShowHelp,
    GetCurrentMode,
    GetLastValCompass,
    GetLastValGyroscope,
    GetLastValJoystick,
    SetModeCompass,
    SetModeGyroscope,
    SetModeJoystick,
  };

void ParseCommand(){
  uint8_t * substr_ptr  = 0;
  uint32_t 	ret         = 0;

  for(uint32_t i = 0; i < number_commands; ++i){
    substr_ptr = (uint8_t *) strstr((char*) tx_buff, (char *) commands[i].name);
    if (substr_ptr){
      if(substr_ptr != tx_buff){
        substr_ptr = 0;
        break;
      }
      ret = hCommands[i](i, substr_ptr);
      break;
    }
  }

  ssd1306_WriteString( (char *) info[ret], Font_7x10, White );
  ssd1306_UpdateScreen();

  HAL_UART_Transmit_IT(   &huart4,
                          (uint8_t * ) info[ret],
                          strlen( (char*) info[ret]) );
}
