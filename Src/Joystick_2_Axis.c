/**
  ******************************************************************************
  * @file   Joystick_2_Axis.c
  * @author Mykola Kvach
  * @brief  This file contains functions to provide configuration and getting
            measurement from joystick. It the values of which are obtained from the ADC1.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Joystick_2_Axis.h"

#define         _DELAY_1      (10    )  // time interval between updates of position joystick on LED panel indicator
#define         _DELAY_2      (250   )  // max delay when LED blinking
#define         _DELAY_3      (500   )  // interval between printing

#define         __DEBUG__     (0)       // if __DEBUG__ and is_dr != 0 then every call function JOYSTICK_UpdateValues prints joystick attributes, state and its calculated values

// macros for calculate ADC resolution from ADCEx_Resolution
#define GET_ADC_MAX_NUMBER(_ADCEx_Resolution) ( 0x01 << ( ( (~(_ADCEx_Resolution >> 3) & 0x03) << 1 ) + 6 ) )


// object of joystick that consist its attributes, state and calculated values
Joystick2 _joystick;

static volatile uint32_t  is_dr         = 0; // set in HAL_ADC_ConvCpltCallback if ADC1 performed two conversions

static volatile uint32_t  i_xy          = 0; // index of array cur_xy from structure Joystick2 _joystick

// old time-tick with [index] for compare with current time and if above that _DELAY_[index]
static uint32_t           time_old_1    = 0; // recalculate intensity, angle and distance of current joystick attitude
static uint32_t           time_old_2    = 0; // invert current LED statee
static uint32_t           time_old_3    = 0; // print joystick attributes, state and its calculated values


/*/  The array must be filled starting at three o'clock and moving counter-clockwise
/*/
uint32_t const leds[] =
  {
      LED7 , LED5 , LED3  , LED4  ,
      LED6 , LED8 , LED10 , LED9
  };

const uint32_t LED_NUM = (sizeof(leds) / sizeof(leds[0]) ); // number of LEDs

static void DistancePercent(Joystick2 * j);

void JOYSTICK_Init(){
  _joystick.min_x      = 0;
  _joystick.mid_x      = 537;
  _joystick.max_x      = GET_ADC_MAX_NUMBER(hadc1.Init.Resolution);

  _joystick.min_y      = 0;
  _joystick.mid_y      = 497;
  _joystick.max_y      = GET_ADC_MAX_NUMBER(hadc1.Init.Resolution);

  _joystick.eps_x      = 5;
  _joystick.eps_y      = 5;

  _joystick.cur_xy[0]  = 0;
  _joystick.cur_xy[1]  = 0;
}

// recalculate intensity, angle and distance of current joystick attitude
void JOYSTICK_UpdateValues(){

  if(!is_dr){
    return;
  }

  uint32_t        curr_tick = HAL_GetTick();
  static uint32_t curr_led  = 0;

  // calculate current index of LED and update joystick parameters
  if ( curr_tick - time_old_1 > _DELAY_1){

    time_old_1 = curr_tick;

    DistancePercent( & _joystick );

    uint32_t old_led = curr_led;

    curr_led = ( ( (uint32_t) ( _joystick.angle / ( 180.f / LED_NUM ) ) + 1) >> 1 ) % LED_NUM;

    if(old_led != curr_led){
      BSP_LED_Off( leds[old_led] );
      BSP_LED_On( leds[curr_led] );
    }
//    LED_Off_ALL();
    //BSP_LED_On( leds[curr_led] );

  }

  // print debug information
  if(__DEBUG__ && curr_tick - time_old_3 > _DELAY_3){

    time_old_3 = curr_tick;

    sprintf( (char * )  tx_buff, "\r\nJOYSTICK:\r\nx = %04li\r\ny = %04li\r\n\r\na = %04li.%03lu\r\ni = %02li.%03lu\r\nd = %04li.%03lu\r\n\r\n>",
                        _joystick.relative_x,
                        _joystick.relative_y,

                        (int32_t) _joystick.angle,
                        (uint32_t) fabs(_joystick.angle * 1000) % 1000,

                        (int32_t) _joystick.intensity,
                        (uint32_t) fabs(_joystick.intensity * 1000) % 1000 ,

                        (int32_t) _joystick.distance,
                        (uint32_t) fabs(_joystick.distance  * 1000) % 1000 );

    UART_TRANSMIT_IT_BLOCK_IF_BUSY(& huart4, tx_buff, (uint16_t) strlen( (char *) tx_buff));
  }

  // organize  blinking from intensity of the current LED
  if ( curr_tick - time_old_2 > _DELAY_2 * fabs(1.1f - _joystick.intensity )){

    time_old_2 = curr_tick;

    BSP_LED_Toggle(leds[curr_led]);
  }

  HAL_ADC_Start_IT(& hadc1);
}

static inline int32_t GetSign(float number){
  return ((number < 0) ? -1 : 1);
}

// calculate the distance relatively the center of the joystick (mid_x, mid_y)
static inline float CalcDistance(Joystick2 * j, float x, float y){
  float x_prj, y_prj;
  x_prj = j->mid_x - x;
  x_prj *= x_prj;

  y_prj = j->mid_y - y;
  y_prj *= y_prj;

  return sqrt(x_prj + y_prj);
}

static void DistancePercent(Joystick2 * j){

  uint16_t xy[2];

  xy[0] = j->cur_xy[0];
  xy[1] = j->cur_xy[1];

  j->relative_x = xy[0] - j->mid_x;
  j->relative_y = xy[1] - j->mid_y;

  if (j->relative_x == 0){
    return;
  }

  /*/ Determining the angle of rotation of the joystick relative to the X (positive part)
      Cartesian coordinate system, degrees increase by moving counterclockwise,
      the range of values from 0 to 360
  /*/
  j->angle  = atan2(j->relative_y, j->relative_x) * 180. / M_PI;

  if (j->angle < 0.f){
    j->angle += 360.f;
  }

  uint32_t  max_x,
            max_y;

  if ( j->angle < 90.f){
    max_x = j->max_x - j->mid_x;
    max_y = j->max_y - j->mid_y;
  }
  else if ( j->angle < 180.f ){
    max_x = j->mid_x;
    max_y = j->max_y - j->mid_y;
  }
  else if ( j->angle < 270.f){
    max_x = j->mid_x;
    max_y = j->mid_y;
  }
  else{
    max_x = j->max_x - j->mid_x;
    max_y = j->mid_y;
  }

  j->distance  = CalcDistance(j, xy[0], xy[1]);

  float round_dist = (j->distance - ( ( uint16_t ) j->distance) % ( j->eps_x * j->eps_x + j->eps_y * j->eps_y ));

  if (round_dist < ( j->eps_x * j->eps_x + j->eps_y * j->eps_y ) ){
     j->angle = 0.f;
  }

  j->intensity =  j->distance / CalcDistance(j, max_x + j->mid_x, max_y + j->mid_y);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

  if( hadc->Instance == ADC1 ){

    if(!i_xy){
      _joystick.cur_xy[0] = GET_ADC_MAX_NUMBER(hadc->Init.Resolution) - HAL_ADC_GetValue(hadc);
      ++i_xy;
    }
    else{
      _joystick.cur_xy[1] = HAL_ADC_GetValue(hadc);
      i_xy = 0;
      is_dr = 1;
      HAL_ADC_Stop_IT(hadc);
    }

  }

}
