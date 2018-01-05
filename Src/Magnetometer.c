#include "Magnetometer.h"

volatile float  magneto_xyz[3];
float           HeadingValue;


void MAGNETO_Init(void){

  if(BSP_MAGNETO_Init() != MAGNETO_OK){
    /* Initialization Error */
    Error_Handler();
  }

}

static inline float GetAngle(float y, float x){
  float angle = atan2f(y, x);
  return ( (angle < 0.f) ? (angle + 2.f * M_PI) : angle );
}

void MAGNETO_UpdateValues(void){

  for(uint32_t i=0; i<3; i++){
    acc_xyz[i] /= 100.0f;
  }

  float pitch   = GetAngle( acc_xyz[1],  sqrtf( acc_xyz[0]*acc_xyz[0] + acc_xyz[2]*acc_xyz[2] ) );
  float roll    = GetAngle( -acc_xyz[0], acc_xyz[2] );

  if ((roll <= M_PI_4) && (pitch <= M_PI_4)){
    float fTiltedX      =  magneto_xyz[0]*cosf(pitch)+magneto_xyz[2]*sinf(pitch);
    float fTiltedY      =  magneto_xyz[0]*sinf(roll)*sinf(pitch)+magneto_xyz[1]*cosf(roll)-magneto_xyz[1]*sinf(roll)*cosf(pitch);
    HeadingValue        =  ( GetAngle( fTiltedY, fTiltedX ) * (180.f / M_PI) + 225.0f + 7.5 /*some error*/);
    HeadingValue        =  HeadingValue - (HeadingValue > 360.f) ? 360.f : 0.f;

    static uint16_t curr_led  = -1;
    uint16_t        old_led   = curr_led;

    curr_led = ( ( (uint16_t) ( (HeadingValue ) / ( 180.f / LED_NUM ) ) + 1) >> 1 ) % LED_NUM;

    if( old_led != curr_led ){
      BSP_LED_Off( leds[old_led]  );
      BSP_LED_On ( leds[curr_led] );
    }

  }

}
