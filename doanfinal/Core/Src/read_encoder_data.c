#include "read_encoder_data.h"


                                          ///////////////
                                        //////////////////
//////////////////////////////////////////////////////////
//slot1///slot2///slot3///slot4///slot5///slot6////slot7//
//tim5////tim2////tim4////tim3////tim1////uart1////uart2//
//////////////////////////////////////////////////////////
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

Encoder_Data encoder_data;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&htim5,TIM_CHANNEL_ALL);
    
}
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//  
//  if(htim->Instance==TIM3)
//  {
//    if(TIM3->CR1 & TIM_COUNTERMODE_DOWN)
//    {
//      encoder_data.encoder_data[1]--;
//    }
//    else
//    {
//      encoder_data.encoder_data[1]++;
//    }
//  }
//   if(htim->Instance==TIM5)
//  {
//    if(TIM4->CR1 & TIM_COUNTERMODE_DOWN)
//    {
//      encoder_data.encoder_data[0]--;
//    }
//    else
//    {
//      encoder_data.encoder_data[0]++;
//    }
//  }
//}
