
#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H


/* Includes ------------------------------------------------------------------*/

#ifdef STM32H723xx 
#include "stm32h7xx_hal.h"
#endif  
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 上位机链路存活状态
 *
 */
typedef enum
{
    Alive_Status_DISABLE = 0,
    Alive_Status_ENABLE,
} Enum_Alive;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif

void Task_Init();
void Task_Loop();
Enum_Alive Get_Alive_Status();
 
#ifdef __cplusplus
}
#endif


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
