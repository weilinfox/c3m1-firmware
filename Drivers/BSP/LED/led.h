/**
 * @file        led.h
 * @brief       C3M1 status LED driver (LED1 on PD9, active LOW).
 */

#ifndef LED_H_
#define LED_H_

#include "../../SYSTEM/sys/sys.h"

/* LED1 (PD9): 0 = ON, 1 = OFF */
#define LED1(x)   do{ (x) ? \
                      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); \
                  }while(0)

#define LED1_TOGGLE()   do{ HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); }while(0)

#endif /* LED_H_ */
