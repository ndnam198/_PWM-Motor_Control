/**
 * @file myF103.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-11-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __MY_F103_BOARD_H
#define __MY_F103_BOARD_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "myDebug.h"


/****************************************** BKFET_BOARD DEFINE - BEGIN */
#define BKFET_BOARD ((void)0U)

#define LED_1   GPIO_PIN_5
#define LED_2   GPIO_PIN_12
#define LED_3   GPIO_PIN_11
#define LED_4   GPIO_PIN_8
#define ON      (0U)
#define OFF     (1U)


#define __MY_TOGGLE_LED(__LED_NUMBER__)                                    \
  (((__LED_NUMBER__) == LED_1) ? (HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5))   :\
   ((__LED_NUMBER__) == LED_2) ? (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12))  :\
   ((__LED_NUMBER__) == LED_3) ? (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11))  :\
   (HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8)))

#define __MY_WRITE_LED(__LED_NUMBER__, __STATE__)                                   \
  (((__LED_NUMBER__) == LED_1) ? (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  __STATE__)) :\
   ((__LED_NUMBER__) == LED_2) ? (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, __STATE__)) :\
   ((__LED_NUMBER__) == LED_3) ? (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, __STATE__)) :\
   (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, __STATE__)))

#define __MY_TOGGLE_ALL_LED() \
    do                           \
    {                            \
        __MY_TOGGLE_LED(LED_1);  \
        __MY_TOGGLE_LED(LED_2);  \
        __MY_TOGGLE_LED(LED_3);  \
        __MY_TOGGLE_LED(LED_4);  \
    } while (0)

#define __MY_ON_ALL_LED() \
    do                             \
    {                              \
        __MY_WRITE_LED(LED_1, 0);  \
        __MY_WRITE_LED(LED_2, 0);  \
        __MY_WRITE_LED(LED_3, 0);  \
        __MY_WRITE_LED(LED_4, 0);  \
    } while (0)

#define __MY_OFF_ALL_LED() \
    do                             \
    {                              \
        __MY_WRITE_LED(LED_1, 1);  \
        __MY_WRITE_LED(LED_2, 1);  \
        __MY_WRITE_LED(LED_3, 1);  \
        __MY_WRITE_LED(LED_4, 1);  \
    } while (0)

/****************************************** BKFET_BOARD DEFINE - END */

#endif /* !__MY_F103_BOARD_H */
