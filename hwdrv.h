/**
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HWDRV_H
#define __HWDRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#define GPIO_sensor GPIOA
#define QTRSENSOR1 	GPIO_Pin_6
#define QTRSENSOR2 	GPIO_Pin_7
#define READSENSOR1 GPIO_sensor->IDR & QTRSENSOR1
#define READSENSOR2 GPIO_sensor->IDR & QTRSENSOR2

#define MOTORx		GPIOB
#define MOTORA1 	GPIO_Pin_6
#define MOTORA2 	GPIO_Pin_7
#define MOTORB1 	GPIO_Pin_8
#define MOTORB2 	GPIO_Pin_9

#define ROBOTADDR   98D3,32,30511C,10

/* PWM consts */
#define MINPWM      0
#define MAXPWM      127
#define MIDPWM      62

#endif
