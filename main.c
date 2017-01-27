#include "stm32f10x.h"
#include "delay.h"
#include "usart_rxtx.h"
#include "hwdrv.h"

#define NUM 5
#define FAILURES 4

#define AUTO	  ((0x51 << 8) | 0x52) /* the command is QR */
#define FORWARD	  ((0x46 << 8) | 0x52) /* the command is FR */
#define BACK 	  ((0x42 << 8) | 0x4B) /* the command is BK */
#define LEFT	  ((0x4C << 8) | 0x46) /* the command is LF */
#define RIGHT 	  ((0x52 << 8) | 0x54) /* the command is RT */
#define STOP 	  ((0x53 << 8) | 0x54) /* the command is ST */
#define OK	      ((0x4F << 8) | 0x4B) /* the command is OK */
#define MOVE	  ((0x4D << 8) | 0x56) /* the command is MV */

/* Bit definition for status register */
#define FORWARD_Status	0
#define BACK_Status		1
#define LEFT_Status		2
#define RIGHT_Status	3
#define MOVE_Status		4
#define WALL_Status		5
#define AUTO_Status		6
#define CONNOK  		7

#define STARTMARKER		0x81
#define STOPMARKER 		0x8F

#define TEN(NUMBER) ((3 << 4) | NUMBER / 10)
#define ONE(NUMBER) ((3 << 4) | NUMBER % 10)

__IO uint8_t cmd_count = 0;
__IO uint8_t status = 0;
__IO uint8_t count = 0;
__IO uint16_t sensor_data = 0xFFFF;

__IO uint8_t i = 0;
__IO uint8_t p = 0;
char cmd[NUM] = { '\0' };
uint8_t response[NUM] = { '\0' };
__IO uint8_t flg = 0;
__IO int8_t drv1 = 0;
__IO int8_t drv2 = 0;

GPIO_InitTypeDef gpio_sensors;

void sensor_handler(void);
void forward(void);
void backward(void);
void left(void);
void right(void);
void stop(void);
void move(void);
void status_handler(void);
void cmd_handler(char*);
void TIM4_Init(void);

typedef struct {
    void (*func)(void);
    uint32_t ticks;
} Task;

Task allTasks[10] = {0};


typedef struct {
	void (*func)(void);
	uint32_t ticks;
} Action;

Action detour[3];

void RCC_Configuration(void)
{
    RCC_APB1PeriphClockCmd(
                    RCC_APB1Periph_TIM2 | 
                    RCC_APB1Periph_TIM3 | 
                    RCC_APB1Periph_TIM4, 
                    ENABLE);
 
	RCC_APB2PeriphClockCmd(
                    RCC_APB2Periph_USART1 | 
                    RCC_APB2Periph_GPIOA  | 
                    RCC_APB2Periph_GPIOB  | 
                    RCC_APB2Periph_GPIOC, 
                    ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef statusPin;
	GPIO_InitTypeDef gpio_motordrv;
    gpio_sensors.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_sensors.GPIO_Pin = QTRSENSOR1 | QTRSENSOR2;

	gpio_motordrv.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_Out_PP;
	gpio_motordrv.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_motordrv.GPIO_Pin = MOTORA1 | MOTORA2 | MOTORB1 | MOTORB2;

	statusPin.GPIO_Mode = GPIO_Mode_Out_PP;
	statusPin.GPIO_Speed = GPIO_Speed_2MHz;
	statusPin.GPIO_Pin = GPIO_Pin_13;

	GPIO_Init(MOTORx, &gpio_motordrv);
	GPIO_Init(GPIOC, &statusPin);
	GPIO_SetBits(GPIOC, statusPin.GPIO_Pin);
}

/**
 * @brief  This function handles USARTx global interrupt request.
 * @param  None
 * @retval None
 */
void USART1_IRQHandler(void)
{
	if ((USART1->SR & USART_FLAG_RXNE) != (u16) RESET)
	{
		i = USART_ReceiveData(USART1);

		if (i == STOPMARKER)
		{
			cmd_count++;
			cmd_handler(&cmd[0]);
			p = 0;
			flg &= ~(1 << 0);
		}
		if (flg & (1 << 0))
		{
			if (p < NUM)
			{
				cmd[p] = i;
				p++;
			}
		}
		if (i == STARTMARKER)
		{
			flg |= (1 << 0);
			p = 0;
		}
	}
}

/* Send response to RC */
void SendResponse(void)
{ 
    response[0] = STARTMARKER;
	response[1] = status;
	response[2] = drv1;
	response[3] = drv2;
	response[4] = STOPMARKER;
	UARTSend(&response[0], 5);
}

void SysTick_Handler(void)
{
    
    SendResponse();

	if (((1 << CONNOK) & status) == 0)
		count++;

	if (count >= FAILURES)
	{
		status = 0x0;
		stop();
		count = 0;
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
	}

	status &= ~(1 << CONNOK);

	if (status & (1 << AUTO_Status))
	{
		if (status & (1 << WALL_Status))
		{
			if (detour[0].ticks > 0)
			{
				detour[0].ticks--;
				detour[0].func();
			}
			else if (detour[1].ticks > 0)
			{
				detour[1].ticks--;
				detour[1].func();
			}
			else
			{
				stop();
				status &= ~(1 << WALL_Status);
                /* restore the previous command */
                status_handler();
			}
		}
		else
		{
			sensor_handler();
		}
	}

}

int main(void)
{
    
    RCC_Configuration();
    
    GPIO_Configuration();
    
	USART_NVIC_Configuration();
	USART_GPIO_Configuration();
	USART_Configuration();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	TIM4_Init();

	SysTick_Config(0XFFFFFF);

	while (1)
	{
		;
	}
}

void forward(void)
{
	drv1 = drv2 = MAXPWM;
	move();
}

void backward(void)
{
	drv1 = drv2 = MINPWM;
	move();
}

void left(void)
{
	drv1 = MINPWM;
	drv2 = MAXPWM;
	move();
}

void right(void)
{
	drv1 = MAXPWM;
	drv2 = MINPWM;
	move();
}

void stop(void)
{
	drv1 = drv2 = MIDPWM;
	move();
}

void move()
{
	if ((drv2 >= MIDPWM) && (drv2 <= MAXPWM))
	{
		TIM_SetCompare1(TIM4, drv2 - MIDPWM);
		TIM_SetCompare2(TIM4, MINPWM);
	}
	else if ((drv2 >= MINPWM) && (drv2 < MIDPWM))
	{
		TIM_SetCompare1(TIM4, MINPWM);
		TIM_SetCompare2(TIM4, (drv2 - MIDPWM) * -1);
	}
	if ((drv1 >= MIDPWM) && (drv1 <= MAXPWM))
	{
		TIM_SetCompare3(TIM4, drv1 - MIDPWM);
		TIM_SetCompare4(TIM4, MINPWM);
	}
	else if ((drv1 >= MINPWM) && (drv1 < MIDPWM))
	{
		TIM_SetCompare3(TIM4, MINPWM);
		TIM_SetCompare4(TIM4, (drv1 - MIDPWM) * -1);
	}
}

void status_handler(void)
{
    uint8_t movestatus = 0x0F & status;
    switch (movestatus)
    {
    case (1 << FORWARD_Status):
        forward();
        break;
    case (1 << BACK_Status):
        backward();
        break;
    case (1 << LEFT_Status):
        left();
        break;
    case (1 << RIGHT_Status):
        right();
        break;
    }
}

void cmd_handler(char *cmd)
{
	uint16_t c;
	c = (uint16_t)(cmd[0] << 8) | cmd[1];
	status |= (1 << CONNOK);
	count = 0;
	switch (c)
	{
	case MOVE:
		if ((status & (1 << AUTO_Status)) == 0)
		{
			status &= 0xF0;
			status |= (1 << MOVE_Status);
			drv1 = (int8_t)cmd[2];
			drv2 = (int8_t)cmd[3];
			move();
		}
		break;
	case FORWARD:
		status &= 0xF0;
		status |= (1 << FORWARD_Status);
		forward();
		break;
	case BACK:
		status &= 0xF0;
		status |= (1 << BACK_Status);
		backward();
		break;
	case LEFT:
		status &= 0xF0;
		status |= (1 << LEFT_Status);
		left();
		break;
	case RIGHT:
		status &= 0xF0;
		status |= (1 << RIGHT_Status);
		right();
		break;
	case STOP:
		status &= 0xF0;
		stop();
		break;
	case AUTO:
		if (status & (1 << AUTO_Status))
		{
			status &= ~(1 << AUTO_Status);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		}
		else
		{
			status |= (1 << AUTO_Status);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		}
		break;
	case OK:
		break;
	}
}

void TIM4_Init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// PWM frequency = 10000 hz with 72,000,000 Hz system clock
	// 72,000,000 / 7200 = 10000
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseStructure.TIM_Period = 64 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration
	// Edge - aligned; not single pulse mode
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	// Enable Timer
	TIM_Cmd(TIM4, ENABLE);
}

void sensor_handler(void)
{
	uint8_t count1 = 0;
	uint8_t count2 = 0;
	uint8_t flg = 0x3;
	/* Set GPIO sensors to output */
	gpio_sensors.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_sensor, &gpio_sensors);
	/* Set GPIO sensors to HIGH for 20 us */
	GPIO_SetBits(GPIO_sensor, gpio_sensors.GPIO_Pin);
	delay_us(20);
	/* Set GPIO sensors to input */
	gpio_sensors.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIO_sensor, &gpio_sensors);
    while (flg > 0)
    {
    	if (READSENSOR1)
    	{
      		if (count1 == 255) flg &= (1 << 1); else count1++;
    	}
    	else
    	{
    		flg &= (1 << 1);
    	}
    	if (READSENSOR2)
    	{
      		if (count2 == 255) flg &= (1 << 0); else count2++;
    	}
    	else
    	{
    		flg &= (1 << 0);
		}
    	delay_us(5);
    }
    if ((count2 < 0xFF) | (count1 < 0xFF))
    {
        stop();
        status |= (1 << WALL_Status);
        detour[0].func = &backward;
        detour[0].ticks = 10;
        if (count2 > count1)
        {
            detour[1].func = &right;
            detour[1].ticks = 5;
        }
        else
        {
            detour[1].func = &left;
            detour[1].ticks = 5;
        }
    }
    //return (uint16_t)((count2 << 8) | count1);
}
