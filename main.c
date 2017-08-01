#include "stm32f10x.h"
#include "delay.h"
#include "usart_rxtx.h"
#include "hwdrv.h"
#include "mpu6050.h"

#define NUM 20
#define FAILURES 4
#define CALIBRATENUM 10

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
__IO uint8_t drv1 = MIDPWM;
__IO uint8_t drv2 = MIDPWM;

uint8_t device;
int8_t dataint, datafract;

GPIO_InitTypeDef gpio_sensors;

uint32_t I2C_Rx_Buffer[14];
uint32_t I2C_DR_Address = 0x40005810;

s16 AccelGyroTemp[8];
s16 offset[6];

void sensor_handler(void);
void forward(void);
void backward(void);
void left(void);
void right(void);
void stop(void);
void move(void);
void status_handler(void);
void cmd_handler(char*);
void calibrate_gyracc(void);
void calc_gyracc(void);
void TIM3_Init(void);
void TIM4_Init(void);
void DMA_Configuration(void);
void EXTI_MPU6050_Configuration(void);

uint8_t calib_count = 0;
s32 offset_gyr_x = 0;
s32 offset_gyr_y = 0;
s32 offset_gyr_z = 0;
s32 offset_acc_x = 0;
s32 offset_acc_y = 0;
s32 offset_acc_z = 0;

s16 gyr_x_ang = 0;
s16 gyr_y_ang = 0;

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
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    RCC_APB1PeriphClockCmd(
                    RCC_APB1Periph_TIM2 | 
                    RCC_APB1Periph_TIM3 | 
                    RCC_APB1Periph_TIM4 |
                    RCC_APB1Periph_I2C2, 
                    ENABLE);
 
	RCC_APB2PeriphClockCmd(
                    RCC_APB2Periph_AFIO   |
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

void EXTI_MPU6050_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    // Configure EXTI Line1 to generate an interrupt on rising or falling edge
    EXTI_InitStruct.EXTI_Line = EXTI_Line1; 
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
    response[4] = (uint8_t)((gyr_x_ang >> 8) & 0xFF);
    response[5] = (uint8_t)(gyr_x_ang & 0xFF);
    /*
    response[6] = (uint8_t)((AccelGyroTemp[1] >> 8) & 0xFF);
    response[7] = (uint8_t)(AccelGyroTemp[1] & 0xFF);
    response[8] = (uint8_t)((AccelGyroTemp[2] >> 8) & 0xFF);
    response[9] = (uint8_t)(AccelGyroTemp[2] & 0xFF);
    response[10] = (uint8_t)((AccelGyroTemp[3] >> 8) & 0xFF);
    response[11] = (uint8_t)(AccelGyroTemp[3] & 0xFF);
    response[12] = (uint8_t)((AccelGyroTemp[4] >> 8) & 0xFF);
    response[13] = (uint8_t)(AccelGyroTemp[4] & 0xFF);
    response[14] = (uint8_t)((AccelGyroTemp[5] >> 8) & 0xFF);
    response[15] = (uint8_t)(AccelGyroTemp[5] & 0xFF);
    response[16] = (uint8_t)((AccelGyroTemp[6] >> 8) & 0xFF);
    response[17] = (uint8_t)(AccelGyroTemp[6] & 0xFF);
    */
	response[18] = STOPMARKER;
	UARTSend(&response[0], 19);
}

void SysTick_Handler(void)
{
    
    
    //MPU6050_GetRawAccelGyroTemp(&AccelGyroTemp[0]);
    calc_gyracc();
    SendResponse();

	if ((Status_OK & status) == 0)
		count++;

	if (count >= FAILURES)
	{
		status = 0x0;
		stop();
		count = 0;
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
	}

	if (status & Status_AUTO)
	{
		if (status & Status_WALL)
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
				status &= ~Status_WALL;
                /* restore the previous command */
                status_handler();
                move();
			}
		}
		else
		{
			sensor_handler();
		}
	}

}

void calc_gyracc(void)
{
    s16 tmp = 0;
    tmp = AccelGyroTemp[3];
    if (offset_gyr_x > 0) 
    {
        tmp -= offset_gyr_x;
    }
    else
    {
        tmp += offset_gyr_x;
    }
    gyr_x_ang += tmp / 131;
}

int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    
    //MPU6050_I2C_Init();
    //MPU6050_Initialize();
    //MPU6050_GetDeviceID();
    
    //status |= Status_CALIB;
    
    //EXTI_MPU6050_Configuration();
    
	USART_NVIC_Configuration();
	USART_GPIO_Configuration();
	USART_Configuration();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	TIM4_Init();
        
    //DMA_Configuration();
    
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
    switch (0x0F & status)
    {
        case Status_FORWARD:
            drv1 = MAXPWM; drv2 = MAXPWM;
            break;
        case Status_BACK:
            drv1 = MINPWM; drv2 = MINPWM;
            break;
        case Status_LEFT:
            drv1 = MINPWM; drv2 = MAXPWM;
            break;
        case Status_RIGHT:
            drv2 = MINPWM; drv1 = MAXPWM;
            break;
        case Status_SLEFT:
            drv1 = SPWM; drv2 = MAXPWM;
            break;
        case Status_SRIGHT:
            drv2 = SPWM; drv1 = MAXPWM;
            break;
        case Status_SBLEFT:
            drv1 = SBPWM; drv2 = MINPWM;
            break;
        case Status_SBRIGHT:
            drv2 = SBPWM; drv1 = MINPWM;
            break;
        default:
            drv1 = MIDPWM; drv2 = MIDPWM;
            break;
    }
}

void cmd_handler(char *cmd)
{
	status = (uint8_t)cmd[0];
	count = 0;
    if  (Status_AUTO & status)
    {
        status_handler();
    }
    else
    {
        drv1 = (uint8_t)cmd[1];
        drv2 = (uint8_t)cmd[2];
    }
    move();
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
        status |= Status_WALL;
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
}
/*
void I2C_DMA_Read(u8 slaveAddr, u8 readAddr)
{
    // Disable DMA channel 
    DMA_Cmd(DMA1_Channel5, DISABLE);
    
    // Set current data number again to 14 for MPU6050, only possible after disabling the DMA channel 
    DMA_SetCurrDataCounter(DMA1_Channel5, 14);
    
    // While the bus is busy 
    while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY));

    // Enable DMA NACK automatic generation 
    I2C_DMALastTransferCmd(MPU6050_I2C, ENABLE);					//Note this one, very important

    // Send START condition 
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);

    // Test on EV5 and clear it 
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    // Send MPU6050 address for write 
    I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter); 

    // Test on EV6 and clear it 
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Clear EV6 by setting again the PE bit 
    I2C_Cmd(MPU6050_I2C, ENABLE);

    // Send the MPU6050's internal address to write to 
    I2C_SendData(MPU6050_I2C, readAddr);

    // Test on EV8 and clear it 
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Send STRAT condition a second time 
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);

    // Test on EV5 and clear it 
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    // Send MPU6050 address for read 
    I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);

    // Test on EV6 and clear it 
    while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // Start DMA to receive data from I2C 
    DMA_Cmd(DMA1_Channel5, ENABLE);
    I2C_DMACmd(MPU6050_I2C, ENABLE);

    // When the data transmission is complete, it will automatically jump to DMA interrupt routine to finish the rest.
    //now go back to the main routine
}
*/
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1))			//MPU6050_INT
    {
        EXTI_ClearITPendingBit(EXTI_Line1);
        MPU6050_GetRawAccelGyroTemp(&AccelGyroTemp[0]);
        if (status & Status_CALIB)
        {
            offset_acc_x += AccelGyroTemp[0];
            offset_acc_y += AccelGyroTemp[1];
            offset_acc_z += AccelGyroTemp[2];
            offset_gyr_x += AccelGyroTemp[3];
            offset_gyr_y += AccelGyroTemp[4];
            offset_gyr_z += AccelGyroTemp[5];
            calib_count++;
            if (calib_count >= CALIBRATENUM)
            {
                offset_acc_x /= CALIBRATENUM;
                offset_acc_y /= CALIBRATENUM;
                offset_acc_z /= CALIBRATENUM;
                offset_gyr_x /= CALIBRATENUM;
                offset_gyr_y /= CALIBRATENUM;
                offset_gyr_z /= CALIBRATENUM;
                status &= ~(Status_CALIB);
            }
        }
        //I2C_DMA_Read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H);
    }
}
/*
void DMA1_Channel5_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_IT_TC5))
    {
        // Clear transmission complete flag 
        DMA_ClearFlag(DMA1_FLAG_TC5);

        I2C_DMACmd(MPU6050_I2C, DISABLE);
        
        // Send I2C2 STOP Condition 
        I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
        
        // Disable DMA channel
        DMA_Cmd(DMA1_Channel5, DISABLE);

        //Read Accel data from byte 0 to byte 2
        for(i=0; i<3; i++) 
            AccelGyroTemp[i] = ((s16)((u16)I2C_Rx_Buffer[2*i] << 8) + I2C_Rx_Buffer[2*i+1]);
        // Get Temperature 
        AccelGyroTemp[6] = ((s16) ((u16) I2C_Rx_Buffer[6] << 8) + I2C_Rx_Buffer[7]);
        //Read Gyro data from byte 4 to byte 6
        for(i=4; i<7; i++)
            AccelGyroTemp[i-1] = ((s16)((u16)I2C_Rx_Buffer[2*i] << 8) + I2C_Rx_Buffer[2*i+1]);
    }
}
*/
/*
void DMA_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    DMA_DeInit(DMA1_Channel5); //reset DMA1 channel to default values;

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C_DR_Address;    // =0x40005810 : address of data reading register of I2C2
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)I2C_Rx_Buffer;         // variable to store data
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // channel will be used for peripheral to memory transfer
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	                        // setting normal mode (non circular)
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	                // medium priority
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	                    // Location assigned to peripheral register will be source
    DMA_InitStructure.DMA_BufferSize = 14;	                                // number of data to be transfered
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // automatic memory increment disable for peripheral
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	                // automatic memory increment enable for memory
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	// source peripheral data size = 8bit
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	        // destination memory data size = 8bit
    
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;                // I2C2 connect to channel 5 of DMA1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
*/
