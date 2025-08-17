/**
 * CT117E-M4 / GPIO - I2C
*/

#include "i2c_hal.h"

#define DELAY_TIME	9



// ��ʼ�� DWT
void DWT_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

// ΢�뼶��ʱ (200MHz ϵͳʱ��)
void delay1(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * 200;  // 200MHz: 1us = 200��ʱ������
    
    // ������������
    if (cycles > 0x7FFFFFFF) {
        cycles = 0x7FFFFFFF;  // �����ʱԼ 10.7 ��
    }
    
    while ((DWT->CYCCNT - start) < cycles) {
        // �ȴ�
    }
}


//
void SDA_Input_Mode()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//
void SDA_Output_Mode()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//
void SDA_Output(uint16_t val)
{
    if (val)
    {
        //GPIOB->BSRR = GPIO_PIN_7; // ??????
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
    }
    else
    {
        //GPIOB->BSRR = (uint32_t)GPIO_PIN_7 << 16; // ??????
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
    }
}

//
void SCL_Output( uint16_t val )
{
     if (val)
    {
        //GPIOB->BSRR = GPIO_PIN_7; // ??????
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
    }
    else
    {
        //GPIOB->BSRR = (uint32_t)GPIO_PIN_7 << 16; // ??????
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
    }
}

//
uint8_t SDA_Input(void)
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET){
		return 1;
	}else{
		return 0;
	}
}

//
//static void delay1(unsigned int n)
//{
//    uint32_t i;
//    for ( i = 0; i < n; ++i);
//}

//
void I2CStart(void)
{
    SDA_Output(1);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SDA_Output(0);
    delay1(DELAY_TIME);
    SCL_Output(0);
    delay1(DELAY_TIME);
}

//
void I2CStop(void)
{
    SCL_Output(0);
    delay1(DELAY_TIME);
    SDA_Output(0);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SDA_Output(1);
    delay1(DELAY_TIME);

}

//
unsigned char I2CWaitAck(void)
{
    unsigned short cErrTime = 5;
    SDA_Input_Mode();
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    while(SDA_Input())
    {
        cErrTime--;
        delay1(DELAY_TIME);
        if (0 == cErrTime)
        {
            SDA_Output_Mode();
            I2CStop();
            return ERROR;
        }
    }
    SCL_Output(0);
    SDA_Output_Mode();
    delay1(DELAY_TIME);
    return SUCCESS;
}

//
void I2CSendAck(void)
{
    SDA_Output(0);
    delay1(DELAY_TIME);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SCL_Output(0);
    delay1(DELAY_TIME);

}

//
void I2CSendNotAck(void)
{
    SDA_Output(1);
    delay1(DELAY_TIME);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SCL_Output(0);
    delay1(DELAY_TIME);

}

//
void I2CSendByte(unsigned char cSendByte)
{
    unsigned char  i = 8;
    while (i--)
    {
        SCL_Output(0);
        delay1(DELAY_TIME);
        SDA_Output(cSendByte & 0x80);
        delay1(DELAY_TIME);
        cSendByte += cSendByte;
        delay1(DELAY_TIME);
        SCL_Output(1);
        delay1(DELAY_TIME);
    }
    SCL_Output(0);
    delay1(DELAY_TIME);
}

//
unsigned char I2CReceiveByte(void)
{
    unsigned char i = 8;
    unsigned char cR_Byte = 0;
    SDA_Input_Mode();
    while (i--)
    {
        cR_Byte += cR_Byte;
        SCL_Output(0);
        delay1(DELAY_TIME);
        delay1(DELAY_TIME);
        SCL_Output(1);
        delay1(DELAY_TIME);
        cR_Byte |=  SDA_Input();
    }
    SCL_Output(0);
    delay1(DELAY_TIME);
    SDA_Output_Mode();
    return cR_Byte;
}

//
void I2CInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void I2CWrite(unsigned int addr , unsigned char *data,unsigned char num)
{
	I2CStart();
	
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte((unsigned char)(addr >> 8));  // ?8?
    I2CWaitAck();
    I2CSendByte((unsigned char)(addr & 0xFF)); // ?8?
    I2CWaitAck();
	
	while(num--)
	{
		I2CSendByte(*data++);
		I2CWaitAck();
	}
	I2CStop();
	delay1(DELAY_TIME);
}

void I2CRead(unsigned int addr , unsigned char *data,unsigned char num)
{
	I2CStart();
	
	I2CSendByte(0xa0 );
	I2CWaitAck();
	
	I2CSendByte((unsigned char)(addr >> 8));  // ?8?
    I2CWaitAck();
    I2CSendByte((unsigned char)(addr & 0xFF)); // ?8?
    I2CWaitAck();
	
	I2CStart();
	
	I2CSendByte(0xa1);
	I2CWaitAck();
	
	
	while(num--)
	{
		if(num)
		{
			*data++=I2CReceiveByte();
			I2CSendAck();
		}
		else
		{
			*data++=I2CReceiveByte();
			I2CSendNotAck();
		}
	
	}
	I2CStop();
	 delay1(DELAY_TIME);
}
