#include "at24cxx.h"
#include "i2c.h"
//��ʼ��IIC�ӿ�

#define AT24CXX_HANDLE	(&hi2c1)	//IIC�ӿ�
#define AT24C_DEV_ADDR  (0XA0) //�豸��ַ

void AT24CXX_Init(void)
{
	//IIC_Init();//IIC��ʼ��
	AT24CXX_Check();
}

/*****************************************
��������void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
������WriteAddr :Ҫд�����ݵĵ�ַ  pBuffer��Ҫд������ݵ��׵�ַ NumToWrite��Ҫд�����ݵĳ���
������������ָ����ַ��ʼд�����ֽ�����
����ֵ����
*****************************************/
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
//	if(EE_TYPE < AT24C16)
//		HAL_I2C_Mem_Write(AT24CXX_HANDLE,AT24C_DEV_ADDR,WriteAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumToWrite,HAL_MAX_DELAY);
//	else
		HAL_I2C_Mem_Write(AT24CXX_HANDLE,AT24C_DEV_ADDR,WriteAddr,I2C_MEMADD_SIZE_16BIT,pBuffer,NumToWrite,HAL_MAX_DELAY);
		HAL_Delay(5);
}
/*****************************************
��������AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
������ ReadAddr��Ҫ��ȡ���ݵĵ�ַ pBuffer�����������׵�ַ NumToRead:���ݳ���
������������ָ����ַ��ʼ��ȡ������ֽ�����
����ֵ����
*****************************************/
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	if(EE_TYPE < AT24C16)
		HAL_I2C_Mem_Read(AT24CXX_HANDLE,AT24C_DEV_ADDR,ReadAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumToRead,HAL_MAX_DELAY);
	else
		HAL_I2C_Mem_Read(AT24CXX_HANDLE,AT24C_DEV_ADDR,ReadAddr,I2C_MEMADD_SIZE_16BIT,pBuffer,NumToRead,HAL_MAX_DELAY);
} 
/*****************************************
��������uint8_t AT24CXX_Check(void)
��������
�������������AT24CXX�Ƿ���������������24XX�����һ����ַ(255)���洢��־��.���������24Cϵ��,�����ַҪ�޸�
����ֵ�����ɹ�����0 ʧ�ܷ���1
*****************************************/
uint8_t AT24CXX_Check(void)
{
	uint8_t temp;
	uint8_t data = 0XAB;
	AT24CXX_Read(EE_TYPE,&temp,1);//����ÿ�ο�����дAT24CXX			   
	if(temp != 0XAB)
		return 1;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_Write(EE_TYPE,&data,1);
	    AT24CXX_Read(EE_TYPE,&temp,1);;	  
		if(temp != 0XAB)
			return 1;
	}
	return 0;											  
}

