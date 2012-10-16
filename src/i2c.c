#include "stdio.h"
#include "stm32f4xx.h"
#include "i2c.h"

#define I2C                          I2C2
#define I2C_CLK                      RCC_APB1Periph_I2C2
#define I2C_SCL_PIN                  GPIO_Pin_10
#define I2C_SCL_GPIO_PORT            GPIOB
#define I2C_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C_SCL_SOURCE               GPIO_PinSource10
#define I2C_SCL_AF                   GPIO_AF_I2C2
#define I2C_SDA_PIN                  GPIO_Pin_11
#define I2C_SDA_GPIO_PORT            GPIOB
#define I2C_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C_SDA_SOURCE               GPIO_PinSource11
#define I2C_SDA_AF                   GPIO_AF_I2C2

#ifndef I2C_SPEED
 #define I2C_SPEED                        100000
#endif /* I2C_SPEED */

#define FLAG_TIMEOUT         ((uint32_t)0x1000)
#define LONG_TIMEOUT         ((uint32_t)(10 * FLAG_TIMEOUT))

/* SB0 = INT0(in)  PB1 = INT1(in)  SLC = PB2(out)  SDA = PB3(in/out)  */
/* I2C2SCL は PB10 I2C2SDA は PB11 */

/* I2C の初期化 */
void I2Cs_Initialize(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  I2C_InitTypeDef  I2C_InitStructure;

  RCC_APB1PeriphClockCmd(I2C_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(I2C_SCL_GPIO_CLK | I2C_SDA_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
  GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(I2C_SCL_GPIO_PORT, I2C_SCL_SOURCE, I2C_SCL_AF);
  GPIO_PinAFConfig(I2C_SDA_GPIO_PORT, I2C_SDA_SOURCE, I2C_SDA_AF);  

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  I2C_Cmd(I2C, ENABLE);
  I2C_Init(I2C, &I2C_InitStructure);
}

uint16_t I2Cs_read(uint16_t d_address,uint16_t address)
{
  uint32_t ack;
  uint16_t r = 0;
  __IO uint32_t  Timeout; 
  
  Timeout = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C BUSY TIMEOUT\r\n");
      return 0xffff;
    }
  }
  I2C_GenerateSTART(I2C, ENABLE);
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT MASTER MODE TIMEOUT\r\n");
      return 0xffff;
    }
  }    
  I2C_Send7bitAddress(I2C,(d_address << 1), I2C_Direction_Transmitter);
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT TRANS MODE TIMEOUT\r\n");
      return 0xffff;
    }
  } 
  I2C_SendData(I2C, address);  
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT TRANSMITTED TIMEOUT\r\n");
      return 0xffff;
    }
  }
  I2C_GenerateSTOP(I2C, ENABLE);
  Timeout = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C BUSY TIMEOUT\r\n");
      return 0xffff;
    }
  }
  I2C_GenerateSTART(I2C, ENABLE);
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT MASTER MODE TIMEOUT\r\n");
      return 0xffff;
    }
  }    
  I2C_Send7bitAddress(I2C,(d_address << 1), I2C_Direction_Receiver);  
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT RECEIVE MODE TIMEOUT\r\n");
      return 0xffff;
    }
  } 
  I2C_AcknowledgeConfig(I2C, DISABLE);   
  I2C_GenerateSTOP(I2C, ENABLE);
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT RECEIVED TIMEOUT\r\n");
      return 0xffff;
    }
  }
  r = I2C_ReceiveData(I2C);
  return r;
}

uint16_t I2Cs_write(uint16_t d_address,uint16_t address,uint16_t data)
{
  uint32_t ack;
  uint16_t r = 0;
  __IO uint32_t  Timeout; 
  
  Timeout = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C BUSY TIMEOUT\r\n");
      return 0xffff;
    }
  }
  I2C_GenerateSTART(I2C, ENABLE);
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT MASTER MODE TIMEOUT\r\n");
      return 0xffff;
    }
  }    
  I2C_Send7bitAddress(I2C,(d_address << 1), I2C_Direction_Transmitter);
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT TRANS MODE TIMEOUT\r\n");
      return 0xffff;
    }
  } 
  I2C_SendData(I2C, address);  
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT TRANSMITTED TIMEOUT\r\n");
      return 0xffff;
    }
  }
  I2C_SendData(I2C, data);  
  Timeout = FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if((Timeout--) == 0) {
      I2C_GenerateSTOP(I2C, ENABLE);
      printf("I2C CHECK EVENT TRANSMITTED TIMEOUT\r\n");
      return 0xffff;
    }
  }
  I2C_GenerateSTOP(I2C, ENABLE);
  return 0x0000;
}

