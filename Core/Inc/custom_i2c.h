/**
 ****************************************************************************************************
 * @file        custom_i2c.h
 * @author      Aki
 * @version     V1.0
 * @date        2024-12-11
 * @brief       软件IIC头文件
 ****************************************************************************************************
 */

#ifndef __MYIIC_H
#define __MYIIC_H

#include "sys.h"


/******************************************************************************************/
/* 引脚定义 */

#define IIC_SCL_GPIO_PORT               GPIOH
#define IIC_SCL_GPIO_PIN                GPIO_PIN_4
#define IIC_SCL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)   /* PH口时钟使能 */

#define IIC_SDA_GPIO_PORT               GPIOH
#define IIC_SDA_GPIO_PIN                GPIO_PIN_5
#define IIC_SDA_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)   /* PH口时钟使能 */

/******************************************************************************************/
/* IO操作 */

#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN)        /* 读取SDA电平 */

/******************************************************************************************/

void iic_init(void);                        
void iic_start(void);                       
void iic_stop(void);                      
void iic_ack(void);                        
void iic_nack(void);                       
uint8_t iic_wait_ack(void);                 
void iic_send_byte(uint8_t data);          
uint8_t iic_read_byte(unsigned char ack);   

#endif