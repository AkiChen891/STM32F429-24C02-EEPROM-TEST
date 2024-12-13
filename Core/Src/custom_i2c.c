/**
  ******************************************************************************
  * @file    custom_i2c.c
  * @brief   客制化I2C底层驱动，软件模拟方式
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "custom_i2c.h"
#include "delay.h"

/** 
    @brief:IIC初始化
    @param:NULL
    @return:NULL
**/
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    /* SCL */
    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;            /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);

    /* SDA */
    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;    /* 开漏输出 */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);

    iic_stop();     /* 停止总线上所有设备 */
}

/** 
    @brief:IIC延时函数
    @param:NULL
    @return:NULL
**/
static void iic_delay(void)
{
    delay_us(2);    /* 读写速度在250Khz以内 */
}

/** 
    @brief:IIC起始信号
    @param:NULL
    @return:NULL
**/
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);   /* START信号 */
    iic_delay();
    IIC_SCL(0);   /* 钳住总线，准备发送或接收数据（SCL只有低电平期间允许改变SDA状态） */
    iic_delay();
}


/** 
    @brief:IIC停止信号
    @param:NULL
    @return:NULL
**/
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP信号 */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);     /* 总线结束信号 */
    iic_delay();
}

/** 
    @brief:IIC发送一个字节
    @param:data：要发送的数据
    @return:NULL
**/
void iic_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* 提取高位 */
        iic_delay();
        IIC_SCL(1);     /* 拉高SCL并维持高电平，开始发送 */
        iic_delay();
        IIC_SCL(0);     /* 发送结束，拉低SCL，准备下一位数据 */
        data <<= 1;     /* 左移1位，循环发送 */
    }
    IIC_SDA(1);     /* 发送完成，释放SDA线 */
}

/** 
    @brief:IIC读取一个字节
    @param:ack: ack=1时，发送ack; ack=0时，发送nack
    @return:接收到的数据
**/
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0x00;

    /* 1个字节共8位，单次接收1位，循环8次 */
    for (i = 0; i < 8; i++)
    {
        receive <<= 1;      /* 输出时高位先输出，因此接收时先收到的数据是高位，要左移 */
        IIC_SCL(1);         /* 拉高SCL，SDA准备接收 */
        iic_delay();

        if (IIC_READ_SDA)   /* 如果SDA为高电平 */
        {
            receive++;      /* 第0位（低位）置1 */
        }

        IIC_SCL(0);         /* 接收结束，拉低SCL */
        iic_delay();
    }

    if (!ack)
    {
        iic_nack();         /* 发送nACK信号 */
    }
    else
    {
        iic_ack();          /* 发送ACK信号 */
    }

    return receive;
}

/** 
    @brief:等待应答信号
    @param:NULL
    @return:1=失败；0=成功
**/
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* 主机释放SDA（外部器件此时可拉低SDA电平） */
    iic_delay();
    IIC_SCL(1);     /* 拉高SCL，从机此时可返回ACK */
    iic_delay();

    while (IIC_READ_SDA)    /* SDA等待应答 */
    {
        waittime++;

        if (waittime > 250)     /* 若超时则直接停机 */
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0，结束ACK检查 */
    iic_delay();
    return rack;
}

/** 
    @brief:产生ACK应答
    @param:NULL
    @return:NULL
    @note：ACK应答时，SDA拉低，SCL=0->1->0
**/
void iic_ack(void)
{
    IIC_SDA(0);     
    iic_delay();
    IIC_SCL(1);     
    iic_delay();
    IIC_SCL(0);     
    iic_delay();
    IIC_SDA(1);     
    iic_delay();
}

/** 
    @brief:不产生ACK应答（NACK）
    @param:NULL
    @return:NULL
    @note：NACK应答时，SDA拉高，SCL=0->1->0
**/
void iic_nack(void)
{
    IIC_SDA(1);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}