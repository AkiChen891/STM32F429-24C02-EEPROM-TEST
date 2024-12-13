/**
  ******************************************************************************
  * @file    24cxx.c
  * @brief   24cxx EEPROM底层驱动
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
#include "24cxx.h"
#include "delay.h"

/**
* @brief 向 AT24CXX 指定地址写入一个数据
* @param addr: 写入数据的目的地址
* @param data: 要写入的数据
* @retval 无
*/
void at24cxx_write_one_byte(uint16_t addr, uint8_t data)
{
    iic_start();       /* IIC起始信号 */

    if (EE_TYPE > AT24C16)  /* 容量大于24C16时分2个字节发送目标内存地址 */
    {
        iic_send_byte(0xA0);    /* 发送写命令，从机设备地址为0xA0，最低位为0表示写入 */
        iic_wait_ack();         /* 发送完一个字节后等待ACK */
        iic_send_byte(addr >> 8);   /* 发送高位内存地址 */
    }
    else
    {
        iic_send_byte(0xA0 + ((addr >> 8) << 1));   /* 发送器件0xA0 + 高位a8/a9/a10地址，写数据 */
    }

    iic_wait_ack();
    iic_send_byte(addr % 256);  /* 发送低位内存地址 */
    iic_wait_ack();

    iic_send_byte(data);    /* 发送1个字节 */
    iic_wait_ack();
    iic_stop();
    delay_ms(10);       /* EEPROM写入速度慢，必须等待写入完成再写下一个 */
}

/**
* @brief 在 AT24CXX 指定地址写入一个数据
* @param addr: 写入数据的目的地址
* @param data: 要写入的数据
* @retval 无
*/
uint8_t at24cxx_read_one_byte(uint16_t addr)
{
    uint8_t temp = 0;
    iic_start();

    /* 根据不同的 24CXX 型号, 发送高位地址
    * 1, 24C16 以上的型号, 分 2 个字节发送地址
    * 2, 24C16 及以下的型号, 分 1 个低字节地址 + 占用器件地址的 bit1 ~ bit3 位
    * 用于表示高位地址, 最多 11 位地址
    * 对于 24C01/02, 其器件地址格式(8bit)为: 1 0 1 0 A2 A1 A0 R/W
    * 对于 24C04, 其器件地址格式(8bit)为: 1 0 1 0 A2 A1 a8 R/W
    * 对于 24C08, 其器件地址格式(8bit)为: 1 0 1 0 A2 a9 a8 R/W
    * 对于 24C16, 其器件地址格式(8bit)为: 1 0 1 0 a10 a9 a8 R/W
    * R/W : 读/写控制位 0,表示写; 1,表示读;
    * A0/A1/A2 : 对应器件的 1,2,3 引脚(只有 24C01/02/04/8 有这些脚)
    * a8/a9/a10: 对应存储整列的高位地址, 11bit 地址最多可以表示 2048 个位置,
    * 可以寻址 24C16 及以内的型号
    */

   if (EE_TYPE > AT24C16)       /* 24C16以上型号分2个字节发送地址 */
   {
        iic_send_byte(0xA0);    /* 最低位为0，表示写入*/
        iic_wait_ack();         /* 发送完一个字节后等待ACK信号 */
        iic_send_byte(addr >> 8);       /* 发送高字节地址 */
   }
   else
   {
        /* 发送器件0xA0 + 高位a8/a9/a10地址，写数据 */
        iic_send_byte(0xA0 + ((addr >> 8) << 1));
   }

   iic_wait_ack();
   iic_send_byte(addr % 256);       /* 发送低位地址*/
   iic_wait_ack();

   iic_start();                     /* 重新发送起始信号*/
   iic_send_byte(0xA1);             /* 进入接收模式，最低位为1，表示读取*/
   iic_wait_ack();
   temp = iic_read_byte(0);         /* 接收一个字节数据 */
   iic_stop();

   return temp;
}

/**
* @brief 检查 AT24CXX 是否正常
* @note 检测原理: 在器件的末地址写入 0X55, 然后再读取, 
* 如果读取值为 0X55 则表示检测正常. 否则,则表示检测失败.
* @param 无
* @retval 检测结果
* 0: 检测成功
* 1: 检测失败
*/
uint8_t at24cxx_check(void)
{
    uint8_t temp;
    uint16_t addr = EE_TYPE;                /* 目标内存地址为EEPROM末地址 */
    temp = at24cxx_read_one_byte(addr);     /* 避免每次开机都写入*/

    if (temp == 0x55)                       /* 读取数据正常 */
    {
        return 0;
    }
    else                                    /* 第一次初始化时 */
    {
        at24cxx_write_one_byte(addr, 0x55); /* 先写入 */
        temp = at24cxx_read_one_byte(255);  /* 再读取 */
        if (temp == 0x55) return 0;
    }

    return 1;
}

/**
* @brief 在 AT24CXX 里面的指定地址开始读出指定个数的数据
* @param addr : 开始读出的地址 对 24c02 为 0~255
* @param pbuf : 数据存放数组（缓冲区）首地址
* @param datalen : 要读出数据的个数
* @retval 无
*/
void at24cxx_read(uint16_t addr, uint8_t *pbuf, uint16_t datalen)
{
    while(datalen--)
    {
        *pbuf++ = at24cxx_read_one_byte(addr++);
    }
}

/**
* @brief 在 AT24CXX 里面的指定地址开始写入指定个数的数据
* @param addr : 开始写入的地址 对 24c02 为 0~255
* @param pbuf : 数据数组首地址
* @param datalen : 要写入数据的个数
* @retval 无
*/
void at24cxx_write(uint16_t addr, uint8_t *pbuf, uint16_t datalen)
{
    while(datalen--)
    {
        at24cxx_write_one_byte(addr, *pbuf);
        addr++;
        pbuf++;
    }
}