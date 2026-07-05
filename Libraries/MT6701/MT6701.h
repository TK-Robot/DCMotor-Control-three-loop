//
// Created by Administrator on 2025/12/7.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_MT6701_H
#define TRIPLE_CASCADECONTROLDCMOTOR_MT6701_H

#include <stdbool.h>
#include "i2c.h"
#include "TypeDefine.h"
#include "Filter.h"

#define MT6701_ADDR_7BIT  0x06
#define MT6701_ADDRESS      (MT6701_ADDR_7BIT << 1)
#define ReadAddressH    0X03        //数据高位寄存器地址

typedef struct
{
    // 属性
    I2C_HandleTypeDef* iic;//接收
    Param* param;
    bool dma_busy;
    LPF_Filter SpeedFilter;
    LPF_Filter AccDecSpeedFilter;
    // 方法（函数指针）
    // void (*MT6701_init)(void*,I2C_HandleTypeDef*,Param*); // 用于打招呼的方法
    // void (*MT6701_Update)(void*);
    // void (*MT6701_EncoderManage)(void*);
}MT6701;

void MT6701_init(MT6701* MT,I2C_HandleTypeDef *hi2c1,Param* params);
void MT6701_Update(MT6701* MT);
void MT6701_CodedManage(MT6701* MT);
void MT6701_SpeedUpdate(MT6701* MT);
#endif //TRIPLE_CASCADECONTROLDCMOTOR_MT6701_H