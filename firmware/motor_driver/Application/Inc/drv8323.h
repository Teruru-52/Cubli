/*
 * drv8323.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef DRV8323_H_
#define DRV8323_H_

#include "main.h"

class DRV8323
{
private:
    SPI_HandleTypeDef *spi;
    GPIO_Value SPI_CS_DRV;
    GPIO_Value nFault;
    GPIO_Value DRV_ENABLE;
    GPIO_Value DRV_CAL;
    GPIO_Value INLx;

    uint16_t fault_status1;
    uint16_t fault_status2;

public:
    DRV8323(SPI_HandleTypeDef *spi, GPIO_Value SPI_CS_DRV, GPIO_Value nFault, GPIO_Value DRV_ENABLE, GPIO_Value DRV_CAL, GPIO_Value INLx);

    uint16_t Read_byte(uint8_t reg);
    void Write_byte(uint8_t reg, uint16_t data);

    void Initialize();
    void CheckFaultStatus();
    void Stop();
};

#endif // DRV8323_H_