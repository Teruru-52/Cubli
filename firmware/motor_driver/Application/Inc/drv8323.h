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
    GPIO_Value INLA;
    GPIO_Value INLB;
    GPIO_Value INLC;

    uint16_t fault_status1;
    uint16_t fault_status2;
    uint16_t test[5];

public:
    DRV8323(SPI_HandleTypeDef *spi, GPIO_Value SPI_CS_DRV, GPIO_Value nFault, GPIO_Value DRV_ENABLE,
            GPIO_Value DRV_CAL, GPIO_Value INLA, GPIO_Value INLB, GPIO_Value INLC);

    uint16_t ReadByte(uint8_t reg);
    void WriteByte(uint8_t reg, uint16_t data);

    void Initialize();
    void StartCalibration();
    void FinishCalibration() { Write_GPIO(DRV_CAL, GPIO_PIN_RESET); }
    void SetINLA() { Write_GPIO(INLA, GPIO_PIN_SET); }
    void SetINLB() { Write_GPIO(INLB, GPIO_PIN_SET); }
    void SetINLC() { Write_GPIO(INLC, GPIO_PIN_SET); }
    void ResetINLA() { Write_GPIO(INLA, GPIO_PIN_RESET); }
    void ResetINLB() { Write_GPIO(INLB, GPIO_PIN_RESET); }
    void ResetINLC() { Write_GPIO(INLC, GPIO_PIN_RESET); }
    void CheckFaultStatus();
    void Stop();
};

#endif // DRV8323_H_