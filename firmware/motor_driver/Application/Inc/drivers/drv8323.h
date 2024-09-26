/*
 * drv8323.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef DRV8323_H_
#define DRV8323_H_

#include "main.h"
#include "drivers/foc_driver.h"

class DRV8323
{
private:
    SPI_HandleTypeDef *hspi;
    GPIO_Value SPI_CS_DRV;
    GPIO_Value nFault;
    GPIO_Value DRV_ENABLE;
    GPIO_Value DRV_CAL;
    GPIO_Value INLA;
    GPIO_Value INLB;
    GPIO_Value INLC;

    uint16_t fault_status1;
    uint16_t fault_status2;

public:
    DRV8323(SPI_HandleTypeDef *hspi, GPIO_Value SPI_CS_DRV, GPIO_Value nFault, GPIO_Value DRV_ENABLE,
            GPIO_Value DRV_CAL, GPIO_Value INLA, GPIO_Value INLB, GPIO_Value INLC);

    inline void SetChipSelect() { Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET); }
    inline void ResetChipSelect() { Write_GPIO(SPI_CS_DRV, GPIO_PIN_RESET); }
    uint16_t ReadByte(uint8_t reg);
    void WriteByte(uint8_t reg, uint16_t data);

    void Initialize();
    void StartCalibration();
    inline void FinishCalibration() { Write_GPIO(DRV_CAL, GPIO_PIN_RESET); }
    void CheckFaultStatus();
    void SetPhaseState(PhaseState sa, PhaseState sb, PhaseState sc);
    void Align();
    void Free();
};

#endif // DRV8323_H_