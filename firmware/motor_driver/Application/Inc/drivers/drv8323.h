/*
 * drv8323.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef DRV8323_H_
#define DRV8323_H_

#include "main.h"
#include "common/base_classes/foc_driver.h"
#include "common/defaults.h"
#include "common/foc_utils.h"

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

    const float Vref = 3.3f;     // [V]
    const float Rsense = 0.005f; // [Ω]
    // const float Gcsa = 20.0f;
    const float Gcsa = 40.0f;
    const float Gcsa_calib = 40.0f;
    const int phase_num = 3; // U, V, W
    uint32_t adc_data[3] = {0, 0, 0};
    uvw_t current_offset; // [A]

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
    int SetCurrentoffsets();
    uvw_t GetPhaseCurrents();
    void CheckFaultStatus();
    void SetPhaseState(PhaseState sa, PhaseState sb, PhaseState sc);
    void Enable();
    void Disable();
};

#endif // DRV8323_H_