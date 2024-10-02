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

#define FAULT_STATUS_1 0x00
#define FAULT_STATUS_2 0x01
#define DRIVER_CONTROL 0x02
#define GATE_DRIVE_HS 0x03
#define GATE_DRIVE_LS 0x04
#define OCP_CONTROL 0x05
#define CSA_CONTROL 0x06

// Fault Status (FS)
#define FS1_VDS_LC 0x01
#define FS1_VDS_HC 0x01 << 1
#define FS1_VDS_LB 0x01 << 2
#define FS1_VDS_HB 0x01 << 3
#define FS1_VDS_LA 0x01 << 4
#define FS1_VDS_HA 0x01 << 5
#define FS1_OTSD 0x01 << 6
#define FS1_UVLO 0x01 << 7
#define FS1_GDF 0x01 << 8
#define FS1_VDS_OCP 0x01 << 9
#define FS1_FAULT 0x01 << 10

#define FS2_VGS_LC 0x01
#define FS2_VGS_HC 0x01 << 1
#define FS2_VGS_LB 0x01 << 2
#define FS2_VGS_HB 0x01 << 3
#define FS2_VGS_LA 0x01 << 4
#define FS2_VGS_HA 0x01 << 5
#define FS2_CPUV 0x01 << 6
#define FS2_OTW 0x01 << 7
#define FS2_SC_OC 0x01 << 8
#define FS2_SB_OC 0x01 << 9
#define FS2_SA_OC 0x01 << 10

// Driver Control
#define DRV_CTRL_PWM_MODE_3x 0x01 << 5
#define DRV_CTRL_DIS_CPUV 0x01 << 9

// Gate Drive HS
#define GATE_DRV_HS_UNLOCK 0x03 << 8
#define GATE_DRV_HS_IDRIVEP_HS_10mA 0x00
#define GATE_DRV_HS_IDRIVEP_HS_1000mA 0x0F << 4
#define GATE_DRV_HS_IDRIVEN_HS_20mA 0x00
#define GATE_DRV_HS_IDRIVEN_HS_2000mA 0x0F

// Gate Drive LS
#define GATE_DRV_LS_CBC 0x01 << 10
#define GATE_DRV_LS_TDRIVE_4000ns 0x03 << 8
#define GATE_DRV_LS_IDRIVEP_LS_10mA 0x00
#define GATE_DRV_LS_IDRIVEP_LS_1000mA 0x0F << 4
#define GATE_DRV_LS_IDRIVEN_LS_20mA 0x00
#define GATE_DRV_LS_IDRIVEN_LS_2000mA 0x0F

// OCP Control
#define OCP_CTRL_TRETRY_50ms 0x01 << 10
#define OCP_CTRL_DEAD_TIME_100ns 0x01 << 8
#define OCP_CTRL_OCP_MODE_RETRY 0x01 << 6
#define OCP_CTRL_OCP_DEG_4us 0x01 << 4
#define OCP_CTRL_VDS_LVL_0_75V 0x09

// CSA Control
#define CSA_CTRL_VREF_DIV 0x01 << 9
#define CSA_CTRL_CSA_GAIN_5VV 0x00 << 6
#define CSA_CTRL_CSA_GAIN_10VV 0x01 << 6
#define CSA_CTRL_CSA_GAIN_20VV 0x02 << 6
#define CSA_CTRL_CSA_GAIN_40VV 0x03 << 6
#define CSA_CTRL_DIS_SEN 0x01 << 5
#define CSA_CTRL_CSA_CAL_A 0x01 << 4
#define CSA_CTRL_CSA_CAL_B 0x01 << 3
#define CSA_CTRL_CSA_CAL_C 0x01 << 2
#define CSA_CTRL_SEN_LVL_0_25V 0x00
#define CSA_CTRL_SEN_LVL_0_5V 0x01
#define CSA_CTRL_SEN_LVL_0_75V 0x02
#define CSA_CTRL_SEN_LVL_1V 0x03

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
    void SetDriverControl();
    void SetGateDriveHS();
    void SetGateDriveLS();
    void SetOCPControl();
    void SetCSAControl();
    int SetCurrentoffsets();
    uvw_t GetPhaseCurrents();
    void CheckFaultStatus();
    void SetPhaseState(PhaseState sa, PhaseState sb, PhaseState sc);
    void Enable();
    void Disable();
};

#endif // DRV8323_H_