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
#define FS2_OTW 0x01
#define FS2_SC_OC 0x01 << 8
#define FS2_SB_OC 0x01 << 9
#define FS2_SA_OC 0x01 << 10

// Driver Control
#define DRV_CTRL_PWM_MODE_3x 0x01 << 5
#define DRV_CTRL_DIS_CPUV 0x01 << 9
#define DRV_CTRL_CLR_FLT 0x01

// Gate Drive HS
#define GATE_DRV_HS_UNLOCK 0x03 << 8

// Gate Drive LS
#define GATE_DRV_LS_CBC 0x01 << 10
#define GATE_DRV_LS_TDRIVE_500ns 0x00
#define GATE_DRV_LS_TDRIVE_1000ns 0x01 << 8
#define GATE_DRV_LS_TDRIVE_2000ns 0x02 << 8
#define GATE_DRV_LS_TDRIVE_4000ns 0x03 << 8

// Gate Drive IDRIVEP
#define GATE_DRV_IDRIVEP_10mA 0x00
#define GATE_DRV_IDRIVEP_30mA 0x01 << 4
#define GATE_DRV_IDRIVEP_60mA 0x02 << 4
#define GATE_DRV_IDRIVEP_80mA 0x03 << 4
#define GATE_DRV_IDRIVEP_120mA 0x04 << 4
#define GATE_DRV_IDRIVEP_140mA 0x05 << 4
#define GATE_DRV_IDRIVEP_170mA 0x06 << 4
#define GATE_DRV_IDRIVEP_190mA 0x07 << 4
#define GATE_DRV_IDRIVEP_260mA 0x08 << 4
#define GATE_DRV_IDRIVEP_330mA 0x09 << 4
#define GATE_DRV_IDRIVEP_370mA 0x0A << 4
#define GATE_DRV_IDRIVEP_440mA 0x0B << 4
#define GATE_DRV_IDRIVEP_570mA 0x0C << 4
#define GATE_DRV_IDRIVEP_680mA 0x0D << 4
#define GATE_DRV_IDRIVEP_820mA 0x0E << 4
#define GATE_DRV_IDRIVEP_1000mA 0x0F << 4

// Gate Drive IDRIVEN
#define GATE_DRV_IDRIVEN_20mA 0x00
#define GATE_DRV_IDRIVEN_60mA 0x01
#define GATE_DRV_IDRIVEN_120mA 0x02
#define GATE_DRV_IDRIVEN_160mA 0x03
#define GATE_DRV_IDRIVEN_240mA 0x04
#define GATE_DRV_IDRIVEN_280mA 0x05
#define GATE_DRV_IDRIVEN_340mA 0x06
#define GATE_DRV_IDRIVEN_380mA 0x07
#define GATE_DRV_IDRIVEN_520mA 0x08
#define GATE_DRV_IDRIVEN_660mA 0x09
#define GATE_DRV_IDRIVEN_740mA 0x0A
#define GATE_DRV_IDRIVEN_880mA 0x0B
#define GATE_DRV_IDRIVEN_1140mA 0x0C
#define GATE_DRV_IDRIVEN_1360mA 0x0D
#define GATE_DRV_IDRIVEN_1640mA 0x0E
#define GATE_DRV_IDRIVEN_2000mA 0x0F

// OCP Control
#define OCP_CTRL_TRETRY_4ms 0x00 << 10
#define OCP_CTRL_TRETRY_50us 0x01 << 10
#define OCP_CTRL_DEAD_TIME_50ns 0x00
#define OCP_CTRL_DEAD_TIME_100ns 0x01 << 8
#define OCP_CTRL_DEAD_TIME_200ns 0x10 << 8
#define OCP_CTRL_DEAD_TIME_400ns 0x11 << 8
#define OCP_CTRL_OCP_MODE_LATCH 0x00
#define OCP_CTRL_OCP_MODE_RETRY 0x01 << 6
#define OCP_CTRL_OCP_MODE_REPORT 0x02 << 6
#define OCP_CTRL_OCP_MODE_NO_ACTION 0x03 << 6
#define OCP_CTRL_OCP_DEG_2us 0x00
#define OCP_CTRL_OCP_DEG_4us 0x01 << 4
#define OCP_CTRL_OCP_DEG_6us 0x02 << 4
#define OCP_CTRL_OCP_DEG_8us 0x03 << 4
#define OCP_CTRL_VDS_LVL_0_06V 0x00
#define OCP_CTRL_VDS_LVL_0_13V 0x01
#define OCP_CTRL_VDS_LVL_0_20V 0x02
#define OCP_CTRL_VDS_LVL_0_26V 0x03
#define OCP_CTRL_VDS_LVL_0_31V 0x04
#define OCP_CTRL_VDS_LVL_0_45V 0x05
#define OCP_CTRL_VDS_LVL_0_53V 0x06
#define OCP_CTRL_VDS_LVL_0_60V 0x07
#define OCP_CTRL_VDS_LVL_0_68V 0x08
#define OCP_CTRL_VDS_LVL_0_75V 0x09
#define OCP_CTRL_VDS_LVL_0_94V 0x0A
#define OCP_CTRL_VDS_LVL_1_13V 0x0B
#define OCP_CTRL_VDS_LVL_1_30V 0x0C
#define OCP_CTRL_VDS_LVL_1_50V 0x0D
#define OCP_CTRL_VDS_LVL_1_70V 0x0E
#define OCP_CTRL_VDS_LVL_1_88V 0x0F

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
    void ClearFaultStatus();
    void SetPhaseState(PhaseState sa, PhaseState sb, PhaseState sc);
    void Enable();
    void Disable();
};

#endif // DRV8323_H_