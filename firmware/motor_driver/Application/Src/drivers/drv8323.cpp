/*
 * drv8323.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "drivers/drv8323.h"

DRV8323::DRV8323(SPI_HandleTypeDef *hspi, GPIO_Value SPI_CS_DRV, GPIO_Value nFault, GPIO_Value DRV_ENABLE,
                 GPIO_Value DRV_CAL, GPIO_Value INLA, GPIO_Value INLB, GPIO_Value INLC)
    : hspi(hspi),
      SPI_CS_DRV(SPI_CS_DRV),
      nFault(nFault),
      DRV_ENABLE(DRV_ENABLE),
      DRV_CAL(DRV_CAL),
      INLA(INLA),
      INLB(INLB),
      INLC(INLC)
{
}

uint16_t DRV8323::ReadByte(uint8_t reg)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = (reg << 3) | 0x80; // MSB
    tx_data[1] = 0x00;              // LSB
    // printf("[LOG] read tx_data = %x\n", ((uint16_t)(tx_data[0] << 8) | tx_data[1]));

    ResetChipSelect();
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, 1);
    SetChipSelect();

    uint16_t data = ((rx_data[0] << 8) | rx_data[1]) & 0x7FF;
    // printf("[LOG] read data = %x\n", data);
    return data;
}

void DRV8323::WriteByte(uint8_t reg, uint16_t data)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];
    // printf("[LOG] write data = %x\n", data);

    tx_data[0] = (reg << 3) | (data >> 8); // MSB
    tx_data[1] = data & 0xFF;              // LSB
    // printf("[LOG] write tx_data = %x\n", ((uint16_t)(tx_data[0] << 8) | tx_data[1]));

    ResetChipSelect();
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, 1);
    SetChipSelect();
}

void DRV8323::Initialize()
{
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);
    Write_GPIO(DRV_ENABLE, GPIO_PIN_SET); // Set gate driver enable
    HAL_Delay(2);

    ClearFaultStatus(); // clear fault status
    HAL_Delay(1);
    SetDriverControl(); // set driver control register
    HAL_Delay(1);
    SetGateDriveHS(); // set gate drive HS register
    HAL_Delay(1);
    SetGateDriveLS(); // set gate drive LS register
    HAL_Delay(1);
    SetOCPControl(); // set OCP control register
    HAL_Delay(1);
    SetCSAControl(); // set CSA control register
    HAL_Delay(1);

    CheckFaultStatus();
    HAL_Delay(1);

    // check register value
    // printf("[LOG] driver control register = %x\n", ReadByte(DRIVER_CONTROL));
    // HAL_Delay(1);
    // printf("[LOG] gate drive HS register = %x\n", ReadByte(GATE_DRIVE_HS));
    // HAL_Delay(1);
    // printf("[LOG] gate drive LS register = %x\n", ReadByte(GATE_DRIVE_LS));
    // HAL_Delay(1);
    // printf("[LOG] OCP control register = %x\n", ReadByte(OCP_CONTROL));
    // HAL_Delay(1);
    // printf("[LOG] CSA control register = %x\n", ReadByte(CSA_CONTROL));
    // HAL_Delay(1);
}

void DRV8323::SetDriverControl()
{
    uint16_t reg = DRV_CTRL_PWM_MODE_3x;
    // reg |= DRV_CTRL_DIS_CPUV;
    WriteByte(DRIVER_CONTROL, reg);
}

void DRV8323::SetGateDriveHS()
{
    uint16_t reg = GATE_DRV_HS_UNLOCK;
    // reg |= GATE_DRV_IDRIVEP_1000mA;
    // reg |= GATE_DRV_IDRIVEN_2000mA;
    reg |= GATE_DRV_IDRIVEP_30mA;
    reg |= GATE_DRV_IDRIVEN_60mA;
    WriteByte(GATE_DRIVE_HS, reg);
}

void DRV8323::SetGateDriveLS()
{
    uint16_t reg = GATE_DRV_LS_CBC;
    reg |= GATE_DRV_LS_TDRIVE_4000ns;
    // reg |= GATE_DRV_LS_TDRIVE_2000ns; // Gate drive fault (GDF) error
    // reg |= GATE_DRV_IDRIVEP_1000mA;
    // reg |= GATE_DRV_IDRIVEN_2000mA;
    reg |= GATE_DRV_IDRIVEP_60mA;
    reg |= GATE_DRV_IDRIVEN_120mA;
    WriteByte(GATE_DRIVE_LS, reg);
}

void DRV8323::SetOCPControl()
{
    uint16_t reg = OCP_CTRL_TRETRY_4ms;
    // uint16_t reg = OCP_CTRL_TRETRY_50us;
    reg |= OCP_CTRL_DEAD_TIME_200ns;
    reg |= OCP_CTRL_OCP_MODE_RETRY;
    // reg |= OCP_CTRL_OCP_MODE_REPORT;
    // reg |= OCP_CTRL_OCP_MODE_NO_ACTION;
    // reg |= OCP_CTRL_OCP_DEG_4us;
    reg |= OCP_CTRL_OCP_DEG_6us;
    // reg |= OCP_CTRL_VDS_LVL_0_31V;
    reg |= OCP_CTRL_VDS_LVL_0_75V;
    WriteByte(OCP_CONTROL, reg);
}

void DRV8323::SetCSAControl()
{
    uint16_t reg = CSA_CTRL_VREF_DIV;
    reg |= CSA_CTRL_CSA_GAIN_40VV;
    reg |= CSA_CTRL_SEN_LVL_1V;
    WriteByte(CSA_CONTROL, reg);
}

int DRV8323::SetCurrentoffsets()
{
    uvw_t current_uvw;
    uvw_t current_sum;

    Disable();
    Write_GPIO(DRV_CAL, GPIO_PIN_SET); // Perform auto offset calbration (amplifier)
    HAL_Delay(1);

    for (int j = 0; j < calibration_rounds; j++)
    {
        current_uvw = GetPhaseCurrents();
        current_sum.u += current_uvw.u;
        current_sum.v += current_uvw.v;
        current_sum.w += current_uvw.w;
    }

    current_offset.u = current_sum.u / static_cast<float>(calibration_rounds);
    current_offset.v = current_sum.v / static_cast<float>(calibration_rounds);
    current_offset.w = current_sum.w / static_cast<float>(calibration_rounds);
    printf("[LOG] current_offset.u = %.3f, current_offset.v = %.3f, current_offset.w = %.3f\n", current_offset.u, current_offset.v, current_offset.w);

    Write_GPIO(DRV_CAL, GPIO_PIN_RESET); // finish calibration
    return 1;
}

uvw_t DRV8323::GetPhaseCurrents()
{
    float Vsox[phase_num] = {0.0, 0.0, 0.0}; // [V]
    uvw_t current_uvw;
    uint32_t adc_data[phase_num] = {0, 0, 0};
    GetAdcValue(adc_data); // get digital current Data

    for (int i = 0; i < phase_num; i++)
        Vsox[i] = static_cast<float>(adc_data[i]) * Vref / static_cast<float>(adc_resolution);
    current_uvw.u = (Vref * 0.5f - Vsox[2]) / (Gcsa * Rsense) - current_offset.u;
    current_uvw.v = (Vref * 0.5f - Vsox[1]) / (Gcsa * Rsense) - current_offset.v;
    current_uvw.w = (Vref * 0.5f - Vsox[0]) / (Gcsa * Rsense) - current_offset.w;

    // current_uvw.u = (Vref * 0.5f - Vsox[2]) / (Gcsa * Rsense);
    // current_uvw.v = (Vref * 0.5f - Vsox[1]) / (Gcsa * Rsense);
    // current_uvw.w = (Vref * 0.5f - Vsox[0]) / (Gcsa * Rsense);
    // printf("%.3f, %.3f, %.3f\n", Vsox[0], Vsox[1], Vsox[2]);

    return current_uvw;
}

void DRV8323::CheckFaultStatus()
{
    fault_status1 = ReadByte(FAULT_STATUS_1); // Fault Status Register1
    fault_status2 = ReadByte(FAULT_STATUS_2); // Fault Status Register2

    if (fault_status1 & FS1_VDS_OCP)
        printf("[ERR] VDS_OCP: VDS overcurrent protection\n");
    if (fault_status1 & FS1_GDF)
        printf("[ERR] GDF: Gate drive fault condition\n");
    if (fault_status1 & FS1_UVLO)
        printf("[ERR] UVLO: VM undervoltage lockout condition\n");
    if (fault_status1 & FS1_OTSD)
        printf("[ERR] OTSD: Overtemperature shutdown\n");
    if (fault_status1 & FS1_VDS_HA)
        printf("[ERR] VDS_HA: VDS overcurrent fault on HA\n");
    if (fault_status1 & FS1_VDS_LA)
        printf("[ERR] VDS_LA: VDS overcurrent fault on LA\n");
    if (fault_status1 & FS1_VDS_HB)
        printf("[ERR] VDS_HB: VDS overcurrent fault on HB\n");
    if (fault_status1 & FS1_VDS_LB)
        printf("[ERR] VDS_LB: VDS overcurrent fault on LB\n");
    if (fault_status1 & FS1_VDS_HC)
        printf("[ERR] VDS_HC: VDS overcurrent fault on HC\n");
    if (fault_status1 & FS1_VDS_LC)
        printf("[ERR] VDS_LC: VDS overcurrent fault on LC\n");
    if (fault_status2 & FS2_SA_OC)
        printf("[ERR] SA_OC: Overcurrent on phase A sense amp\n");
    if (fault_status2 & FS2_SB_OC)
        printf("[ERR] SB_OC: Overcurrent on phase B sense amp\n");
    if (fault_status2 & FS2_SC_OC)
        printf("[ERR] SC_OC: Overcurrent on phase C sense amp\n");
    if (fault_status2 & FS2_OTW)
        printf("[ERR] OTW: Overtemperature warning\n");
    if (fault_status2 & FS2_CPUV)
        printf("[ERR] CPUV: Charge pump undervoltage fault condition\n");
    if (fault_status2 & FS2_VGS_HA)
        printf("[ERR] VGS_HA: Gate drive fault on HA\n");
    if (fault_status2 & FS2_VGS_LA)
        printf("[ERR] VGS_LA: Gate drive fault on LA\n");
    if (fault_status2 & FS2_VGS_HB)
        printf("[ERR] VGS_HB: Gate drive fault on HB\n");
    if (fault_status2 & FS2_VGS_LB)
        printf("[ERR] VGS_LB: Gate drive fault on LB\n");
    if (fault_status2 & FS2_VGS_HC)
        printf("[ERR] VGS_HC: Gate drive fault on HC\n");
    if (fault_status2 & FS2_VGS_LC)
        printf("[ERR] VGS_LC: Gate drive fault on LC\n");

    if (!Read_GPIO(nFault)) // if (fault_status1 & FS1_FAULT)
        Write_GPIO(LED_RED, GPIO_PIN_SET);
}

void DRV8323::ClearFaultStatus()
{
    uint16_t reg = ReadByte(DRIVER_CONTROL);
    reg |= DRV_CTRL_CLR_FLT;
    WriteByte(DRIVER_CONTROL, reg);
}

void DRV8323::SetPhaseState(PhaseState sa, PhaseState sb, PhaseState sc)
{
    Write_GPIO(INLA, sa == PhaseState::PHASE_ON ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Write_GPIO(INLB, sb == PhaseState::PHASE_ON ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Write_GPIO(INLC, sc == PhaseState::PHASE_ON ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void DRV8323::Enable()
{
    SetPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
}

void DRV8323::Disable()
{
    SetPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
}