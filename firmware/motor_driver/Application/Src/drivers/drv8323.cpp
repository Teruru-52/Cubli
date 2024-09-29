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
    // printf("read tx_data = %x\n", ((uint16_t)(tx_data[0] << 8) | tx_data[1]));

    ResetChipSelect();
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, 1);
    SetChipSelect();

    uint16_t data = ((rx_data[0] << 8) | rx_data[1]) & 0x7FF;
    // printf("read data = %x\n", data);
    return data;
}

void DRV8323::WriteByte(uint8_t reg, uint16_t data)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];
    // printf("write data = %x\n", data);

    tx_data[0] = (reg << 3) | (data >> 8); // MSB
    tx_data[1] = data & 0xFF;              // LSB
    // printf("write tx_data = %x\n", ((uint16_t)(tx_data[0] << 8) | tx_data[1]));

    ResetChipSelect();
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, 1);
    SetChipSelect();
}

void DRV8323::Initialize()
{
    if (!Read_GPIO(nFault))
        Write_GPIO(LED_RED, GPIO_PIN_SET);
    Write_GPIO(DRV_ENABLE, GPIO_PIN_SET); // Set gate driver enable
    HAL_Delay(1);
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(hspi); // clockが動かないように、あらかじめEnableにしておく

    HAL_Delay(1);
    WriteByte(DRIVER_CONTROL, 0x0220); // 3x PWM Mode, DIS_CPUV = 1
    // WriteByte(DRIVER_CONTROL, 0x0020); // 3x PWM Mode
    HAL_Delay(1);
    WriteByte(GATE_DRIVE_HS, 0x3FF); // IDRIVEP_HS = 1000mA, IDRIVEN_HS = 2000mA
    HAL_Delay(1);
    WriteByte(GATE_DRIVE_LS, 0x7FF); // TDRIVE = 4000ns, IDRIVEP_LS = 1000mA, IDRIVEN_LS = 2000mA
    HAL_Delay(1);
    WriteByte(OCP_CONTROL, 0x159); // TRETRY = 4ms, DEAD_TIME = 100ns, OCP_MODE = automatic retrying fault, OCP_DEG = 4us, VDS_LVL = 0.75V
    HAL_Delay(1);
    // WriteByte(CSA_CONTROL, 0x283); // VREF_DIV = bidirectional mode, CSA_GAIN = 20, SEN_LVL = 1V
    WriteByte(CSA_CONTROL, 0x2C3); // VREF_DIV = bidirectional mode, CSA_GAIN = 40, SEN_LVL = 1V
    HAL_Delay(1);

    CheckFaultStatus();
    HAL_Delay(1);

    // check register value
    // ReadByte(DRIVER_CONTROL);
    // HAL_Delay(1);
    // ReadByte(GATE_DRIVE_HS);
    // HAL_Delay(1);
    // ReadByte(GATE_DRIVE_LS);
    // HAL_Delay(1);
    // ReadByte(OCP_CONTROL);
    // HAL_Delay(1);
    // ReadByte(CSA_CONTROL);
    // HAL_Delay(1);
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
    printf("current_offset.u = %.3f, current_offset.v = %.3f, current_offset.w = %.3f\n", current_offset.u, current_offset.v, current_offset.w);

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
        printf("VDS Over Current Protection Error!\n");
    if (fault_status1 & FS1_GDF)
        printf("Gate Drive Fault error\n");
    if (fault_status1 & FS1_UVLO)
        printf("UVLO error\n");
    if (fault_status1 & FS1_OTSD)
        printf("Overtemp shutdown error\n");
    if (fault_status1 & FS1_VDS_HA)
        printf("VDS HA error\n");
    if (fault_status1 & FS1_VDS_LA)
        printf("VDS LA error\n");
    if (fault_status1 & FS1_VDS_HB)
        printf("VDS HB error\n");
    if (fault_status1 & FS1_VDS_LB)
        printf("VDS LB error\n");
    if (fault_status1 & FS1_VDS_HC)
        printf("VDS HC error\n");
    if (fault_status1 & FS1_VDS_LC)
        printf("VDS LC error\n");
    if (fault_status2 & FS2_SA_OC)
        printf("overcurrent on phase A\n");
    if (fault_status2 & FS2_SB_OC)
        printf("overcurrent on phase B\n");
    if (fault_status2 & FS2_SC_OC)
        printf("overcurrent on phase C\n");
    if (fault_status2 & FS2_OTW)
        printf("overtemp warning\n");
    if (fault_status2 & FS2_CPUV)
        printf("Charge pump undervoltage fault condition\n");
    if (fault_status2 & FS2_VGS_HA)
        printf("VGS HA error\n");
    if (fault_status2 & FS2_VGS_LA)
        printf("VGS LA error\n");
    if (fault_status2 & FS2_VGS_HB)
        printf("VGS HB error\n");
    if (fault_status2 & FS2_VGS_LB)
        printf("VGS LB error\n");
    if (fault_status2 & FS2_VGS_HC)
        printf("VGS HC error\n");
    if (fault_status2 & FS2_VGS_LC)
        printf("VDS LC error\n");

    if (!Read_GPIO(nFault)) // if (fault_status1 & FS1_FAULT)
        Write_GPIO(LED_RED, GPIO_PIN_SET);
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