/*
 * drv8323.cpp
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#include "drv8323.h"

#define FAULT_STATUS_1 0x00
#define FAULT_STATUS_2 0x01
#define DRIVER_CONTROL 0x02
#define GATE_DRIVE_HS 0x03
#define GATE_DRIVE_LS 0x04
#define OCP_CONTROL 0x05
#define CSA_CONTROL 0x06

DRV8323::DRV8323(SPI_HandleTypeDef *spi, GPIO_Value SPI_CS_DRV, GPIO_Value nFault, GPIO_Value DRV_ENABLE,
                 GPIO_Value DRV_CAL, GPIO_Value INLA, GPIO_Value INLB, GPIO_Value INLC)
    : spi(spi),
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

    tx_data[0] = (reg << 3) | 0x80;
    tx_data[1] = 0x00; // dummy
    printf("read tx_data = %x\n", ((uint16_t)(tx_data[0] << 8) | tx_data[1]));

    Write_GPIO(SPI_CS_DRV, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);

    uint16_t data = (int16_t)((int16_t)((rx_data[0] & 0x07) << 8) | rx_data[1]);
    // printf("read rx_data[0] = %x\n", rx_data[0] & 0b00000111);
    // printf("read rx_data[1] = %x\n", rx_data[1]);
    printf("read rx_data = %x\n", data);
    // data = ((int16_t)(rx_data[0]) & 0x07) << 8 | (uint16_t)(rx_data[1]);
    // printf("read rx_data = %x\n", data);
    return data;
}

void DRV8323::WriteByte(uint8_t reg, uint16_t data)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    uint8_t upper_data = (data >> 8) & 0x07;
    // printf("upper = %x\n", upper_data);
    uint8_t lower_data = data & 0xFF;
    // printf("lower = %x\n", lower_data);
    tx_data[0] = ((reg << 3) | upper_data) & 0x7F;
    // printf("tx_data[0] = %x\n", tx_data[0]);
    tx_data[1] = lower_data;

    Write_GPIO(SPI_CS_DRV, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);

    printf("write tx_data = %x\n", ((uint16_t)(tx_data[0] << 8) | tx_data[1]));
}

void DRV8323::Initialize()
{
    if (!Read_GPIO(nFault))
        Write_GPIO(LED_RED, GPIO_PIN_SET);
    Write_GPIO(DRV_ENABLE, GPIO_PIN_SET); // Set gate driver enable
    HAL_Delay(1);
    Write_GPIO(SPI_CS_DRV, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(spi); // clockが動かないように、あらかじめEnableにしておく

    HAL_Delay(1);
    // WriteByte(DRIVER_CONTROL, 0x0000); // 6x PWM Mode
    WriteByte(DRIVER_CONTROL, 0x0020); // 3x PWM Mode
    HAL_Delay(1);
    WriteByte(GATE_DRIVE_HS, 0x03FF); // IDRIVEP_HS = 1000mA, IDRIVEN_HS = 2000mA
    HAL_Delay(1);
    WriteByte(GATE_DRIVE_LS, 0x07FF); // TDRIVE = 4000ns, IDRIVEP_LS = 1000mA, IDRIVEN_LS = 2000mA
    HAL_Delay(1);
    WriteByte(OCP_CONTROL, 0x0159); // TRETRY = 4ms, DEAD_TIME = 100ns, OCP_MODE = automatic retrying fault, OCP_DEG = 4us, VDS_LVL = 0.75V
    HAL_Delay(1);
    // WriteByte(CSA_CONTROL, 0x0283); // VREF_DIV = bidirectional mode, CSA_GAIN = 20, SEN_LVL = 1V>
    WriteByte(CSA_CONTROL, 0x02C3); // VREF_DIV = bidirectional mode, CSA_GAIN = 40, SEN_LVL = 1V>
    // WriteByte(CSA_CONTROL, 0x0683); // CSA_FET = 1, VREF_DIV = bidirectional mode, CSA_GAIN = 20, SEN_LVL = 1V

    // check register value
    HAL_Delay(1);
    test[0] = ReadByte(DRIVER_CONTROL);
    HAL_Delay(1);
    test[1] = ReadByte(GATE_DRIVE_HS);
    HAL_Delay(1);
    test[2] = ReadByte(GATE_DRIVE_LS);
    HAL_Delay(1);
    test[3] = ReadByte(OCP_CONTROL);
    HAL_Delay(1);
    test[4] = ReadByte(CSA_CONTROL);
}

void DRV8323::StartCalibration()
{
    Stop();
    Write_GPIO(DRV_CAL, GPIO_PIN_SET); // Perform auto offset calbration (amplifier)
    HAL_Delay(1);
}

void DRV8323::CheckFaultStatus()
{
    fault_status1 = ReadByte(FAULT_STATUS_1) & 0x07FF; // Fault Status Register1
    fault_status2 = ReadByte(FAULT_STATUS_2) & 0x07FF; // Fault Status Register2

    if (!Read_GPIO(nFault))
        Write_GPIO(LED_RED, GPIO_PIN_SET);
}

void DRV8323::Stop()
{
    ResetINLA();
    ResetINLB();
    ResetINLC();
}