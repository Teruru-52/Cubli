/*
 * main_exec.cpp
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#define FDCAN_RECEIVE_INTERVAL_TIMEOUT_VALUE 1000

#include "main_exec.h"
#include "instance.h"
#include "driver_controller.h"
#include "fdcan.h"
#include "SEGGER_RTT.h"

bool initialized = false;
uint32_t cali_count = 0;
bool ADC_calibration = true;
bool can_received = false;
uint32_t can_receive_interval = 0;

uint32_t period_count = 0;
bool TIM_rising_edge = false;
bool Update_and_send = false;

using namespace protocol;
enum class LEDState
{
    yet_inited,
    inited,
    stop
};
LEDState led_state = LEDState::yet_inited;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Hall_U_Pin)
        hall.SetHallValueU(Read_GPIO(HALL_U));
    if (GPIO_Pin == Hall_V_Pin)
        hall.SetHallValueV(Read_GPIO(HALL_V));
    if (GPIO_Pin == Hall_W_Pin)
        hall.SetHallValueW(Read_GPIO(HALL_W));
}

void ReadHallSensor()
{
    hall.SetHallValueU(Read_GPIO(HALL_U));
    hall.SetHallValueV(Read_GPIO(HALL_V));
    hall.SetHallValueW(Read_GPIO(HALL_W));
    // uint8_t hall_value = hall.GetHallValue();
    // static int cnt = 0;
    // if (cnt == 0)
    //     printf("hall = %d\n", hall_value);
    // cnt = (cnt + 1) % 10000;
}

void TIMUpdate()
{
    // TIM_rising_edge = !TIM_rising_edge;
    // if (TIM_rising_edge)
    // {
    //     Write_GPIO(LED_YELLOW, GPIO_PIN_SET);
    //     LEDUpdate();
    //     can_receive_interval++;
    //     if (can_receive_interval > FDCAN_RECEIVE_INTERVAL_TIMEOUT_VALUE)
    //     {
    //         led_state = LEDState::yet_inited;
    //         can_received = false;
    //         if (driver_controller != nullptr)
    //             driver_controller->Reset();
    //         driver_controller = nullptr;
    //         initialized = false;
    //     }
    //     if (driver_controller == nullptr || !initialized)
    //         return;
    //     else
    //         period_count++;

    //     // if (period_count % driver_controller->GetPeriod() == 0)
    //     // {
    //     // Update_and_send = true;
    //     // driver_controller->UpdateInTimerCallback(TIM_rising_edge);
    //     // }
    //     // else
    //     // {
    //     //     Update_and_send = false;
    //     // }
    //     Write_GPIO(LED_YELLOW, GPIO_PIN_RESET);
    // }
    // else
    // {
    //     if (driver_controller == nullptr || !initialized)
    //         return;
    //     if (Update_and_send)
    //     {
    //         // driver_controller->UpdateInTimerCallback(TIM_rising_edge);
    //         // driver_controller->SendData();
    //     }
    // }

    BLMD_Access_Lamp.FDCAN_TX = ENABLE;
    TxData[0] = 3;
    FDCAN_Send(TxData);
}

void UpdateControl()
{
    if (ADC_calibration == true)
    {
        driver_controller->SetCurrentoffset();
    }
    if (driver_controller == nullptr)
    {
        PWM_Update(&blcd_pwm, 0.0, 0.0, 0.0);
        return;
    }
    else
    {
        if (!can_received)
            PWM_Update(&blcd_pwm, 0.0, 0.0, 0.0);
        else
        {
            uvw_t input_duty = driver_controller->Control();
            PWM_Update(&blcd_pwm, input_duty.u, input_duty.v, input_duty.w);
        }
    }
}

void UpdateEncoder()
{
    encoder.Update();
}

void LogPrint()
{
    float angle = encoder.GetAngle();
    printf("angle = %.3f\n", angle);

    // uint16_t flag_err = encoder.GetErrFlag();
    // printf("flag_err = %x\n", flag_err);
}

void FDCANReceiveCallback(uint8_t *pRxData)
{
    // Buffer<64> can_buf;
    // uint8_t DataSize = 0;
    // uint32_t ID = 0;
    // uint8_t RxData[64] = {0};
    // DataSize = FDCAN_Receive(&ID, RxData);
    // if (ID != CAN_ID_TX || DataSize == 0)
    // {
    //     return;
    // }
    // BLMD_Access_Lamp.FDCAN_RX = ENABLE;
    // can_receive_interval = 0;
    // can_received = true;
    // for (uint8_t i = 0; i < DataSize; i++)
    //     can_buf.push_back(RxData[i]);

    // switch (led_state)
    // {
    // case LEDState::yet_inited:
    // {
    //     MessageHeader header = static_cast<MessageHeader>(can_buf[0]);
    //     if (header == MessageHeader::init)
    //     {
    //         encoder.Reset();

    //         // ControlMode control_mode = static_cast<ControlMode>(can_buf[1]);
    //         // if (control_mode == ControlMode::voltage)
    //         //     driver_controller = &voltage_driver;
    //         // else if (control_mode == ControlMode::current)
    //         //     driver_controller = &current_driver;
    //         // else if (control_mode == ControlMode::motor_identification)
    //         //     driver_controller = &motor_identification;
    //         // else
    //         //     return;

    //         driver_controller->Initialize(can_buf);
    //         led_state = LEDState::inited;
    //     }
    //     return;
    // }
    // case LEDState::inited:
    // case LEDState::stop:
    // {
    //     MessageHeader header = static_cast<MessageHeader>(can_buf[0]);
    //     if (header == MessageHeader::tx)
    //     {
    //         driver_controller->ReceiveAndUpdateData(can_buf);
    //         led_state = LEDState::inited;
    //     }
    //     else if (header == MessageHeader::stop)
    //     {
    //         PWM_Update(&blcd_pwm, 0.0, 0.0, 0.0);
    //         led_state = LEDState::stop;
    //     }
    //     else if (header == MessageHeader::free)
    //     {
    //         PWM_Stop();
    //         led_state = LEDState::stop;
    //     }
    //     else
    //     {
    //         PWM_Update(&blcd_pwm, 0.0, 0.0, 0.0);
    //         can_buf.pop_front();
    //     }
    // }
    // }

    BLMD_Access_Lamp.FDCAN_RX = ENABLE;
    printf("data=%d\r\n", pRxData[0]);
}

void TestADC()
{
    uint32_t ADC_Data[3];
    ADC_Get_Value(ADC_Data);

    // static int cnt = 0;
    // if (cnt == 0)
    //     printf("%ld, %ld, %ld\n", ADC_Data[0], ADC_Data[1], ADC_Data[2]);
    // cnt = (cnt + 1) % 10000;
}

void LEDUpdate()
{
    switch (led_state)
    {
    case LEDState::yet_inited:
        Write_GPIO(LED_WHITE, GPIO_PIN_SET);
        Write_GPIO(LED_BLUE, GPIO_PIN_RESET);
        break;
    case LEDState::inited:
        Write_GPIO(LED_WHITE, GPIO_PIN_RESET);
        Write_GPIO(LED_BLUE, GPIO_PIN_SET);
        break;
    case LEDState::stop:
        Write_GPIO(LED_WHITE, GPIO_PIN_SET);
        Write_GPIO(LED_BLUE, GPIO_PIN_SET);
        break;
    }
}