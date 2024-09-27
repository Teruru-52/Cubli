/*
 * main_exec.cpp
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#define FDCAN_RECEIVE_INTERVAL_TIMEOUT_VALUE 1000

#include "main_exec.h"
#include "instance.h"
#include "drivers/driver_controller.h"
#include "fdcan.h"
#include "SEGGER_RTT.h"

using namespace protocol;
enum class LEDState
{
    not_inited,
    inited,
    stop
};
LEDState led_state = LEDState::not_inited;
bool can_received = false;
uint32_t can_receive_interval = 0;
uint8_t TxData[8];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    hall.HandleCallback(GPIO_Pin);
}

void InitializeDriver()
{
    driver_controller.torque_controller = TorqueControlType::foc_current;
    // driver_controller.foc_modulation = FOCModulationType::SpaceVectorPWM;
    driver_controller.foc_modulation = FOCModulationType::Trapezoid_120;
    // driver_controller.controller = MotionControlType::torque;
    driver_controller.controller = MotionControlType::velocity;
    driver_controller.Initialize();
    led_state = LEDState::inited;
}

void TimUpdate()
{
    LEDUpdate();
    encoder.Update();

    can_receive_interval++;
    if (can_receive_interval > FDCAN_RECEIVE_INTERVAL_TIMEOUT_VALUE)
    {
        led_state = LEDState::stop;
        can_received = false;
    }
    else
    {
        if (driver_controller.GetInitilizationFlag() == false)
            return;
        else
        {
            TxData[0] = 3;
            FDCAN_Send(TxData);
        }
    }
}

void AdcCpltCallback()
{
    // Write_GPIO(LED_RED, GPIO_PIN_RESET);
    if (!can_received)
        driver_controller.Stop();
    else
    {
        driver_controller.Update();
        driver_controller.SetPwm();
    }
}

void FDCANReceiveCallback(uint8_t *pRxData)
{
    can_received = true;
    // switch (led_state)
    // {
    // case LEDState::not_inited:
    // {
    //     MessageHeader header = static_cast<MessageHeader>(pRxData[0]);
    //     if (header == MessageHeader::init)
    //     {
    //         driver_controller.Reset();
    //         driver_controller.Initialize();
    //         led_state = LEDState::inited;
    //     }
    //     return;
    // }
    // case LEDState::inited:
    // case LEDState::stop:
    // {
    //     MessageHeader header = static_cast<MessageHeader>(pRxData[0]);
    //     if (header == MessageHeader::tx)
    //     {
    //         float ref = static_cast<float>(pRxData[1]); // to do
    //         driver_controller.UpdateTarget(ref);
    //         led_state = LEDState::inited;
    //     }
    //     else if (header == MessageHeader::stop)
    //     {
    //         driver_controller.Stop();
    //         led_state = LEDState::stop;
    //     }
    //     else if (header == MessageHeader::free)
    //     {
    //         driver_controller.Free();
    //         led_state = LEDState::stop;
    //     }
    //     else
    //         driver_controller.Stop();
    // }
    // }
}

void PrintLog()
{
    // encoder.PrintLog();
    // hall.PrintLog();
    driver_controller.PrintLog();
}

void LEDUpdate()
{
    switch (led_state)
    {
    case LEDState::not_inited:
        Write_GPIO(LED_YELLOW, GPIO_PIN_SET);
        Write_GPIO(LED_BLUE, GPIO_PIN_RESET);
        break;
    case LEDState::inited:
        Write_GPIO(LED_YELLOW, GPIO_PIN_RESET);
        Write_GPIO(LED_BLUE, GPIO_PIN_SET);
        break;
    case LEDState::stop:
        Write_GPIO(LED_YELLOW, GPIO_PIN_RESET);
        Write_GPIO(LED_BLUE, GPIO_PIN_RESET);
        break;
    }
}