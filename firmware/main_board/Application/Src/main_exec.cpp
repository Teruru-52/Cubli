/*
 * main_exec.cpp
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#include "main_exec.h"
#include "can.h"
#include "instance.h"
#include "controller/controller.h"

CAN_TxHeaderTypeDef TxHeader;
// CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
// uint8_t RxData[8];
uint32_t TxMailbox;

Pose input_torque;

void Initialize()
{
    state->Initialize();
}

void UpdateControl()
{
    state->Update();
    controller->CalcInput(state->vec_x);
    input_torque = controller->GetInput();
}

void UpdateIMUs()
{
    // state->UpdateIMUs();
    state->Update();
}

void LogPrint()
{
    state->LogPrint();
}

void CAN_Send()
{
    TxHeader.StdId = CAN_ID_MB;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxData[0] = 1;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        // printf("FreeLevel = %ld\n", HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        {
            Error_Handler();
        }
        MB_Access_Lamp.CAN_TX = ENABLE;
    }
}

void CANReceiveCallback(uint8_t *pRxData)
{
    MB_Access_Lamp.CAN_RX = ENABLE;
}