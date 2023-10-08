/*
 * main_exec.h
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef MAIN_EXEC_H_
#define MAIN_EXEC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"
    void Initialize();
    void UpdateControl();
    void UpdateIMUs();
    void LogPrint();
    void CAN_Send();
    void CANReceiveCallback(uint8_t *pRxData);

#ifdef __cplusplus
};
#endif

CAN_TxHeaderTypeDef TxHeader;
// CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
// uint8_t RxData[8];
uint32_t TxMailbox;

#endif // MAIN_EXEC_H_