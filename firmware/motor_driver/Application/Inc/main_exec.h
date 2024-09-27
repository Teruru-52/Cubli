/*
 * main_exec.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef MAIN_EXEC_H_
#define MAIN_EXEC_H_

#define FDCAN_RECEIVE_INTERVAL_TIMEOUT_VALUE 1000

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

    void InitializeDriver();
    void TimUpdate();
    void AdcCpltCallback();
    void FDCAN_ReceiveCallback(uint8_t *pRxData);
    void PrintLog();

#ifdef __cplusplus
};
#endif

extern bool can_received;
extern uint32_t can_receive_interval;

extern uint8_t TxData[8];

void LedUpdate(void);

#endif // MAIN_EXEC_H_