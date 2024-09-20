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

#endif // MAIN_EXEC_H_