/*
 * main_exec.h
 *
 *  Created on: Aug 15th, 2023
 *      Author: Reiji Terunuma
 */

#ifndef MAIN_EXEC_H_
#define MAIN_EXEC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

    void TIMUpdate();
    void UpdateControl();
    void UpdateEncoder();
    void LogPrint();
    void FDCANReceiveCallback();

#ifdef __cplusplus
};
#endif

extern bool initialized;
extern uint32_t cali_count;
extern bool ADC_calibration;
extern bool can_received;
extern uint32_t can_receive_interval;

void LEDUpdate(void);
uint8_t BLMDFDCANReceive(uint32_t *id, uint8_t *buffer);

#endif // MAIN_EXEC_H_