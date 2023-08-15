/*
 * main_exec.cpp
 *
 *  Created on: Aug 13th, 2023
 *      Author: Reiji Terunuma
 */

#include "main_exec.h"
#include "instance.h"
#include "controller/controller.h"

void UpdateControl()
{
    state->Update();
    controller->CalcInput(state->vec_x);
}