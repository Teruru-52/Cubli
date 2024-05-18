#include "SEGGER_RTT.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
  SEGGER_RTT_Write(0, ptr, len);
  return len;
}

// int _write(int file, char *ptr, int len)
// {
//   HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
//   return len;
// }