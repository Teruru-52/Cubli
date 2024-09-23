/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file   fatfs.c
 * @brief  Code for fatfs applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSD;  /* Return value for SD */
char SDPath[4]; /* SD logical drive path */
FATFS SDFatFS;  /* File system object for SD logical drive */
FIL SDFile;     /* File object for SD */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
 * @brief  Gets Time from RTC
 * @param  None
 * @retval Time in DWORD
 */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
void SDIO_Test()
{
  FRESULT res;                                              /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                         /* File write/read counts */
  uint8_t wtext[] = "Hi, this is STM32 working with FatFs"; /* File write buffer */
  uint8_t rtext[_MAX_SS];                                   /* File read buffer */

  /*##-1- Link the SD disk I/O driver ########################################*/
  if (retSD == 0)
  {
    /*##-2- Register the file system object to the FatFs module ##############*/
    if (f_mount(&SDFatFS, (TCHAR const *)SDPath, 0) != FR_OK)
    {
      Error_Handler();
    }
    else
    {
      /*##-3- Create a FAT file system (format) on the logical drive #########*/
      FRESULT fr = f_mkfs((TCHAR const *)SDPath, FM_ANY, 0, rtext, sizeof(rtext));
      if (fr != FR_OK)
      {
        for (int i = 0; i < 20; i++)
        {
          if (fr == i)
          {
            printf("FRESULT: %d\n", i); // see ff.h for FRESULT values
            break;
          }
        }
        Write_GPIO(LED_RED, GPIO_PIN_SET); // Error is here
        Error_Handler();
      }
      else
      {
        /*##-4- Create and Open a new text file object with write access #####*/
        if (f_open(&SDFile, "STM32.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
        {
          Error_Handler();
        }
        else
        {
          /*##-5- Write data to the text file ################################*/
          res = f_write(&SDFile, wtext, sizeof(wtext), (void *)&byteswritten);

          if ((byteswritten == 0) || (res != FR_OK))
          {
            Error_Handler();
          }
          else
          {
            /*##-6- Close the open text file #################################*/
            f_close(&SDFile);
            /*##-7- Open the text file object with read access ###############*/
            if (f_open(&SDFile, "STM32.txt", FA_READ) != FR_OK)
            {
              Error_Handler();
            }
            else
            {
              /*##-8- Read data from the text file ###########################*/
              res = f_read(&SDFile, rtext, sizeof(rtext), (UINT *)&bytesread);

              if ((bytesread == 0) || (res != FR_OK)) /* EOF or Error */
              {
                Error_Handler();
              }
              else
              {
                /*##-9- Close the open text file #############################*/
                f_close(&SDFile);
                /*##-10- Compare read data with the expected data ############*/
                if ((bytesread != byteswritten))
                {
                  /* Read data is different from the expected data */
                  Error_Handler();
                }
              }
            }
          }
        }
      }
    }
  }

  /*##-11- Unlink the SD disk I/O driver ####################################*/
  FATFS_UnLinkDriver(SDPath);
}
/* USER CODE END Application */
