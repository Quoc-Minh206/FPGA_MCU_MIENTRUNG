/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Printf(const char *format, ...);
const char* fresult_to_string(FRESULT fr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}

const char* fresult_to_string(FRESULT fr)
{
    switch(fr) {
        case FR_OK:                  return "FR_OK (Thanh cong)";
        case FR_DISK_ERR:            return "FR_DISK_ERR (Loi doc/ghi the nho - SPI hardware error)";
        case FR_INT_ERR:             return "FR_INT_ERR (Loi noi bo FatFS)";
        case FR_NOT_READY:           return "FR_NOT_READY (The SD chua san sang hoac khong nhan the)";
        case FR_NO_FILE:             return "FR_NO_FILE (Khong tim thay tap tin)";
        case FR_NO_PATH:             return "FR_NO_PATH (Khong tim thay duong dan)";
        case FR_INVALID_NAME:        return "FR_INVALID_NAME (Ten file khong hop le)";
        case FR_DENIED:              return "FR_DENIED (Quyen truy cap bi tu choi)";
        case FR_EXIST:               return "FR_EXIST (File da ton tai)";
        case FR_INVALID_OBJECT:      return "FR_INVALID_OBJECT (Doi tuong khong hop le)";
        case FR_WRITE_PROTECTED:     return "FR_WRITE_PROTECTED (The bi khoa che do ghi)";
        case FR_INVALID_DRIVE:       return "FR_INVALID_DRIVE (O dia khong hop le)";
        case FR_NOT_ENABLED:         return "FR_NOT_ENABLED (Chua mount volume)";
        case FR_NO_FILESYSTEM:       return "FR_NO_FILESYSTEM (Khong co phan vung FAT/Can format the FAT32)";
        case FR_MKFS_ABORTED:        return "FR_MKFS_ABORTED (Tao he thong tap tin that bai)";
        case FR_TIMEOUT:             return "FR_TIMEOUT (Het thoi gian cho)";
        case FR_LOCKED:              return "FR_LOCKED (File bi khoa boi tien trinh khac)";
        case FR_NOT_ENOUGH_CORE:     return "FR_NOT_ENOUGH_CORE (Khong du RAM)";
        case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES (Qua nhieu file mo cung luc)";
        case FR_INVALID_PARAMETER:   return "FR_INVALID_PARAMETER (Tham so khong hop le)";
        default:                     return "UNKNOWN_ERROR";
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  UART_Printf("\r\n\r\n==================================================\r\n");
  UART_Printf("      STM32F103C8T6 - TEST MODULE MICROSD CARD     \r\n");
  UART_Printf("==================================================\r\n");
  UART_Printf("[INFO] Ket noi SPI: PA4(CS), PA5(SCK), PA6(MISO), PA7(MOSI)\r\n");
  UART_Printf("[INFO] Debug UART:  PA9(TX) @ 115200 bps\r\n");
  UART_Printf("[INFO] Onboard LED: PC13\r\n");
  UART_Printf("[INFO] Dang cho on dinh nguon the SD...\r\n");

  HAL_Delay(500);

  FATFS myFatFS;
  FIL myFile;
  FRESULT fresult;
  UINT bytesWritten = 0, bytesRead = 0;
  char writeBuffer[256];
  char readBuffer[256];
  uint8_t testSuccess = 0;

  /* 1. Mount the nho */
  UART_Printf("\r\n[STEP 1] Dang Mount the nho (f_mount)... \r\n");
  fresult = f_mount(&myFatFS, "", 1);
  UART_Printf("-> Ket qua: [%d] %s\r\n", fresult, fresult_to_string(fresult));

  if (fresult == FR_OK)
  {
      /* 2. Mo / Tao file test.txt */
      UART_Printf("\r\n[STEP 2] Dang mo/tao file 'test.txt' (f_open)... \r\n");
      fresult = f_open(&myFile, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
      UART_Printf("-> Ket qua: [%d] %s\r\n", fresult, fresult_to_string(fresult));

      if (fresult == FR_OK)
      {
          /* 3. Chuan bi du lieu va ghi vao file */
          snprintf(writeBuffer, sizeof(writeBuffer),
                   "STM32F103 ghi the SD thanh cong!\r\n"
                   "Thoi gian khoi dong: %lu ms\r\n"
                   "Kiem tra: SPI MicroSD Adapter hoat dong hoan hao.\r\n",
                   HAL_GetTick());

          UART_Printf("\r\n[STEP 3] Dang ghi du lieu vao file (f_write)... \r\n");
          UART_Printf("-> Noi dung ghi:\r\n%s", writeBuffer);
          fresult = f_write(&myFile, writeBuffer, strlen(writeBuffer), &bytesWritten);
          UART_Printf("-> Ket qua: [%d] %s (Da ghi: %u bytes)\r\n", fresult, fresult_to_string(fresult), bytesWritten);

          /* 4. Dong file de luu du lieu vao the */
          UART_Printf("\r\n[STEP 4] Dang dong file (f_close)... \r\n");
          fresult = f_close(&myFile);
          UART_Printf("-> Ket qua: [%d] %s\r\n", fresult, fresult_to_string(fresult));

          /* 5. Mo lai file o che do READ de kiem chung */
          UART_Printf("\r\n[STEP 5] Mo lai file 'test.txt' de doc kiem chung... \r\n");
          fresult = f_open(&myFile, "test.txt", FA_READ);
          if (fresult == FR_OK)
          {
              memset(readBuffer, 0, sizeof(readBuffer));
              fresult = f_read(&myFile, readBuffer, sizeof(readBuffer) - 1, &bytesRead);
              f_close(&myFile);

              UART_Printf("-> Doc ve %u bytes. Noi dung thuc te tren the:\r\n", bytesRead);
              UART_Printf("--------------------------------------------------\r\n");
              UART_Printf("%s", readBuffer);
              UART_Printf("--------------------------------------------------\r\n");

              if (bytesRead > 0 && strcmp(writeBuffer, readBuffer) == 0)
              {
                  testSuccess = 1;
                  UART_Printf(">>> XAC NHAN: Du lieu doc va ghi TRUNG KHOP 100%%! <<<\r\n");
              }
              else
              {
                  UART_Printf(">>> CANH BAO: Du lieu doc ve khac voi du lieu da ghi! <<<\r\n");
              }
          }
          else
          {
              UART_Printf("-> Khong the mo lai file de doc: [%d] %s\r\n", fresult, fresult_to_string(fresult));
          }
      }

      /* 6. Unmount */
      UART_Printf("\r\n[STEP 6] Unmount the nho... \r\n");
      fresult = f_mount(NULL, "", 1);
      UART_Printf("-> Ket qua: [%d] %s\r\n", fresult, fresult_to_string(fresult));
  }

  if (testSuccess)
  {
      UART_Printf("\r\n==================================================\r\n");
      UART_Printf("     KET QUA: TEST THE SD THANH CONG 100%%!        \r\n");
      UART_Printf("==================================================\r\n");
      UART_Printf("LED PC13 nhap nhay cham (500ms) bao hieu he thong OK.\r\n");
  }
  else
  {
      UART_Printf("\r\n==================================================\r\n");
      UART_Printf("            KET QUA: TEST CHUA DAT!               \r\n");
      UART_Printf("==================================================\r\n");
      UART_Printf("Huong dan khac phuc nhanh:\r\n");
      UART_Printf("1. Kiem tra so do chan:\r\n");
      UART_Printf("   - CS   -> PA4\r\n");
      UART_Printf("   - SCK  -> PA5\r\n");
      UART_Printf("   - MISO -> PA6\r\n");
      UART_Printf("   - MOSI -> PA7\r\n");
      UART_Printf("2. Kiem tra nguon VCC cua module:\r\n");
      UART_Printf("   - Neu module co IC hạ ap AMS1117 tren mach: Phai cap 5V vao VCC.\r\n");
      UART_Printf("   - Neu module chi co the breakout: Cap 3.3V vao VCC.\r\n");
      UART_Printf("3. Kiem tra the MicroSD:\r\n");
      UART_Printf("   - Phai duoc dinh dang dinh dang FAT32 (hoac FAT16).\r\n");
      UART_Printf("   - Dung luong khuyen dung: <= 32GB.\r\n");
      UART_Printf("LED PC13 nhap nhay nhanh (100ms) bao loi.\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    if (testSuccess)
    {
      HAL_Delay(500);
    }
    else
    {
      HAL_Delay(100);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
