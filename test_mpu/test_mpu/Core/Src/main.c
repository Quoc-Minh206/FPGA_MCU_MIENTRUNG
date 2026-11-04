/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Chuong trinh doc MPU-6050 qua I2C1 va xuat du lieu len PuTTY qua UART1
  ******************************************************************************
  * Cau hinh chan STM32F103:
  * - I2C1:  SCL = PB6, SDA = PB7 (MPU-6050 GY-521)
  * - UART1: TX = PA9,  RX = PA10 (115200 baud, noi vao USB-TTL de xem tren PuTTY)
  * - Nguon: VCC MPU-6050 noi vao 5V (hoac 3.3V), GND noi GND chung.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t mpu_found = 0;
uint8_t mpu_dev_addr = 0; // Dia chi 8-bit cho HAL (0xD0 hoac 0xD2)
uint8_t who_am_i = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_Print(const char *str);
uint8_t MPU6050_Init_Sequence(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Print(const char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 200);
}

// Ham khoi tao va cau hinh MPU-6050
uint8_t MPU6050_Init_Sequence(void)
{
  char log_buf[128];
  uint8_t check_addr[2] = {0x68 << 1, 0x69 << 1}; // 0xD0 va 0xD2

  mpu_found = 0;

  // 1. Quet tim xem MPU o dia chi 0x68 hay 0x69
  for (int i = 0; i < 2; i++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, check_addr[i], 3, 100) == HAL_OK) {
      mpu_found = 1;
      mpu_dev_addr = check_addr[i];
      break;
    }
  }

  // 2. Neu chua thay, quet toan bo bus 1..127 phong truong hop clone ID la
  if (!mpu_found) {
    for (uint8_t a = 1; a < 128; a++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 2, 50) == HAL_OK) {
        mpu_found = 1;
        mpu_dev_addr = a << 1;
        break;
      }
    }
  }

  if (!mpu_found) {
    UART_Print("[-] KHONG TIM THAY MPU-6050! Vui long kiem tra day:\r\n");
    UART_Print("    PB6 -> SCL | PB7 -> SDA | 5V -> VCC | GND -> GND\r\n\r\n");
    return 0;
  }

  // Thong bao tim thay
  snprintf(log_buf, sizeof(log_buf), 
           "[+] TIM THAY THIET BI I2C tai dia chi 7-bit: 0x%02X (HAL Addr: 0x%02X)\r\n", 
           mpu_dev_addr >> 1, mpu_dev_addr);
  UART_Print(log_buf);

  // 3. Doc thanh ghi WHO_AM_I (0x75)
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, mpu_dev_addr, 0x75, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
  if (ret == HAL_OK) {
    snprintf(log_buf, sizeof(log_buf), "[+] Doc WHO_AM_I thanh cong: 0x%02X ", who_am_i);
    UART_Print(log_buf);
    if (who_am_i == 0x68) UART_Print("(MPU-6050 Chinh hang)\r\n");
    else if (who_am_i == 0x70) UART_Print("(MPU-6500)\r\n");
    else if (who_am_i == 0x71) UART_Print("(MPU-9250)\r\n");
    else if (who_am_i == 0x72) UART_Print("(MPU-6515)\r\n");
    else if (who_am_i == 0x98) UART_Print("(MPU-6050 Clone)\r\n");
    else UART_Print("(Chip MPU/ICM Tuong thich)\r\n");
  } else {
    UART_Print("[-] Loi khi doc WHO_AM_I tu thanh ghi 0x75!\r\n");
  }

  // 4. Danh thuc MPU (Ghi 0x00 vao PWR_MGMT_1 tai 0x6B)
  uint8_t data_pwr = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, mpu_dev_addr, 0x6B, I2C_MEMADD_SIZE_8BIT, &data_pwr, 1, 100);

  // 5. Bat ca 3 truc (Ghi 0x00 vao PWR_MGMT_2 tai 0x6C)
  uint8_t data_axes = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, mpu_dev_addr, 0x6C, I2C_MEMADD_SIZE_8BIT, &data_axes, 1, 100);

  // 6. Cau hinh bo loc so DLPF 44Hz (Ghi 0x03 vao CONFIG tai 0x1A)
  uint8_t data_dlpf = 0x03;
  HAL_I2C_Mem_Write(&hi2c1, mpu_dev_addr, 0x1A, I2C_MEMADD_SIZE_8BIT, &data_dlpf, 1, 100);

  // 7. Cau hinh dai do gia toc +-2g (Ghi 0x00 vao ACCEL_CONFIG tai 0x1C)
  uint8_t data_scale = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, mpu_dev_addr, 0x1C, I2C_MEMADD_SIZE_8BIT, &data_scale, 1, 100);

  UART_Print("[+] Da gui lenh danh thuc va cau hinh MPU thanh cong!\r\n");
  UART_Print("------------------------------------------------------------------------\r\n");
  return 1;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(500); // Cho MPU on dinh nguon

  UART_Print("\r\n========================================================================\r\n");
  UART_Print("   HE THONG KIEM THU CAM BIEN MPU-6050 (GY-521) QUA STM32 & PUTTY\r\n");
  UART_Print("   Baudrate: 115200 | SCL: PB6 | SDA: PB7 | TX: PA9 (Noi vao RX USB-TTL)\r\n");
  UART_Print("========================================================================\r\n\r\n");

  MPU6050_Init_Sequence();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t raw_accel[6];
  char sample_buf[128];
  uint32_t sample_count = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (mpu_found) {
      // Doc 6 bytes gia toc tu thanh ghi 0x3B (ACCEL_XOUT_H)
      HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, mpu_dev_addr, 0x3B, 
                                                  I2C_MEMADD_SIZE_8BIT, raw_accel, 6, 100);

      if (status == HAL_OK) {
        sample_count++;
        // Ghep 2 byte thanh so nguyen 16-bit co dau
        int16_t raw_ax = (int16_t)((raw_accel[0] << 8) | raw_accel[1]);
        int16_t raw_ay = (int16_t)((raw_accel[2] << 8) | raw_accel[3]);
        int16_t raw_az = (int16_t)((raw_accel[4] << 8) | raw_accel[5]);

        // Tinh gia tri gia toc don vi g (dài +-2g: 16384 LSB = 1.0g)
        float ax_g = (float)raw_ax / 16384.0f;
        float ay_g = (float)raw_ay / 16384.0f;
        float az_g = (float)raw_az / 16384.0f;

        // In ra PuTTY (format dep de quan sat su bien doi khi lac/nghieng MPU)
        snprintf(sample_buf, sizeof(sample_buf),
                 "#%05lu | Ax=%+6d (%+.2fg) | Ay=%+6d (%+.2fg) | Az=%+6d (%+.2fg)\r\n",
                 sample_count,
                 raw_ax, ax_g,
                 raw_ay, ay_g,
                 raw_az, az_g);
        UART_Print(sample_buf);
      } else {
        UART_Print("[-] Mat ket noi I2C khi doc gia toc! Dang thu ket noi lai...\r\n");
        HAL_Delay(500);
        MPU6050_Init_Sequence();
      }

      HAL_Delay(100); // Doc 10 lan/giay (10 Hz) de PuTTY hien thi ro rang, de nhin
    } else {
      // Neu chua tim thay, lap lai do tim sau moi 2 giay
      HAL_Delay(2000);
      MPU6050_Init_Sequence();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
