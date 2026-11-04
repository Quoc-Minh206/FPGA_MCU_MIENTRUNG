/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - He thong SIM800L / SIM Module tren STM32
  *                   - Tu dong xoa sach bo nho SIM khi khoi dong (AT+CMGD=1,4)
  *                   - Tu dong gui tin nhan khi Run
  *                   - Tu dong doc va xoa tin nhan ngay sau khi nhan tin (+CMTI)
  *                   - Toc do Baud: 115200 bps cho ca PuTTY va SIM
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
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
/* ==============================================================================
 * THÔNG SỐ CẤU HÌNH GỬI TIN NHẮN TỰ ĐỘNG KHI RUN
 * ============================================================================== */
#define SIM_PHONE_NUMBER    "0899244478"         /* <-- Số điện thoại người nhận */
#define SIM_SMS_MESSAGE     "dcm m minh"  /* <-- Nội dung tin nhắn không dấu */
/* ============================================================================== */

#define SIM_RX_BUFFER_SIZE  512
#define RX_BUFFER_LEN       256

/* Biến nhận 1 byte ngắt UART */
uint8_t rx_byte_pc;
uint8_t rx_byte_sim;

/* Bộ đệm chuỗi tổng quát để kiểm tra phản hồi AT */
uint8_t sim_rx_buffer[SIM_RX_BUFFER_SIZE];
volatile uint16_t sim_rx_idx = 0;

/* Bộ đệm nhận từng dòng để phân tích sự kiện +CMTI */
char rx_line_sim[RX_BUFFER_LEN];
volatile uint16_t rx_idx = 0;

/* Cờ báo hiệu có sự kiện tin nhắn mới từ ngắt đẩy ra vòng lặp while(1) */
volatile uint8_t flag_new_sms = 0;
volatile int target_sms_index = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PC_Print(const char *msg);
void PC_Println(const char *msg);
void SIM800_SendCmd(const char *cmd);
void SIM_ParseLine(char *line);
uint8_t SIM_Send_Command(const char *cmd, const char *expected_reply, uint32_t timeout_ms);
uint8_t SIM_Send_SMS_Core(const char *number, const char *msg);
uint8_t SIM_AutoSend_SMS(const char *number, const char *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Gửi chuỗi lệnh UART sang Module SIM
  */
void SIM800_SendCmd(const char *cmd)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 1000);
}

/**
  * @brief Xử lý từng dòng phản hồi từ Module SIM để bắt sự kiện +CMTI
  */
void SIM_ParseLine(char *line)
{
    /* Bắt sự kiện: +CMTI: "SM", <index> hoặc +CMTI: "ME", <index> */
    char *cmti = strstr(line, "+CMTI:");
    if (cmti != NULL)
    {
        char *comma = strchr(cmti, ',');
        if (comma != NULL)
        {
            int idx = atoi(comma + 1);
            if (idx > 0)
            {
                target_sms_index = idx;
                flag_new_sms = 1; /* Bật cờ để vòng while(1) xử lý đọc & xóa */
            }
        }
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Bật ngắt nhận byte đầu tiên cho cả 2 cổng UART */
  HAL_UART_Receive_IT(&huart1, &rx_byte_pc, 1);
  HAL_UART_Receive_IT(&huart2, &rx_byte_sim, 1);

  /* Thông báo khởi động lên màn hình PuTTY */
  PC_Println("\r\n=======================================================================");
  PC_Println("         STM32F103C8T6 - HE THONG SMS SIM800L (BAUDRATE: 115200)       ");
  PC_Println("=======================================================================");
  PC_Println("Dang khoi dong, vui long cho 3 giay de SIM on dinh nguon...");
  HAL_Delay(3000);

  /* 1. Kiểm tra kết nối SIM ở 115200 bps */
  PC_Print("[1/5] Kiem tra ket noi SIM (AT @ 115200 bps)... ");
  uint8_t at_ok = 0;
  for (int retry = 0; retry < 5; retry++)
  {
      if (SIM_Send_Command("AT\r\n", "OK", 1000))
      {
          at_ok = 1;
          break;
      }
      HAL_Delay(500);
  }
  if (!at_ok)
  {
      PC_Println("THAT BAI! (Module SIM khong phan hoi AT)");
  }
  else
  {
      PC_Println("OK!");
  }

  /* 2. Cấu hình cơ bản: Tắt echo, bật báo lỗi chi tiết, bật Text Mode, bảng mã GSM */
  PC_Print("[2/5] Cau hinh Text Mode & Parameters... ");
  SIM_Send_Command("ATE0\r\n", "OK", 1000);
  SIM_Send_Command("AT+CMEE=1\r\n", "OK", 1000);
  SIM_Send_Command("AT+CMGF=1\r\n", "OK", 1000);
  SIM_Send_Command("AT+CSCS=\"GSM\"\r\n", "OK", 1000);
  PC_Println("OK!");

  /* 3. DỌN SẠCH TOÀN BỘ KHAY CHỨA TIN NHẮN TRÊN SIM (CHỐNG TRÀN BỘ NHỚ +SMS FULL) */
  PC_Print("[3/5] Don sach toan bo tin nhan cu (AT+CMGD=1,4)... ");
  SIM_Send_Command("AT+CMGD=1,4\r\n", "OK", 3000);
  PC_Println("DA XOA SACH! (Bo nho SIM da duoc giai phong hoan toan)");

  /* 4. Kiểm tra SIM & Trạng thái vào mạng */
  PC_Println("[4/5] Kiem tra the SIM & Mang di dong:");
  PC_Print("   - The SIM (AT+CPIN?)... ");
  if (SIM_Send_Command("AT+CPIN?\r\n", "READY", 3000))
  {
      PC_Println("READY!");
  }
  else
  {
      PC_Println("CHUA SAN SANG!");
  }

  PC_Print("   - Song mang (AT+CREG?)... ");
  uint8_t net_ok = 0;
  for (int i = 0; i < 15; i++)
  {
      sim_rx_idx = 0;
      memset((void *)sim_rx_buffer, 0, SIM_RX_BUFFER_SIZE);
      HAL_UART_Transmit(&huart2, (uint8_t *)"AT+CREG?\r\n", 10, 1000);
      HAL_Delay(1000);
      if (strstr((char *)sim_rx_buffer, ",1") != NULL || strstr((char *)sim_rx_buffer, ",5") != NULL)
      {
          net_ok = 1;
          break;
      }
  }
  if (net_ok)
  {
      PC_Println("DA VAO MANG!");
  }
  else
  {
      PC_Println("CHUA VAO MANG!");
  }

  /* 5. TỰ ĐỘNG GỬI TIN NHẮN KHỞI ĐỘNG */
  PC_Println("[5/5] Tien hanh gui tin nhan khoi dong:");
  SIM_AutoSend_SMS(SIM_PHONE_NUMBER, SIM_SMS_MESSAGE);

  PC_Println("\r\n-----------------------------------------------------------------------");
  PC_Println("  HE THONG DA SAN SANG: DANG LANG NGHE TIN NHAN DEN (TU DONG DOC & XOA)");
  PC_Println("  Ban co the go lenh AT truc tiep tren PuTTY bat ky luc nao.");
  PC_Println("-----------------------------------------------------------------------\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* =========================================================================
     * LUỒNG SỰ KIỆN: TỰ ĐỘNG ĐỌC VÀ XÓA NGAY KHI CÓ TIN NHẮN TỚI
     * ========================================================================= */
    if (flag_new_sms && target_sms_index > 0)
    {
        int idx = target_sms_index;
        char cmd_buf[64];

        PC_Println("\r\n=======================================================");
        snprintf(cmd_buf, sizeof(cmd_buf), ">>> PHAT HIEN TIN NHAN MOI O O NHO: %d <<<", idx);
        PC_Println(cmd_buf);
        PC_Println("=======================================================");

        /* 1. Gửi lệnh đọc nội dung tin nhắn */
        PC_Println("-> Dang doc noi dung tin nhan (AT+CMGR):");
        snprintf(cmd_buf, sizeof(cmd_buf), "AT+CMGR=%d\r\n", idx);
        SIM800_SendCmd(cmd_buf);
        HAL_Delay(1000); /* Đợi module trả nội dung tin nhắn về và in lên PuTTY */

        /* 2. Gửi lệnh xóa ngay ô nhớ vừa đọc để giải phóng bộ nhớ */
        PC_Println("\r\n-> Dang xoa tin nhan de giai phong bo nho (AT+CMGD)...");
        snprintf(cmd_buf, sizeof(cmd_buf), "AT+CMGD=%d\r\n", idx);
        SIM800_SendCmd(cmd_buf);
        HAL_Delay(500);

        PC_Println("-> XAC NHAN: Da xoa o nho, khay SIM luon trong (0 tin nhan).");
        PC_Println("=======================================================\r\n");

        /* Reset cờ trạng thái */
        flag_new_sms = 0;
        target_sms_index = -1;
    }

    HAL_Delay(50);
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
/**
  * @brief In chuỗi thông báo lên máy tính qua USART1
  */
void PC_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
}

/**
  * @brief In chuỗi thông báo kèm xuống dòng lên máy tính qua USART1
  */
void PC_Println(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 1000);
}

/**
  * @brief Gửi lệnh AT xuống module SIM và chờ phản hồi mong đợi
  */
uint8_t SIM_Send_Command(const char *cmd, const char *expected_reply, uint32_t timeout_ms)
{
    sim_rx_idx = 0;
    memset((void *)sim_rx_buffer, 0, SIM_RX_BUFFER_SIZE);

    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 1000);

    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < timeout_ms)
    {
        if (strstr((char *)sim_rx_buffer, expected_reply) != NULL)
        {
            return 1;
        }
        if (strstr((char *)sim_rx_buffer, "ERROR") != NULL)
        {
            return 0;
        }
        HAL_Delay(10);
    }
    return 0;
}

/**
  * @brief Thực hiện 4 bước gửi tin nhắn SMS tới 1 số điện thoại cụ thể
  */
uint8_t SIM_Send_SMS_Core(const char *number, const char *msg)
{
    char cmd[64];

    /* 1. Đảm bảo ở chế độ Text Mode */
    if (!SIM_Send_Command("AT+CMGF=1\r\n", "OK", 2000))
    {
        PC_Println("   -> [Loi] Khong the bat Text Mode!");
        return 0;
    }

    /* 2. Cấu hình bảng mã ký tự GSM */
    SIM_Send_Command("AT+CSCS=\"GSM\"\r\n", "OK", 2000);

    /* 3. Phát lệnh gửi tới số điện thoại, kết thúc bằng \r */
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r", number);
    PC_Print("   -> Dang ket noi toi so: ");
    PC_Println(number);

    sim_rx_idx = 0;
    memset((void *)sim_rx_buffer, 0, SIM_RX_BUFFER_SIZE);
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 1000);

    /* Chờ ký tự prompt '>' từ module SIM */
    uint32_t t_prompt = HAL_GetTick();
    uint8_t got_prompt = 0;
    while ((HAL_GetTick() - t_prompt) < 5000)
    {
        if (strchr((char *)sim_rx_buffer, '>') != NULL)
        {
            got_prompt = 1;
            break;
        }
        if (strstr((char *)sim_rx_buffer, "ERROR") != NULL)
        {
            break;
        }
        HAL_Delay(10);
    }

    if (!got_prompt)
    {
        PC_Println("   -> [Loi] Module SIM khong xuat hien dau nhac '>' !");
        PC_Print("   -> [Phan hoi SIM]: ");
        PC_Println((char *)sim_rx_buffer);
        return 0;
    }

    PC_Println("   -> Da nhan duoc dau nhac '>', cho module on dinh 300ms...");
    HAL_Delay(300);

    /* 4. Gửi nội dung tin nhắn và kết thúc bằng phím Ctrl+Z (0x1A) */
    PC_Print("   -> Noi dung tin nhan: \"");
    PC_Print(msg);
    PC_Println("\"");

    sim_rx_idx = 0;
    memset((void *)sim_rx_buffer, 0, SIM_RX_BUFFER_SIZE);

    /* Truyền nội dung tin nhắn */
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 3000);
    HAL_Delay(200);

    /* Truyền ký tự kết thúc Ctrl+Z (0x1A) */
    uint8_t ctrl_z = 0x1A;
    HAL_UART_Transmit(&huart2, &ctrl_z, 1, 1000);

    PC_Println("   -> Da phat lenh Ctrl+Z (0x1A)!");
    PC_Println("   -> Dang cho nha mang phan hoi +CMGS va OK (thoi gian cho toi da 60s)...");

    /* Chờ bắt tín hiệu +CMGS: và OK từ nhà mạng */
    uint32_t t_start = HAL_GetTick();
    uint8_t got_cmgs = 0;
    uint8_t got_ok = 0;
    uint8_t got_error = 0;

    while ((HAL_GetTick() - t_start) < 60000)
    {
        if (strstr((char *)sim_rx_buffer, "+CMGS:") != NULL)
        {
            got_cmgs = 1;
        }

        if (got_cmgs && (strstr((char *)sim_rx_buffer, "OK") != NULL))
        {
            got_ok = 1;
            break;
        }

        if (strstr((char *)sim_rx_buffer, "ERROR") != NULL)
        {
            HAL_Delay(500);
            got_error = 1;
            break;
        }

        HAL_Delay(50);
    }

    if (got_cmgs || got_ok)
    {
        PC_Println("\r\n=======================================================================");
        PC_Println("            >>> XAC NHAN: GUI TIN NHAN SMS THANH CONG! <<<             ");
        PC_Println("=======================================================================");
        PC_Print("Phan hoi tu Module SIM: ");
        PC_Println((char *)sim_rx_buffer);
        return 1;
    }
    else
    {
        PC_Println("\r\n=======================================================================");
        PC_Println("                 >>> GUI TIN NHAN THAT BAI! <<<                        ");
        PC_Println("=======================================================================");
        PC_Print("Chi tiet phan hoi tu Module SIM:\r\n");
        PC_Println((char *)sim_rx_buffer);

        if (strstr((char *)sim_rx_buffer, "302") != NULL || strstr((char *)sim_rx_buffer, "304") != NULL)
        {
            PC_Println("[THONG BAO LOI]: +CMS ERROR: 302/304: The SIM het tien hoac tram mang tu choi gui.");
        }
        else if (strstr((char *)sim_rx_buffer, "305") != NULL)
        {
            PC_Println("[THONG BAO LOI]: +CMS ERROR: 305: Module chua o che do Text Mode (AT+CMGF=1).");
        }
        else if (strstr((char *)sim_rx_buffer, "500") != NULL)
        {
            PC_Println("[THONG BAO LOI]: +CMS ERROR: 500: Loi mang hoac module bi sut ap khi phat song.");
        }
        else if (got_error)
        {
            PC_Println("[THONG BAO LOI]: Module SIM tra ve ERROR.");
        }
        else
        {
            PC_Println("[THONG BAO LOI]: Het thoi gian cho (Timeout 60s) khong nhan duoc tin hieu +CMGS va OK tu mang.");
        }
        return 0;
    }
}

/**
  * @brief Tự động kiểm tra hệ thống và gửi tin nhắn SMS
  */
uint8_t SIM_AutoSend_SMS(const char *number, const char *msg)
{
    uint8_t res = SIM_Send_SMS_Core(number, msg);

    /* Nếu gửi với định dạng 09... thất bại, tự động thử lại với định dạng quốc tế +84... */
    if (!res && number[0] == '0')
    {
        char intl_number[32];
        snprintf(intl_number, sizeof(intl_number), "+84%s", number + 1);
        PC_Println("\r\n-> Tu dong thu lai voi dinh dang so quoc te (+84):");
        res = SIM_Send_SMS_Core(intl_number, msg);
    }

    return res;
}

/**
  * @brief Callback nhận ngắt 1 byte theo chuẩn HAL
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Dữ liệu hoặc phản hồi từ Module SIM trả về */
    if (huart->Instance == USART2)
    {
        /* 1. Lưu vào buffer tổng quát để SIM_Send_Command kiểm tra */
        if (sim_rx_idx < SIM_RX_BUFFER_SIZE - 1)
        {
            sim_rx_buffer[sim_rx_idx++] = (uint8_t)rx_byte_sim;
            sim_rx_buffer[sim_rx_idx] = '\0';
        }

        /* 2. Gom dòng để bắt sự kiện +CMTI: "SM", <index> */
        if (rx_byte_sim == '\n')
        {
            rx_line_sim[rx_idx] = '\0';
            if (rx_idx > 0)
            {
                SIM_ParseLine(rx_line_sim);
            }
            rx_idx = 0;
        }
        else if (rx_byte_sim != '\r')
        {
            if (rx_idx < RX_BUFFER_LEN - 1)
            {
                rx_line_sim[rx_idx++] = (char)rx_byte_sim;
            }
            else
            {
                rx_idx = 0; /* Tránh tràn bộ đệm dòng */
            }
        }

        /* 3. Đẩy tức thì lên màn hình PuTTY qua USART1 bằng thanh ghi trực tiếp */
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = rx_byte_sim;

        /* Mồi lại ngắt để đón byte tiếp theo từ SIM */
        HAL_UART_Receive_IT(&huart2, &rx_byte_sim, 1);
    }
    /* Ký tự người dùng gõ từ bàn phím PuTTY -> Chuyển tiếp sang Module SIM */
    else if (huart->Instance == USART1)
    {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = rx_byte_pc;

        /* Mồi lại ngắt để đón byte tiếp theo từ PuTTY */
        HAL_UART_Receive_IT(&huart1, &rx_byte_pc, 1);
    }
}

/**
  * @brief Xử lý lỗi Overrun Error (ORE) tránh treo cờ UART
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte_pc, 1);
    }
    else if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart2, &rx_byte_sim, 1);
    }
}
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
