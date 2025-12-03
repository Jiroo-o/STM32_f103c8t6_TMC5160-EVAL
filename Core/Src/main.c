/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (Updated for TMC5160 Velocity Mode)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h> // เผื่อใช้ abs
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---- PIN DEFINES (Based on your wiring) ----
#define TMC_CS_Pin      GPIO_PIN_4
#define TMC_CS_Port     GPIOA

#define TMC_EN_Pin      GPIO_PIN_0
#define TMC_EN_Port     GPIOB

#define TMC5160_FCLK 12000000.0f // Internal Clock Frequency
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TMC_CS_LOW()    HAL_GPIO_WritePin(TMC_CS_Port, TMC_CS_Pin, GPIO_PIN_RESET)
#define TMC_CS_HIGH()   HAL_GPIO_WritePin(TMC_CS_Port, TMC_CS_Pin, GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void uart_printf(const char *fmt, ...);
void TMC5160_WriteReg(uint8_t address, uint32_t value);
uint32_t TMC5160_ReadReg(uint8_t address);

// เพิ่ม Prototypes ใหม่
void TMC5160_SetupMotion(void);
void TMC5160_Rotate(int32_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ฟังก์ชันแปลง PPS -> VMAX Value
int32_t TMC5160_ConvertSpeed(float steps_per_sec) {
	// สูตร: (PPS * 2^24) / fCLK
	return (int32_t) ((steps_per_sec * 16777216.0f) / TMC5160_FCLK);
}

// อัปเดตฟังก์ชัน Rotate ให้รับค่า PPS ตรงๆ
void TMC5160_Rotate_PPS(float pps) {
	uint32_t vmax_val = (uint32_t) fabs(TMC5160_ConvertSpeed(pps));

	if (pps >= 0) {
		TMC5160_WriteReg(0x20, 1); // RAMPMODE = Velocity Positive
	} else {
		TMC5160_WriteReg(0x20, 2); // RAMPMODE = Velocity Negative
	}

	TMC5160_WriteReg(0x27, vmax_val); // เขียนค่า VMAX ที่แปลงแล้ว
}

void TMC5160_PrintStatus(void) {
	// อ่าน GSTAT (0x01): ดูสถานะทั่วไป (Reset, Voltage, Driver Error)
	uint32_t gstat = TMC5160_ReadReg(0x01);

	// อ่าน DRV_STATUS (0x6F): ดูสถานะ Driver (Open Load, Short, Over Temp)
	uint32_t drv_status = TMC5160_ReadReg(0x6F);

	uart_printf("Status -> GSTAT: 0x%08X | DRV: 0x%08X\r\n", gstat, drv_status);

	if (gstat & 0x02)
		uart_printf("!! DRIVER ERROR DETECTED !!\r\n");
	if (gstat & 0x04)
		uart_printf("!! UNDER VOLTAGE !!\r\n");
	if (drv_status & 0x80000000)
		uart_printf("!! STANDSTILL !!\r\n");
}

// 1. ฟังก์ชันสั่งวิ่ง (High Speed, Normal Accel)
// speed: ความเร็วเป้าหมาย
// accel: อัตราเร่งตอนออกตัว (ใส่สัก 500-1000 ให้มีแรงออกตัว)
void TMC5160_Run(int32_t speed, uint32_t accel) {
	// ตั้งค่าอัตราเร่งสำหรับ "ขาไป"
	TMC5160_WriteReg(0x26, accel);

	// สั่งความเร็ว
	if (speed >= 0) {
		TMC5160_WriteReg(0x20, 1); // RAMPMODE +
		TMC5160_WriteReg(0x27, speed);
	} else {
		TMC5160_WriteReg(0x20, 2); // RAMPMODE -
		TMC5160_WriteReg(0x27, -speed);
	}
}

// 2. ฟังก์ชันสั่งหยุดนุ่ม (Soft Stop)
// decel: อัตราเบรก (ใส่ต่ำๆ เช่น 50 หรือ 100 เพื่อให้หยุดช้าๆ)
void TMC5160_SoftStop(uint32_t decel) {
	// *** ทีเด็ดอยู่ตรงนี้: เปลี่ยน AMAX ให้ต่ำก่อนสั่งหยุด ***
	TMC5160_WriteReg(0x26, decel);

	// สั่งหยุด (VMAX = 0)
	// Driver จะใช้ค่า AMAX ใหม่ (ที่ต่ำๆ) ในการคำนวณ Ramp ลง
	TMC5160_WriteReg(0x27, 0);
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
  /* USER CODE BEGIN 2 */

	// 1. Disable driver first (Safety)
	TMC_CS_HIGH();
	HAL_GPIO_WritePin(TMC_EN_Port, TMC_EN_Pin, GPIO_PIN_SET); // EN High = Disable
	HAL_Delay(100);

	uart_printf("\r\n--- TMC5160 SPI DEBUG START ---\r\n");

	// 2. Enable driver (EN = Low)
	HAL_GPIO_WritePin(TMC_EN_Port, TMC_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);

	// 3. Init GCONF (Global Config)
	// GCONF = 0x00000004 (EN_PWM_MODE = 1) -> แล้วแต่การใช้งาน ปกติใช้ 0 ก็ได้ถ้าไม่ได้ใช้ external step
	TMC5160_WriteReg(0x00, 0x00000000);
	HAL_Delay(10);

	// Readback Check
	uint32_t gconf = TMC5160_ReadReg(0x00);
	uart_printf("GCONF Readback = 0x%08lX\r\n", gconf);

	// 4. Setup Motion Parameters (Current, Chopper, Accel)
	TMC5160_SetupMotion();

	uart_printf("Ready to Run!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// --- Step 1: วิ่งขวา (เร็ว!) ---
		if (TMC5160_ReadReg(0x00) != 0) { // เช็ค Reset (GCONF ต้องเป็น 0 ในโหมดนี้)
			// ถ้าอ่านได้ค่าไม่ใช่ 0 (หรือ Default 0x04 กลับมา) แปลว่า Reset
			// แต่เดี๋ยวก่อน... SetupMotion เราสั่งเขียน 0 ลงไป
			// ถ้า Reset มันจะเด้งกลับเป็นค่า Default โรงงาน
			// เอาเป็นว่าสั่ง Setup ทุกรอบเพื่อความชัวร์ดีกว่า
		}
		// "Hack" Setup ทุกรอบ กันเหนียวสุดๆ
		TMC5160_SetupMotion();

		uart_printf(">>> Turbo Right (Speed 50k)\r\n");
		// ลอง 50,000 ดูก่อน (เร็วกว่าตะกี้เยอะ)
		// Accel 2000 (ออกตัวไวขึ้น)
		TMC5160_Run(50000, 2000);

		HAL_Delay(4000);

		// --- Step 2: เบรก (นุ่มๆ) ---
		uart_printf("--- Soft Stop ---\r\n");
		TMC5160_SoftStop(1000);
		HAL_Delay(1000);

		// --- Step 3: วิ่งซ้าย (เร็ว!) ---
		TMC5160_SetupMotion(); // กันเหนียวอีกที

		uart_printf("<<< Turbo Left (Speed -50k)\r\n");
		TMC5160_Run(-50000, 2000);

		HAL_Delay(4000);

		// --- Step 4: เบรก ---
		uart_printf("--- Soft Stop ---\r\n");
		TMC5160_SoftStop(1000);
		HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TMC_CS_Pin */
  GPIO_InitStruct.Pin = TMC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TMC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_EN_Pin */
  GPIO_InitStruct.Pin = TMC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TMC_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void uart_printf(const char *fmt, ...) {
	char buffer[128];
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	if (len > 0) {
		if (len > (int) sizeof(buffer))
			len = sizeof(buffer);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, len, HAL_MAX_DELAY);
	}
}

void TMC5160_WriteReg(uint8_t address, uint32_t value) {
	uint8_t tx[5];
	uint8_t rx[5];

	tx[0] = 0x80 | (address & 0x7F);  // Write bit = 1
	tx[1] = (value >> 24) & 0xFF;
	tx[2] = (value >> 16) & 0xFF;
	tx[3] = (value >> 8) & 0xFF;
	tx[4] = (value >> 0) & 0xFF;

	TMC_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, 100);
	TMC_CS_HIGH();
}

uint32_t TMC5160_ReadReg(uint8_t address) {
	uint8_t tx[5] = { 0 };
	uint8_t rx[5] = { 0 };
	uint32_t value = 0;

	// Frame 1: Send Read Command
	tx[0] = address & 0x7F; // Write bit = 0

	TMC_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, 100);
	TMC_CS_HIGH();

	// Frame 2: Dummy to get data out
	tx[0] = address & 0x7F;

	TMC_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, 100);
	TMC_CS_HIGH();

	value = (uint32_t) rx[1] << 24;
	value |= (uint32_t) rx[2] << 16;
	value |= (uint32_t) rx[3] << 8;
	value |= (uint32_t) rx[4];

	return value;
}

// ---- NEW FUNCTIONS ----

void TMC5160_SetupMotion(void) {
	// 1. CHOPCONF:
	// เปลี่ยน MRES เป็น 4 (16 Microsteps) -> นี่คือเกียร์สำหรับความเร็ว!
	// TOFF=3, HSTRT=4, HEND=1, TBL=2 (สูตร NEMA 34)
	TMC5160_WriteReg(0x6C, 0x14410153);

	// 2. IHOLD_IRUN: อัดกระแสเพิ่ม! (PSU 10A ไหวอยู่แล้ว)
	// IRUN = 20 (ประมาณ 2.5A) -> แรงบิดมาเต็ม
	// IHOLD = 5
	TMC5160_WriteReg(0x10, 0x00061405);

	// 3. AMAX:
	// พอใช้ 16 uStep ค่า AMAX ไม่ต้องเยอะมากก็พุ่งแล้ว
	// ตั้งไว้ 5000 (ออกตัวกระชับ)
	TMC5160_WriteReg(0x26, 5000);

	// 4. Clear GCONF (SpreadCycle Only)
	TMC5160_WriteReg(0x00, 0x00000000);

	// ... (VSTART/STOP ปิดเหมือนเดิม) ...
	TMC5160_WriteReg(0x23, 0);
	TMC5160_WriteReg(0x24, 0);
	TMC5160_WriteReg(0x34, 0x00000000);
	TMC5160_WriteReg(0x6D, 0x00000000);

	uart_printf("Setup: SPORT MODE (16 uSteps, High Current).\r\n");
}
void TMC5160_Rotate(int32_t speed) {
	// RAMPMODE (0x20): 1 = Pos Velocity, 2 = Neg Velocity
	// VMAX (0x27): Target Speed (Always Positive)

	if (speed >= 0) {
		TMC5160_WriteReg(0x20, 1);       // Direction +
		TMC5160_WriteReg(0x27, speed);   // Speed value
	} else {
		TMC5160_WriteReg(0x20, 2);       // Direction -
		// แปลง speed เป็นบวก เพราะ VMAX รับค่า Unsigned
		// (ใช้ -speed ก็ได้ เพราะ input เป็น int32_t ที่ติดลบอยู่แล้ว)
		TMC5160_WriteReg(0x27, -speed);
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
