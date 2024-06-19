/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "complex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum print_option
{
	DONT_PRINT,
	PRINT
} print_option;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PRINT_LENGTH 100 // Maximum lpuart1 serial data buffer length
#define GRAVITY 9.8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Maximum time-out to wait for any print ACK
const uint32_t MAXIMUM_PRINT_TIMEOUT = 100;

const uint16_t SAMPLE_SIZE = 1 << 10; // Must be = 2^n where n is an integer
// Note that a SAMPLE_POWER < 2 will record samples slower than the set sample rate
const uint8_t SAMPLE_POWER = 6; // 0 <= SAMPLE_POWER <= 8: 8 ~4.3Hz 7 ~8.6Hz etc
const uint8_t SAMPLE_VALUE = (uint8_t) (((uint16_t) 1 << SAMPLE_POWER) - 1);
const uint32_t SAMPLE_PERIOD_MS = 10 * (1 + (uint32_t) SAMPLE_VALUE) / 11 + 1;
const float SAMPLE_FREQUENCY = 11000.f / (10 * (1 + (uint32_t) SAMPLE_VALUE));

// How long to wait before checking external sensor registers for magnetometer data
const uint32_t MAG_SAFTEY_WAIT = (SAMPLE_PERIOD_MS / 4 < 10) ? SAMPLE_PERIOD_MS / 4 : 10;

// Gets added to measured gyro int16_vector3 via offset_gyro function
const int16_vector3 gyro_offset = { 0, -40, -79 }; // Measured for my specific device

const uint8_t GYRO_FS_SEL = 0; // 0 equivalent to GYRO_FS_SEL_250
// 131 is typical value for GYRO_FS_SEL = 0 (DS p11)
// 131 = 0xFFFF / 250 (lowest dps range of gyro)
const uint8_t GYRO_SENSITIVITY_DIVISOR = 1 << (GYRO_FS_SEL >> 1);
const float GYRO_SENSITIVITY_SCALE_FACTOR = M_PI / (180 * 131.f / GYRO_SENSITIVITY_DIVISOR);

const uint8_t ACCEL_FS_SEL = ACCEL_FS_SEL_4g; // 0 equivalent to ACCEL_FS_SEL_2g
// 16384 is typical value for ACCEL_FS_SEL = 0 (DS p11)
const float ACCEL_SENSITIVITY_SCALE_FACTOR = GRAVITY * 1.f / (16384 >> (ACCEL_FS_SEL >> 1));

// Outputs range from [-32752, 32752] and converts to equivalent values from [-4912, 4912]
// In magnetic flux density uT
const float MAG_SENSITIVITY_SCALE_FACTOR = 0.15;

// Number of times to repeat integrating (values should converge very quickly)
const uint8_t INTEGRAL_REPETITIONS = 2;

const float M_2PI = 2 * M_PI;
const float M_2PI_SAMPLE_SIZE = 2 * M_PI / SAMPLE_SIZE;
const float M_PI_SAMPLE_FREQUENCY = 1 / (SAMPLE_FREQUENCY * M_PI);

/** Earth-fixed local magnetic field in [uT]
 * https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
 * For Santa Rosa CA with WMM model on 5/14/24
 */
const float BEY = 22.1544; // Uncertainty of .131uT and change of -0.0344uT/yr
const float BEZ = 42.1813; // Uncertainty of .157uT and change of -0.0981uT/yr

/** Magnetic delcination in [rad]
 * https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
 * For Santa Rosa CA with WMM model on 5/14/24 with estimated 5 arcminute change west per year
 */
const float B_DECLINATION = 0.230674349; // 13 degrees 13 arcminutes east +- 22 arcminutes

/* These can be determined by spinning the buoy at constant rate so
 *  roll and pitch are both 0 and A can be determined by gyro then,
 *  Bi = Bi0 + (BEY*cosA)*Bi1 + (-BEY*sinA)*Bi2 and solve for Bij that minimize error
 * I can't, so I will instead align B1 to known compass headings to solve for Bij.
 * In python: Bijs = numpy.linalg.pinv(X)@Bi where X is the Nx3 matrix of Bij coefficients
 */
const float B10 = -2.21720753, // residual hull magnetic effect (bow) 0
    B11 = 1.09226593, // induced hull magnetic effect (bow) 1
    B12 = -0.12668346, // induced hull magnetic effect (bow) 0
    B20 = 8.57181184, // residual hull magnetic effect (starboard) 0
    B21 = 0.02056748, // induced hull magnetic effect (starboard) 0
    B22 = 1.07178173; // induced hull magnetic effect (starboard) 1

const float B_delta = B11 * B22 - B12 * B21;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void device_init();
HAL_StatusTypeDef init_registers();
void serial_print(const char* buffer);
void offset_gyro(int16_vector3* gyro);
void measure_gyro_offset();
void fft(float complex* f, uint16_t size);
void integrate(float* f, float* df);
void collect_samples(int16_vector3* accel_samples, float* wx, float* wy, float* wz, float* bx, float* by, print_option option);
void integrate_w(float* roll, float* pitch, float* wx, float* wy, float* wz, print_option option);
void calculate_headings(float* zx, float* zy, float* roll, float* pitch, float* bx, float* by, print_option option);
void calculate_heave(float* heave, int16_vector3* accel_samples, float* roll, float* pitch, print_option option);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[MAX_PRINT_LENGTH];
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
	MX_LPUART1_UART_Init();
	MX_USB_HOST_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	device_init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	sprintf(str, "Accel scale: %.9f\r\nGyro scale: %.10f\n\rMag scale: %.2f\r\n", ACCEL_SENSITIVITY_SCALE_FACTOR,
	    GYRO_SENSITIVITY_SCALE_FACTOR, MAG_SENSITIVITY_SCALE_FACTOR);
	serial_print(str);

	sprintf(str, "Sample size: %u\r\n", SAMPLE_SIZE);
	serial_print(str);

	sprintf(str, "Sample rate: %lums, %f/s\r\n\r\n", SAMPLE_PERIOD_MS, SAMPLE_FREQUENCY);
	serial_print(str);

	while (1)
	{
		uint32_t timeit;

		/* DATA COLLECTION BEGIN */
		// Definitions
		int16_vector3* accel_samples = (int16_vector3*) malloc(sizeof(int16_vector3) * SAMPLE_SIZE);
		float* wx = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // gyro_samples.x in rad/s
		float* wy = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // gyro_samples.y in rad/s
		float* wz = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // gyro_samples.z in rad/s
		float* bx = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // mag_samples.x in uT
		float* by = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // mag_samples.y in uT

		// Body
		serial_print("Data collection starting.\r\n");
		timeit = HAL_GetTick();
		collect_samples(accel_samples, wx, wy, wz, bx, by, DONT_PRINT);
		timeit = HAL_GetTick() - timeit;
		sprintf(str, "Data collection done in %lums.\r\n\r\n", timeit);
		serial_print(str);
		/* DATA COLLECTION END */

		/* ROLL PITCH CALCULATION BEGIN */
		// Definitions
		float* roll = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // Positive down from starboard
		float* pitch = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // Positive up from bow

		// Body
		serial_print("Roll/Pitch calculation starting.\r\n");
		timeit = HAL_GetTick();
		integrate_w(roll, pitch, wx, wy, wz, DONT_PRINT);
		timeit = HAL_GetTick() - timeit;
		sprintf(str, "Roll/Pitch calculation done in %lums.\r\n\r\n", timeit);
		serial_print(str);

		// Closing
		free(wx);
		free(wy);
		free(wz);
		/* ROLL PITCH CALCULATION END */

		/* HEAVE CALCULATION BEGIN */
		// Definitions
		float* heave = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // Earth-fixed reference

		// Body
		serial_print("Heave calculation starting.\r\n");
		timeit = HAL_GetTick();
		calculate_heave(heave, accel_samples, roll, pitch, DONT_PRINT);
		timeit = HAL_GetTick() - timeit;
		sprintf(str, "Heave calculation done in %lums.\r\n\r\n", timeit);
		serial_print(str);

		// Closing
		free(accel_samples);
		/* HEAVE CALCULATION END */

		/* NORTH AND EAST DECK SLOPE CALCULATION BEGIN */
		// Definitions
		float* zx = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // East deck slope
		float* zy = (float*) malloc(sizeof(float) * SAMPLE_SIZE); // North deck slope

		// Body
		serial_print("Deck slopes calculation starting.\r\n");
		timeit = HAL_GetTick();
		calculate_headings(zx, zy, roll, pitch, bx, by, DONT_PRINT);
		timeit = HAL_GetTick() - timeit;
		sprintf(str, "Heave calculation done in %lums\r\n\r\n", timeit);
		serial_print(str);

		// Closing
		free(bx);
		free(by);
		free(roll);
		free(pitch);
		/* NORTH AND EAST DECK SLOPE CALCULATION END */

		free(zx);
		free(zy);
		free(heave);

		/* USER CODE END WHILE */
		MX_USB_HOST_Process();

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 71;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
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
	hi2c1.Init.Timing = 0x00101A26;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 9600;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

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
	hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG,
	USB_PowerSwitchOn_Pin | SMPS_V1_Pin | SMPS_EN_Pin | SMPS_SW_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OverCurrent_Pin SMPS_PG_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin | SMPS_PG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_PowerSwitchOn_Pin SMPS_V1_Pin SMPS_EN_Pin SMPS_SW_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin | SMPS_V1_Pin | SMPS_EN_Pin | SMPS_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void device_init()
{
	HAL_StatusTypeDef status;

	serial_print("\r\n\r\n\r\n\r\n\r\nNew Session\r\n");

	status = ICM20948_Init(&hi2c1, SDO_LOW);
	if (status != HAL_OK)
	{
		sprintf(str, "Could not initialize ICM20948.\r\nStatus = %d\r\n", status);
		serial_print(str);
		Error_Handler();
	}
	serial_print("Initialized ICM20948.\r\n");

	status = ICM20948_Reset();
	if (status != HAL_OK)
	{
		sprintf(str, "Could not reset ICM20948.\r\nStatus = %d\r\n", status);
		serial_print(str);
		Error_Handler();
	}
	serial_print("Reset ICM20948.\r\n");

	status = ICM20948_Wake();
	if (status != HAL_OK)
	{
		sprintf(str, "Could not wake ICM20948.\r\nStatus = %d\r\n", status);
		serial_print(str);
		Error_Handler();
	}
	serial_print("Woke ICM20948.\r\n");

	status = AK09916_Init(CONT_MEASURE_4);
	if (status != HAL_OK)
	{
		sprintf(str, "Could not find AK09916.\r\nStatus = %d\r\n", status);
		serial_print(str);
		Error_Handler();
	}
	serial_print("Initialized AK09916.\r\n");

	status = init_registers();
	if (status != HAL_OK)
	{
		sprintf(str, "Could initialize ICM20948 registers.\r\nStatus = %d\r\n", status);
		serial_print(str);
		Error_Handler();
	}
	serial_print("Initialized ICM20948 registers.\r\n");
}

HAL_StatusTypeDef init_registers()
{
	HAL_StatusTypeDef status;

	status = ICM20948_WriteRegister(&REG_GYRO_SMPLRT_DIV, (uint8_t) SAMPLE_VALUE);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_CONFIG_1, GYRO_FCHOICE | GYRO_DLPFCFG_6 | GYRO_FS_SEL);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_CONFIG_2, GYRO_AVGCFG_128x);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (SAMPLE_VALUE & 0b111100000000) >> 8);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_SMPLRT_DIV_2, (uint8_t) SAMPLE_VALUE);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_CONFIG_1, ACCEL_FCHOICE | ACCEL_FS_SEL | ACCEL_DLPFCFG_7);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_CONFIG_2, DEC3_CFG_8);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_I2C_MST_ODR_CONFIG, SAMPLE_POWER);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ODR_ALIGN_EN, ODR_ALIGN_EN);
	if (status != HAL_OK)
		return status;

	status = ICM20948_Sleep();
	if (status != HAL_OK)
		return status;

	return ICM20948_Wake();
}

void serial_print(const char* buffer)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) buffer, strlen(buffer), MAXIMUM_PRINT_TIMEOUT);
}

void offset_gyro(int16_vector3* gyro)
{
	gyro->x += gyro_offset.x;
	gyro->y += gyro_offset.y;
	gyro->z += gyro_offset.z;
}

void measure_gyro_offset()
{
	int16_vector3 gyro_off;

	serial_print("Measuring gyroscope offset. Keep device still.\r\n");

	if (ICM20948_MeasureGyroOffset((uint32_t) SAMPLE_SIZE, &gyro_off, SAMPLE_PERIOD_MS) != HAL_OK)
	{
		serial_print("Couldn't measure gyroscope offset.");
		Error_Handler();
	} else
	{
		sprintf(str, "Gyro Offset: %hd, %hd, %hd\r\n", gyro_off.x, gyro_off.y, gyro_off.z);
		serial_print(str);
	}
}

/*
 * Computes the fft of f and stores the result in f
 * size = 2^n where n is a positive non-zero integer
 */
void fft(float complex* f, uint16_t size)
{
	if (size == 2)
	{
		float complex F_even = f[0];
		f[0] = F_even + f[1];
		f[1] = F_even - f[1];
		return;
	}

	uint16_t size_half = size >> 1;

	// Compute even half of f
	float complex* F_even = (float complex*) malloc(sizeof(float complex) * size_half);
	for (uint16_t i = 0; i < size_half; i++)
		F_even[i] = f[2 * i];
	fft(F_even, size_half);

	// Compute odd half of f
	float complex* F_odd = (float complex*) malloc(sizeof(float complex) * size_half);
	for (uint16_t i = 0; i < size_half; i++)
		F_odd[i] = f[2 * i + 1];
	fft(F_odd, size_half);

	// f = fft(f)
	float complex w = cexpf(-M_2PI * I / size);
	for (uint16_t i = 0; i < size_half; i++)
	{
		float complex wi = cpowf(w, i) * F_odd[i];
		f[i] = F_even[i] + wi;
		f[i + size_half] = F_even[i] - wi;
	}
	free(F_even);
	free(F_odd);
}

/*
 * Populates f with the integral of df
 * f is defined to have an average of 0
 * f and df must be of length SAMPLE_SIZE
 */
void integrate(float* f, float* df)
{
	float complex* f_complex = (float complex*) malloc(sizeof(float complex) * SAMPLE_SIZE);
	for (int i = 0; i < SAMPLE_SIZE; i++)
		f_complex[i] = df[i];

	fft(f_complex, SAMPLE_SIZE);

	float* sinf_nk = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
	float* cosf_nk = (float*) malloc(sizeof(float) * SAMPLE_SIZE);

	for (int i = 0; i < SAMPLE_SIZE; i++)
	{
		sinf_nk[i] = sinf(M_2PI_SAMPLE_SIZE * i);
		cosf_nk[i] = cosf(M_2PI_SAMPLE_SIZE * i);
	}

	float f_0 = crealf(f_complex[0]) * M_PI / SAMPLE_SIZE;

	long double sum = 0;

	for (int k = 0; k < SAMPLE_SIZE; k++)
	{
		f[k] = 0;
		for (int n = 1; n < SAMPLE_SIZE / 2; n++)
		{
			uint16_t index = (k * n) % SAMPLE_SIZE;
			f[k] += (crealf(f_complex[n]) * sinf_nk[index] + cimagf(f_complex[n]) * cosf_nk[index]) / n;
		}
		f[k] += f_0 * k;
		f[k] *= M_PI_SAMPLE_FREQUENCY;
		sum += f[k];
	}

	free(f_complex);
	free(sinf_nk);
	free(cosf_nk);

	float average = sum / SAMPLE_SIZE;

	for (int i = 0; i < SAMPLE_SIZE; i++)
		f[i] -= average;
}

void collect_samples(int16_vector3* accel_samples, float* wx, float* wy, float* wz, float* bx, float* by, print_option option)
{
	int16_vector3* gyro_samples = (int16_vector3*) malloc(sizeof(int16_vector3) * SAMPLE_SIZE);
	int16_vector3* mag_samples = (int16_vector3*) malloc(sizeof(int16_vector3) * SAMPLE_SIZE);

	// Read once to get values updating
	ICM20948_ReadAccelGryoRegisters(accel_samples, gyro_samples);
	HAL_Delay(STARTUP_DELAY);

	for (uint16_t i = 0; i < SAMPLE_SIZE; i++)
	{
		uint32_t start_time = HAL_GetTick();

		ICM20948_ReadAccelGryoRegisters(accel_samples + i, gyro_samples + i);

		HAL_Delay(MAG_SAFTEY_WAIT); // Small delay to make sure mag data is ready
		ICM20948_ReadMagRegisters(mag_samples + i);

		int64_t wait_time = SAMPLE_PERIOD_MS + start_time - (int64_t) HAL_GetTick();
		if (wait_time > 0)
			HAL_Delay((uint32_t) wait_time);
	}

	for (uint16_t i = 0; i < SAMPLE_SIZE; i++)
	{
		// Manually corrects sensor and scales to rad/s
		offset_gyro(gyro_samples + i);

		// Standard aircraft roll, pitch, yaw directions
		// Roll is positive when starboard goes down
		// Pitch is positive when nose goes up
		// Yaw is positive when nose goes starboard
		wx[i] = gyro_samples[i].x * GYRO_SENSITIVITY_SCALE_FACTOR;
		wy[i] = -gyro_samples[i].y * GYRO_SENSITIVITY_SCALE_FACTOR;
		wz[i] = -gyro_samples[i].z * GYRO_SENSITIVITY_SCALE_FACTOR;

		// Converts to magnetic flux density [uT]
		bx[i] = mag_samples[i].x * MAG_SENSITIVITY_SCALE_FACTOR;
		by[i] = mag_samples[i].y * MAG_SENSITIVITY_SCALE_FACTOR;
	}

	if (option == PRINT)
	{
		serial_print("AccelX,\tAccelY,\tAccelZ,\tGyroX,\tGyroY,\tGyroZ,\tMagX,\tMagY,\tMagZ\r\n");
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			sprintf(str, "%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d\r\n", accel_samples[i].x, accel_samples[i].y,
			    accel_samples[i].z, gyro_samples[i].x, gyro_samples[i].y, gyro_samples[i].z, mag_samples[i].x, mag_samples[i].y,
			    mag_samples[i].z);
			serial_print(str);
		}
	}

	free(gyro_samples);
	free(mag_samples);
}

void integrate_w(float* roll, float* pitch, float* wx, float* wy, float* wz, print_option option)
{
	integrate(roll, wx);
	integrate(pitch, wy);

	float* droll = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
	float* dpitch = (float*) malloc(sizeof(float) * SAMPLE_SIZE);

	for (uint8_t i = 0; i < INTEGRAL_REPETITIONS; i++)
	{
		for (int j = 0; j < SAMPLE_SIZE; j++)
		{
			droll[j] = wx[j] + tanf(pitch[j]) * (wy[j] * sinf(roll[j]) + wz[j] * cosf(roll[j]));
			dpitch[j] = wy[j] * cosf(roll[j]) - wz[j] * sinf(roll[j]);
		}

		integrate(roll, droll);
		integrate(pitch, dpitch);
	}

	free(droll);
	free(dpitch);

	if (option == PRINT)
	{
		serial_print("\r\nRoll,\t\tPitch,\t\tWx\t\tWy\t\tWz\r\n");
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			sprintf(str, "%f,\t%f,\t%f,\t%f,\t%f\r\n", pitch[i] * 180 / M_PI, roll[i] * 180 / M_PI, wx[i], wy[i], wz[i]);
			serial_print(str);
		}
	}
}

void calculate_headings(float* zx, float* zy, float* roll, float* pitch, float* bx, float* by, print_option option)
{
	if (option == PRINT)
		serial_print("\r\nBx,\t\tBy,\t\tAzimuth\t\tEast Slope,\tNorth Slope\r\n");

	float B_cosD = cosf(B_DECLINATION);
	float B_sinD = sinf(B_DECLINATION);

	for (int i = 0; i < SAMPLE_SIZE; i++)
	{
		float cosP = cosf(pitch[i]);
		float sinP = sinf(pitch[i]);
		float cosR = cosf(roll[i]);
		float sinR = sinf(roll[i]);

		float D = BEY * B_delta * cosP * cosR;

		// Azimuth clockwise from magnetic north
		float sinA = ((B21 * cosP + B22 * sinP * sinR) * (bx[i] - B10) - (B11 * cosP - B12 * sinP * sinR) * (by[i] - B20)
		    - BEZ * B_delta * sinR) / D;
		float cosA = ((B22 * (bx[i] - B10) - B12 * (by[i] - B20)) * cosR - BEZ * B_delta * sinP * cosR) / D;

		// Recalculate for true-north azimuth
		float true_cosA = cosA * B_cosD - sinA * B_sinD;
		float true_sinA = sinA * B_cosD + cosA * B_sinD;

		zx[i] = sinP * true_sinA / cosP - sinR * true_cosA / (cosP * cosR);
		zy[i] = sinP * true_cosA / cosP + sinR * true_sinA / (cosP * cosR);

		if (option == PRINT)
		{
			sprintf(str, "%f,\t%f,\t%f,\t%f,\t%f\r\n", bx[i], by[i], atan2f(true_sinA, true_cosA) * 180 / M_PI, zx[i], zy[i]);
			serial_print(str);
		}
	}
}

void calculate_heave(float* heave, int16_vector3* accel_samples, float* roll, float* pitch, print_option option)
{
	if (option == PRINT)
		serial_print("roll,\t\tpitch\t\t,x,\t\ty,\t\tz,\t\theave\r\n");

	for (int i = 0; i < SAMPLE_SIZE; i++)
	{
		heave[i] = accel_samples[i].x * ACCEL_SENSITIVITY_SCALE_FACTOR * sinf(pitch[i]) * cosf(roll[i])
		    + accel_samples[i].y * ACCEL_SENSITIVITY_SCALE_FACTOR * cosf(pitch[i]) * sinf(roll[i])
		    + accel_samples[i].z * ACCEL_SENSITIVITY_SCALE_FACTOR * cosf(pitch[i]) * cosf(roll[i]);

		if (option == PRINT)
		{
			sprintf(str, "%f,\t%f,\t%f,\t%f,\t%f,\t%f\r\n", roll[i] * 180 / M_PI, pitch[i] * 180 / M_PI,
			    accel_samples[i].x * ACCEL_SENSITIVITY_SCALE_FACTOR, accel_samples[i].y * ACCEL_SENSITIVITY_SCALE_FACTOR,
			    accel_samples[i].z * ACCEL_SENSITIVITY_SCALE_FACTOR, heave[i]);
			serial_print(str);
		}
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
	serial_print("ERROR :(");
	while (1)
	{
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
	sprintf(str, "Wrong parameters value: file %s on line %d\r\n", file, line) */
	serial_print(str);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
