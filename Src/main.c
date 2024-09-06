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
#include "nrf24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum print_option
{
	DONT_PRINT,
	PRINT
} print_option;

// The only sample rates where the accel and gyro are synchronized
typedef enum sample_rate
{
	SAMPLE_RATE_25_HZ = 1,
	SAMPLE_RATE_12_5_HZ = 2,
	SAMPLE_RATE_8_33_HZ = 3,
	SAMPLE_RATE_6_25_HZ = 4,
	SAMPLE_RATE_5_HZ = 5
} sample_rate;

typedef enum transmit_size
{
	SIZE_REQUIRED_DATA_ONLY = 5,
	SIZE_SPECTRA_DATA_ONLY = 9,
	SIZE_VALIDATION_DATA = 10, // required + 5 validation parameters
	SIZE_SPECTRA_DATA = 13, // required data + other 8 spectra data
	SIZE_ALL_DATA = 18,
} transmit_size;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAXIMUM_PRINT_TIMEOUT 100 // Maximum time-out to wait for any print ACK
#define MAX_PRINT_LENGTH 100 // Maximum lpuart1 serial data buffer length
#define GRAVITY 9.8
#define GYRO_SAMPLE_RATE_COEFFICIENT 44
#define GYRO_SAMPLE_RATE_MODIFIER 2 // Gyro samples amples twice as fast as accel
#define ACCEL_SAMPLE_RATE_COEFFICIENT 45
#define SAMPLE_RATE_BUFFER 5 // How many ms to wait after sampling before reading
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
const uint16_t SAMPLE_SIZE = 1 << 10; // Must be = 2^n where n is an integer

const sample_rate SAMPLE_RATE = SAMPLE_RATE_12_5_HZ;
// Gyroscope sample rate = 1100 / (1 + value)
const uint8_t GYRO_SAMPLE_RATE_VALUE = SAMPLE_RATE * GYRO_SAMPLE_RATE_COEFFICIENT / GYRO_SAMPLE_RATE_MODIFIER - 1;
// Accelerometer sample rate = 1125 / (1 + value)
const uint8_t ACCEL_SAMPLE_RATE_VALUE = SAMPLE_RATE * ACCEL_SAMPLE_RATE_COEFFICIENT - 1;

// Gyroscope sample period = ((1 + value) / (1100s)) * (1000ms) / (1s)
const uint32_t GYRO_SAMPLE_PERIOD_MS = 10 * (GYRO_SAMPLE_RATE_VALUE + 1) / 11;
const uint32_t SAMPLE_PERIOD_MS = GYRO_SAMPLE_PERIOD_MS * GYRO_SAMPLE_RATE_MODIFIER;
const float SAMPLE_FREQUENCY = 1000.f / SAMPLE_PERIOD_MS;

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
const float M_3PI2 = 3 * M_PI * 0.5f;
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
HAL_StatusTypeDef device_init();
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
void co_spectral_density(float* c, float complex* x, float complex* y);
void quad_spectral_density(float* q, float complex* x, float complex* y);
void HAL_GPIO_EXTI_Callback(uint16_t pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[MAX_PRINT_LENGTH];

transmit_size data_outf_size = SIZE_SPECTRA_DATA;
float** data_outf;
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
	HAL_StatusTypeDef status = device_init();
	if (status != HAL_OK)
	{
		sprintf(str, "Device errored on startup.\r\nStatus = %d\r\n", status);
		serial_print(str);
		Error_Handler();
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	sprintf(str, "Accel scale: %.9f\r\nGyro scale: %.10f\n\rMag scale: %.2f\r\n", ACCEL_SENSITIVITY_SCALE_FACTOR,
	    GYRO_SENSITIVITY_SCALE_FACTOR, MAG_SENSITIVITY_SCALE_FACTOR);
	serial_print(str);

	sprintf(str, "Sample size: %u\r\n", SAMPLE_SIZE);
	serial_print(str);

	sprintf(str, "Sample rate: %lums, %.2f/s\r\n\r\n", SAMPLE_PERIOD_MS, SAMPLE_FREQUENCY);
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


		/* FFT ON RESULTS START */
		// Definitions
		float complex* heave_f = (float complex*) malloc(sizeof(float complex) * SAMPLE_SIZE); // // Earth-fixed heave wrt. freq.
		float complex* zx_f = (float complex*) malloc(sizeof(float complex) * SAMPLE_SIZE); // North deck slope wrt. freq.
		float complex* zy_f = (float complex*) malloc(sizeof(float complex) * SAMPLE_SIZE); // East deck slope wrt. freq.

		// Body
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			heave_f[i] = heave[i];
			zx_f[i] = zx[i];
			zy_f[i] = zy[i];
		}

		fft(heave_f, SAMPLE_SIZE);
		fft(zx_f, SAMPLE_SIZE);
		fft(zy_f, SAMPLE_SIZE);

		// Closing
		free(heave);
		free(zx);
		free(zy);
		/* FFT ON RESULTS END */


		/* CO&QUAD-SPECTRAL DENSITY CALCULATION START */
		// Definitions 1
		float* C11, * C12, * C13, * C22, * C23, * C33, * Q12, * Q13, * Q23;
		C11 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		C12 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		C13 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		Q12 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		Q13 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);

		// Body 1
		co_spectral_density(C11, heave_f, heave_f);
		co_spectral_density(C12, heave_f, zx_f);
		co_spectral_density(C13, heave_f, zy_f);
		quad_spectral_density(Q12, heave_f, zx_f);
		quad_spectral_density(Q13, heave_f, zy_f);

		// Closing 1
		free(heave_f);

		// Definitions 2
		C22 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		C23 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		C33 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		Q23 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);

		// Body 2
		co_spectral_density(C22, zx_f, zx_f);
		co_spectral_density(C23, zx_f, zy_f);
		co_spectral_density(C33, zy_f, zy_f);
		quad_spectral_density(Q23, zx_f, zy_f);

		// Closing 2
		free(zx_f);
		free(zy_f);
		/* CO&QUAD-SPECTRAL DENSITY CALCULATION END */


		/* CO&QUAD-SPECTRAL DENSITY ADJUSTMENTS END */
		// Definitions
		float p = 1;
		float* Q12p = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		float* Q13p = (float*) malloc(sizeof(float) * SAMPLE_SIZE);

		// Body
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			Q12p[i] = Q12[i]*cosf(p) + C12[i]*sinf(p);
			Q13p[i] = Q13[i]*cosf(p) + C13[i]*sinf(p);
		}
		/* CO&QUAD-SPECTRAL DENSITY ADJUSTMENTS END */


		/* MAIN PARAMETER CALCULATIONS START */
		// Definitions
		float* r1 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		float* a1 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		float* r2 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
		float* a2 = (float*) malloc(sizeof(float) * SAMPLE_SIZE);

		// Body
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			r1[i] = powf((Q12p[i] * Q12p[i] + Q13p[i] * Q13p[i]) / (C11[i] * (C22[i] + C33[i])), 0.5f);
			a1[i] = M_3PI2 - atan2f(Q13p[i], Q12p[i]);
			r2[i] = powf(powf(C22[i] - C33[i], 2) + 4 * C23[i] * C23[i], 0.5f) / (C22[i] + C33[i]);
			a2[i] = -M_3PI2 - 0.5f * atan2f(2 * C23[i], C22[i] - C33[i]); // + either 0 or pi...
			// ??? whichever makes the angle between a1 and a2 ???
		}

		// Closing
		free(Q12p);
		free(Q13p);
		/* MAIN PARAMETER CALCULATIONS END */


		/* TRANSMIT DATA START */
		float* temp[SIZE_ALL_DATA] = {NULL, NULL, NULL, NULL, NULL, r1, a1, r2, a2, C11, C22, C33, C23, Q12, C12, Q13, C13, Q23};

		uint8_t offset;
		if (data_outf_size == SIZE_VALIDATION_DATA || data_outf_size == SIZE_ALL_DATA)
			offset = 0;
		else
			offset = SIZE_VALIDATION_DATA - SIZE_REQUIRED_DATA_ONLY;

		data_outf = malloc(sizeof(float*) * data_outf_size);
		memcpy(data_outf, temp + offset, data_outf_size);

		nRF24_SetPowerMode(nRF24_PWR_UP); // wake transceiver

		// transmit first piece of data

		// Closing
		free(C11);
		free(C12);
		free(C13);
		free(Q12);
		free(Q13);
		free(C22);
		free(C23);
		free(C33);
		free(Q23);
		free(r1);
		free(a1);
		free(r2);
		free(a2);
		free(data_outf);
		/* TRANSMIT DATA END */

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
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin | SMPS_V1_Pin | SMPS_EN_Pin | SMPS_SW_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF24_IRQ_Pin */
	GPIO_InitStruct.Pin = NRF24_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF24_CE_Pin NRF24_CSN_Pin */
	GPIO_InitStruct.Pin = NRF24_CE_Pin | NRF24_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
HAL_StatusTypeDef device_init()
{
	HAL_StatusTypeDef status;

	serial_print("\r\n\r\n\r\n\r\n\r\nNew Session\r\n");


	serial_print("Initializing ICM20948.\r\n");
	status = ICM20948_Init(&hi2c1, SDO_LOW);
	if (status != HAL_OK)
		return status;

	status = ICM20948_Reset();
	if (status != HAL_OK)
		return status;

	status = ICM20948_Wake();
	if (status != HAL_OK)
		return status;
	serial_print("Initialized.\r\n");


	serial_print("Initializing AK09916.\r\n");
	status = AK09916_Init(SINGLE_MEASURE);
	if (status != HAL_OK)
		return status;
	serial_print("Initialized.\r\n");


	serial_print("Initializing ICM20948 registers.\r\n");
	status = init_registers();
	if (status != HAL_OK)
		return status;
	serial_print("Initialized.\r\n");


	serial_print("Initializing nRF24.\r\n");
	nRF24_HAL_Init(&hspi1);

	nRF24_Init();
	if (!nRF24_Check())
		return HAL_ERROR;
	serial_print("Initialized.\r\n");


	serial_print("Configuring nRF24.\r\n");
	uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the TX address
	nRF24_SetRFChannel(24); // set RF channel to 2424MHz
	nRF24_SetDataRate(nRF24_DR_1Mbps); // 1Mbit/s data rate
	nRF24_SetCRCScheme(nRF24_CRC_2byte); // 2-byte CRC scheme
	nRF24_SetAddrWidth(5); // address width is 5 bytes
	nRF24_SetAddr(nRF24_PIPETX, ADDR); // program TX address
	nRF24_SetAddr(nRF24_PIPE0, ADDR); // program pipe#0 RX address, must be same as TX (for ACK packets)
	nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power
	nRF24_SetAutoRetr(nRF24_ARD_1000us, 10); // configure auto retransmit: 10 retransmissions with pause of 1000us in between
	nRF24_EnableAA(nRF24_PIPE0); // enable Auto-ACK for pipe#0 (for ACK packets)
	nRF24_SetOperationalMode(nRF24_MODE_TX); // switch transceiver to the TX mode
	nRF24_SetPowerMode(nRF24_PWR_DOWN); // put transceiver to sleep
	// the nRF24 is ready for transmission, upload a payload, then pull CE pin to HIGH and it will transmit a packet...
	serial_print("Configured.\r\n");

	return HAL_OK;
}

HAL_StatusTypeDef init_registers()
{
	HAL_StatusTypeDef status;

	status = ICM20948_WriteRegister(&REG_INT_PIN_CFG, FSYNC_INT_MODE_EN);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_FSYNC_CONFIG, EXT_SYNC_GYRO_XOUT_L | DELAY_TIME_EN);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_SMPLRT_DIV, GYRO_SAMPLE_RATE_VALUE);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_CONFIG_1, GYRO_FCHOICE | GYRO_DLPFCFG_6 | GYRO_FS_SEL);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_GYRO_CONFIG_2, GYRO_AVGCFG_128x);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_SMPLRT_DIV_2, ACCEL_SAMPLE_RATE_VALUE);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_CONFIG_1, ACCEL_FCHOICE | ACCEL_FS_SEL | ACCEL_DLPFCFG_7);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ACCEL_CONFIG_2, DEC3_CFG_8);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_ODR_ALIGN_EN, ODR_ALIGN_EN);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_I2C_SLV0_CTRL, 0);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_I2C_SLV0_ADDR, AK09916_ADDR_READ);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_I2C_SLV0_REG, HXL);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_I2C_SLV4_ADDR, AK09916_ADDR_WRITE);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_I2C_SLV4_REG, CNTL2);
	if (status != HAL_OK)
		return status;

	status = ICM20948_WriteRegister(&REG_I2C_SLV4_DO, SINGLE_MEASURE);
	if (status != HAL_OK)
		return status;

	return ICM20948_Sleep();
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

	// Undefined for first index
	float* f_real = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
	float* f_imag = (float*) malloc(sizeof(float) * SAMPLE_SIZE);
	float* n_inv = (float*) malloc(sizeof(float) * SAMPLE_SIZE / 2);

	sinf_nk[0] = 0;
	cosf_nk[0] = 1;

	for (int i = 1; i < SAMPLE_SIZE; i++)
	{
		sinf_nk[i] = sinf(M_2PI_SAMPLE_SIZE * i);
		cosf_nk[i] = cosf(M_2PI_SAMPLE_SIZE * i);
		f_real[i] = crealf(f_complex[i]);
		f_imag[i] = cimagf(f_complex[i]);
		n_inv[i] = 1.f / i;
	}

	free(f_complex);

	float f_0 = crealf(f_complex[0]) * M_PI / SAMPLE_SIZE;

	long double sum = 0;

	for (int k = 0; k < SAMPLE_SIZE; k++)
	{
		f[k] = 0;
		uint16_t index = 0;
		for (int n = 1; n < SAMPLE_SIZE / 2; n++)
		{
			index += k;
			if (index >= SAMPLE_SIZE)
				index -= SAMPLE_SIZE;
			f[k] += (f_real[n] * sinf_nk[index] + f_imag[n] * cosf_nk[index]) * n_inv[n];
		}
		f[k] += f_0 * k;
		f[k] *= M_PI_SAMPLE_FREQUENCY;
		sum += f[k];
	}

	free(sinf_nk);
	free(cosf_nk);
	free(f_real);
	free(f_imag);
	free(n_inv);

	float average = sum / SAMPLE_SIZE;

	for (int i = 0; i < SAMPLE_SIZE; i++)
		f[i] -= average;
}

void collect_samples(int16_vector3* accel_samples, float* wx, float* wy, float* wz, float* bx, float* by, print_option option)
{
	int16_vector3* gyro_samples = (int16_vector3*) malloc(sizeof(int16_vector3) * SAMPLE_SIZE);
	int16_vector3* mag_samples = (int16_vector3*) malloc(sizeof(int16_vector3) * SAMPLE_SIZE);

	ICM20948_Wake();

	// Find when gyro odr events happen
	float delay_ms;
	uint32_t fsync_time;
	do
	{
		// Send fsync event
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		fsync_time = HAL_GetTick();

		HAL_Delay(GYRO_SAMPLE_PERIOD_MS);

		// Read delay between fsync event and gyro_odr
		uint8_t data[2];
		ICM20948_ReadRegisters(&REG_DELAY_TIMEH, data, 2);
		delay_ms = (data[0] * 256 + data[1]) * 0.0009645f;

	} while (delay_ms > 1);

	// Find when accel odr events happen
	HAL_Delay(SAMPLE_PERIOD_MS - (HAL_GetTick() - fsync_time) + SAMPLE_RATE_BUFFER);
	uint32_t start_time = HAL_GetTick();

	ICM20948_ReadAccelRegisters(accel_samples);

	HAL_Delay(GYRO_SAMPLE_PERIOD_MS + start_time - HAL_GetTick());
	start_time = HAL_GetTick();

	ICM20948_ReadAccelRegisters(accel_samples + 1);

	if (accel_samples[0].x != accel_samples[1].x)
		HAL_Delay(GYRO_SAMPLE_PERIOD_MS + start_time - HAL_GetTick());
	else
		HAL_Delay(SAMPLE_PERIOD_MS + start_time - HAL_GetTick());

	start_time = HAL_GetTick();
	ICM20948_WriteRegister(&REG_I2C_SLV4_CTRL, I2C_SLV_EN | I2C_SLV_LENG_1);

	for (uint16_t i = 0; i < SAMPLE_SIZE; i++)
	{
		HAL_Delay(GYRO_SAMPLE_PERIOD_MS + start_time - HAL_GetTick()); // Sample wait
		start_time = HAL_GetTick();

		// Set magnetometer to read sensor
		ICM20948_WriteRegister(&REG_I2C_SLV0_CTRL, I2C_SLV_EN | I2C_SLV_LENG_8);

		HAL_Delay(GYRO_SAMPLE_PERIOD_MS + start_time - HAL_GetTick()); // Read wait
		start_time = HAL_GetTick();

		// Read Sensors
		ICM20948_ReadAccelGryoRegisters(accel_samples + i, gyro_samples + i);
		ICM20948_ReadMagRegisters(mag_samples + i);

		// Set magnetometer to sample sensor
		ICM20948_WriteRegister(&REG_I2C_SLV4_CTRL, I2C_SLV_EN | I2C_SLV_LENG_1);
		ICM20948_WriteRegister(&REG_I2C_SLV0_CTRL, 0);
	}

	ICM20948_Sleep();

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

void co_spectral_density(float* c, float complex* x, float complex* y)
{
	for (int i = 0; i < SAMPLE_SIZE; i++)
		c[i] = 1000 * (crealf(x[i]) * crealf(y[i]) + cimagf(x[i]) * cimagf(y[i])) / (SAMPLE_SIZE * SAMPLE_PERIOD_MS);
}

void quad_spectral_density(float* q, float complex* x, float complex* y)
{
	for (int i = 0; i < SAMPLE_SIZE; i++)
		q[i] = 1000 * (cimagf(x[i]) * crealf(y[i]) - crealf(x[i]) * cimagf(y[i])) / (SAMPLE_SIZE * SAMPLE_PERIOD_MS);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
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
