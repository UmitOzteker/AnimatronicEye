/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG1_PIN GPIO_PIN_6
#define TRIG1_PORT GPIOA
#define ECHO1_PIN GPIO_PIN_4
#define ECHO1_PORT GPIOA

#define TRIG2_PIN GPIO_PIN_7
#define TRIG2_PORT GPIOA
#define ECHO2_PIN GPIO_PIN_12
#define ECHO2_PORT GPIOA
#define SW_Pin GPIO_PIN_8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
volatile uint32_t start_time_1 = 0;
volatile uint32_t stop_time_1 = 0;
volatile uint8_t measurement_done_1 = 0;
float distance_cm_1 = 0;

volatile uint32_t start_time_2 = 0;
volatile uint32_t stop_time_2 = 0;
volatile uint8_t measurement_done_2 = 0;
float distance_cm_2 = 0;

typedef enum {
	STATE_RANDOM_MOVEMENT, STATE_MANUAL_CONTROL, SOLA_BAK, SAGA_BAK
} ControlState_TypeDef;

volatile ControlState_TypeDef current_state = STATE_RANDOM_MOVEMENT;

typedef struct {
	float sol;
	float sag;
} MesafeSonucu;

// ADC DMA Buffer (ADC 12-bit ise uint16_t)
#define ADC_NUM_CHANNELS 2
volatile uint16_t adc_dma_buffer[ADC_NUM_CHANNELS]; // X için [0], Y için [1]

// İşlenmiş (ham) ADC değerleri
volatile uint16_t adc_raw_x = 0; // DMA buffer'dan okunan değerler uint16_t
volatile uint16_t adc_raw_y = 0;

// Servo için hesaplanmış PWM değerleri
uint16_t pwm_x_value = 1500;
uint16_t pwm_y_value = 1500;

// Kullanıcı etkileşimi
uint32_t last_significant_user_input_time = 0;
const uint32_t IDLE_TIMEOUT_TO_RANDOM_MS = 1000;

// Değişiklik tespiti
uint16_t prev_adc_x_for_detect = 0; // ADC değerleri uint16_t olduğu için
uint16_t prev_adc_y_for_detect = 0;
const uint16_t ADC_NOISE_THRESHOLD = 200; // uint16_t ADC değerleri için eşik

volatile int joystick_pressed;

////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc_ptr) {
	// Bu callback DMA tarafından tetiklenir (ADC->DMA ilişkisi doğru ayarlandıysa
	// ve ADC Continuous + DMA Circular modda ise)
	if (hadc_ptr->Instance == ADC1) { // ADC1 kullandığınızı varsayıyorum
		// DMA buffer'dan değerleri global değişkenlere al
		adc_raw_x = adc_dma_buffer[0]; // İlk kanal (Rank 1 - X ekseni)
		adc_raw_y = adc_dma_buffer[1]; // İkinci kanal (Rank 2 - Y ekseni)

		// Joystick'te anlamlı bir değişiklik var mı kontrol et
		// abs() için <stdlib.h> gerekir
		uint8_t x_changed = abs(
				(int16_t) adc_raw_x - (int16_t) prev_adc_x_for_detect)
				> ADC_NOISE_THRESHOLD;
		uint8_t y_changed = abs(
				(int16_t) adc_raw_y - (int16_t) prev_adc_y_for_detect)
				> ADC_NOISE_THRESHOLD;

		if (x_changed || y_changed) {
			if (x_changed) {
				prev_adc_x_for_detect = adc_raw_x;
			}
			if (y_changed) {
				prev_adc_y_for_detect = adc_raw_y;
			}

			current_state = STATE_MANUAL_CONTROL;
			last_significant_user_input_time = HAL_GetTick();
			pwm_x_value = ((adc_raw_x * 1100UL) / 4095UL) + 1300;
			if (pwm_x_value < 1300)
				pwm_x_value = 1300;
			if (pwm_x_value > 2400)
				pwm_x_value = 2400;

			// Y ekseni için PWM hesapla (1200-1500 aralığı)
			// Maksimum PWM - Minimum PWM = 1500 - 1200 = 300
			pwm_y_value = ((adc_raw_y * 550UL) / 4095UL) + 900;
			if (pwm_y_value < 900)
				pwm_y_value = 900;
			if (pwm_y_value > 1450)
				pwm_y_value = 1450;
		}
		// ADC sürekli modda ve DMA circular modda olduğu için,

	}
}

void trigger_sensor(GPIO_TypeDef *port, uint16_t pin) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // Sayaç sıfırla
	HAL_TIM_Base_Start(&htim1);        // Timer başlat

	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		; // Belirtilen süreyi bekle

	HAL_TIM_Base_Stop(&htim1);         // Timer durdur (isteğe bağlı)
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static uint32_t last_interrupt_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (GPIO_Pin == SW_Pin && (current_time - last_interrupt_time > 50)) // 50ms debounce
			{
		if (HAL_GPIO_ReadPin(GPIOA, SW_Pin) == GPIO_PIN_RESET) {
			joystick_pressed = 1;
		}
		last_interrupt_time = current_time;
	}

	if (GPIO_Pin == ECHO1_PIN) {
		if (HAL_GPIO_ReadPin(ECHO1_PORT, ECHO1_PIN) == GPIO_PIN_SET) {
			__HAL_TIM_SET_COUNTER(&htim17, 0);
			start_time_1 = __HAL_TIM_GET_COUNTER(&htim17);
		} else {
			stop_time_1 = __HAL_TIM_GET_COUNTER(&htim17);
			//HAL_TIM_Base_Stop(&htim17);
			measurement_done_1 = 1;
		}
	}
}

uint16_t get_random_positionX(void) {
	static const uint16_t values[] = { 1300, 1400, 1500, 1600, 1700, 1800, 1900,
			2000, 2100, 2200, 2300, 2400 };
	const uint8_t num_values = sizeof(values) / sizeof(values[0]);

	// rand() % num_values ifadesi 0 ile num_values-1 arasında sayı üretir
	uint8_t index = rand() % num_values;

	return values[index];
}

uint16_t get_random_positionY(void) {
	static const uint16_t values[] = { 900, 950, 1000, 1050, 1100, 1150, 1200,
			1250, 1300, 1350, 1400, 1450 };
	const uint8_t num_values = sizeof(values) / sizeof(values[0]);

	// rand() % num_values ifadesi 0 ile num_values-1 arasında sayı üretir
	uint8_t index = rand() % num_values;

	return values[index];
}

uint8_t get_random_time(void) {
	return (rand() % 1) + 1;
}

void movement(void) {
	TIM3->CCR3 = get_random_positionX();
	HAL_Delay(get_random_time() * 700);

	TIM3->CCR4 = get_random_positionY();
	HAL_Delay(get_random_time() * 700);

	trigger_sensor(TRIG1_PORT, TRIG1_PIN);

			if (measurement_done_1) {
				measurement_done_1 = 0;
				distance_cm_1 = (stop_time_1 - start_time_1) / 58.0f;
			}



}

void eyelid_movement() {
	TIM3->CCR1 = 1800; // Sağ üst göz
	TIM2->CCR4 = 1200; // Sol üst göz
	TIM3->CCR2 = 1550; // Sağ Alt
	TIM2->CCR3 = 800; // Sol alt

	HAL_Delay(220);
	TIM3->CCR1 = 2250;
	TIM2->CCR4 = 650;
	TIM3->CCR2 = 1250; // Sağ Alt
	TIM2->CCR3 = 1100;
	HAL_Delay(220);
}

void blink() {
	int random = (rand() % 2) + 1;
	if (random == 1) {
		TIM3->CCR1 = 1800; // Sağ üst göz
		TIM3->CCR2 = 1550; // Sağ Alt
		HAL_Delay(220);
		TIM3->CCR1 = 2250;
		TIM3->CCR2 = 1250; // Sağ Alt
	} else {
		TIM2->CCR4 = 1200; // Sol üst göz
		TIM2->CCR3 = 800; // Sol alt
		HAL_Delay(220);
		TIM2->CCR4 = 650;
		TIM2->CCR3 = 1100;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_ADC_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM14_Init();
	MX_TIM16_Init();
	MX_TIM17_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	// MESAFE İÇİN
	//HAL_TIM_Base_Start(&htim2); // Timer başlatılıyor
	// TRIG pinini bir defa sıfırla
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(50); // Stabilite için küçük bir gecikme
	/////////////////////////////////////
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // X ekseni servosu için timer başlatılır.
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_Base_Start(&htim16);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim1);

	HAL_TIM_Base_Start(&htim17);

	srand(HAL_GetTick());

	// İlk ADC çevrimini başlat
	/*if (HAL_ADC_Start_IT(&hadc) != HAL_OK) {
	 Error_Handler();
	 }*/

	// ADC kalibrasyonu
	if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) { // hadc1 sizin ADC handle'ınız
		Error_Handler();
	}

	// ADC'yi DMA ile başlat
	if (HAL_ADC_Start_DMA(&hadc, (uint16_t*) adc_dma_buffer, ADC_NUM_CHANNELS)
			!= HAL_OK) {
		Error_Handler();
	}

	// İlk değerlerin callback tarafından atanmasını bekle ve prev değerleri ayarla
	HAL_Delay(10); // Callback'in en az bir kez çalışması için kısa bekleme
	prev_adc_x_for_detect = adc_raw_x; // Callback global değişkenleri güncelledi
	prev_adc_y_for_detect = adc_raw_y;

	last_significant_user_input_time = HAL_GetTick();
	current_state = STATE_RANDOM_MOVEMENT;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1800); // X servosu ortaya
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1350); // Y SERVOSU
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2200);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1200); // Sağ alt AÇIK
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2250); // SAĞ ÜST
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 650); // SOL ÜST
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1100); // SOL ALT

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t last_distance_measure_time = 0;
	const uint32_t distance_measure_interval = 150;
	uint32_t last_trigger_time = 0;
	uint8_t current_sensor = 1;

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (joystick_pressed) {
						joystick_pressed = 0;
						blink();
					}

		switch (current_state) {
		case STATE_MANUAL_CONTROL:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_x_value);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_y_value);

			if (HAL_GetTick() - last_significant_user_input_time
					> IDLE_TIMEOUT_TO_RANDOM_MS) {
				current_state = STATE_RANDOM_MOVEMENT;
			}
			break;

		case STATE_RANDOM_MOVEMENT:
			movement();
			if (rand() % 5 == 0) { // %20 ihtimalle göz kırp
				eyelid_movement();
			}

			break;
		case SOLA_BAK:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1300);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1450);
			//current_state = STATE_RANDOM_MOVEMENT;
			break;
		case SAGA_BAK:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2400);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1450);
			//current_state = STATE_RANDOM_MOVEMENT;
			break;

		default:
			current_state = STATE_RANDOM_MOVEMENT;
			break;
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 15;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 15;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 15;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 19999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 15;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 65535;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 15;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 65535;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 15;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 0xffff;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA4 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA6 PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
