
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define T 10000
#define MODT (uint16_t) ((float)T / 100.0 * 80.0)
#define FI int32_t
#define FPQ 16

/*
#define MAXX 11.4350
#define MINX -9.1053
*/

#define MAXX 12.08
#define MINX -9.5



//---- constants ---//
typedef struct ressler
{

	FI a, b, c, om;
	FI a1, b1, c1, om1;
	FI a2, b2, c2, om2;


	FI cr1; // -1

	FI Y0[3];
	FI h;

	FI c2d3; // 2/3
	FI c1d4; // 1/4
	FI c3d4; // 3/4


	// for DAC
	FI DMAX;
	FI FIMINX;
	FI FIMAXX;
	FI cdac;
} Ressler;
Ressler ress;


FI fadd(FI a, FI b);
FI fsub(FI a, FI b);
FI fmul(FI a, FI b, uint8_t q);
FI fdiv(FI a, FI b, uint8_t q);
FI toFix(double d, uint8_t q);
double toFlt(FI a, uint8_t q);






FI* rosslerX(FI* inX, FI* outX);
FI* mulMatA(FI* mat, FI a);
FI* addMatA(FI* mat, FI a);
FI* addMat(FI* mat, FI* mat2);

uint16_t FiToDAC(FI a);



uint8_t comps = 0;

void initResslerConsts(void)
{
	ress.om1 = toFix(200.0, FPQ);

	double r2 = toFlt(ress.om1, FPQ);

	if (r2 == 200.0)
		comps++;

	ress.a1 =  toFix(0.2, FPQ);
	ress.b1 = toFix(0.2, FPQ);
	ress.c1 = toFix(5.7, FPQ);


		ress.om2 = toFix(200.0, FPQ);
		ress.a2 =  toFix(0.3, FPQ);
		ress.b2 = toFix(0.3, FPQ);
		ress.c2 = toFix(4.0, FPQ);

			ress.om = ress.om1;
			ress.a =  ress.a1;
			ress.b = ress.b1;
			ress.c = ress.c1;

	ress.cr1 = toFix(-1, FPQ);

	ress.Y0[0] = toFix(0.1, FPQ);
	ress.Y0[1] = 0;
	ress.Y0[2] = 0;
	ress.h = toFix(1.0/T, FPQ);

	ress.c2d3 = toFix(2.0/3.0, FPQ);
	ress.c1d4 = toFix(1.0/4.0, FPQ);
	ress.c3d4 = toFix(3.0/4.0, FPQ);


	ress.DMAX = toFix(0xFFF, FPQ);
	ress.FIMINX = toFix(MINX, FPQ);
	ress.FIMAXX = toFix(MAXX, FPQ);
	//ress.cdac = fdiv(fsub(ress.DMAX, ress.FIMINX), ress.FIMAXX, FPQ);

	ress.cdac = fdiv(ress.DMAX, fsub(ress.FIMAXX, ress.FIMINX), FPQ);
}

volatile uint8_t mode = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		if ((mode = !mode)) {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
						ress.om = ress.om2;
						ress.a =  ress.a2;
						ress.b = ress.b2;
						ress.c = ress.c2;
		}
		else {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
						ress.om = ress.om1;
						ress.a =  ress.a1;
						ress.b = ress.b1;
						ress.c = ress.c1;
		}
	}
}




volatile uint16_t count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


if (htim->Instance==TIM1) //check if the interrupt comes from TIM1
	{








								//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

								++count;

								FI K1[3];
								rosslerX(ress.Y0, K1);
								mulMatA(K1, ress.h);

								FI K2[3];
							    rosslerX( addMat(ress.Y0, mulMatA(K1, ress.c2d3)), K2);
							    mulMatA(K2, ress.h);

							    mulMatA(K1, ress.c1d4);
							    mulMatA(K2, ress.c3d4);

							    addMat(ress.Y0, K1);
							    addMat(ress.Y0, K2);

							    uint16_t dacVal = FiToDAC(ress.Y0[0]);
							    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacVal);

							    if (count % (T+1) == 0) {
							    	count = 0;
							    }

				//}

		    /*


	if (count > 0)
		count--;
	if (count == 0) {

		if (val) {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
			//uint8_t rx_buf = 'A';
			//HAL_UART_Transmit(&huart2, &val, 1, 0xFF);
			val = 0;
		}
		else {
						HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
						//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
						//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
						val = 4095;
		}

		count = N;
	}
	*/
}
}


FI* rosslerX(FI* inX, FI* outX)
{

	FI a, b, c;


	if (count % MODT == 0) {
		/*
		a = ress.a >> 1;
		b = ress.b << 1;
		c = ress.c << 2;
		*/
		a = ress.a;
		b = ress.b;
		c = ress.c;
	}
	else {
		a = ress.a;
		b = ress.b;
		c = ress.c;
	}

	outX[0] = fsub(fmul(ress.cr1, inX[1], FPQ), inX[2]);
	outX[1] = fadd(fmul(a, inX[1], FPQ), inX[0]);
	outX[2] = fadd(b, fmul(inX[2], fsub(inX[0], c), FPQ));

	outX[0] = fmul(ress.om, outX[0], FPQ);
	outX[1] = fmul(ress.om, outX[1], FPQ);
	outX[2] = fmul(ress.om, outX[2], FPQ);

	return outX;
}




/*
double arr[] = {
0.099930, 0.099701, 0.099365, 0.098938, 0.098404, 0.097763, 0.097046, 0.096237, 0.095337, 0.094376, 0.093338, 0.092224, 0.091049, 0.089813, 0.088516, 0.087158, 0.085754, 0.084290, 0.082764, 0.081207, 0.079590, 0.077927, 0.076218, 0.074463, 0.072678, 0.070847, 0.068954, 0.067047, 0.065094, 0.063110, 0.061096, 0.059021, 0.056931, 0.054794, 0.052643, 0.050446, 0.048233, 0.045975, 0.043701, 0.041382, 0.039062, 0.036697, 0.034317, 0.031921, 0.029495, 0.027054, 0.024597, 0.022110, 0.019608, 0.017105, 0.014587, 0.012039, 0.009476, 0.006912, 0.004349, 0.001770, -0.000824, -0.003433, -0.006042, -0.008652, -0.011276, -0.013901, -0.016525, -0.019150, -0.021774, -0.024384, -0.026993, -0.029602, -0.032196, -0.034790, -0.037369, -0.039932, -0.042496, -0.045044, -0.047577, -0.050095, -0.052597, -0.055084, -0.057556, -0.059998, -0.062424, -0.064819, -0.067200, -0.069550, -0.071884, -0.074188, -0.076462, -0.078705, -0.080917, -0.083099, -0.085251, -0.087372, -0.089462, -0.091507, -0.093506, -0.095474, -0.097397, -0.099289, -0.101135, -0.102936, -0.104691, -0.106415, -0.108093, -0.109711, -0.111282, -0.112823, -0.114304, -0.115738, -0.117111, -0.118439, -0.119736, -0.120972, -0.122147, -0.123260, -0.124313, -0.125320, -0.126266, -0.127151, -0.127975, -0.128738, -0.129456, -0.130112, -0.130707, -0.131241, -0.131714, -0.132126, -0.132477, -0.132751, -0.132965, -0.133118, -0.133209, -0.133240, -0.133194, -0.133087, -0.132919, -0.132690, -0.132401, -0.132050, -0.131638, -0.131165, -0.130615, -0.130005, -0.129318, -0.128555, -0.127731, -0.126846, -0.125885, -0.124863, -0.123779, -0.122635, -0.121429, -0.120163, -0.118835, -0.117447, -0.115997, -0.114487, -0.112900, -0.111252, -0.109543, -0.107773, -0.105942, -0.104065, -0.102127, -0.100128, -0.098068, -0.095947, -0.093765, -0.091522, -0.089218, -0.086868, -0.084457, -0.082001, -0.079514, -0.076965, -0.074356, -0.071701, -0.069000, -0.066269, -0.063477, -0.060638, -0.057755, -0.054840, -0.051880, -0.048874, -0.045837, -0.042755, -0.039642, -0.036484, -0.033295, -0.030075, -0.026825, -0.023544, -0.020248, -0.016922, -0.013565, -0.010193, -0.006790, -0.003372, 0.000076, 0.003540, 0.007019, 0.010513, 0.014023, 0.017532, 0.021072, 0.024612, 0.028152, 0.031693, 0.035248, 0.038803, 0.042358, 0.045914, 0.049454, 0.052994, 0.056534, 0.060074, 0.063614, 0.067123, 0.070618, 0.074097, 0.077576, 0.081039, 0.084473, 0.087891, 0.091293, 0.094650, 0.097992, 0.101303, 0.104599, 0.107849, 0.111084, 0.114258, 0.117416, 0.120529, 0.123611, 0.126648, 0.129639, 0.132599, 0.135513, 0.138382, 0.141190, 0.143967, 0.146683, 0.149353, 0.151962, 0.154526, 0.157028, 0.159470, 0.161850, 0.164169,
};
*/


//float arr2[10000];
uint16_t arr2[10000];

double merror = 0.24;
uint16_t mmax = 0;
uint16_t mmin = 0xFFF;

double dmax = -12;
double dmin = 12;


uint16_t FiToDAC(FI a) {


	double r1 = toFlt(fmul(ress.cdac, fsub(a, ress.FIMINX), FPQ), FPQ);

	double r2 = toFlt(a, FPQ);


	arr2[count-1] = (uint16_t) r1;

	if (r2 > dmax )
		dmax = r2;
	if (r2 < dmin )
			dmin = r2;

	if (arr2[count-1]  > mmax )
			mmax = arr2[count-1] ;
	if (arr2[count-1]  < mmin )
				mmin = arr2[count-1] ;


	if (count == T) {
			merror = arr2[count-1];
			mmin;
			mmax;
			dmax;
			dmin;
		}

	return (uint16_t) r1;
	//return (uint16_t)( ((double)0xFFF-MINX) / MAXX * (a+MINX));
}




FI* mulMatA(FI* mat, FI a)
{
	//for (uint8_t i = 0; i < 3; ++i)
	//	mat[i] *= a;
	mat[0] = fmul(mat[0], a, FPQ);
	mat[1] = fmul(mat[1], a, FPQ);
	mat[2] = fmul(mat[2], a, FPQ);
	return mat;
}


FI* addMatA(FI* mat, FI a)
{
	//for (uint8_t i = 0; i < 3; ++i)
	//	mat[i] *= a;
	mat[0] = fadd(mat[0], a);
	mat[1] = fadd(mat[1], a);
	mat[2] = fadd(mat[2], a);
	return mat;
}

FI* addMat(FI* mat, FI* mat2)
{
	//for (uint8_t i = 0; i < 3; ++i)
	//	mat[i] *= a;
	mat[0] = fadd(mat[0], mat2[0]);
	mat[1] = fadd(mat[1], mat2[1]);
	mat[2] = fadd(mat[2], mat2[2]);
	return mat;
}



float* plusMatA(float* mat, float a)
{
	for (uint8_t i = 0; i < 3; ++i)
		mat[i] += a;
	return mat;
}

float* plusMatMat(float* mat1, float* mat2)
{
	for (uint8_t i = 0; i < 3; ++i)
		mat1[i] += mat2[i];
	return mat1;
}

uint16_t floatToDAC(float a) {
	return (uint16_t)(0xFFF / 25.0 * (a+10.0));
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	initResslerConsts();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  //MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */



  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start(&htim6);

  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);

  //HAL_UART_Receive_IT(&huart2, &rx_data, 1);



  //HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R, 3000);
  //HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sine, N, DAC_ALIGN_12B_R);

  //__HAL_DAC_ENABLE(&hdac, DAC_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 27999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;//115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}







/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
