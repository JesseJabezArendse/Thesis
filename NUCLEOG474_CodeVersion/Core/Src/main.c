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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <fastmath.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t sampleRate = 1;
uint8_t currFreq = 9;

#define BUFFER_SIZE             180

#define PI                      3.141592654

#define NUM_FREQS               41
#define DISTURBANCE_AMPLITUDE   0.05
#define MAX_DISTURBANCE         0.0577 // maximum disturbance possible [is intended to be 0.06]
#define DAC_RESOLUTION          2300   // effectively
#define ADC_RESOLUTION          4095  // 12-bit
#define MAX_SAMPLES             BUFFER_SIZE + 1    // Maximum number of samples
#define NUMBER_OF_PULSES        5

#define VREF                    3.31
#define CORDIC_SCALE_FACTOR 2147483648.0f  // 2^31 for Q1.31 format

#define R_SHUNT                 0.0957

typedef struct {
    float buffer[BUFFER_SIZE];  // Buffer to hold float data
    int head;                   // Index of the next write position
    int count;                  // Count of the current number of samples
    float global_max;           // Global maximum value over all samples
    float global_min;           // Global minimum value over all samples
    float sum;                  // Running sum of all samples (for running mean)
    
    // Local min/max/mean for the latest 2000 values
    float local_max;            // Local maximum value in the buffer
    float local_min;            // Local minimum value in the buffer
    float local_mean;           // Local mean of the buffer
} DataBuffer;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

CORDIC_HandleTypeDef hcordic;
DMA_HandleTypeDef hdma_cordic_read;
DMA_HandleTypeDef hdma_cordic_write;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;
DMA_HandleTypeDef hdma_dac1_ch1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;

OPAMP_HandleTypeDef hopamp6;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_OPAMP6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t ADCValue_V_Batt = 0;
uint32_t ADCValue_I_Shunt = 0;
int16_t ADC_V_Buffer[BUFFER_SIZE];
int16_t ADC_I_Buffer[BUFFER_SIZE];

float V_Batt = 0.0f;
float I_Batt = 0.0f;

DataBuffer V_buffer;
DataBuffer I_buffer;
DataBuffer Mag_buffer;
DataBuffer Phase_buffer;

uint32_t sineWave[MAX_SAMPLES];  // Array to hold sine wave samples

float All_Mag[NUM_FREQS];
float All_Phase[NUM_FREQS];
float V_mag, I_mag;

float prevFreq = -1;

uint8_t finishedSweeping = 0;

float array_max(float arr[]) {
  int size = BUFFER_SIZE;
    // Initialize max with the first element
    float max_val = arr[0];

    // Iterate through the array to find the max value
    for (int i = 1; i < size; i++) {
        if (arr[i] > max_val) {
            max_val = arr[i];
        }
    }
    
    return max_val;
}

float array_min(float arr[]) {
  int size = BUFFER_SIZE;
    // Initialize min with the first element
    float min_val = arr[0];

    // Iterate through the array to find the min value
    for (int i = 1; i < size; i++) {
        if (arr[i] < min_val) {
            min_val = arr[i];
        }
    }
    
    return min_val;
}


void initBuffer(DataBuffer *buf) {
    memset(buf->buffer, 0, sizeof(buf->buffer));  // Clear the buffer
    buf->head = 0;
    buf->count = 0;
    buf->global_max = -0;  // Initialize global max to the lowest possible float
    buf->global_min = 6;   // Initialize global min to the highest possible float
    buf->sum = 0;                // Initialize the sum to 0
    buf->local_max = -0;   // Initialize local max
    buf->local_min = 6;    // Initialize local min
    buf->local_mean = 0;         // Initialize local mean
}

void addSample(DataBuffer *buf, float sample) {
    // If the buffer is full, we need to remove the oldest sample
    if (buf->count == BUFFER_SIZE) {
        float oldest_sample = buf->buffer[buf->head];  // Get the oldest sample
        buf->sum -= oldest_sample;  // Subtract the oldest sample from the sum

      // sample = (buf->buffer[buf->head]) +  sample/(NUMBER_OF_PULSES);
      // buf->buffer[buf->head] = sample;


        // Check if we need to recalculate local max and min
        if (oldest_sample == buf->local_max || oldest_sample == buf->local_min) {
            buf->local_max = -0;  // Reset local max
            buf->local_min = 6;   // Reset local min

            // Recalculate local max and min by iterating through the buffer
            for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
                if (buf->buffer[i] > buf->local_max) {
                    buf->local_max = buf->buffer[i];
                }
                if (buf->buffer[i] < buf->local_min) {
                    buf->local_min = buf->buffer[i];
                }
            }
        }
    } else {
        buf->count++;  // Increment the count until the buffer is full
    }

    sample = (buf->buffer[buf->head]) +  sample/(NUMBER_OF_PULSES);
    buf->buffer[buf->head] = sample;

        // Add the new sample
    buf->sum += sample;  // Add the new sample to the sum
    buf->local_mean = buf->sum / buf->count;  // Calculate the local mean
    buf->head = (buf->head + 1) % BUFFER_SIZE;  // Move head forward
}

int16_t floatToCordicInput(float value) {
    // Scale the float value to Q1.31 fixed-point format and cast to int32_t
    int16_t scaledValue = (int16_t)(value * CORDIC_SCALE_FACTOR);
    return scaledValue;
}

float cordicOutputToFloat(int32_t cordicOutput) {
    // Convert Q1.31 fixed-point output to float
    float result = (float)cordicOutput / CORDIC_SCALE_FACTOR;
    return result;
}

// Function to compute the mag using CORDIC
void cordicMag(int16_t x, int16_t y){

  CORDIC_ConfigTypeDef sConfig = {0};
  sConfig.Function = CORDIC_FUNCTION_MODULUS;       // Set function to arctangent
  sConfig.Precision = CORDIC_PRECISION_6CYCLES;   // Set precision (e.g., 6 cycles)
  sConfig.Scale = CORDIC_SCALE_0;                 // No scaling
  sConfig.NbWrite = CORDIC_NBWRITE_2;             // Two inputs (y and x)
  sConfig.NbRead = CORDIC_NBREAD_1;               // Single output (arctangent)
  sConfig.InSize = CORDIC_INSIZE_16BITS;          // Input size (16-bit)
  sConfig.OutSize = CORDIC_OUTSIZE_32BITS;        // Output size (16-bit)
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CORDIC_Configure(&hcordic,&sConfig);

    int32_t cordicOutput;
    int16_t cordicInput[2];
    cordicInput[0] = x;
    cordicInput[1] = y;

    // Calculate phases using Zero-Overhead (blocking mode)
    HAL_CORDIC_CalculateZO(&hcordic, &cordicInput, &cordicOutput, 1 , HAL_MAX_DELAY);
    float mag = cordicOutputToFloat(cordicOutput);

    // add resulting cordic to buffer
    addSample(&Mag_buffer, mag);
}

float arm_phase(float in , float mag){
  return (float) (acos(-2*(in/mag)+1));
}

// Function to compute the phase using CORDIC
float cordicPhase(int16_t x, int16_t y){
  CORDIC_ConfigTypeDef sConfig = {0};
  sConfig.Function = CORDIC_FUNCTION_PHASE;       // Set function to arctangent
  sConfig.Precision = CORDIC_PRECISION_6CYCLES;   // Set precision (e.g., 6 cycles)
  sConfig.Scale = CORDIC_SCALE_0;                 // No scaling
  sConfig.NbWrite = CORDIC_NBWRITE_2;             // Two inputs (y and x)
  sConfig.NbRead = CORDIC_NBREAD_1;               // Single output (arctangent)
  sConfig.InSize = CORDIC_INSIZE_16BITS;          // Input size (16-bit)
  sConfig.OutSize = CORDIC_OUTSIZE_32BITS;        // Output size (16-bit)
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CORDIC_Configure(&hcordic,&sConfig);

    int32_t cordicOutput;
    int16_t cordicInput[2];
    cordicInput[0] = x;
    cordicInput[1] = y;

    // Calculate phases using Zero-Overhead (blocking mode)
    HAL_CORDIC_CalculateZO(&hcordic, &cordicInput, &cordicOutput, 1 , HAL_MAX_DELAY);
    float phase = cordicOutputToFloat(cordicOutput) * PI;

    // add resulting cordic to buffer
    
    return phase;
}

const float frequencies[NUM_FREQS] = {
0.1,
0.2,
0.3,
0.4,
0.5,
0.6,
0.7,
0.8,
0.9,
1,
2,
3,
4,
5,
6,
7,
8,
9,
10,
20,
30,
40,
50,
60,
70,
80,
90,
100,
200,
300,
400,
500,
600,
700,
800,
900,
1000,
2000,
3000,
4000,
5000
};


volatile uint32_t psc;
volatile uint32_t arr;

ADC_ChannelConfTypeDef ADC_config_V;
ADC_ChannelConfTypeDef ADC_config_I;

const uint32_t ADC_SamplingTime_confs[8] = {
ADC_SAMPLETIME_2CYCLES_5  ,
ADC_SAMPLETIME_6CYCLES_5  ,
ADC_SAMPLETIME_12CYCLES_5 ,
ADC_SAMPLETIME_24CYCLES_5 ,
ADC_SAMPLETIME_47CYCLES_5 ,
ADC_SAMPLETIME_92CYCLES_5 ,
ADC_SAMPLETIME_247CYCLES_5,
ADC_SAMPLETIME_640CYCLES_5  
};

const float ADC_SamplingTime_floats[8] = {
2.5  ,
6.5  ,
12.5 ,
24.5 ,
47.5 ,
92.5 ,
247.5,
640.5
};



void configureTimer(TIM_HandleTypeDef *htim, float desired_frequency) {
    // Assuming the clock frequency is 60 MHz
    float clock_frequency = 60000000.0f;  // 60 MHz
    float timer_frequency = clock_frequency / desired_frequency;  // Timer ticks per desired frequency
    float time_per_sample = 1.0f / (desired_frequency * BUFFER_SIZE);  // Time per ADC sample

    // Set timer prescaler and auto-reload register
    TIM2->PSC = (12 * 1000 / BUFFER_SIZE);  // Example PSC setting (can be modified based on requirements)
    TIM2->ARR = (uint32_t)round(10000 / desired_frequency);  // Set the timer auto-reload value

    // ADC sampling time setup
    uint32_t selected_sampling_time = ADC_SAMPLETIME_640CYCLES_5;  // Default minimum sample time
    for (int i = 7; i >= 0; i--) {  // Start from the longest sampling time
        float adc_sample_time_cycles = ADC_SamplingTime_floats[i];
        float adc_sample_time =  (adc_sample_time_cycles-16.5) / clock_frequency;  // Convert cycles to seconds

        // Check if this sample time fits within the time per sample
        if (adc_sample_time < time_per_sample) {
            selected_sampling_time = ADC_SamplingTime_confs[i];  // Use the longest possible sample time
            break;
        }
    }

    HAL_ADC_DeInit(&hadc3);
    MX_ADC3_Init();
    HAL_ADC_DeInit(&hadc1);
    MX_ADC1_Init();

    ADC_config_V.SamplingTime = selected_sampling_time;
    ADC_config_I.SamplingTime = selected_sampling_time;
    if (HAL_ADC_ConfigChannel(&hadc3, &ADC_config_V) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_ConfigChannel(&hadc1, &ADC_config_I) != HAL_OK) {
        Error_Handler();
    }
    HAL_ADC_Start_DMA(&hadc3,&ADC_V_Buffer,BUFFER_SIZE);  // start getting V BATT with DMA
    HAL_ADC_Start_DMA(&hadc1,&ADC_I_Buffer,BUFFER_SIZE);  // start getting V BATT with DMA
}

// Function to generate a sine wave and configure the DAC with DMA
void GenerateSineWave(float frequency) {

    // 1. Calculate the sample rate based on the requested frequency
    const uint32_t systemClock = SystemCoreClock;  // System clock frequency
    const uint32_t numSamples = round(MAX_SAMPLES)-1;  // Number of samples per sine wave cycle

    // Sample rate to generate the sine wave
    sampleRate = frequency * numSamples;  // Sample rate = frequency * number of samples per cycle

    // 2. Generate the sine wave samples
    for (int i = 0; i < numSamples; i++) {
        sineWave[i] = (round(( (float) cos(2 * PI * i/numSamples) -1)/(-2) * DAC_RESOLUTION * DISTURBANCE_AMPLITUDE/MAX_DISTURBANCE)) ;
    }

    configureTimer(&htim2,frequency);

    // 5. Start DAC with DMA for both channels, using the sine wave samples
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *) sineWave, numSamples, DAC_ALIGN_12B_R);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t *) sineWave, numSamples, DAC_ALIGN_12B_R);

    // HAL_OPAMP_Start(&hopamp6);
    // Enable the timer
    HAL_TIM_Base_Start(&htim2);
}

// Function to convert ADC value to Voltage
float ADC2Volts(uint16_t ADCValue) {
    // Convert the ADC value to voltage
    float voltage = ((float)ADCValue / (float)ADC_RESOLUTION) * VREF;

    return voltage*3 + 0.2; // Return the battery voltage
}

// Function to convert ADC value to Amperes
float ADC2Current(uint16_t ADCValue) {
    // Convert the ADC value to voltage
    float voltage = ((float)ADCValue / (float)ADC_RESOLUTION) * VREF;

    return (voltage + 0.2)/R_SHUNT; // Return the battery voltage
}

uint16_t Current2ADC(float current){
   float voltage = (current + 33/2 -0.01)*R_SHUNT;
   return voltage / VREF * ADC_RESOLUTION;
}

uint16_t Voltage2ADC(float voltage){
   return (voltage - 0.2)/3  / VREF * ADC_RESOLUTION;
}

uint16_t bufferIndex = 0;
float I_Phase;
float V_Phase;
float phaseDiff;
float I_NoDC;
float V_NoDC;
float currentReal;

void doCalculations(){
    float V_minus_mean, I_minus_mean;  // Variables for storing offset-removed values

    I_buffer.local_max = array_max(I_buffer.buffer);
    I_buffer.local_min = array_min(I_buffer.buffer);
    
    V_buffer.local_max = array_max(V_buffer.buffer);
    V_buffer.local_min = array_min(V_buffer.buffer);


    I_mag = I_buffer.local_max;
    V_mag = (V_buffer.local_max - V_buffer.local_min);

    currentReal = (V_mag/I_mag);
    uint8_t V_peak_passed = 1;
    uint8_t I_peak_passed = 1;

    // do phase fancy buffers
    for (int i = 0 ; i < BUFFER_SIZE ; i++){
      I_NoDC = fabsf(I_buffer.buffer[i] - I_buffer.local_min);
      V_NoDC = fabsf(V_buffer.buffer[i] - V_buffer.local_min);

      I_Phase = I_NoDC/I_mag;

      V_Phase = V_NoDC/V_mag;

      if (V_buffer.buffer[i] ==  V_buffer.local_min){
        All_Phase[currFreq] = i * 180/BUFFER_SIZE - 90;
      }

      phaseDiff = (V_Phase - I_Phase)*180.0f;
      addSample(&Phase_buffer, phaseDiff);

    }

    All_Mag[currFreq] = currentReal;
}

uint8_t pulseCounter = 0;
// HAL ADC ConvCpltCallback (called when ADC conversion via DMA is complete)
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    if (hdac->Instance == DAC1) {
        HAL_TIM_Base_Stop(&htim2);
        pulseCounter++; 
        // do V and I fancy buffers
        for (int i = 0 ; i < BUFFER_SIZE ; i++){
          addSample(&V_buffer ,ADC2Volts    ( ADC_V_Buffer[i]));
          addSample(&I_buffer , sineWave[i] * MAX_DISTURBANCE / (DAC_RESOLUTION));
          // addSample(&I_buffer , ADC2Current  ( ADC_I_Buffer[i]) / 200);
        }
        HAL_TIM_Base_Start(&htim2);
        if (pulseCounter == NUMBER_OF_PULSES){ 
          pulseCounter = 0;
          // HAL_OPAMP_Stop(&hopamp6);
          HAL_DAC_SetValue(&hdac1,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,0);
          HAL_DAC_SetValue(&hdac1,DAC1_CHANNEL_2,DAC_ALIGN_12B_R,0);
          HAL_TIM_Base_Stop(&htim2);
          HAL_ADC_Stop_DMA(&hadc1);  // stop getting V BATT with DMA
          HAL_ADC_Stop_DMA(&hadc3);  // stop getting V BATT with DMA


          bufferIndex = 0;
          dumpBufferOverUSB();

          doCalculations();

          initBuffer(&V_buffer); // initialize buffers
          initBuffer(&I_buffer); // initialize buffers
          initBuffer(&Mag_buffer); // initialize buffers
          initBuffer(&Phase_buffer); // initialize buffers

          currFreq++;

          if (currFreq == NUM_FREQS){
            finishedSweeping = 1;
            HAL_OPAMP_Stop(&hopamp6);
            while (1){dumpOverUSB(); }
          }
          else{
            for (int i = 0 ; i < 150000*2 ; i++){
            }
            GenerateSineWave(frequencies[currFreq]);
          }
        }
    }
}


void dumpBufferOverUSB(){
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) "JJA"      , 3                          , HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) &V_buffer.buffer  , sizeof(float)*BUFFER_SIZE  , HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) &I_buffer.buffer  , sizeof(float)*BUFFER_SIZE  , HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) &currFreq  ,  4                         , HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) "4022"     , 4                          , HAL_MAX_DELAY);
}

void dumpOverUSB(){
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) "JJA"      , 3                        , HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) &All_Mag   , sizeof(float)*NUM_FREQS  , HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) &All_Phase , sizeof(float)*NUM_FREQS  , HAL_MAX_DELAY);
  HAL_UART_Transmit(&hlpuart1 ,(uint8_t *) "4022"     , 4                        , HAL_MAX_DELAY);
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_CORDIC_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_OPAMP6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  TIM2 -> ARR = 1;    // Set ARR
  
  initBuffer(&V_buffer); // initialize buffers
  initBuffer(&I_buffer); // initialize buffers
  initBuffer(&Mag_buffer); // initialize buffers
  initBuffer(&Phase_buffer); // initialize buffers


  HAL_ADC_Start_DMA(&hadc3,&ADC_V_Buffer,BUFFER_SIZE);  // start getting V BATT with DMA
  HAL_ADC_Start_DMA(&hadc1,&ADC_I_Buffer,BUFFER_SIZE);  // start getting V BATT with DMA

  HAL_OPAMP_Start(&hopamp6); // start current sink

  GenerateSineWave(frequencies[currFreq]);



  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // dumpOverUSB();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
    ADC_config_I = sConfig;
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  ADC_config_V = sConfig;

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */
  CORDIC_ConfigTypeDef sConfig = {0};

  sConfig.Function = CORDIC_FUNCTION_PHASE;       // Set function to arctangent
  sConfig.Precision = CORDIC_PRECISION_6CYCLES;   // Set precision (e.g., 6 cycles)
  sConfig.Scale = CORDIC_SCALE_0;                 // No scaling
  sConfig.NbWrite = CORDIC_NBWRITE_2;             // Two inputs (y and x)
  sConfig.NbRead = CORDIC_NBREAD_1;               // Single output (arctangent)
  sConfig.InSize = CORDIC_INSIZE_32BITS;          // Input size (32-bit)
  sConfig.OutSize = CORDIC_OUTSIZE_32BITS;        // Output size (32-bit)
  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */
  HAL_CORDIC_Configure(&hcordic,&sConfig);
  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 1152000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief OPAMP6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP6_Init(void)
{

  /* USER CODE BEGIN OPAMP6_Init 0 */

  /* USER CODE END OPAMP6_Init 0 */

  /* USER CODE BEGIN OPAMP6_Init 1 */

  /* USER CODE END OPAMP6_Init 1 */
  hopamp6.Instance = OPAMP6;
  hopamp6.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp6.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp6.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO1;
  hopamp6.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp6.Init.InternalOutput = DISABLE;
  hopamp6.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp6.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP6_Init 2 */

  /* USER CODE END OPAMP6_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA1_Channel8_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel8_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel8_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
