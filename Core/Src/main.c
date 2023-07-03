/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : b4Pressure main
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "wavetables.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_RANGE 4095.0f
#define SR 96000.f
#define WAVE_TABLE_SIZE 4096.f
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
//#define TracePads

struct OscData {
	// everything phase related
	float inc;
	float phase_accumulator;

	// the rest
	/*
  uint32_t waveform1 = maxAnalogIn;
  uint32_t waveform2;
	 */
	//float crossFMint;
	float crossFM;
	float volume;
	float tableIndex;
	float output2;
	float outputFilter;
	//GPIO_PinState waveChange; // chooses alternative wavetable
};

typedef enum {
	stop,
	attack,
	sustain,
	decay
} egStates;

struct EgData {
	//float attack;
	//float decay;
	float destinations[6];
	float value;
	float factor;
	float inc;
	float dec;
	float amount;
	egStates state;
	GPIO_PinState loop;
	GPIO_PinState trigger;
};

struct PadData {
	float mode1;
	float mode2;
	float pressure;
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint16_t adc1Buffer[8];
uint16_t adc2Buffer[5];
uint16_t adc3Buffer[7];
volatile uint8_t adc1Completed = 0;
volatile uint8_t adc2Completed = 0;
volatile uint8_t adc3Completed = 0;

//float analogIn = 0.0f;
//float aInOsc1, aInOsc2, aInFilter;
volatile float cutoff = 0.f;
volatile float reso = 0.1f;
volatile float low=0.f;
volatile float high=0.f;
volatile float band=0.f;
volatile float notch = 0.f;
volatile float filterIndex = 0.f;
volatile float filterVolume = 0.f;
volatile float *filterStates[5]={&low,&high,&band,&notch,&low};
volatile float filterOutput1,filterOutput2;

volatile struct EgData eg1 = {.state = stop};
volatile struct EgData eg2 = {.state = stop,.loop=GPIO_PIN_SET,.inc = 0.0001f,.dec=0.00001f};

volatile struct OscData osc1 = {.inc = WAVE_TABLE_SIZE * 100.f / SR, .phase_accumulator = 0.f,.tableIndex = 0};
volatile struct OscData osc2 = {.inc = WAVE_TABLE_SIZE * 101.f / SR, .phase_accumulator = 0.f,.tableIndex = 0};

volatile float osc1FMSample = 0.f;
volatile float osc2FMSample = 0.f;
volatile GPIO_PinState filterModus;
volatile float oldx;
volatile float oldy1,oldy2,oldy3;
volatile float my1 = 0.f;
volatile float y2 = 0.f;
volatile float y3 = 0.f;
volatile float y4 = 0.f;
static volatile struct PadData pad[4];

// wavetableset definitions, each one is made of several individual single cycle wave tables
const float *waveset1[8]={sintable,tritable,sinsawtable,sawtable,squaretable,sinsquaretable,randomtable,sintable};
const float *shapeset[8]={sawtable,sinsawtable,sintable,tritable,squaretable,sinsquaretable,randomtable,sawtable};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void UART_SEND(UART_HandleTypeDef *huart, char buffer[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	//HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
	if (hadc == &hadc1){
		adc1Completed = 1;
		return;
	}
	if (hadc == &hadc2){
		adc2Completed = 1;
		return;
	}
	if (hadc == &hadc3){
		adc3Completed = 1;
		return;
	}
	//HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// since we only use one timer, there is no need to check the timer

	// calculate envelope generator 1
	float egValue = eg1.value + eg1.factor;
	egValue = fminf(egValue,1.f);
	egValue = fmaxf(egValue,0.f);
	eg1.value = egValue;
	float env1Value = egValue * egValue * eg1.amount; // making the envelope snappy

	// calculate envelope generator 2
	egValue = eg2.value + eg2.factor;
	egValue = fminf(egValue,1.f);
	egValue = fmaxf(egValue,0.f);
	eg2.value = egValue;
	float env2Value = egValue * egValue * eg2.amount;

	// Oscillators -----------------------------------------------------------------
	float sample;
	float sample2;

	float accumulator;

	// calculate oscillator 1
	accumulator = osc1.phase_accumulator + osc1.inc;
	accumulator = (accumulator >= WAVE_TABLE_SIZE) ? accumulator - WAVE_TABLE_SIZE: accumulator;

	// apply FM

	accumulator += eg1.destinations[0] * (env1Value * 200.f) + osc2.crossFM * osc2FMSample + pad[0].mode2 * pad[0].pressure * 200.f;

	accumulator = (accumulator >= WAVE_TABLE_SIZE) ? accumulator - WAVE_TABLE_SIZE: accumulator;
	accumulator = (accumulator < 0.f) ? accumulator + WAVE_TABLE_SIZE : accumulator;

	// store accumulator
	osc1.phase_accumulator = accumulator;

	// get wavetable
	float tableIndex = osc1.tableIndex + eg1.destinations[1] * (env1Value * 6.f) + pad[0].mode1 * pad[0].pressure * 6.f;
	uint16_t tindex = (uint16_t)tableIndex;
	//if (tindex>6)tindex = 6;
	tindex = min(tindex,6);
	uint16_t tindexPlus = tindex + 1;


	// interpolated table value
	uint16_t index = (uint16_t)accumulator;
	uint16_t indexPlus = index + 1;
	float fIndex = (float)index;
	float fIndexPlus = (float)indexPlus;
	float sampleA,sampleB;

	sampleA = (fIndexPlus - accumulator) * *(waveset1[tindex]+index) + (accumulator-fIndex) * *(waveset1[tindex]+indexPlus);
	sampleB = (fIndexPlus - accumulator) * *(waveset1[tindexPlus]+index) + (accumulator-fIndex) * *(waveset1[tindexPlus]+indexPlus);

	float volumeEg = eg1.destinations[2] == 0.f ? 1.f: env1Value;

	sample = (((float)(tindexPlus - tableIndex)) * sampleA + ((float)(tableIndex-tindex))*sampleB);

	float padValue = pad[1].mode1 == 0.f ? 1.f : pad[1].pressure;

	sample = ((sample * osc1.volume) * volumeEg) * padValue;
	osc1FMSample = sample * 500.f;

	// calculate oscillator 2 ===========================================================================
	accumulator = osc2.phase_accumulator + osc2.inc;
	accumulator = (accumulator >= WAVE_TABLE_SIZE) ? accumulator - WAVE_TABLE_SIZE: accumulator;

	accumulator += eg2.destinations[0] * (env2Value * 200.f) + osc1.crossFM * osc1FMSample;

	accumulator = (accumulator >= WAVE_TABLE_SIZE) ? accumulator - WAVE_TABLE_SIZE: accumulator;
	accumulator = (accumulator < 0.f) ? accumulator + WAVE_TABLE_SIZE : accumulator;

	osc2.phase_accumulator = accumulator;

	// get wavetable
	tableIndex = osc2.tableIndex + eg2.destinations[1]* (env2Value * 6.f) + pad[3].mode1 * pad[3].pressure * 6.f;
	tindex = (uint16_t)tableIndex;

	tindex = min(tindex,6);
	tindexPlus = tindex + 1;


	// interpolated table value
	index = (uint16_t)accumulator;
	indexPlus = index + 1;
	fIndex = (float)index;
	fIndexPlus = (float)indexPlus;
	sampleA = (fIndexPlus - accumulator) * *(waveset1[tindex]+index) + (accumulator-fIndex) * *(waveset1[tindex]+indexPlus);
	sampleB = (fIndexPlus - accumulator) * *(waveset1[tindexPlus]+index) + (accumulator-fIndex) * *(waveset1[tindexPlus]+indexPlus);

	float volumeEg2 = eg2.destinations[2] == 0.f ? 1.f: env2Value;
	padValue = pad[2].mode1 == 0.f ? 1.f : pad[2].pressure;
	sample2 = ((( ((float)(tindexPlus - tableIndex)) * sampleA + ((float)(tableIndex-tindex))*sampleB) * osc2.volume) * volumeEg2)*padValue;
	osc2FMSample = sample2 * 500.f;

	// filter ----------------------------------------------------------------------------------
	//cutoff = cutoff freq in Hz
	//fs = sampling frequency //(e.g. 44100Hz)
	//f = 2 sin (pi * cutoff / fs) //[approximately]
	//q = resonance/bandwidth [0 < q <= 1]  most res: q=1, less: q=0

	// adding the oscillators
	float filterInput = (sample * osc1.outputFilter + sample2 * osc2.outputFilter) * 0.5f;
	float filterOutput = 0.f;
	volumeEg = eg1.destinations[5] == 0.f ? 1.f: env1Value;
	volumeEg2 = eg2.destinations[5] == 0.f ? 1.f: env2Value;

	if (filterModus == GPIO_PIN_RESET){
		low = low + (cutoff + (eg1.destinations[3]*env1Value*2.f) + (eg2.destinations[3]*env2Value*2.f) + pad[1].mode2 * pad[1].pressure + pad[2].mode2 * pad[2].pressure) * band;
		high = reso * filterInput - low - reso*band;
		band = cutoff * high + band;
		notch = high + low;
		float compensation = 1.f;
		float antiReso = 1.f - reso;
		//if (reso < 0.4f)
		compensation = 4.f*antiReso*antiReso*antiReso*antiReso*antiReso*antiReso*antiReso * antiReso * antiReso * antiReso * antiReso + 1.f;
		// filter blend
		float filterIndexMod = (filterIndex + eg1.destinations[4]* (env1Value * 3.f)+ eg2.destinations[4] * (env2Value * 3.f));// + pad[3].mode1 * (pad[3].pressure * 3.f));
		filterIndexMod = fminf(filterIndexMod,3.f);
		index = (uint16_t)filterIndexMod;
		indexPlus = index + 1;
		fIndex = (float)index;
		fIndexPlus = (float)indexPlus;
		padValue = pad[3].mode2 == 0.f ? 1.f : pad[3].pressure;
		filterOutput = (((((fIndexPlus - filterIndexMod) * *filterStates[index] + (filterIndexMod - fIndex) * *filterStates[indexPlus])*filterVolume * compensation) * volumeEg) * volumeEg2)*padValue;
	}
	else {
		// waveshaper
		// scale the input signal for lut
		float signal = (((filterInput + 1.0f)*0.5f) * DAC_RANGE); // coincedence that dac range is wavetable size - 1
		index = (uint16_t)signal;
		indexPlus = index + 1;
		fIndex = (float)index;
		fIndexPlus = (float)indexPlus;

		float shapeMorph =(filterIndex * 2.f + eg1.destinations[4]* (env1Value * 6.f)+ eg2.destinations[4] * (env2Value * 6.f));// + pad[2].mode2 * (pad[2].pressure * 6.f));
		shapeMorph = fminf(shapeMorph,6.f);
		tindex = (uint16_t)shapeMorph;
		tindexPlus = tindex + 1;

		sampleA = (fIndexPlus - signal) * *(shapeset[tindex]+index) + (signal-fIndex) * *(shapeset[tindex]+indexPlus);
		sampleB = (fIndexPlus - signal) * *(shapeset[tindexPlus]+index) + (signal-fIndex) * *(shapeset[tindexPlus]+indexPlus);

		fIndex = (float)tindex;
		fIndexPlus = (float)tindexPlus;
		filterInput = ((fIndexPlus - shapeMorph) * sampleA + (shapeMorph-fIndex)*sampleB)*0.5f;

		// Moogish filter
		float f = (cutoff + (eg1.destinations[3]*env1Value*0.2f) + (eg2.destinations[3]*env2Value*0.2f) + pad[1].mode2 * pad[1].pressure + pad[2].mode2 * pad[2].pressure * 1.f);
		float p=f*(1.8f-0.8f*f);
		float k=p+p-1.f;

		float t=(1.f-p)*1.386249f;
		float t2=12.f+t*t;
		float r = (1.f - reso)*(t2+6.f*t)/(t2-6.f*t);

		float x = filterInput - r*y4;

		//Four cascaded onepole filters (bilinear transform)
		my1= x*p +  oldx*p - k*my1;
		y2=my1*p + oldy1*p - k*y2;
		y3=y2*p + oldy2*p - k*y3;
		y4=y3*p + oldy3*p - k*y4;

		//Clipper band limited sigmoid
		y4-=(y4*y4*y4)/6.f;

		oldx = x; oldy1 = my1; oldy2 = y2; oldy3 = y3;
		padValue = pad[3].mode2 == 0.f ? 1.f : pad[3].pressure;
		filterOutput = (((y4 *filterVolume) * volumeEg) * volumeEg2)*padValue;
	}
	float output1 = filterOutput;
	float output2 = (sample * osc1.output2 + sample2 * osc2.output2) * 0.5f;// 4.5f;

	uint32_t DACData = ((uint32_t) (((output1 + 1.0f)*0.5f * DAC_RANGE)* 0.5f) );
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACData);

	DACData = ((uint32_t) (((output2 + 1.0f)*0.5f * DAC_RANGE)* 0.5f) );
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DACData);

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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  float freqFactor = WAVE_TABLE_SIZE / SR;
  	float dacLUT[4096];
  	float cutLUT[4096];
  	float oscLUT[4096];

  	for (int i=0;i<4096;i++){
  		dacLUT[i] = ((float)i) / 4095.0f;
  		cutLUT[i] = (2.f * sinf (3.14159265359f *  (dacLUT[i]*11000.f + 20.f) / SR));
  		oscLUT[i] = freqFactor * (dacLUT[i]*2095.f + 0.01f);
  	}

  	float eg1AmountReadings[32];
  	uint16_t eg1AmountIndex = 0;
  	float eg1AttackReadings[8];
  	uint16_t eg1AttackIndex = 0;
  	float eg1DecayReadings[8];
  	uint16_t eg1DecayIndex = 0;

  	float eg2AmountReadings[32];
  	uint16_t eg2AmountIndex = 0;
  	float eg2AttackReadings[8];
  	uint16_t eg2AttackIndex = 0;
  	float eg2DecayReadings[8];
  	uint16_t eg2DecayIndex = 0;

  	uint16_t pad1Readings[8];
  	uint16_t pad1Index = 0;
  	uint16_t pad2Readings[8];
  	uint16_t pad2Index = 0;
  	uint16_t pad3Readings[8];
  	uint16_t pad3Index = 0;
  	uint16_t pad4Readings[8];
  	uint16_t pad4Index = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
#ifdef TracePads
  MX_USB_OTG_FS_PCD_Init();
#endif
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
#ifdef TracePads
  MX_USART3_UART_Init();
#endif
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
#ifdef TracePads
  char buffer[30];
  	sprintf(buffer, "not completed");
  	char CRLF[3] ={0x0A,0x0D,0x0};
  	strcat(buffer,CRLF);
#endif
  //UART_SEND(&huart3, buffer);


    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  	HAL_TIM_Base_Start_IT(&htim6); // start the timer in interrupt mode

  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Buffer, 8);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Buffer, 5);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3Buffer, 7);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	while (1)
	{
		//UART_SEND(&huart3, buffer);
		while(!adc1Completed && !adc2Completed && !adc3Completed){
			HAL_Delay(4);
		}
		if (adc1Completed){
			HAL_ADC_Stop_DMA(&hadc1);
			adc1Completed = 0;

			osc1.inc = oscLUT[adc1Buffer[0]];
			osc1.tableIndex = dacLUT[adc1Buffer[2]] * 6.f;
			osc1.volume = dacLUT[adc1Buffer[3]];

			osc2.inc = oscLUT[adc1Buffer[1]];
			osc2.tableIndex = dacLUT[adc1Buffer[5]] * 6.f;
			osc2.volume = dacLUT[adc1Buffer[7]];

			filterIndex = dacLUT[adc1Buffer[4]] * 3.f;
			filterVolume = dacLUT[adc1Buffer[6]];


			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Buffer, 8);
		}

		if (adc2Completed){
			HAL_ADC_Stop_DMA(&hadc2);
			adc2Completed = 0;

			//10 13
			pad1Readings[pad1Index] = adc2Buffer[0];
			pad1Index++;
			if (pad1Index == 8){
				pad1Index = 0;
				uint16_t value = (pad1Readings[0] + pad1Readings[1] + pad1Readings[2] + pad1Readings[3] + pad1Readings[4] + pad1Readings[5] + pad1Readings[6] + pad1Readings[7]) / 8;
				pad[0].pressure = dacLUT[value];
			}

			pad2Readings[pad2Index] = adc2Buffer[1];
			pad2Index++;
			if (pad2Index == 8){
				pad2Index = 0;
				uint16_t value = (pad2Readings[0] + pad2Readings[1] + pad2Readings[2] + pad2Readings[3] + pad2Readings[4] + pad2Readings[5] + pad2Readings[6] + pad2Readings[7]) / 8;
				pad[1].pressure = dacLUT[value];
			}
#ifdef TracePads
			 itoa(adc2Buffer[0],buffer,10);
				  strcat(buffer,CRLF);
				  UART_SEND(&huart3, buffer);

			 itoa(adc2Buffer[1],buffer,10);
							  strcat(buffer,CRLF);
							  UART_SEND(&huart3, buffer);
#endif
			pad3Readings[pad3Index] = adc2Buffer[2];
			pad3Index++;
			if (pad3Index == 8){
				pad3Index = 0;
				uint16_t value = (pad3Readings[0] + pad3Readings[1] + pad3Readings[2] + pad3Readings[3] + pad3Readings[4] + pad3Readings[5] + pad3Readings[6] + pad3Readings[7]) / 8;
				pad[2].pressure = dacLUT[value];
			}
#ifdef TracePads
			 itoa(adc2Buffer[2],buffer,10);
							  strcat(buffer,CRLF);
							  UART_SEND(&huart3, buffer);
#endif
			pad4Readings[pad4Index] = adc2Buffer[3];
			pad4Index++;
			if (pad4Index == 8){
				pad4Index = 0;
				uint16_t value = (pad4Readings[0] + pad4Readings[1] + pad4Readings[2] + pad4Readings[3] + pad4Readings[4] + pad4Readings[5] + pad4Readings[6] + pad4Readings[7]) / 8;
				pad[3].pressure = dacLUT[value];
			}
#ifdef TracePads
			itoa(adc2Buffer[3],buffer,10);
							  strcat(buffer,CRLF);
							  strcat(buffer,CRLF);
							  UART_SEND(&huart3, buffer);
#endif

			cutoff = cutLUT[adc2Buffer[4]];//(2.f * sinf (3.14159265359f *  (dacLUT[adc2Buffer[1]]*11000.f + 20.f + analogIn*aInFilter*100.f) * SRFactor));

			HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Buffer, 5);
		}

		if (adc3Completed){
			HAL_ADC_Stop_DMA(&hadc3);
			adc3Completed = 0;

			reso = 1.f - dacLUT[adc3Buffer[0]] + 0.01f;

			eg1AttackReadings[eg1AttackIndex] = dacLUT[adc3Buffer[5]];
			eg1AttackIndex++;
			if (eg1AttackIndex == 8) {
				float eAtt = (eg1AttackReadings[0]+eg1AttackReadings[1]+eg1AttackReadings[2]+eg1AttackReadings[3]+eg1AttackReadings[4]+eg1AttackReadings[5]+eg1AttackReadings[6]+eg1AttackReadings[7]) * 0.125f;
				eg1.inc = eAtt * 0.00225f + 0.000001f;
				eg1AttackIndex = 0;
			}

			eg1DecayReadings[eg1DecayIndex] = dacLUT[adc3Buffer[6]];
			eg1DecayIndex++;
			if (eg1DecayIndex == 8){
				float eDec = (eg1DecayReadings[0]+eg1DecayReadings[1]+eg1DecayReadings[2]+eg1DecayReadings[3]+eg1DecayReadings[4]+eg1DecayReadings[5]+eg1DecayReadings[6]+eg1DecayReadings[7]) * 0.125f;
				eg1.dec = (eDec * 0.00225f + 0.000001f) * -1.f;
				eg1DecayIndex = 0;
			}


			eg1AmountReadings[eg1AmountIndex] = dacLUT[adc3Buffer[4]];
			eg1AmountIndex++;
			if (eg1AmountIndex == 32){
				eg1AmountIndex = 0;
				eg1.amount = (eg1AmountReadings[0] + eg1AmountReadings[1] + eg1AmountReadings[2] + eg1AmountReadings[3]
																													 + eg1AmountReadings[4] + eg1AmountReadings[5] + eg1AmountReadings[6] + eg1AmountReadings[7]
																																																			  + eg1AmountReadings[8] + eg1AmountReadings[9] + eg1AmountReadings[10] + eg1AmountReadings[11]
																																																																										+ eg1AmountReadings[12] + eg1AmountReadings[13] + eg1AmountReadings[14] + eg1AmountReadings[15]
																																																																																																	+ eg1AmountReadings[16] + eg1AmountReadings[17] + eg1AmountReadings[18] + eg1AmountReadings[19]
																																																																																																																								+ eg1AmountReadings[20] + eg1AmountReadings[21] + eg1AmountReadings[22] + eg1AmountReadings[23]
																																																																																																																																															+ eg1AmountReadings[24] + eg1AmountReadings[25] + eg1AmountReadings[26] + eg1AmountReadings[27]
																																																																																																																																																																						+ eg1AmountReadings[28] + eg1AmountReadings[29] + eg1AmountReadings[30] + eg1AmountReadings[31]
				) * 0.03125f;
			}

			eg2AttackReadings[eg2AttackIndex] = dacLUT[adc3Buffer[2]];
			eg2AttackIndex++;
			if (eg2AttackIndex == 8) {
				float eAtt = (eg2AttackReadings[0]+eg2AttackReadings[1]+eg2AttackReadings[2]+eg2AttackReadings[3]+eg2AttackReadings[4]+eg2AttackReadings[5]+eg2AttackReadings[6]+eg2AttackReadings[7]) * 0.125f;
				eg2.inc = eAtt * 0.00225f + 0.000001f;
				eg2AttackIndex = 0;
			}

			eg2DecayReadings[eg2DecayIndex] = dacLUT[adc3Buffer[1]];
			eg2DecayIndex++;
			if (eg2DecayIndex == 8){
				float eDec = (eg2DecayReadings[0]+eg2DecayReadings[1]+eg2DecayReadings[2]+eg2DecayReadings[3]+eg2DecayReadings[4]+eg2DecayReadings[5]+eg2DecayReadings[6]+eg2DecayReadings[7]) * 0.125f;
				eg2.dec = (eDec * 0.00225f + 0.000001f) * -1.f;
				eg2DecayIndex = 0;
			}


			eg2AmountReadings[eg2AmountIndex] = dacLUT[adc3Buffer[3]];
			eg2AmountIndex++;
			if (eg2AmountIndex == 32){
				eg2AmountIndex = 0;
				eg2.amount = (eg2AmountReadings[0] + eg2AmountReadings[1] + eg2AmountReadings[2] + eg2AmountReadings[3]+ eg2AmountReadings[4] + eg2AmountReadings[5] + eg2AmountReadings[6] + eg2AmountReadings[7] + eg2AmountReadings[8] + eg2AmountReadings[9] + eg2AmountReadings[10] + eg2AmountReadings[11] + eg2AmountReadings[12] + eg2AmountReadings[13] + eg2AmountReadings[14] + eg2AmountReadings[15]
																																																																																																	+ eg2AmountReadings[16] + eg2AmountReadings[17] + eg2AmountReadings[18] + eg2AmountReadings[19]
																																																																																																																								+ eg2AmountReadings[20] + eg2AmountReadings[21] + eg2AmountReadings[22] + eg2AmountReadings[23]
																																																																																																																																															+ eg2AmountReadings[24] + eg2AmountReadings[25] + eg2AmountReadings[26] + eg2AmountReadings[27]
																																																																																																																																																																						+ eg2AmountReadings[28] + eg2AmountReadings[29] + eg2AmountReadings[30] + eg2AmountReadings[31]
				) * 0.03125f;
			}

			HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3Buffer, 7);
		}

		// read GPIOs -------------------------------------------------------------------------------------

		osc1.output2 = HAL_GPIO_ReadPin(GPIOE, Osc1Out2_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
		osc1.outputFilter = HAL_GPIO_ReadPin(GPIOE, Osc1Filter_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;

		osc2.output2 = HAL_GPIO_ReadPin(GPIOE, Osc2Out2_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
		osc2.outputFilter =  HAL_GPIO_ReadPin(GPIOE, Osc2Filter_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
		GPIO_PinState altModus = filterModus;
		filterModus = HAL_GPIO_ReadPin(GPIOB, FilterModus_Pin);
		if (filterModus != altModus){
			oldx = 0.f;
			oldy1=oldy2=oldy3=0.f;
			my1 = 0.f;
			y2 = 0.f;
			y3 = 0.f;
			y4 = 0.f;
		}
		HAL_GPIO_WritePin(GPIOB, LD1_Pin, filterModus);

		eg1.loop = HAL_GPIO_ReadPin(GPIOB, Eg1Loop_Pin);// == GPIO_PIN_RESET ? 1.0f:0.f;
		eg1.trigger = eg2.trigger = HAL_GPIO_ReadPin(GPIOB, EgTrigger_Pin);// == GPIO_PIN_RESET ? 1.0f:0.f;

		eg1.destinations[0] = HAL_GPIO_ReadPin(GPIOB, Eg1FM1_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
		eg1.destinations[1] = HAL_GPIO_ReadPin(GPIOB, Eg1Wave1_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg1.destinations[2] = HAL_GPIO_ReadPin(GPIOD, Eg1Vol1_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg1.destinations[3] = HAL_GPIO_ReadPin(GPIOD, Eg1Cut_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg1.destinations[4] = HAL_GPIO_ReadPin(GPIOD, Eg1Morph_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg1.destinations[5] = HAL_GPIO_ReadPin(GPIOD, Eg1FilterVol_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;

		eg2.loop = HAL_GPIO_ReadPin(GPIOB, Eg2Loop_Pin);

		eg2.destinations[0] = HAL_GPIO_ReadPin(GPIOD, Eg2FM2_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
		eg2.destinations[1] = HAL_GPIO_ReadPin(GPIOE, Eg2Wave2_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg2.destinations[2] = HAL_GPIO_ReadPin(GPIOE, Eg2Vol2_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg2.destinations[3] = HAL_GPIO_ReadPin(GPIOE, Eg2Cut_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg2.destinations[4] = HAL_GPIO_ReadPin(GPIOE, Eg2Morph_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		eg2.destinations[5] = HAL_GPIO_ReadPin(GPIOE, Eg2FilterVol_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;


		osc1.crossFM = HAL_GPIO_ReadPin(GPIOF, Osc1FM_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		osc2.crossFM = HAL_GPIO_ReadPin(GPIOE, Osc2FM_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;

		pad[0].mode1 = HAL_GPIO_ReadPin(GPIOF, PadMode1_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		pad[0].mode2 = pad[0].mode1 == 0.f ? 1.f : 0.f;

		pad[1].mode1 = HAL_GPIO_ReadPin(GPIOF, PadMode2_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		pad[1].mode2 = pad[1].mode1 == 0.f ? 1.f : 0.f;

		pad[2].mode1 = HAL_GPIO_ReadPin(GPIOF, PadMode3_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		pad[2].mode2 = pad[2].mode1 == 0.f ? 1.f : 0.f;

		pad[3].mode1 = HAL_GPIO_ReadPin(GPIOF, PadMode4_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
		pad[3].mode2 = pad[3].mode1 == 0.f ? 1.f : 0.f;

// control the egs
		switch(eg1.state){
		case attack:
			if (eg1.loop == GPIO_PIN_RESET || eg1.trigger == GPIO_PIN_SET){
				if (eg1.value < 1.f) {
					eg1.factor = eg1.inc;
				}
				else{
					eg1.state = sustain;
					eg1.factor = 0.f;
				}
			}
			else {
				eg1.state = decay;
				HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
			}
			break;
		case sustain:
			if (eg1.trigger != GPIO_PIN_SET){
				eg1.state = decay;
				HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
			}
			break;
		case decay:
			if (eg1.value > 0.f){
				eg1.factor = eg1.dec;
			}
			else{
				eg1.state = stop;
				eg1.value = 0.f;
				eg1.factor = 0.f;
			}
			if (eg1.loop == GPIO_PIN_SET && eg1.trigger == GPIO_PIN_SET){ // when not looping and trigger is pressed, go into attack
				eg1.state = attack;
				eg1.factor = eg1.inc;
				HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
			}
			break;
		case stop:
			if (eg1.loop == GPIO_PIN_RESET || eg1.trigger == GPIO_PIN_SET){
				eg1.state = attack;
				eg1.factor = eg1.inc;
				HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
			}
			break;
		}

		switch(eg2.state){
		case attack:
			if (eg2.loop == GPIO_PIN_RESET || eg2.trigger == GPIO_PIN_SET){

				if (eg2.value < 1.f) {
					eg2.factor = eg2.inc;
				}
				else {
					eg2.state=sustain;
					eg2.factor = 0.f;
				}
			}
			else {
				eg2.state = decay;
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
			}
			break;
		case sustain:
			if (eg2.trigger != GPIO_PIN_SET){
				eg2.state = decay;
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
			}
			break;
		case decay:
			if (eg2.value > 0.f){
				eg2.factor = eg2.dec;
			}
			else {
				eg2.state = stop;
				eg2.value = 0.f;
				eg2.factor = 0.f;
			}
			if (eg2.loop == GPIO_PIN_SET && eg2.trigger == GPIO_PIN_SET){ // if not looping and trigger is pressed, go into attack
				eg2.state = attack;
				eg2.factor = eg2.inc;
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
			}
			break;
		case stop:
			if (eg2.loop == GPIO_PIN_RESET || eg2.trigger == GPIO_PIN_SET){
				eg2.state = attack;
				eg2.factor = eg2.inc;
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
			}
			break;
		}


		//UART_SEND(&huart3, buffer);
		HAL_Delay(2);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_5;
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
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 7;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 216000000/96000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Eg2Vol2_Pin Eg2Cut_Pin Eg2Morph_Pin Eg2FilterVol_Pin
                           Osc1Filter_Pin Osc1Out2_Pin Osc2Filter_Pin Osc2Out2_Pin
                           Osc2FM_Pin Eg2Wave2_Pin */
  GPIO_InitStruct.Pin = Eg2Vol2_Pin|Eg2Cut_Pin|Eg2Morph_Pin|Eg2FilterVol_Pin
                          |Osc1Filter_Pin|Osc1Out2_Pin|Osc2Filter_Pin|Osc2Out2_Pin
                          |Osc2FM_Pin|Eg2Wave2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EgTrigger_Pin Eg1FM1_Pin Eg1Wave1_Pin FilterModus_Pin
                           Eg1Loop_Pin Eg2Loop_Pin */
  GPIO_InitStruct.Pin = EgTrigger_Pin|Eg1FM1_Pin|Eg1Wave1_Pin|FilterModus_Pin
                          |Eg1Loop_Pin|Eg2Loop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Osc1FM_Pin PadMode1_Pin PadMode2_Pin PadMode3_Pin
                           PadMode4_Pin */
  GPIO_InitStruct.Pin = Osc1FM_Pin|PadMode1_Pin|PadMode2_Pin|PadMode3_Pin
                          |PadMode4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Eg1Vol1_Pin Eg1Cut_Pin Eg1Morph_Pin Eg1FilterVol_Pin
                           Eg2FM2_Pin */
  GPIO_InitStruct.Pin = Eg1Vol1_Pin|Eg1Cut_Pin|Eg1Morph_Pin|Eg1FilterVol_Pin
                          |Eg2FM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART_SEND(UART_HandleTypeDef *huart, char buffer[]){
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
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
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
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
