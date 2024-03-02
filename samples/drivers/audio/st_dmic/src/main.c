/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32wbxx_hal_sai.h>

#include <zephyr/logging/log.h>
#include "stm32wb5mm_dk_audio.h"
#include "stm32wb5mm_dk_errno.h"

LOG_MODULE_REGISTER(dmic_sample);
BSP_AUDIO_Init_t MicParams;
uint16_t PDM_Buffer[((((2 * AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY) / 1000) * MAX_DECIMATION_FACTOR) / 16)* N_MS_PER_INTERRUPT ];
// SAI_HandleTypeDef hsai_BlockA1;
// DMA_HandleTypeDef hdma_sai1_a;


void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        // Error_Handler();
        printk("Error HAL_RCC_OscConfig\n");
    }

    /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                                  |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        printk("Error Clock init....\n");
    }

    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();
}

void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Initializes the peripherals clock
    */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
    PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
    PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        printk("Error PeriphCommonClock_Config\n");
    }
    /* USER CODE BEGIN Smps */

    /* USER CODE END Smps */
}

static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hAudioInSai.Instance = SAI1_Block_A;
  hAudioInSai.Init.Protocol = SAI_FREE_PROTOCOL;
  hAudioInSai.Init.AudioMode = SAI_MODEMASTER_RX;
  hAudioInSai.Init.DataSize = SAI_DATASIZE_8;
  hAudioInSai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hAudioInSai.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hAudioInSai.Init.Synchro = SAI_ASYNCHRONOUS;
  hAudioInSai.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hAudioInSai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hAudioInSai.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hAudioInSai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hAudioInSai.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hAudioInSai.Init.MonoStereoMode = SAI_STEREOMODE;
  hAudioInSai.Init.CompandingMode = SAI_NOCOMPANDING;
  hAudioInSai.Init.PdmInit.Activation = ENABLE;
  hAudioInSai.Init.PdmInit.MicPairsNbr = 1;
  hAudioInSai.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hAudioInSai.FrameInit.FrameLength = 8;
  hAudioInSai.FrameInit.ActiveFrameLength = 1;
  hAudioInSai.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hAudioInSai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hAudioInSai.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hAudioInSai.SlotInit.FirstBitOffset = 0;
  hAudioInSai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hAudioInSai.SlotInit.SlotNumber = 1;
  hAudioInSai.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hAudioInSai) != HAL_OK)
  {
    // Error_Handler();
    printk("Error HAL_SAI_Init\n");

  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/////////////////////////////
static void Init_Acquisition_Peripherals(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
    int fret = 0;

    MicParams.BitsPerSample = 16;
    MicParams.ChannelsNbr = ChnlNbrIn;
    MicParams.Device = AUDIO_IN_DIGITAL_MIC;
    MicParams.SampleRate = AudioFreq;
    MicParams.Volume = AUDIO_VOLUME_INPUT;

    fret = BSP_AUDIO_IN_Init(BSP_AUDIO_INSTANCE, &MicParams);
    if ( fret != BSP_ERROR_NONE)
    {
        // Error_Handler();
        printk("Error BSP_AUDIO_IN_Init: %d\n", fret);
    }
}

static void Start_Acquisition(void)
{
    int fret = 0;
    fret = BSP_AUDIO_IN_Record(BSP_AUDIO_INSTANCE, (uint8_t *) PDM_Buffer, AUDIO_IN_BUFFER_SIZE);
    if (fret != BSP_ERROR_NONE)
    {
        printk("Error in BSP_AUDIO_IN_Record: %d\n", fret);
    }
}
int main(void)
{
    int fret = 0;

    // HAL_Init();

    // SystemClock_Config();

    PeriphCommonClock_Config();
    // MX_GPIO_Init();
    // MX_DMA_Init();
    // MX_SAI1_Init();

    printk("Start Init_Acquisition_Peripherals...\n");
    // Init_Acquisition_Peripherals(AUDIO_IN_SAMPLING_FREQUENCY, AUDIO_IN_CHANNELS, 0);
    printk("Init_Acquisition_Peripherals finished...\n");

    printk("Start Audio acquisition...\n");
    Start_Acquisition();
    printk("Init Clock: %d\n", fret);
    return 0;
}
