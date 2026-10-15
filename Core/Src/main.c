/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief        m
  *    : Board A receiver/master
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_frame.h"
#include "combined_audio_frame.h"
#include <stdint.h>
#include <string.h>
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
SPI_HandleTypeDef hspi1;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t spi_valid_frames = 0;
volatile uint32_t spi_timeouts = 0;
volatile uint32_t spi_transfer_errors = 0;
volatile uint32_t spi_bad_magic = 0;
volatile uint32_t spi_bad_length = 0;
volatile uint32_t spi_bad_checksum = 0;
volatile uint32_t spi_frame_id_gaps = 0;
volatile uint16_t spi_last_frame_id = 0;
volatile uint16_t spi_last_received_checksum = 0;
volatile uint16_t spi_last_expected_checksum = 0;
static uint32_t valid_frame_count = 0;
#define RECEIVER_STARTUP_DELAY_MS 250U
#define SPI_BAD_MAGIC_RECOVERY_THRESHOLD 32U
static uint32_t spi_bad_magic_streak = 0;

#define MIC_DMA_SAMPLES 1024U
#define MIC_HALF_SAMPLES (MIC_DMA_SAMPLES / 2U)
#define MIC_GAIN 4
static int32_t mic0_dma[MIC_DMA_SAMPLES];
static int32_t mic1_dma[MIC_DMA_SAMPLES];
static volatile uint8_t mic0_half_ready[2];
static volatile uint8_t mic1_half_ready[2];
#define LOCAL_AUDIO_QUEUE_DEPTH 4U
static int16_t local_audio_queue[LOCAL_AUDIO_QUEUE_DEPTH]
                         [AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS];
static volatile uint8_t local_queue_head = 0;
static volatile uint8_t local_queue_tail = 0;
static int16_t latest_local_samples[AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS];
static volatile uint32_t local_audio_underruns = 0;
static volatile uint32_t local_audio_overruns = 0;
static volatile uint32_t combined_audio_frames = 0;
static uint16_t combined_frame_id = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static int ReceiveAudioFrame_SPI(AudioFrame_t *frame);
static void ProcessRemoteAudioFrame(AudioFrame_t *frame);
static int16_t ClampAudio(int32_t value);
static void ProcessLocalMicHalf(uint32_t offset);
static void ServiceLocalMicrophones(void);
static void TakeLocalAudioFrame(int16_t *samples);
static void BuildAndForwardCombinedFrame(const AudioFrame_t *remote_frame);
static void RecoverSPI1(void);
static int TransmitterReady(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Receive one complete frame using hardware SPI1 NSS on PA4. */
static int ReceiveAudioFrame_SPI(AudioFrame_t *frame)
{
    static uint8_t clock_bytes[AUDIO_FRAME_TOTAL_BYTES] = {0};
    HAL_StatusTypeDef status;

    status = HAL_SPI_TransmitReceive(
        &hspi1,
        clock_bytes,
        (uint8_t *)frame,
        AUDIO_FRAME_TOTAL_BYTES,
        100
    );
    if (status != HAL_OK)
    {
        if (status == HAL_TIMEOUT)
        {
            spi_timeouts++;
        }
        else
        {
            spi_transfer_errors++;
        }
        return 0;
    }

    if (frame->magic0 != AUDIO_FRAME_MAGIC_0 ||
        frame->magic1 != AUDIO_FRAME_MAGIC_1)
    {
        spi_bad_magic++;
        spi_bad_magic_streak++;
        if (spi_bad_magic_streak >= SPI_BAD_MAGIC_RECOVERY_THRESHOLD)
        {
            RecoverSPI1();
            spi_bad_magic_streak = 0;
            HAL_Delay(1);
        }
        return 0;
    }

    spi_bad_magic_streak = 0;

    if (frame->payload_size != AUDIO_PAYLOAD_BYTES)
    {
        spi_bad_length++;
        return 0;
    }

    uint16_t expected_checksum = audio_checksum_u8(
        (uint8_t *)frame,
        AUDIO_FRAME_TOTAL_BYTES - AUDIO_FRAME_CRC_BYTES
    );
    spi_last_frame_id = frame->frame_id;
    spi_last_received_checksum = frame->checksum;
    spi_last_expected_checksum = expected_checksum;

    if (frame->checksum != expected_checksum)
    {
        spi_bad_checksum++;
        return 0;
    }

    spi_valid_frames++;
    return 1;
}

/*
 * Placeholder for using the received Board B audio.
 */
static int16_t ClampAudio(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static void ProcessLocalMicHalf(uint32_t offset)
{
    /* Each DMA half contains 512 samples, enough for two 256-sample frames. */
    for (uint32_t base = 0; base < MIC_HALF_SAMPLES; base += AUDIO_FRAME_SAMPLES)
    {
        uint8_t next_head = (uint8_t)((local_queue_head + 1U) % LOCAL_AUDIO_QUEUE_DEPTH);
        if (next_head == local_queue_tail)
        {
            local_audio_overruns++;
            continue;
        }
        int16_t *local_frame = local_audio_queue[local_queue_head];
        for (uint32_t i = 0; i < AUDIO_FRAME_SAMPLES; i++)
        {
            uint32_t index = offset + base + i;
            local_frame[2U * i] =
                ClampAudio((mic0_dma[index] >> 8) * MIC_GAIN);
            local_frame[2U * i + 1U] =
                ClampAudio((mic1_dma[index] >> 8) * MIC_GAIN);
        }
        memcpy(latest_local_samples, local_frame, sizeof(latest_local_samples));
        local_queue_head = next_head;
    }
}

static void TakeLocalAudioFrame(int16_t *samples)
{
    if (local_queue_tail != local_queue_head)
    {
        memcpy(samples, local_audio_queue[local_queue_tail],
               sizeof(latest_local_samples));
        local_queue_tail = (uint8_t)((local_queue_tail + 1U) % LOCAL_AUDIO_QUEUE_DEPTH);
    }
    else
    {
        memcpy(samples, latest_local_samples, sizeof(latest_local_samples));
        local_audio_underruns++;
    }
}

/* Re-arm the master peripheral after a slave was unplugged mid-transaction. */
static void RecoverSPI1(void)
{
    HAL_SPI_DeInit(&hspi1);
    HAL_Delay(1);
    MX_SPI1_Init();
}

static int TransmitterReady(void)
{
    return HAL_GPIO_ReadPin(TRANSMITTER_READY_GPIO_Port,
                            TRANSMITTER_READY_Pin) == GPIO_PIN_SET;
}

static void ServiceLocalMicrophones(void)
{
    if (mic0_half_ready[0] && mic1_half_ready[0])
    {
        mic0_half_ready[0] = 0;
        mic1_half_ready[0] = 0;
        ProcessLocalMicHalf(0);
    }
    if (mic0_half_ready[1] && mic1_half_ready[1])
    {
        mic0_half_ready[1] = 0;
        mic1_half_ready[1] = 0;
        ProcessLocalMicHalf(MIC_HALF_SAMPLES);
    }
}

static void BuildAndForwardCombinedFrame(const AudioFrame_t *remote_frame)
{
    static CombinedAudioFrame_t combined;
    int16_t local_samples[AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS];

    TakeLocalAudioFrame(local_samples);

    combined.magic0 = COMBINED_AUDIO_MAGIC_0;
    combined.magic1 = COMBINED_AUDIO_MAGIC_1;
    combined.frame_id = combined_frame_id++;
    combined.payload_size = COMBINED_AUDIO_PAYLOAD_BYTES;

    for (uint32_t i = 0; i < AUDIO_FRAME_SAMPLES; i++)
    {
        /* Channels 0/1 are transmitter mics; 2/3 are receiver mics. */
        combined.samples[4U * i] = remote_frame->samples[2U * i];
        combined.samples[4U * i + 1U] = remote_frame->samples[2U * i + 1U];
        combined.samples[4U * i + 2U] = local_samples[2U * i];
        combined.samples[4U * i + 3U] = local_samples[2U * i + 1U];
    }

    combined.checksum = audio_checksum_u8(
        (uint8_t *)&combined,
        COMBINED_AUDIO_TOTAL_BYTES - COMBINED_AUDIO_CRC_BYTES);

    if (HAL_UART_Transmit(&huart1, (uint8_t *)&combined,
                          COMBINED_AUDIO_TOTAL_BYTES, 100) == HAL_OK)
    {
        combined_audio_frames++;
    }
}

static void ProcessRemoteAudioFrame(AudioFrame_t *frame)
{
    valid_frame_count++;
    if ((valid_frame_count % 16U) == 0U)
    {
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    }
    BuildAndForwardCombinedFrame(frame);
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0) mic0_half_ready[0] = 1;
    if (hdfsdm_filter == &hdfsdm1_filter1) mic1_half_ready[0] = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0) mic0_half_ready[1] = 1;
    if (hdfsdm_filter == &hdfsdm1_filter1) mic1_half_ready[1] = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  AudioFrame_t rx_frame;
  uint16_t expected_frame_id = 0;
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
  MX_DFSDM1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, mic1_dma,
                                       MIC_DMA_SAMPLES) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, mic0_dma,
                                       MIC_DMA_SAMPLES) != HAL_OK)
  {
      Error_Handler();
  }
  /* Let the transmitter boot and arm its SPI slave before we clock it. */
  HAL_Delay(RECEIVER_STARTUP_DELAY_MS);
  /*
   * Board A is the SPI master. It drives hardware NSS on PA4 low and
   * supplies SPI clocks,
   * and reads one complete frame from the transmitter slave.
   */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    ServiceLocalMicrophones();

    /* Do not clock SPI while the transmitter is booting or recovering. */
    if (!TransmitterReady())
    {
        HAL_Delay(1);
        continue;
    }

    if (ReceiveAudioFrame_SPI(&rx_frame))
    {
        /*
         * Optional frame ID check.
         */
        if (rx_frame.frame_id != expected_frame_id)
        {
            /*
             * Frame skip/desync happened.
             * For now, resync the expected ID.
             */
            if (spi_valid_frames > 1U)
            {
                spi_frame_id_gaps +=
                    (uint16_t)(rx_frame.frame_id - expected_frame_id);
            }
            expected_frame_id = rx_frame.frame_id;
        }

        expected_frame_id++;

        ProcessRemoteAudioFrame(&rx_frame);
    }
    else
    {
        /*
         * Receive failed or the slave was not ready.
         * The next SPI transaction will retry the frame.
         */
    }

    /* The SPI transaction and UART forwarding determine the transfer rate. */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 200;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SYNC_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 200;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 25;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x03;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 25;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @note  Receiver A is the SPI master; PA4 is hardware NSS.
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : TRANSMITTER_READY_Pin */
  GPIO_InitStruct.Pin = TRANSMITTER_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TRANSMITTER_READY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
