#include "stm32_it.h"
#include "usb_regs.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;
extern __IO uint32_t packet_sent;
extern __IO uint8_t  Send_Buffer[BULK_MAX_PACKET_SIZE];
extern __IO uint32_t packet_receive;
extern __IO uint8_t  Receive_length;

uint8_t Receive_Buffer[BULK_MAX_PACKET_SIZE];
uint32_t Send_length;

static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);


uint32_t  active_ch_nums=1;
bool      CH_active[MAX_ADC_CH]={TRUE, TRUE, TRUE, TRUE};
__IO uint8_t ADC_Values[MAX_ADC_CH];
uint32_t  to_fill=0, to_send=0, fill_count=0;  // buffer number for filling and sending
//uint8_t   PacketNum=0;
uint8_t   buf[BUF_NUMS][BULK_MAX_PACKET_SIZE];


void set_DMA1_ch1()
{
//  DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);
  // Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits
  /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
  /* Set DIR bit according to DMA_DIR value */
  /* Set CIRC bit according to DMA_Mode value */
  /* Set PINC bit according to DMA_PeripheralInc value */
  /* Set MINC bit according to DMA_MemoryInc value */
  /* Set PSIZE bits according to DMA_PeripheralDataSize value */
  /* Set MSIZE bits according to DMA_MemoryDataSize value */
  /* Set PL bits according to DMA_Priority value */
  /* Set the MEM2MEM bit according to DMA_M2M value */
  // Write to DMAy Channelx CCR
  DMA1_Channel1->CCR = DMA_DIR_PeripheralSRC | DMA_Mode_Circular | DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
                       DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte | DMA_Priority_High | DMA_M2M_Disable;
  // Write to DMAy Channelx CNDTR
  DMA1_Channel1->CNDTR = active_ch_nums ? active_ch_nums : 1;
  // Write to DMAy Channelx CPAR
  DMA1_Channel1->CPAR = ((uint32_t)&ADC1->DR)+1;
  // Write to DMAy Channelx CMAR
  DMA1_Channel1->CMAR = (uint32_t)&ADC_Values;  // memory address

  DMA1_Channel1->CCR |= DMA_CCR1_EN | DMA1_IT_TC1;  // DMA_Cmd( DMA1_Channel1, ENABLE );
//  DMA1_Channel1->CCR |= DMA1_IT_TC1;             // DMA_ITConfig( DMA1_Channel1, DMA1_IT_TC1, ENABLE );
}

uint32_t DWT_CNT=0;
uint16_t TIM_CNT=0;
uint32_t TIM_OVR=0;
// Обработчик прерывания TIM6_DAC
void TIM1_UP_IRQHandler(void)
{
  TIM1->SR &= ~TIM_SR_UIF; //Сбрасываем флаг UIF
  TIM_OVR +=1;
}


void DMA1_Channel1_IRQHandler(void)
{
//   DMA_ITConfig( DMA1_Channel1, DMA1_IT_TC1, DISABLE );

        DWT_CNT = DWT_CYCCNT;
        DWT_CYCCNT = 0;
        TIM_CNT = TIM1->CNT;
        TIM1->CNT = 0;

    uint32_t  i=0;
    for( i=0; i<active_ch_nums; i++ )
        buf[to_fill][fill_count++] = ADC_Values[i];

    if( fill_count>=BULK_MAX_PACKET_SIZE )
    {
       fill_count=0;
       if( ++to_fill>=BUF_NUMS ) to_fill=0;
    }

//   DMA_ITConfig( DMA1_Channel1, DMA1_IT_TC1, ENABLE );
    DMA_ClearITPendingBit( DMA1_IT_TC1 );
}


void DMA_init()
{
    // configure interrupt on DMA transfer complete
    NVIC_InitTypeDef    NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); // */

    //  Enable DMA1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    //  Configure DMA1 - Channel1 for ADC
    set_DMA1_ch1();

//    DMA_ITConfig(DMA1_Channel1, DMA1_IT_TC1, ENABLE);

#define CR2_DMA_Set                 ((uint32_t)0x00000100)
    ADC1->CR2 |= CR2_DMA_Set;          // ADC_DMACmd(ADC1, ENABLE);       // Enable ADC1 DMA
//    DMA1_Channel1->CCR |= DMA_CCR1_EN; // DMA_Cmd(DMA1_Channel1, ENABLE);
}

void SetADCChannels()
{
    uint32_t i, n=0, SQR=0;

    DMA_Cmd(DMA1_Channel1, DISABLE);

    for( i=0; i< MAX_ADC_CH; i++) // count active channels and add them to scan list SQR
        if( CH_active[i]==TRUE )
        {
            SQR |= i << (5*n);
            n++;
        }

    active_ch_nums = n;
    if( n )
    {
        ADC1->SQR1 = (n-1) << 20;  // numbers of channels for scan
        ADC1->SQR3 = SQR;          // channel numbers for scan list
    }
    else
    {  // if no channels for scan - scan only channel 0
        ADC1->SQR1 = 0;
        ADC1->SQR3 = 0;
    }
    DMA_init();
}

void ADC_init()
{
    GPIO_InitTypeDef PORT;
    //  clock for ports A & B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // PB8 and PB9 for LED
    PORT.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    PORT.GPIO_Mode  = GPIO_Mode_Out_PP;
    PORT.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( GPIOB , &PORT );

    // Configure PA0-3 (ADC Channel0-3) as analog input
    PORT.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    PORT.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init( GPIOA, &PORT );

//    RCC-> CFGR |= RCC_CFGR_ADCPRE_DIV6;
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE); // ADC clock
//    ADC1->CR2 |= ADC_CR2_RSTCAL;            // reset ADC calibration
//    while( !(ADC1->CR2 & ADC_CR2_RSTCAL) ); // wait for reset end
    ADC1->CR2 |= ADC_CR2_CAL;               // start ADC calibration
    while( !(ADC1->CR2 & ADC_CR2_CAL) );    // wait for calibration end

/*    ADC1->SMPR2 |= ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0 |
                   ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0 |
                   ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0 |
                   ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0 ; // sample time for channels 0-3 */
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 |// ADC_SMPR2_SMP0_0 |
                   ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 |// ADC_SMPR2_SMP1_0 |
                   ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP2_1 |// ADC_SMPR2_SMP2_0 |
                   ADC_SMPR2_SMP3_2 | ADC_SMPR2_SMP3_1 ;//| ADC_SMPR2_SMP3_0 ; // sample time for channels 0-3 */
    ADC1->CR1 |= ADC_CR1_SCAN;      // scan channels mode
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ALIGN;      // continuous conversion

    SetADCChannels();  // configure channels for ADC and DMA

    ADC1->CR2 |= ADC_CR2_ADON;      // ADC power on
    int j;  for(j=0; j<1000; j++);  // ADC stabilization time
    ADC1->CR2 |= ADC_CR2_ADON;      // start conversion
}


void Timer_init()
{
    GPIO_InitTypeDef PORT;
    //  clock for ports A, B and Alternate functions
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM4, ENABLE);

    // PB5 for TIM3 ch2  and  PB6 for TIM4 ch1
    PORT.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    PORT.GPIO_Mode  = GPIO_Mode_AF_PP;
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &PORT );
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // Remap TIM3 CH2 to PB5

  SCB_DEMCR  |= 0x01000000;
  DWT_CONTROL|= 1; // включаем счётчик
  DWT_CYCCNT  = 0;

// Timer 1 for time count
    TIM1->PSC = 0;              // Настраиваем делитель
    TIM1->ARR = 0xFFFF;         // Предел счета
    TIM1->DIER |= TIM_DIER_UIE; // разрешаем прерывание от таймера
    TIM1->CR1 |= TIM_CR1_CEN;   // Начать отсчёт!
//    NVIC_EnableIRQ(TIM2_IRQn);   // Разрешение TIM2_IRQn прерывания
    NVIC_EnableIRQ(TIM1_UP_IRQn);  // Разрешение TIM_IRQn прерывания


    // initialization TIM3 ch2 and TIM4 ch1 as generators
    TIM3->CR1 |= TIM_CR1_ARPE;     // Включен режим предварительной записи регистра автоперезагрузки
    TIM3->CCMR1 |= TIM_CCMR1_OC2PE;// Включен режим предварительной загрузки регистра сравнения
    TIM3->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); // OC1M = 110 - PWM mode 1
    TIM3->PSC = 2;                 // Prescale
    TIM3->ARR = 63999;             // Период выходного сигнала T
    TIM3->CCR2 = 13200;            // Длительность импульса Duty cycle
    TIM3->CCER |= TIM_CCER_CC2E;   // Выход канала захвата/сравнения включен
    TIM3->CR1 |= TIM_CR1_CEN;      // start timer 3 */

    TIM4->CR1 |= TIM_CR1_ARPE;     // Включен режим предварительной записи регистра автоперезагрузки
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;// Включен режим предварительной загрузки регистра сравнения
    TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // OC1M = 110 - PWM mode 1
    TIM4->PSC = 2;                 // Prescale
    TIM4->ARR = 63999;             // Период выходного сигнала T
    TIM4->CCR1 = 20200;            // Длительность импульса Duty cycle
    TIM4->CCER |= TIM_CCER_CC1E;   // Выход канала захвата/сравнения включен
    TIM4->CR1 |= TIM_CR1_CEN;      // start timer 4  */
}


void Set_System(void)
{
#if !defined(STM32L1XX_MD) && !defined(STM32L1XX_HD) && !defined(STM32L1XX_MD_PLUS)
  GPIO_InitTypeDef GPIO_InitStructure;
#endif /* STM32L1XX_MD && STM32L1XX_XD */

#if defined(USB_USE_EXTERNAL_PULLUP)
  GPIO_InitTypeDef  GPIO_InitStructure;
#endif /* USB_USE_EXTERNAL_PULLUP */

  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS) || defined(STM32F37X) || defined(STM32F30X)
  /* Enable the SYSCFG module clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif /* STM32L1XX_XD */

#if !defined(STM32L1XX_MD) && !defined(STM32L1XX_HD) && !defined(STM32L1XX_MD_PLUS) && !defined(STM32F37X) && !defined(STM32F30X)
  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up pin */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32L1XX_MD && STM32L1XX_XD */

#if defined(USB_USE_EXTERNAL_PULLUP)
  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);

  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* USB_USE_EXTERNAL_PULLUP */

#if defined(STM32F37X) || defined(STM32F30X)

  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);

  /*Set PA11,12 as IN - USB_DM,DP*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*SET PA11,12 for USB: USB_DM,DP*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_14);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_14);

  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32F37X  && STM32F30X)*/

   /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  ADC_init();
  Timer_init();
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD) || defined(STM32L1XX_MD_PLUS)
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

#else
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
    /*Enable SystemCoreClock*/
  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

#if defined(STM32L1XX_MD)|| defined(STM32L1XX_HD) || defined(STM32L1XX_MD_PLUS)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#elif defined(STM32F37X)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#else
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
#endif
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#if defined(STM32L1XX_MD) || defined (STM32L1XX_HD)|| (STM32L1XX_MD_PLUS)
  if (NewState != DISABLE)
  {
    STM32L15_USB_CONNECT;
  }
  else
  {
    STM32L15_USB_DISCONNECT;
  }

#else /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}


/*******************************************************************************
* Function Name  : Send DATA.
* Description    : send the data received from the STM32 to the PC through USB
* Input          : None.
* Output         : None.
* Return         : None.
******************************************************************************
uint32_t CDC_Send_DATA (uint8_t *ptrBuffer, uint8_t Send_length)
{
  //if max buffer is Not reached
  if(Send_length <= VIRTUAL_COM_PORT_DATA_SIZE)
  {
    //Sent flag
    packet_sent = 0;
    // send  packet to PMA
    UserToPMABufferCopy((unsigned char*)ptrBuffer, ENDP1_TXADDR, Send_length);
    SetEPTxCount(ENDP1, Send_length);
    SetEPTxValid(ENDP1);
  }
  else
  {
    return 0;
  }
  return 1;
}*/

//static uint32_t  qq=1;

uint32_t CDC_Send_DATA(uint8_t* ptrBuffer, uint8_t Send_length)
{
//  if(Send_length > BULK_MAX_PACKET_SIZE) return 1;

  if (GetENDPOINT(ENDP1) & EP_DTOG_RX) // NOT TX ie SW_BUF
  {
    UserToPMABufferCopy((uint8_t*)ptrBuffer, ENDP1_BUF0ADDR, Send_length);
    SetEPDblBuf0Count(ENDP1, EP_DBUF_IN, Send_length);
  }
  else
  {
    UserToPMABufferCopy((uint8_t*)ptrBuffer, ENDP1_BUF1ADDR, Send_length);
    SetEPDblBuf1Count(ENDP1, EP_DBUF_IN, Send_length);
  }

  FreeUserBuffer(ENDP1, EP_DBUF_IN); // Toggles EP_DTOG_RX / SW_BUF
  //if( qq ) qq=0,
        SetEPTxValid(ENDP1);

  packet_sent = 0;

  return 0;
}


/*******************************************************************************
* Function Name  : Receive DATA .
* Description    : receive the data from the PC to STM32 and send it through USB
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t CDC_Receive_DATA(void)
{
  /*Receive flag*/
  packet_receive = 0;
  SetEPRxValid(ENDP3);
  return 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
