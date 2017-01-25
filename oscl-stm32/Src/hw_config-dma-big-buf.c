#include "stm32_it.h"
#include "usb_regs.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "stm32f10x_adc.h"

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
bool      CH_active[MAX_ADC_CH]={TRUE, FALSE, FALSE, FALSE};
__IO uint8_t ADC_Values[MAX_ADC_CH*2];
uint32_t  to_fill=0, to_send=0; //, fill_count=0;  // buffer number for filling and sending
//uint8_t   PacketNum=0;
__IO uint8_t   buf[BUF_NUMS][BULK_MAX_PACKET_SIZE];


void set_DMA1_ch1()
{
  DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);
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
  DMA1_Channel1->CCR = DMA_DIR_PeripheralSRC | DMA_Mode_Normal | DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
                       DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte | DMA_Priority_High | DMA_M2M_Disable;
  // Write to DMAy Channelx CNDTR
  DMA1_Channel1->CNDTR = BULK_MAX_PACKET_SIZE; // -2
  // Write to DMAy Channelx CPAR
  DMA1_Channel1->CPAR = ((uint32_t)&ADC1->DR)+1;
  // Write to DMAy Channelx CMAR
  DMA1_Channel1->CMAR = (uint32_t)(buf[to_fill]);  // memory address
  buf[to_fill][0]=to_fill;
  if( ++to_fill>=BUF_NUMS ) to_fill=0;

  DMA1_Channel1->CCR |= DMA_CCR1_EN | DMA_IT;  // DMA_Cmd( DMA1_Channel1, ENABLE );
  // DMA1_Channel1->CCR |= DMA_IT;       // DMA_ITConfig( DMA1_Channel1, DMA1_IT_TC1, ENABLE );
}

void DMA1_Channel1_IRQHandler(void)
{
//    DMA_ITConfig( DMA1_Channel1, DMA1_IT_TC1, DISABLE );
    *((uint32_t*)(buf[to_fill])+0)=0xffffffff;
    *((uint32_t*)(buf[to_fill])+1)=to_fill;
    *((uint32_t*)(buf[to_fill])+2)=DMA1_Channel1->CNDTR;
    *((uint32_t*)(buf[to_fill])+3)=DMA1_Channel1->CMAR;
    if( !to_fill ) return;
    set_DMA1_ch1();
//    DMA_ITConfig( DMA1_Channel1, DMA1_IT_TC1, ENABLE );
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
    DMA1_Channel1->CCR |= DMA_CCR1_EN; // DMA_Cmd(DMA1_Channel1, ENABLE);
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

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE); // ADC clock
//    ADC1->CR2 |= ADC_CR2_RSTCAL;            // reset ADC calibration
//    while( !(ADC1->CR2 & ADC_CR2_RSTCAL) ); // wait for reset end
    ADC1->CR2 |= ADC_CR2_CAL;               // start ADC calibration
    while( !(ADC1->CR2 & ADC_CR2_CAL) );    // wait for calibration end

    ADC1->SMPR2 |= ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0 |
                   ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0 |
                   ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0 |
                   ADC_SMPR2_SMP3_2 | ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0 ; // sample time for channels 0-3
    ADC1->CR1 |= ADC_CR1_SCAN;      // scan channels mode
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ALIGN;      // continuous conversion

    SetADCChannels();  // configure channels for ADC and DMA

    ADC1->CR2 |= ADC_CR2_ADON;      // ADC power on
    int j;  for(j=0; j<1000; j++);  // ADC stabilization time
    ADC1->CR2 |= ADC_CR2_ADON;      // start conversion
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

uint32_t CDC_Send_DATA( __IO uint8_t* ptrBuffer, uint8_t Send_length)
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
  return 1 ;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/*
// Have to send a program to calculate the total number of data tables (including ZLP)
// For each IN interrupt, put the total number of packets to be sent -1
usb_in_numofpackage -;
if (GetENDPOINT (ENDP1) & EP_DTOG_RX)
{
      if( usb_in_numofpackage>0 )
           // Enable next package
           // FreeUserBuffer is to switch the current cache Role
           // If there is a packet to be transmitted, the switch cache (data ready)
           FreeUserBuffer (ENDP1, EP_DBUF_IN);

      // Usb_in_data_remain is needed to put the cached data length
      // Send function in USB, first fills two buffers, usb_in_data_remain -2 total length of the buffer length
      if( usb_in_data_remain>0 )
      {
           // There is data to be transmitted
           if (usb_in_data_remain> VIRTUAL_COM_PORT_DATA_SIZE)
               len = VIRTUAL_COM_PORT_DATA_SIZE;
           else
               len = usb_in_data_remain;
           // Copy the data to the free buffer, and set the length
           UserToPMABufferCopy (buffer_in, ENDP1_BUF0ADDR, len);
           SetEPDblBuf0Count (ENDP1, EP_DBUF_IN, len);
           // Usb_in_data_remain minus the length of the buffer has been placed
           usb_in_data_remain - = len;
           // Update the data pointer
           buffer_in + = len;
      }
      else
          SetEPDblBuf0Count (ENDP1, EP_DBUF_IN, 0);   // Data has been placed in the buffer, the buffer length is set to 0 idle
}
else
{
      if (usb_in_numofpackage> 0)
          FreeUserBuffer (ENDP1, EP_DBUF_IN);    // Enable next package
      if (usb_in_data_remain> 0)
      {
           if (usb_in_data_remain> VIRTUAL_COM_PORT_DATA_SIZE)
               len = VIRTUAL_COM_PORT_DATA_SIZE;
           else
               len = usb_in_data_remain;
           UserToPMABufferCopy (buffer_in, ENDP1_BUF1ADDR, len);
           SetEPDblBuf1Count (ENDP1, EP_DBUF_IN, len);
           usb_in_data_remain -= len;
           buffer_in + = len;
      }
      else
         SetEPDblBuf1Count (ENDP1, EP_DBUF_IN, 0);
}
// */

