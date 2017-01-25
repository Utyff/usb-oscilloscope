//#include "stm32f10x_adc.h"
#include "stm32_it.h"
#include "usb_regs.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;
extern __IO uint32_t packet_sent;
extern __IO uint8_t Send_Buffer[VIRTUAL_COM_PORT_DATA_SIZE] ;
extern __IO  uint32_t packet_receive;
extern __IO uint8_t Receive_length;

uint8_t Receive_Buffer[64];
uint32_t Send_length;
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

#define ADC1_DR_Address    ((uint32_t)0x4001244C)

ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
__IO uint16_t ADCConvertedValue;


void ADC_init()
{
    GPIO_InitTypeDef PORT;
   //Включаем порты А и B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    //Настраиваем ноги PB8 и PB9 на выход. Там у нас висят светодиоды
    PORT.GPIO_Pin = (GPIO_Pin_8 | GPIO_Pin_9);
    PORT.GPIO_Mode = GPIO_Mode_Out_PP;
    PORT.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( GPIOB , &PORT );
    // Configure PA00 (ADC Channel0) as analog input
    PORT.GPIO_Pin = GPIO_Pin_0;
    PORT.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &PORT); // */

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE); //Включаем тактирование АЦП
    ADC1->CR2 |= ADC_CR2_CAL;           //Запуск калибровки АЦП
    while (!(ADC1->CR2 & ADC_CR2_CAL)); //Ожидаем окончания калибровки

    ADC1->SMPR2 |= (ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0); //Задаем длительность выборки
    ADC1->CR2 |= ADC_CR2_JEXTSEL;   //Преобразование инжектированной группы
                                    //запустится установкой бита JSWSTART
    ADC1->CR2 |= ADC_CR2_JEXTTRIG;  //Разрешаем внешний запуск инжектированной группы
    ADC1->CR2 |= ADC_CR2_CONT;      //Преобразования запускаются одно за другим
    ADC1->CR1 |= ADC_CR1_JAUTO;     //Разрешить преобразование инжектированной группы
                                    //после регулярной. Не понятно зачем, но без этого не работает
    //ADC1->JSQR |= (1<<15);          //Задаем номер канала (выбран ADC1)
    ADC1->CR2 |= ADC_CR2_ADON;      //Теперь включаем АЦП

    ADC1->CR2 |= ADC_CR2_JSWSTART;  //Запуск преобразований
    while (!(ADC1->SR & ADC_SR_JEOC)); //ждем пока первое преобразование завершится

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

uint32_t CDC_Send_DATA(uint8_t* ptrBuffer, uint8_t Send_length)
{
  if(Send_length > VIRTUAL_COM_PORT_DATA_SIZE) return 1;


  if (GetENDPOINT(ENDP1) & EP_DTOG_RX) // NOT TX ie SW_BUF
  {
    UserToPMABufferCopy(ptrBuffer, ENDP1_BUF0ADDR, Send_length);
    SetEPDblBuf0Count(ENDP1, EP_DBUF_IN, Send_length);
  }
  else
  {
    UserToPMABufferCopy(ptrBuffer, ENDP1_BUF1ADDR, Send_length);
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

