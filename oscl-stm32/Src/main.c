#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "string.h"

extern __IO  uint8_t  Receive_Buffer[64];
extern __IO  uint32_t Receive_length;
uint32_t packet_sent=1;
uint32_t packet_receive=1;


void SendVersion()
{
    uint8_t  cmd_buf[BULK_MAX_PACKET_SIZE];
    int  i;

#define CR2_DMA_Set                 ((uint32_t)0x00000100)
#define CR2_DMA_Reset               ((uint32_t)0xFFFFFEFF)

    ADC1->CR2 &= CR2_DMA_Reset; // Disable DMA

    cmd_buf[0]=1; cmd_buf[1]=0; cmd_buf[2]=1;
    strcpy( (char*)cmd_buf+3, "Oscilograph 32 (M3)" );
    while( packet_sent != 1 );
    for( i=0; i<10000; i++);
    CDC_Send_DATA( cmd_buf, BULK_MAX_PACKET_SIZE );

    ADC1->CR2 |= CR2_DMA_Set;   // Enable DMA
}

void ParseCommand()
{
    if( Receive_Buffer[0]!='A' || Receive_Buffer[1]!='T' )  return;

    if( Receive_Buffer[2]=='L' && Receive_Buffer[3]=='1' )
    {
        if( Receive_Buffer[4]=='1' )      GPIOB->ODR &= ~GPIO_Pin_9;
        else if( Receive_Buffer[4]=='0' ) GPIOB->ODR |= GPIO_Pin_9;
        return;
    }

/*    if( Receive_Buffer[2]=='I' )
    {
        SendVersion();
        return;
    }*/

}


#define pin_S1  GPIO_Pin_13  // C13
#define pin_S2  GPIO_Pin_6   // A6
#define pin_S3  GPIO_Pin_7   // A7
#define pin_S4  GPIO_Pin_0   // B0
#define pin_S5  GPIO_Pin_1   // B1
#define pin_S6  GPIO_Pin_2   // B2
#define pin_S7  GPIO_Pin_10  // B10
#define pin_S8  GPIO_Pin_11  // B11
#define pin_S9  GPIO_Pin_12  // B12
#define pin_S10 GPIO_Pin_13  // B13
#define pin_S11 GPIO_Pin_14  // B14
#define pin_S12 GPIO_Pin_15  // B15




int CheckButtons()
{
    uint32_t    result; // =0;

    result  = (GPIOC->IDR & pin_S1) >> 13;
    result |= (GPIOA->IDR & (pin_S2|pin_S3)) >> 5;
    result |= (GPIOB->IDR & (pin_S4|pin_S5|pin_S6)) << 3;
    result |= (GPIOB->IDR & (pin_S7|pin_S8|pin_S9|pin_S10|pin_S11|pin_S12)) >> 4;

    return result;
}

void InitButtonsPins()
{

    GPIO_InitTypeDef Port;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    Port.GPIO_Pin = pin_S1;
    Port.GPIO_Mode = GPIO_Mode_IPU;
    Port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( GPIOC, &Port );

    Port.GPIO_Pin = pin_S2 | pin_S3;
//    Port.GPIO_Mode = GPIO_Mode_IPU;
//    Port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( GPIOA, &Port );

    Port.GPIO_Pin = pin_S4 | pin_S5 | pin_S6 | pin_S7 | pin_S8 | pin_S9 | pin_S10 | pin_S11 | pin_S12 ;
//    Port.GPIO_Mode = GPIO_Mode_IPU;
//    Port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( GPIOB, &Port );

}

int  iButtons;

int main(void)
{
    uint32_t i=0, j;

    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
    InitButtonsPins();

    GPIOB->ODR ^= GPIO_Pin_8;

    i=0;
    while(1)
    {
        iButtons = CheckButtons();

        if( bDeviceState == CONFIGURED )
        {
            if( Receive_length != 0 )  //  Check if we have data yet
            {
                ParseCommand();
                Receive_Buffer[63]=0xa5;
                for(j=0; j<BULK_MAX_PACKET_SIZE/4; j++) ((uint32_t*)Receive_Buffer)[j]=0;
                CDC_Receive_DATA();
                Receive_length = 0;
            }

            if( packet_sent == 1 )
                if( to_send!=to_fill )
                {
                    CDC_Send_DATA( buf[to_send], BULK_MAX_PACKET_SIZE );
                    if( ++to_send >= BUF_NUMS ) to_send=0;
                }

        }

        if( i++ > 0x3FFFF )
        {
            GPIOB->ODR ^= GPIO_Pin_8;
            i=0;
        }
    } // while(1)

} // main()



#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void a-ssert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif
