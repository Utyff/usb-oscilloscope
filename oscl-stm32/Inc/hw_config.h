#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#include "platform_config.h"
#include "usb_type.h"

#define BULK_MAX_PACKET_SIZE  0x00000040
#define BUF_NUMS 16
#define MAX_ADC_CH 4


extern uint8_t  buf[BUF_NUMS][BULK_MAX_PACKET_SIZE];
extern uint32_t to_fill, to_send, fill_count;  // buffer number for filling and sending


void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void Get_SerialNum(void);
void LCD_Control(void);
uint32_t CDC_Send_DATA(uint8_t *ptrBuffer, uint8_t Send_length);
uint32_t CDC_Receive_DATA(void);

void SetADCChannels();

#endif  /*__HW_CONFIG_H*/
