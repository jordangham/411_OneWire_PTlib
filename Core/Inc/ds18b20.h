#ifndef __DS18B20_H__
#define __DS18B20_H__
#include "usart.h"
void OneWire_Init(void);
void OneWire_UARTInit(uint32_t baudRate);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uarth);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uarth);
void OneWire_Execute(uint8_t ROM_Command,uint8_t* ROM_Buffer,
                     uint8_t Function_Command,uint8_t* Function_buffer);
void StateMachine(void);
void OneWire_SetCallback(void(*OnComplete)(void), void(*OnErr)(void));
uint8_t ROMStateMachine(void);
uint8_t FunctionStateMachine(void);
void OneWire_TxCpltCallback(void);
void OneWire_RxCpltCallback(void);
#endif
