/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
void prvvUARTTxReadyISR( void );
void prvvUARTRxISR( void );
static CHAR rxBuffer[1];
CHAR rxBuffer1[256];
/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    if (xRxEnable)															
        {
            /* ����2�����ж�ʹ�� */
            __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);	
            #if defined(MODBUS_MASTER_USE_CONTROL_PIN)	
                /* 485�͵�ƽ���� */
                HAL_GPIO_WritePin(MODBUS_MASTER_GPIO_PORT,MODBUS_MASTER_GPIO_PIN,MODBUS_MASTER_GPIO_PIN_LOW);
            #endif
        }
    else
        {
            /* ����2�����жϹر� */
            __HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
            #if defined(MODBUS_MASTER_USE_CONTROL_PIN)
                /* 485�ߵ�ƽ���� */
                HAL_GPIO_WritePin(MODBUS_MASTER_GPIO_PORT,MODBUS_MASTER_GPIO_PIN,MODBUS_MASTER_GPIO_PIN_HIGH);
            #endif
        }
    if (xTxEnable)
        {
            /* ����2�����ж�ʹ�� */
            __HAL_UART_ENABLE_IT(&huart2,UART_IT_TXE);
            #if defined(MODBUS_MASTER_USE_CONTROL_PIN)
                /* 485�ߵ�ƽ����*/
                HAL_GPIO_WritePin(MODBUS_MASTER_GPIO_PORT,MODBUS_MASTER_GPIO_PIN,MODBUS_MASTER_GPIO_PIN_HIGH);
            #endif
        }
    else
        {
            /* ����2�����жϹر� */
            __HAL_UART_DISABLE_IT(&huart2,UART_IT_TXE);
            #if defined(MODBUS_MASTER_USE_CONTROL_PIN)	
                /* 485�͵�ƽ����*/
                HAL_GPIO_WritePin(MODBUS_MASTER_GPIO_PORT,MODBUS_MASTER_GPIO_PIN,MODBUS_MASTER_GPIO_PIN_LOW);
            #endif
        }
}

// TODO: GPIO����
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
		/* ʹ��485ʱ��Ҫ��usart.h�д�RT_MODBUS_MASTER_USE_CONTROL_PIN�궨�� */
	#if defined(MODBUS_MASTER_USE_CONTROL_PIN)	
		modbus_master_control_init();
	#endif

	MX_USART2_UART_Init();
	return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */

    if(HAL_UART_Transmit (&huart2 ,(uint8_t *)&ucByte,1,0x01) != HAL_OK )	//添加发送一位代码
        return FALSE ;
    else
        return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
    if(HAL_UART_Receive (&huart2 ,(uint8_t *)pucByte,1,0x01) != HAL_OK )//添加接收一位代码
            return FALSE ;
    else
			return TRUE;
	// *pucByte = rxBuffer[0];
//	return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
// int i = 0;
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if(huart->Instance == USART2)
//     {
// 			HAL_UART_Receive_IT(&huart2, rxBuffer, 1);  // �?动�??一次接�?
// 							rxBuffer1[i++] = rxBuffer[0];
// 							if(i >= 254)
// 									i = 0;
//        prvvUARTRxISR();
//     }
//     HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
//     HAL_UART_IRQHandler(&huart2);
// }

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if(huart->Instance == USART2)
//     {
//         prvvUARTTxReadyISR();
//     }

//     HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
//     HAL_UART_IRQHandler(&huart2);
// }
