/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2016 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_sau.h
* Version      : Applilet4 for R7F0C003/004 V1.00.00.01 [01 Mar 2016]
* Device(s)    : R7F0C004M
* Tool-Chain   : CCRL
* Description  : This file implements device driver for SAU module.
* Creation Date: 2023/10/10 星期二
***********************************************************************************************************************/
#ifndef SAU_H
#define SAU_H

#define MD_STATUSBASE        (0x00U)
#define MD_OK                (MD_STATUSBASE + 0x00U) /* register setting OK */
#define MD_SPT               (MD_STATUSBASE + 0x01U) /* IIC stop */
#define MD_NACK              (MD_STATUSBASE + 0x02U) /* IIC no ACK */
#define MD_BUSY1             (MD_STATUSBASE + 0x03U) /* busy 1 */
#define MD_BUSY2             (MD_STATUSBASE + 0x04U) /* busy 2 */
#define MD_OVERRUN           (MD_STATUSBASE + 0x05U) /* IIC OVERRUN occur */

/* Error list definition */
#define MD_ERRORBASE         (0x80U)
#define MD_ERROR             (MD_ERRORBASE + 0x00U)  /* error */
#define MD_ARGERROR          (MD_ERRORBASE + 0x01U)  /* error agrument input error */
#define MD_ERROR1            (MD_ERRORBASE + 0x02U)  /* error 1 */
#define MD_ERROR2            (MD_ERRORBASE + 0x03U)  /* error 2 */
#define MD_ERROR3            (MD_ERRORBASE + 0x04U)  /* error 3 */
#define MD_ERROR4            (MD_ERRORBASE + 0x05U)  /* error 4 */
#define MD_ERROR5            (MD_ERRORBASE + 0x06U)  /* error 5 */


typedef int16_t      MD_STATUS;

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void App_LpUart0Cfg(uint32_t baud,int16_t parity);
void App_LpUart0DeCfg(void);
void R_LPUART0_Start(void);
void R_LPUART0_Stop(void);
MD_STATUS R_LPUART0_Send(uint8_t * const tx_buf, uint16_t tx_num);
MD_STATUS R_LPUART0_Receive(uint8_t * const rx_buf, uint16_t rx_num);

void App_LpUart1Cfg(uint32_t baud,int16_t parity);
void R_LPUART1_Start(void);
void R_LPUART1_Stop(void);
MD_STATUS R_LPUART1_Send(uint8_t * const tx_buf, uint16_t tx_num);
MD_STATUS R_LPUART1_Receive(uint8_t * const rx_buf, uint16_t rx_num);

void App_Uart0Cfg(uint32_t baud,int16_t parity);
void R_UART0_Start(void);
void R_UART0_Stop(void);
MD_STATUS R_UART0_Send(uint8_t * const tx_buf, uint16_t tx_num);
MD_STATUS R_UART0_Receive(uint8_t * const rx_buf, uint16_t rx_num);

void App_Uart1Cfg(uint32_t baud,int16_t parity);
void R_UART1_Start(void);
void R_UART1_Stop(void);
MD_STATUS R_UART1_Send(uint8_t * const tx_buf, uint16_t tx_num);
MD_STATUS R_UART1_Receive(uint8_t * const rx_buf, uint16_t rx_num);

void App_Uart2Cfg(uint32_t baud,int16_t parity);
void R_UART2_Start(void);
void R_UART2_Stop(void);
MD_STATUS R_UART2_Send(uint8_t * const tx_buf, uint16_t tx_num);
MD_STATUS R_UART2_Receive(uint8_t * const rx_buf, uint16_t rx_num);

void App_Uart3Cfg(uint32_t baud,int16_t parity);
void R_UART3_Start(void);
void R_UART3_Stop(void);
MD_STATUS R_UART3_Send(uint8_t * const tx_buf, uint16_t tx_num);
MD_STATUS R_UART3_Receive(uint8_t * const rx_buf, uint16_t rx_num);



/* Start user code for function. Do not edit comment generated here */

void enable_LPuart0(void);
void disable_LPuart0(void);
void enable_LPuart1(void);
void disable_LPuart1(void);
void enable_uart0(void);
void disable_uart0(void);
void enable_uart1(void);
void disable_uart1(void);
void enable_uart2(void);
void disable_uart2(void);
void disable_uart3(void);
void enable_uart3(void);



//void enable_irc_receive(void);
//void disable_irc_receive(void);
//void disable_irc(void);
//void enable_irc(void);
//int16_t irc_modify_baud(int16_t baud,int16_t parity);


/* End user code. Do not edit comment generated here */
#endif