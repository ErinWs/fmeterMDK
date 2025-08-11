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
* File Name    : r_cg_sau.c
* Version      : Applilet4 for R7F0C003/004 V1.00.00.01 [01 Mar 2016]
* Device(s)    : R7F0C004M
* Tool-Chain   : CCRL
* Description  : This file implements device driver for SAU module.
* Creation Date: 2023/10/10 星期二
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "ddl.h"
#include "lpuart.h"
#include "uart.h"
#include "gpio.h"
/* Start user code for include. Do not edit comment generated here */
#include "r_cg_sau.h"
/* End user code. Do not edit comment generated here */


/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */


/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
volatile uint8_t * gp_LPuart0_tx_address;        /* LPuart0 transmit buffer address */
volatile uint16_t  g_LPuart0_tx_count;           /* LPuart0 transmit data number */
volatile uint8_t * gp_LPuart0_rx_address;        /* LPuart0 receive buffer address */
volatile uint16_t  g_LPuart0_rx_count;           /* LPuart0 receive data number */
volatile uint16_t  g_LPuart0_rx_length;          /* LPuart0 receive data length */
volatile uint8_t * gp_LPuart1_tx_address;        /* LPuart1 transmit buffer address */
volatile uint16_t  g_LPuart1_tx_count;           /* LPuart1 transmit data number */
volatile uint8_t * gp_LPuart1_rx_address;        /* LPuart1 receive buffer address */
volatile uint16_t  g_LPuart1_rx_count;           /* LPuart1 receive data number */
volatile uint16_t  g_LPuart1_rx_length;          /* LPuart1 receive data length */

volatile uint8_t * gp_uart0_tx_address;        /* uart0 transmit buffer address */
volatile uint16_t  g_uart0_tx_count;           /* uart0 transmit data number */
volatile uint8_t * gp_uart0_rx_address;        /* uart0 receive buffer address */
volatile uint16_t  g_uart0_rx_count;           /* uart0 receive data number */
volatile uint16_t  g_uart0_rx_length;          /* uart0 receive data length */
volatile uint8_t * gp_uart1_tx_address;        /* uart1 transmit buffer address */
volatile uint16_t  g_uart1_tx_count;           /* uart1 transmit data number */
volatile uint8_t * gp_uart1_rx_address;        /* uart1 receive buffer address */
volatile uint16_t  g_uart1_rx_count;           /* uart1 receive data number */
volatile uint16_t  g_uart1_rx_length;          /* uart1 receive data length */
volatile uint8_t * gp_uart2_tx_address;        /* uart2 transmit buffer address */
volatile uint16_t  g_uart2_tx_count;           /* uart2 transmit data number */
volatile uint8_t * gp_uart2_rx_address;        /* uart2 receive buffer address */
volatile uint16_t  g_uart2_rx_count;           /* uart2 receive data number */
volatile uint16_t  g_uart2_rx_length;          /* uart2 receive data length */
volatile uint8_t * gp_uart3_tx_address;        /* uart3 transmit buffer address */
volatile uint16_t  g_uart3_tx_count;           /* uart3 transmit data number */
volatile uint8_t * gp_uart3_rx_address;        /* uart3 receive buffer address */
volatile uint16_t  g_uart3_rx_count;           /* uart3 receive data number */
volatile uint16_t  g_uart3_rx_length;          /* uart3 receive data length */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

void App_LpUart0DeCfg(void)
{
    stc_lpuart_cfg_t  stcLpuartCfg;
    stc_gpio_cfg_t     stcGpioCfg;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 使能GPIO时钟
    LPUart_DisableFunc(M0P_LPUART0,LPUartRenFunc);
    LPUart_ClrStatus(M0P_LPUART0,LPUartRC);         ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART0,LPUartTC);         ///<清发送中断请求
    LPUart_DisableIrq(M0P_LPUART0,LPUartRxIrq);     ///<禁止接收中断
    LPUart_DisableIrq(M0P_LPUART0,LPUartTxIrq);      ///<使能发送中断
    EnableNvic(LPUART0_IRQn,IrqLevel3,FALSE);        ///<系统中断使能

    DDL_ZERO_STRUCT(stcGpioCfg);                            ///< 结构体变量初始值置零
    //<RX
    stcGpioCfg.enDir =  GpioDirIn;
    Gpio_Init(MD_RS485_M0P_LPUART0_RXD_PORT,MD_RS485_M0P_LPUART0_RXD_PIN,&stcGpioCfg);
   // Gpio_SetAfMode(MD_RS485_M0P_LPUART0_RXD_PORT,MD_RS485_M0P_LPUART0_RXD_PIN,MD_RS485_M0P_LPUART0_RXD_PIN_AFN); 
 
    stcGpioCfg.enPd=GpioPdEnable;
    Gpio_Init(MD_RS485_M0P_LPUART0_TXD_PORT,MD_RS485_M0P_LPUART0_TXD_PIN,&stcGpioCfg);
   
}



void App_LpUart0Cfg(uint32_t baud,int16_t parity)
{
    stc_lpuart_cfg_t  stcLpuartCfg;
    stc_gpio_cfg_t     stcGpioCfg;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 使能GPIO时钟
    LPUart_DisableFunc(M0P_LPUART0,LPUartRenFunc);
    LPUart_ClrStatus(M0P_LPUART0,LPUartRC);         ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART0,LPUartTC);         ///<清发送中断请求
    LPUart_DisableIrq(M0P_LPUART0,LPUartRxIrq);     ///<禁止接收中断
    LPUart_DisableIrq(M0P_LPUART0,LPUartTxIrq);      ///<使能发送中断
    EnableNvic(LPUART0_IRQn,IrqLevel3,FALSE);        ///<系统中断使能

    DDL_ZERO_STRUCT(stcGpioCfg);                            ///< 结构体变量初始值置零
    //<RX
    stcGpioCfg.enDir =  GpioDirIn;
    stcGpioCfg.enPu =  GpioPuEnable;
    Gpio_Init(MD_RS485_M0P_LPUART0_RXD_PORT,MD_RS485_M0P_LPUART0_RXD_PIN,&stcGpioCfg);
    Gpio_SetAfMode(MD_RS485_M0P_LPUART0_RXD_PORT,MD_RS485_M0P_LPUART0_RXD_PIN,MD_RS485_M0P_LPUART0_RXD_PIN_AFN); 

    //<TX
    stcGpioCfg.enDir =  GpioDirOut;
    Gpio_Init(MD_RS485_M0P_LPUART0_TXD_PORT,MD_RS485_M0P_LPUART0_TXD_PIN,&stcGpioCfg);
    Gpio_SetAfMode(MD_RS485_M0P_LPUART0_TXD_PORT,MD_RS485_M0P_LPUART0_TXD_PIN,MD_RS485_M0P_LPUART0_TXD_PIN_AFN);


    DDL_ZERO_STRUCT(stcLpuartCfg);                        ///< 结构体变量初始值置零
    ///<LPUART 初始化
    stcLpuartCfg.enStopBit = LPUart1bit;                   ///<1停止位    
                      ///<偶校验
    stcLpuartCfg.stcBaud.enSclkSel = LPUartMskRcl;         ///<传输时钟源
    stcLpuartCfg.stcBaud.u32Sclk = 38400;                  ///<RCL时钟频率 38400Hz
    stcLpuartCfg.stcBaud.enSclkDiv = LPUartMsk4Or8Div;     ///<采样分频
    stcLpuartCfg.stcBaud.u32Baud = baud;                   ///<波特率
    if(parity==2 || parity==3)
    {
        if(parity==2)
        {
             stcLpuartCfg.enMmdorCk = LPUartEven; 
        }
        else
        {
            stcLpuartCfg.enMmdorCk = LPUartOdd;
        }
        stcLpuartCfg.enRunMode = LPUartMskMode3;               ///<工作模式
    }
    else
    {
        stcLpuartCfg.enRunMode = LPUartMskMode1; 
    }
    LPUart_Init(M0P_LPUART0, &stcLpuartCfg);
    LPUart_DisableFunc(M0P_LPUART0,LPUartRenFunc);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,FALSE); 
 }

/***********************************************************************************************************************
* Function Name: R_LPUART0_Start
* Description  : This function starts the LPUART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LPUART0_Start(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,TRUE); 
    LPUart_ClrStatus(M0P_LPUART0,LPUartRC);         ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART0,LPUartTC);         ///<清发送中断请求
    LPUart_EnableIrq(M0P_LPUART0,LPUartRxIrq);     ///<禁止接收中断
    LPUart_EnableIrq(M0P_LPUART0,LPUartTxIrq);      ///<使能发送中断
    EnableNvic(LPUART0_IRQn,IrqLevel3,TRUE);        ///<系统中断使能
    LPUart_EnableFunc(M0P_LPUART0,LPUartRenFunc);

}

/***********************************************************************************************************************
* Function Name: R_LPUART0_Stop
* Description  : This function stops the LPUART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LPUART0_Stop(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,TRUE); 
    LPUart_DisableFunc(M0P_LPUART0,LPUartRenFunc);
    LPUart_DisableIrq(M0P_LPUART0,LPUartRxIrq);     ///<禁止接收中断
    LPUart_DisableIrq(M0P_LPUART0,LPUartTxIrq);      ///<使能发送中断
    EnableNvic(LPUART0_IRQn,IrqLevel3,FALSE);        ///<
    LPUart_ClrStatus(M0P_LPUART0,LPUartRC);         ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART0,LPUartTC);         ///<清发送中断请求
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,FALSE); 

}
/***********************************************************************************************************************
* Function Name: R_LPUART0_Receive
* Description  : This function receives LPUART0 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_LPUART0_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_LPuart0_rx_count = 0U;
        g_LPuart0_rx_length = rx_num;
        gp_LPuart0_rx_address = rx_buf;
    }

    return (status);
}
/***********************************************************************************************************************
* Function Name: R_LPUART0_Send
* Description  : This function sends LPUART0 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_LPUART0_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_LPuart0_tx_address = tx_buf;
        g_LPuart0_tx_count = tx_num;
        //STMK0 = 1U;    /* disable INTST0 interrupt */
        //TXD0 = *gp_LPuart0_tx_address;
        LPUart_DisableIrq(M0P_LPUART0,LPUartTxIrq);
        LPUart_SendDataIt(M0P_LPUART0, *gp_LPuart0_tx_address);
        gp_LPuart0_tx_address++;
        g_LPuart0_tx_count--;
       // STMK0 = 0U;    /* enable INTST0 interrupt */
        LPUart_EnableIrq(M0P_LPUART0,LPUartTxIrq);
    }

    return (status);
}
/***********************************************************************************************************************
* Function Name: R_LPUART1_Create
* Description  : This function initializes the LPUART1 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
 void App_LpUart1Cfg(uint32_t baud,int16_t parity)
{
   
   
}

/***********************************************************************************************************************
* Function Name: R_LPUART1_Start
* Description  : This function starts the LPUART1 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LPUART1_Start(void)
{

    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart1,TRUE); 
    LPUart_ClrStatus(M0P_LPUART1,LPUartRC);         ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART1,LPUartTC);         ///<清发送中断请求
    LPUart_EnableIrq(M0P_LPUART1,LPUartRxIrq);     ///<禁止接收中断
    LPUart_EnableIrq(M0P_LPUART1,LPUartTxIrq);      ///<使能发送中断
    EnableNvic(LPUART1_IRQn,IrqLevel3,TRUE);        ///<系统中断使能
    LPUart_EnableFunc(M0P_LPUART1,LPUartRenFunc);

}
/***********************************************************************************************************************
* Function Name: R_LPUART1_Stop
* Description  : This function stops the LPUART1 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LPUART1_Stop(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart1,TRUE); 
    LPUart_DisableFunc(M0P_LPUART1,LPUartRenFunc);
    LPUart_DisableIrq(M0P_LPUART1,LPUartRxIrq);     ///<禁止接收中断
    LPUart_DisableIrq(M0P_LPUART1,LPUartTxIrq);      ///<使能发送中断
    EnableNvic(LPUART1_IRQn,IrqLevel3,FALSE);        ///<
    LPUart_ClrStatus(M0P_LPUART1,LPUartRC);         ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART1,LPUartTC);         ///<清发送中断请求
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart1,FALSE); 
}
/***********************************************************************************************************************
* Function Name: R_LPUART1_Receive
* Description  : This function receives LPUART1 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_LPUART1_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_LPuart1_rx_count = 0U;
        g_LPuart1_rx_length = rx_num;
        gp_LPuart1_rx_address = rx_buf;
    }

    return (status);
}
/***********************************************************************************************************************
* Function Name: R_LPUART1_Send
* Description  : This function sends LPUART1 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_LPUART1_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_LPuart1_tx_address = tx_buf;
        g_LPuart1_tx_count = tx_num;
       // STMK1 = 1U;    /* disable INTST1 interrupt */
       // TXD1 = *gp_uart1_tx_address;
        LPUart_DisableIrq(M0P_LPUART1,LPUartTxIrq);
        LPUart_SendDataIt(M0P_LPUART1, *gp_LPuart1_tx_address);
        gp_LPuart1_tx_address++;
        g_LPuart1_tx_count--;
       //  STMK1 = 0U;    /* enable INTST1 interrupt */
        LPUart_EnableIrq(M0P_LPUART1,LPUartTxIrq);
    }

    return (status);
}








 void App_Uart0Cfg(uint32_t baud,int16_t parity)
{
   
}

/***********************************************************************************************************************
* Function Name: R_UART0_Start
* Description  : This function starts the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Start(void)
{
    

}
/***********************************************************************************************************************
* Function Name: R_UART0_Stop
* Description  : This function stops the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Stop(void)
{
   

}
/***********************************************************************************************************************
* Function Name: R_UART0_Receive
* Description  : This function receives UART0 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART0_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_uart0_rx_count = 0U;
        g_uart0_rx_length = rx_num;
        gp_uart0_rx_address = rx_buf;
    }

    return (status);
}
/***********************************************************************************************************************
* Function Name: R_UART0_Send
* Description  : This function sends UART0 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART0_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_uart0_tx_address = tx_buf;
        g_uart0_tx_count = tx_num;
        //STMK0 = 1U;    /* disable INTST0 interrupt */
       // TXD0 = *gp_uart0_tx_address;
        Uart_DisableIrq(M0P_UART0,UartTxIrq);
        Uart_SendDataIt(M0P_UART0, *gp_uart0_tx_address);
        gp_uart0_tx_address++;
        g_uart0_tx_count--;
        Uart_EnableIrq(M0P_UART0,UartTxIrq);
       // STMK0 = 0U;    /* enable INTST0 interrupt */
    }
   return (status);
}
/***********************************************************************************************************************
* Function Name: R_UART1_Create
* Description  : This function initializes the UART1 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
 void App_Uart1Cfg(uint32_t baud,int16_t parity)
{
    stc_uart_cfg_t  stcUartCfg;
    stc_gpio_cfg_t stcGpioCfg;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);///<使能uart1模块时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 使能GPIO时钟
     
    Uart_DisableFunc(M0P_UART1,UartRenFunc);
    Uart_ClrStatus(M0P_UART1,UartRC);         ///<清接收中断请求
    Uart_ClrStatus(M0P_UART1,UartTC);         ///<清发送中断请求
    Uart_DisableIrq(M0P_UART1,UartRxIrq);     ///<禁止接收中断
    Uart_DisableIrq(M0P_UART1,UartTxIrq);      ///<使能发送中断
    EnableNvic(UART1_3_IRQn,IrqLevel3,TRUE);        ///<系统中断使能

    DDL_ZERO_STRUCT(stcGpioCfg);                            ///< 结构体变量初始值置零
     ///<TX
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(MD_IOT_M0P_UART1_TXD_PORT, MD_IOT_M0P_UART1_TXD_PIN, &stcGpioCfg);
    Gpio_SetAfMode(MD_IOT_M0P_UART1_TXD_PORT, MD_IOT_M0P_UART1_TXD_PIN, MD_IOT_M0P_UART1_TXD_PIN_AFN);          //配置PA02 端口为URART1_TX
    
    ///<RX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(MD_IOT_M0P_UART1_RXD_PORT, MD_IOT_M0P_UART1_RXD_PIN, &stcGpioCfg);
    Gpio_SetAfMode(MD_IOT_M0P_UART1_RXD_PORT, MD_IOT_M0P_UART1_RXD_PIN, MD_IOT_M0P_UART1_RXD_PIN_AFN);          //配置PA03 端口为URART1_RX
    
    DDL_ZERO_STRUCT(stcUartCfg);                        ///< 结构体变量初始值置零
    ///<UART Init
     stcUartCfg.enStopBit        = UartMsk1bit;           ///<1bit停止位
   
    stcUartCfg.stcBaud.u32Baud  = baud;                  ///<波特率9600
    stcUartCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       ///<通道采样分频配置
    stcUartCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); ///<获得外设时钟（PCLK）频率值
    if(parity==2 || parity==3)
    {
        if(parity==2)
        {
             stcUartCfg.enMmdorCk        = UartMskEven;  
        }
        else
        {
            stcUartCfg.enMmdorCk        = UartMskOdd; 
        }
        stcUartCfg.enRunMode        = UartMskMode3;              ///<工作模式
    }
    else
    {
        stcUartCfg.enRunMode        = UartMskMode1;
    }
    Uart_Init(M0P_UART1, &stcUartCfg);                   ///<串口初始化
    Uart_DisableFunc(M0P_UART1,UartRenFunc);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,FALSE); 
}

/***********************************************************************************************************************
* Function Name: R_UART1_Start
* Description  : This function starts the UART1 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART1_Start(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE); 
    Uart_ClrStatus(M0P_UART1,UartRC);         ///<清接收中断请求
    Uart_ClrStatus(M0P_UART1,UartTC);         ///<清发送中断请求
    Uart_EnableIrq(M0P_UART1,UartRxIrq);     ///<禁止接收中断
    Uart_EnableIrq(M0P_UART1,UartTxIrq);      ///<使能发送中断
    EnableNvic(UART1_3_IRQn,IrqLevel3,TRUE);        ///<系统中断使能
    Uart_EnableFunc(M0P_UART1,UartRenFunc);

}
/***********************************************************************************************************************
* Function Name: R_UART1_Stop
* Description  : This function stops the UART1 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART1_Stop(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE); 
    Uart_DisableFunc(M0P_UART1,UartRenFunc);
    Uart_DisableIrq(M0P_UART1,UartRxIrq);     ///<禁止接收中断
    Uart_DisableIrq(M0P_UART1,UartTxIrq);      ///<使能发送中断
   // EnableNvic(UART1_3_IRQn,IrqLevel3,FALSE);        ///<
    Uart_ClrStatus(M0P_UART1,UartRC);         ///<清接收中断请求
    Uart_ClrStatus(M0P_UART1,UartTC);         ///<清发送中断请求
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,FALSE); 
}
/***********************************************************************************************************************
* Function Name: R_UART1_Receive
* Description  : This function receives UART1 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART1_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
   MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_uart1_rx_count = 0U;
        g_uart1_rx_length = rx_num;
        gp_uart1_rx_address = rx_buf;
    }
    return (status);
}
/***********************************************************************************************************************
* Function Name: R_UART1_Send
* Description  : This function sends UART1 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART1_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_uart1_tx_address = tx_buf;
        g_uart1_tx_count = tx_num;
        // STMK1 = 1U;    /* disable INTST1 interrupt */
        //TXD1 = *gp_uart1_tx_address;
        Uart_DisableIrq(M0P_UART1, UartTxIrq);
        Uart_SendDataIt(M0P_UART1, *gp_uart1_tx_address);
        gp_uart1_tx_address++;
        g_uart1_tx_count--;
       // STMK1 = 0U;    /* enable INTST1 interrupt */
        Uart_EnableIrq(M0P_UART1, UartTxIrq);
    }

    return (status);
}

 void App_Uart2Cfg(uint32_t baud,int16_t parity)
{
    
    
   
}

/***********************************************************************************************************************
* Function Name: R_UART2_Start
* Description  : This function starts the UART2 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART2_Start(void)
{
//    SO1 |= _0001_SAU_CH0_DATA_OUTPUT_1;    /* output level normal */
//    SOE1 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable UART2 output */
//    SS1 |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON;    /* enable UART2 receive and transmit */
//    STIF2 = 0U;    /* clear INTST2 interrupt flag */
//    SRIF2 = 0U;    /* clear INTSR2 interrupt flag */
//    STMK2 = 0U;    /* enable INTST2 interrupt */
//    SRMK2 = 0U;    /* enable INTSR2 interrupt */
}
/***********************************************************************************************************************
* Function Name: R_UART2_Stop
* Description  : This function stops the UART2 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART2_Stop(void)
{
//    STMK2 = 1U;    /* disable INTST2 interrupt */
//    SRMK2 = 1U;    /* disable INTSR2 interrupt */
//    ST1 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON;    /* disable UART2 receive and transmit */
//    SOE1 &= ~_0001_SAU_CH0_OUTPUT_ENABLE;    /* disable UART2 output */
//    STIF2 = 0U;    /* clear INTST2 interrupt flag */
//    SRIF2 = 0U;    /* clear INTSR2 interrupt flag */
}
/***********************************************************************************************************************
* Function Name: R_UART2_Receive
* Description  : This function receives UART2 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART2_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_uart2_rx_count = 0U;
        g_uart2_rx_length = rx_num;
        gp_uart2_rx_address = rx_buf;
    }
    return (status);
}
/***********************************************************************************************************************
* Function Name: R_UART2_Send
* Description  : This function sends UART2 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART2_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_uart2_tx_address = tx_buf;
        g_uart2_tx_count = tx_num;
        //STMK2 = 1U;    /* disable INTST2 interrupt */
        //TXD2 = *gp_uart2_tx_address;
        Uart_DisableIrq(M0P_UART2, UartTxIrq);
        Uart_SendDataIt(M0P_UART2,*gp_uart2_tx_address);
        gp_uart2_tx_address++;
        g_uart2_tx_count--;
        //STMK2 = 0U;    /* enable INTST2 interrupt */
        Uart_EnableIrq(M0P_UART2,UartTxIrq);
    }

   return (status);
}
/***********************************************************************************************************************
* Function Name: R_UART3_Create
* Description  : This function initializes the UART3 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
 void App_Uart3Cfg(uint32_t baud,int16_t parity)
{
    
    
   
}

/***********************************************************************************************************************
* Function Name: R_UART3_Start
* Description  : This function starts the UART3 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART3_Start(void)
{
//    SO1 |= _0004_SAU_CH2_DATA_OUTPUT_1;    /* output level normal */
//    SOE1 |= _0004_SAU_CH2_OUTPUT_ENABLE;    /* enable UART3 output */
//    SS1 |= _0008_SAU_CH3_START_TRG_ON | _0004_SAU_CH2_START_TRG_ON;    /* enable UART3 receive and transmit */
//    STIF3 = 0U;    /* clear INTST3 interrupt flag */
//    SRIF3 = 0U;    /* clear INTSR3 interrupt flag */
//    STMK3 = 0U;    /* enable INTST3 interrupt */
//    SRMK3 = 0U;    /* enable INTSR3 interrupt */
}
/***********************************************************************************************************************
* Function Name: R_UART3_Stop
* Description  : This function stops the UART3 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART3_Stop(void)
{
//    STMK3 = 1U;    /* disable INTST3 interrupt */
//    SRMK3 = 1U;    /* disable INTSR3 interrupt */
//    ST1 |= _0008_SAU_CH3_STOP_TRG_ON | _0004_SAU_CH2_STOP_TRG_ON;    /* disable UART3 receive and transmit */
//    SOE1 &= ~_0004_SAU_CH2_OUTPUT_ENABLE;    /* disable UART3 output */
//    STIF3 = 0U;    /* clear INTST3 interrupt flag */
//    SRIF3 = 0U;    /* clear INTSR3 interrupt flag */
}
/***********************************************************************************************************************
* Function Name: R_UART3_Receive
* Description  : This function receives UART3 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART3_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
   MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_uart3_rx_count = 0U;
        g_uart3_rx_length = rx_num;
        gp_uart3_rx_address = rx_buf;
    }

    return (status);
}
/***********************************************************************************************************************
* Function Name: R_UART3_Send
* Description  : This function sends UART3 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_UART3_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_uart3_tx_address = tx_buf;
        g_uart3_tx_count = tx_num;
//        STMK3 = 1U;    /* disable INTST3 interrupt */
//        TXD3 = *gp_uart3_tx_address;
        Uart_DisableIrq(M0P_UART3, UartTxIrq);
        Uart_SendDataIt(M0P_UART3,*gp_uart3_tx_address);
        gp_uart3_tx_address++;
        g_uart3_tx_count--;
        Uart_EnableIrq(M0P_UART3, UartTxIrq);
//      STMK3 = 0U;    /* enable INTST3 interrupt */
    }

    return (status);
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
