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
* File Name    : r_cg_sau_user.c
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
#include "rtc.h"
#include "lpuart.h"
#include "uart.h"

/* Start user code for include. Do not edit comment generated here */
#include "r_cg_sau.h"
#include "irc.h"
#include "net.h"
#include "elora.h"
#include "modbus.h"
#include "device.h"




/* End user code. Do not edit comment generated here */


/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/




/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern  volatile uint8_t * gp_LPuart0_tx_address;        /* LPuart0 transmit buffer address */
extern  volatile uint16_t  g_LPuart0_tx_count;           /* LPuart0 transmit data number */
extern  volatile uint8_t * gp_LPuart0_rx_address;        /* LPuart0 receive buffer address */
extern  volatile uint16_t  g_LPuart0_rx_count;           /* LPuart0 receive data number */
extern  volatile uint16_t  g_LPuart0_rx_length;          /* LPuart0 receive data length */
extern  volatile uint8_t * gp_LPuart1_tx_address;        /* LPuart1 transmit buffer address */
extern  volatile uint16_t  g_LPuart1_tx_count;           /* LPuart1 transmit data number */
extern  volatile uint8_t * gp_LPuart1_rx_address;        /* LPuart1 receive buffer address */
extern  volatile uint16_t  g_LPuart1_rx_count;           /* LPuart1 receive data number */
extern  volatile uint16_t  g_LPuart1_rx_length;          /* LPuart1 receive data length */

extern volatile uint8_t * gp_uart0_tx_address;         /* uart0 send buffer address */
extern volatile uint16_t  g_uart0_tx_count;            /* uart0 send data number */
extern volatile uint8_t * gp_uart0_rx_address;         /* uart0 receive buffer address */
extern volatile uint16_t  g_uart0_rx_count;            /* uart0 receive data number */
extern volatile uint16_t  g_uart0_rx_length;           /* uart0 receive data length */
extern volatile uint8_t * gp_uart1_tx_address;         /* uart1 send buffer address */
extern volatile uint16_t  g_uart1_tx_count;            /* uart1 send data number */
extern volatile uint8_t * gp_uart1_rx_address;         /* uart1 receive buffer address */
extern volatile uint16_t  g_uart1_rx_count;            /* uart1 receive data number */
extern volatile uint16_t  g_uart1_rx_length;           /* uart1 receive data length */
extern volatile uint8_t * gp_uart2_tx_address;         /* uart2 send buffer address */
extern volatile uint16_t  g_uart2_tx_count;            /* uart2 send data number */
extern volatile uint8_t * gp_uart2_rx_address;         /* uart2 receive buffer address */
extern volatile uint16_t  g_uart2_rx_count;            /* uart2 receive data number */
extern volatile uint16_t  g_uart2_rx_length;           /* uart2 receive data length */
extern volatile uint8_t * gp_uart3_tx_address;         /* uart3 send buffer address */
extern volatile uint16_t  g_uart3_tx_count;            /* uart3 send data number */
extern volatile uint8_t * gp_uart3_rx_address;         /* uart3 receive buffer address */
extern volatile uint16_t  g_uart3_rx_count;            /* uart3 receive data number */
extern volatile uint16_t  g_uart3_rx_length;           /* uart3 receive data length */
/* Start user code for global. Do not edit comment generated here */
uint8_t LPuart0_rx_data;
uint8_t LPuart1_rx_data;
uint8_t uart0_rx_data;
uint8_t uart1_rx_data;
uint8_t uart2_rx_data;
uint8_t uart3_rx_data;

#define   BK_LN

/* End user code. Do not edit comment generated here */



static void r_LPuart0_callback_receiveend(void)
{
    ircComps.store_buffer(LPuart0_rx_data);
    modbusComps.store_buffer(LPuart0_rx_data);
    R_LPUART0_Receive(&LPuart0_rx_data,1);

}

static void  r_LPuart0_callback_softwareoverrun(uint16_t rx_data)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void  r_LPuart0_callback_sendend(void)
{
    if(!modbusComps.sw._bit.runing)
    {
       // enable_irc_receive(); 
    }
    else
    {
        if(modbusComps.sw._bit.baud_modified)
        {
           modbusComps.modify_baud(modbusComps.param_pt->baud,0);
           modbusComps.sw._bit.baud_modified=0;
        }
        modbusComps.sendend_callback();
	}
    
//    if(loraComps.sw._bit.param_modified)
//    {
//          __disable_irq();
//    	NVIC_SystemReset();
//    }
    
}

static void r_LPuart0_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void  r_LPuart0_interrupt_receive(void)
{
    uint8_t rx_data;
    uint8_t err_type;
    
//    err_type = (uint8_t)(SSR01 & 0x0007U);
//    SIR01 = (uint16_t)err_type;

//    if (err_type != 0U)
//    {
//        r_LPuart0_callback_error(err_type);
//    }
    
 //   rx_data = RXD0;
    rx_data =LPUart_ReceiveData(M0P_LPUART0);

    if (g_LPuart0_rx_length > g_LPuart0_rx_count)
    {
        *gp_LPuart0_rx_address = rx_data;
        gp_LPuart0_rx_address++;
        g_LPuart0_rx_count++;

        if (g_LPuart0_rx_length == g_LPuart0_rx_count)
        {
            r_LPuart0_callback_receiveend();
        }
    }
    else
    {
       // r_LPuart0_callback_softwareoverrun(rx_data);
    }
}

static void  r_LPuart0_interrupt_send(void)
{
    if (g_LPuart0_tx_count > 0U)
    {
       // TXD0 = *gp_LPuart0_tx_address;
        LPUart_SendDataIt(M0P_LPUART0, *gp_LPuart0_tx_address); 
        gp_LPuart0_tx_address++;
        g_LPuart0_tx_count--;
    }
    else
    {
        r_LPuart0_callback_sendend();
    }
}

void LpUart0_IRQHandler(void)
{
    if(LPUart_GetStatus(M0P_LPUART0, LPUartTC))       ///接收数据中断
    {
        LPUart_ClrStatus(M0P_LPUART0, LPUartTC);      ///<清发送中断请求
        r_LPuart0_interrupt_send();
    }
    
    if(LPUart_GetStatus(M0P_LPUART0, LPUartRC))       ///接收数据中断
    {
       
        LPUart_ClrStatus(M0P_LPUART0, LPUartRC);      ///<清接收中断请求
        r_LPuart0_interrupt_receive();

    }
}

#define   BK_LN

static void r_LPuart1_callback_receiveend(void)
{
     
    R_LPUART1_Receive(&LPuart1_rx_data,1);

}
static void  r_LPuart1_callback_softwareoverrun(uint16_t rx_data)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void  r_LPuart1_callback_sendend(void)
{

}


static void r_LPuart1_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}




static void  r_LPuart1_interrupt_receive(void)
{
    uint8_t rx_data;
    uint8_t err_type;
    
//    err_type = (uint8_t)(SSR01 & 0x0007U);
//    SIR01 = (uint16_t)err_type;

//    if (err_type != 0U)
//    {
//        r_LPuart1_callback_error(err_type);
//    }
    
 //   rx_data = RXD0;
    rx_data =LPUart_ReceiveData(M0P_LPUART1);

    if (g_LPuart1_rx_length > g_LPuart1_rx_count)
    {
        *gp_LPuart1_rx_address = rx_data;
        gp_LPuart1_rx_address++;
        g_LPuart1_rx_count++;

        if (g_LPuart1_rx_length == g_LPuart1_rx_count)
        {
            r_LPuart1_callback_receiveend();
        }
    }
    else
    {
       // r_LPuart1_callback_softwareoverrun(rx_data);
    }
}

static void  r_LPuart1_interrupt_send(void)
{
    if (g_LPuart1_tx_count > 0U)
    {
       // TXD0 = *gp_LPuart1_tx_address;
        LPUart_SendDataIt(M0P_LPUART0, *gp_LPuart1_tx_address); 
        gp_LPuart1_tx_address++;
        g_LPuart1_tx_count--;
    }
    else
    {
        r_LPuart1_callback_sendend();
    }
}


void LpUart1_IRQHandler(void)
{
     if(LPUart_GetStatus(M0P_LPUART1, LPUartTC))       ///接收数据中断
    {
        LPUart_ClrStatus(M0P_LPUART1, LPUartTC);      ///<清发送中断请求
        r_LPuart1_interrupt_send();
    }
    
    if(LPUart_GetStatus(M0P_LPUART1, LPUartRC))       ///接收数据中断
    {
       
        LPUart_ClrStatus(M0P_LPUART1, LPUartRC);      ///<清接收中断请求
        r_LPuart1_interrupt_receive();

    }

}
#define   BK_LN

static void r_uart0_callback_receiveend(void)
{
//    /* Start user code. Do not edit comment generated here */
    
   

    R_UART0_Receive(&uart0_rx_data,1);

//    /* End user code. Do not edit comment generated here */
}

static void r_uart0_callback_softwareoverrun(uint16_t rx_data)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void r_uart0_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
//    if(!modbusComps.sw._bit.runing)
//    {
//        disable_irc_send() ;   
//        enable_irc_receive(); 
//    }
//    else
//    {
//        if(modbusComps.sw._bit.baud_modified)
//        {
//           modbus_modify_baud(modbusComps.param_pt->baud,0);
//           modbusComps.sw._bit.baud_modified=0;
//        }
//        disable_modbus_send();
//        enable_modbus_receive();
//    }
    
//    if(loraComps.sw._bit.param_modified)
//    {
//        WDTE='R';
//    }
 /* End user code. Do not edit comment generated here */
}

static void r_uart0_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}


static void  r_uart0_interrupt_receive(void)
{
    uint8_t rx_data;
    uint8_t err_type;
    
//    err_type = (uint8_t)(SSR01 & 0x0007U);
//    SIR01 = (uint16_t)err_type;

//    if (err_type != 0U)
//    {
//        r_uart0_callback_error(err_type);
//    }
    
    
    rx_data = Uart_ReceiveData(M0P_UART0);   //接收数据字节


    if (g_uart0_rx_length > g_uart0_rx_count)
    {
        *gp_uart0_rx_address = rx_data;
        gp_uart0_rx_address++;
        g_uart0_rx_count++;

        if (g_uart0_rx_length == g_uart0_rx_count)
        {
            r_uart0_callback_receiveend();
        }
    }
    else
    {
        r_uart0_callback_softwareoverrun(rx_data);
    }
}

static void  r_uart0_interrupt_send(void)
{
    if (g_uart0_tx_count > 0U)
    {
        //TXD0 = *gp_uart0_tx_address;
      
        Uart_SendDataIt(M0P_UART0, *gp_LPuart0_tx_address); 
        gp_uart0_tx_address++;
        g_uart0_tx_count--;
    }
    else
    {
        r_uart0_callback_sendend();
    }
}


void Uart0_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART0, UartRC))         //UART0数据接收
    {
        Uart_ClrStatus(M0P_UART0, UartRC);        //清中断状态位
        r_uart0_interrupt_receive();
       
    }
    
    if(Uart_GetStatus(M0P_UART0, UartTC))         //UART0数据发送
    {
        Uart_ClrStatus(M0P_UART0, UartTC);        //清中断状态位
        r_uart0_interrupt_send();
       
    }

}

#define   BK_LN

static void r_uart1_callback_receiveend(void)
{
    /* Start user code. Do not edit comment generated here */
    
     netComps.store_buffer(uart1_rx_data);
     loraComps.store_buffer(uart1_rx_data);
     R_UART1_Receive(&uart1_rx_data,1);
    /* End user code. Do not edit comment generated here */
}

static void r_uart1_callback_softwareoverrun(uint16_t rx_data)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void r_uart1_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    #if(MD_PRODUCT_NAME == MD_LORA)
     {
          loraComps.sw._bit.busy=0;
          if(loraComps.work_st.mode==LORA_EM_WORK_MODE)
          {
              loraComps.op_window_time=1;//muset set 1 delay 50ms sleep
          }
    
      }
    #endif


    /* End user code. Do not edit comment generated here */
}

static void r_uart1_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}


static void  r_uart1_interrupt_receive(void)
{
    uint8_t rx_data;
    uint8_t err_type;
    
//    err_type = (uint8_t)(SSR03 & 0x0007U);
//    SIR03 = (uint16_t)err_type;

//    if (err_type != 0U)
//    {
//        r_uart1_callback_error(err_type);
//    }
    
//    rx_data = RXD1;
    rx_data=Uart_ReceiveData(M0P_UART1);
    if (g_uart1_rx_length > g_uart1_rx_count)
    {
        *gp_uart1_rx_address = rx_data;
        gp_uart1_rx_address++;
        g_uart1_rx_count++;

        if (g_uart1_rx_length == g_uart1_rx_count)
        {
            r_uart1_callback_receiveend();
        }
    }
    else
    {
        r_uart1_callback_softwareoverrun(rx_data);
    }
}

static void  r_uart1_interrupt_send(void)
{
    if (g_uart1_tx_count > 0U)
    {
        //TXD1 = *gp_uart1_tx_address;
        Uart_SendDataIt(M0P_UART1, *gp_uart1_tx_address);
        gp_uart1_tx_address++;
        g_uart1_tx_count--;
    }
    else
    {
        r_uart1_callback_sendend();
    }
}

void Uart1_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART1, UartRC))         //UART1数据接收
    {
        Uart_ClrStatus(M0P_UART1, UartRC);        //清中断状态位
        r_uart1_interrupt_receive();
    }
    
    if(Uart_GetStatus(M0P_UART1, UartTC))         //UART1数据发送
    {
        Uart_ClrStatus(M0P_UART1, UartTC);        //清中断状态位
        r_uart1_interrupt_send();
    }

}

#define   BK_LN

static void r_uart2_callback_receiveend(void)
{
    
     /* Start user code. Do not edit comment generated here */
   
   
     R_UART2_Receive(&uart2_rx_data,1);

    /* End user code. Do not edit comment generated here */
}

static void r_uart2_callback_softwareoverrun(uint16_t rx_data)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void r_uart2_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */

    /* End user code. Do not edit comment generated here */
}

static void r_uart2_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}


static void  r_uart2_interrupt_receive(void)
{
    uint8_t rx_data;
    uint8_t err_type;
    
//    err_type = (uint8_t)(SSR11 & 0x0007U);
//    SIR11 = (uint16_t)err_type;

//    if (err_type != 0U)
//    {
//        r_uart2_callback_error(err_type);
//    }
    
    //rx_data = RXD2;
    rx_data=Uart_ReceiveData(M0P_UART2);

    if (g_uart2_rx_length > g_uart2_rx_count)
    {
        *gp_uart2_rx_address = rx_data;
        gp_uart2_rx_address++;
        g_uart2_rx_count++;

        if (g_uart2_rx_length == g_uart2_rx_count)
        {
            r_uart2_callback_receiveend();
        }
    }
    else
    {
        r_uart2_callback_softwareoverrun(rx_data);
    }
}

static void  r_uart2_interrupt_send(void)
{
    if (g_uart2_tx_count > 0U)
    {
        //TXD2 = *gp_uart2_tx_address;
        Uart_SendDataIt(M0P_UART2, *gp_uart2_tx_address);
        gp_uart2_tx_address++;
        g_uart2_tx_count--;
    }
    else
    {
        r_uart2_callback_sendend();
    }
}



void Uart2_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART2, UartRC))         //UART2数据接收
    {
        Uart_ClrStatus(M0P_UART2, UartRC);        //清中断状态位
        r_uart2_interrupt_receive();
    }
    
    if(Uart_GetStatus(M0P_UART2, UartTC))         //UART2数据发送
    {
        Uart_ClrStatus(M0P_UART2, UartTC);        //清中断状态位
        r_uart2_interrupt_send();
    }

}

#define   BK_LN

static void r_uart3_callback_receiveend(void)
{
    /* Start user code. Do not edit comment generated here */


    R_UART3_Receive(&uart3_rx_data,1);

    /* End user code. Do not edit comment generated here */
}

static void r_uart3_callback_softwareoverrun(uint16_t rx_data)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void r_uart3_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void r_uart3_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

static void  r_uart3_interrupt_receive(void)
{
    uint8_t rx_data;
    uint8_t err_type;
    
//    err_type = (uint8_t)(SSR13 & 0x0007U);
//    SIR13 = (uint16_t)err_type;

//    if (err_type != 0U)
//    {
//        r_uart3_callback_error(err_type);
//    }
    
   // rx_data = RXD3;
    rx_data= Uart_ReceiveData(M0P_UART3);
    if (g_uart3_rx_length > g_uart3_rx_count)
    {
        *gp_uart3_rx_address = rx_data;
        gp_uart3_rx_address++;
        g_uart3_rx_count++;

        if (g_uart3_rx_length == g_uart3_rx_count)
        {
            r_uart3_callback_receiveend();
        }
    }
    else
    {
        r_uart3_callback_softwareoverrun(rx_data);
    }
}

static void  r_uart3_interrupt_send(void)
{
    if (g_uart3_tx_count > 0U)
    {
        //TXD3 = *gp_uart3_tx_address;
        Uart_SendDataIt(M0P_UART3, *gp_uart3_tx_address);
        gp_uart3_tx_address++;
        g_uart3_tx_count--;
    }
    else
    {
        r_uart3_callback_sendend();
    }
}


void Uart3_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART3, UartRC))         //UART3数据接收
    {
        Uart_ClrStatus(M0P_UART3, UartRC);        //清中断状态位
        r_uart3_interrupt_receive();
    }
    
    if(Uart_GetStatus(M0P_UART3, UartTC))         //UART3数据发送
    {
        Uart_ClrStatus(M0P_UART3, UartTC);        //清中断状态位
        r_uart3_interrupt_send();
    }

}

#define   BK_LN

void enable_LPuart0(void)
{
   
    R_LPUART0_Receive(&LPuart0_rx_data,1);
    R_LPUART0_Start();
}

void disable_LPuart0(void)
{
    R_LPUART0_Stop();
    
}
void enable_LPuart1(void)
{
   
    R_LPUART1_Receive(&LPuart1_rx_data,1);
    R_LPUART1_Start();
}

void disable_LPuart1(void)
{
    R_LPUART1_Stop();
    
}

void enable_uart0(void)
{
    R_UART0_Receive(&uart0_rx_data,1);
    R_UART0_Start();
}
void disable_uart0(void)
{
    R_UART0_Stop();
}

void enable_uart1(void)
{
    R_UART1_Receive(&uart1_rx_data,1);
    R_UART1_Start();
}
void disable_uart1(void)
{
    R_UART1_Stop();
}

void enable_uart2(void)
{
    R_UART2_Receive(&uart2_rx_data,1);
    R_UART2_Start();
}

void disable_uart2(void)
{
    R_UART2_Stop();
}


void disable_uart3(void)
{
    R_UART3_Stop();
}

void enable_uart3(void)
{
    R_UART3_Receive(&uart3_rx_data,1);
    R_UART3_Start();
}

#define  BK_LN






//void enable_irc_receive(void)
//{
//   LPUart_EnableFunc(M0P_LPUART1,LPUartRenFunc);
//}

//void disable_irc_receive(void)
//{
//   LPUart_DisableFunc(M0P_LPUART1,LPUartRenFunc);
//}













