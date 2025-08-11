/******************************************************************************
 * Copyright (C) 2021, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************/

/******************************************************************************
 * @file   board_hc32l19x.h
 *
 * @brief  Header file for BSP functions
 *
 * @author MADS Team 
 *
 ******************************************************************************/
#ifndef __BOARD_HC32L19X_H__
#define __BOARD_HC32L19X_H__
#include "system_hc32l19x.h"

#if defined(HC32L19xPxxx)	      //100PIN MCU	 
 #define   MD_NC_PORT GpioPortF
 #define   MD_NC_PIN  GpioPin4 
	
#elif defined(HC32L19xMxxx)     //80PIN MCU	  
 #define   MD_NC_PORT GpioPortD
 #define   MD_NC_PIN  GpioPin5

#elif defined(HC32L19xKxxx)     //64PIN MCU
 #define   MD_NC_PORT GpioPortD
 #define   MD_NC_PIN  GpioPin0
		
#elif defined(HC32L19xJxxx)     //48PIN MCU
 #define   MD_NC_PORT GpioPortC
 #define   MD_NC_PIN  GpioPin0
			
#elif defined(HC32L19xFxxx)     //32PIN MCU
 #define   MD_NC_PORT GpioPortA
 #define   MD_NC_PIN  GpioPin0
#endif

#define  MD_INVALID_PORT      MD_NC_PORT
#define  MD_INVALID_PIN       MD_NC_PIN

/*********************hum***************************/

#define   MD_BACK_LED_PORT          GpioPortC
#define   MD_BACK_LED_PIN           GpioPin13
#define   MD_KEY_VDD_PORT           MD_INVALID_PORT
#define   MD_KEY_VDD_PIN            MD_INVALID_PIN
#define   MD_S_KEY_PORT             GpioPortF
#define   MD_S_KEY_PIN              GpioPin4
#define   MD_M_KEY_PORT             GpioPortF
#define   MD_M_KEY_PIN              GpioPin5
#define   MD_J_KEY_PORT             GpioPortA
#define   MD_J_KEY_PIN              GpioPin7


#define  MD_LCD_I2CX_SEL            M0P_I2C0  
#define  MD_LCD_POWER_PORT          GpioPortE
#define  MD_LCD_POWER_PIN           GpioPin5
///< I2C0 lcd_driver_ic          
#define  MD_LCD_I2C0_SDA_PORT       GpioPortF
#define  MD_LCD_I2C0_SDA_PIN        GpioPin0
#define  MD_LCD_I2C0_SDA_PIN_AFN    GpioAf1
#define  MD_LCD_I2C0_SCL_PORT       GpioPortF
#define  MD_LCD_I2C0_SCL_PIN        GpioPin1
#define  MD_LCD_I2C0_SCL_PIN_AFN    GpioAf1




/************************************************/

#define        MD_ADI_CS_PORT      MD_INVALID_PORT  
#define        MD_ADI_CS_PIN       MD_INVALID_PIN
#define        MD_ADI_SCLK_PORT    MD_INVALID_PORT 
#define        MD_ADI_SCLK_PIN     MD_INVALID_PIN
#define        MD_ADI_DIN_PORT     MD_INVALID_PORT 
#define        MD_ADI_DIN_PIN      MD_INVALID_PIN
#define        MD_ADI_DOUT_PORT    MD_INVALID_PORT 
#define        MD_ADI_DOUT_PIN     MD_INVALID_PIN


#define        MD_SDQ_PORT              GpioPortC
#define        MD_SDQ_PIN               GpioPin6
#define        MD_AVDD_CTL_PORT         GpioPortD
#define        MD_AVDD_CTL_PIN          GpioPin11
#define        MD_MCU_EXVREF_PORT       GpioPortB
#define        MD_MCU_EXVREF_PIN        GpioPin1
#define        MD_MCU_EXVREF_CTL_PORT   GpioPortE       
#define        MD_MCU_EXVREF_CTL_PIN    GpioPin12
#define        MD_BAT_CTL_PORT          GpioPortB
#define        MD_BAT_CTL_PIN           GpioPin15
#define        MD_BAT_AIN4_PORT         GpioPortA
#define        MD_BAT_AIN4_PIN          GpioPin4
#define        MD_NTC_AIN25_PORT        GpioPortE
#define        MD_NTC_AIN25_PIN         GpioPin13


#define        MD_BAT_BLUNT_CTL_PORT    MD_INVALID_PORT
#define        MD_BAT_BLUNT_CTL_PIN     MD_INVALID_PIN


/*************digiht********************/

#define        MD_METRE_M0P_PCNT_S0_IN_PORT      GpioPortC
#define        MD_METRE_M0P_PCNT_S0_IN_PIN       GpioPin0
#define        MD_METRE_M0P_PCNT_S0_IN_PIN_AFN   GpioAf2




#define     MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PORT      GpioPortC
#define     MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PIN       GpioPin1
#define     MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PIN_AFN   GpioAf1

#define     MD_METER_FREQ_M0P_TIM0_CHA_OUT_PORT      GpioPortA
#define     MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN       GpioPin15
#define     MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN_AFN   GpioAf5

#define     MD_METER_FREQ_OUT_POWER_PORT      GpioPortC
#define     MD_METER_FREQ_OUT_POWER_PIN       GpioPin11
#define     MD_METER_FREQ_OUT_CTL_PORT        GpioPortC
#define     MD_METER_FREQ_OUT_CTL_PIN         GpioPin10



#define     MD_IR_VCM_PORT              GpioPortB
#define     MD_IR_VCM_PIN               GpioPin2

#define     MD_COLLECTOR_VCM_PORT      MD_INVALID_PORT
#define     MD_COLLECTOR_VCM_PIN       MD_INVALID_PIN

#define     MD_EXT_POWER_CONNECT_DETECT_PORT    GpioPortD 
#define     MD_EXT_POWER_CONNECT_DETECT_PIN     GpioPin8
#define     MD_4_20MA_CONNECT_DETECT_PORT   GpioPortC 
#define     MD_4_20MA_CONNECT_DETECT_PIN    GpioPin7


//IOT ctl

#define     MD_GSM_POWER_PORT        GpioPortE
#define     MD_GSM_POWER_PIN         GpioPin2
#define     MD_GSM_RST_PORT          GpioPortE
#define     MD_GSM_RST_PIN           GpioPin3
#define     MD_GSM_ON_PORT           GpioPortE
#define     MD_GSM_ON_PIN            GpioPin4
                                                  
#define     MD_LORA_AUX_PORT         GpioPortC   //falling edge interrupt
#define     MD_LORA_AUX_PIN          GpioPin3
#define     MD_LORA_MD0_PORT         MD_INVALID_PORT   //falling edge interrupt
#define     MD_LORA_MD0_PIN          MD_INVALID_PIN
#define     MD_LORA_MD1_PORT         MD_INVALID_PORT   //falling edge interrupt
#define     MD_LORA_MD1_PIN          MD_INVALID_PIN


//simulate  I2C XROM
#define  MD_XROM_I2C_VCC_PORT          GpioPortE
#define  MD_XROM_I2C_VCC_PIN           GpioPin14
#define  MD_XROM_I2C_WP_PORT           MD_INVALID_PORT
#define  MD_XROM_I2C_WP_PIN            MD_INVALID_PIN
#define  MD_XROM_I2C_SCL_PORT          GpioPortB
#define  MD_XROM_I2C_SCL_PIN           GpioPin10
#define  MD_XROM_I2C_SDA_PORT          GpioPortB
#define  MD_XROM_I2C_SDA_PIN           GpioPin11

//simulate  I2C sht4x etc
#define  MD_SW_I2C_SCL_PORT          GpioPortB
#define  MD_SW_I2C_SCL_PIN           GpioPin8
#define  MD_SW_I2C_SDA_PORT          GpioPortB
#define  MD_SW_I2C_SDA_PIN           GpioPin9



//simulate  4-20ma port
#define  MD_4_20MA_SCI_LATCH_PORT          GpioPortB
#define  MD_4_20MA_SCI_LATCH_PIN           GpioPin14
#define  MD_4_20MA_SCI_DATA_PORT           GpioPortB
#define  MD_4_20MA_SCI_DATA_PIN            GpioPin12
#define  MD_4_20MA_SCI_CLK_PORT            GpioPortB
#define  MD_4_20MA_SCI_CLK_PIN             GpioPin13



/////< LCD
//#define MD_LCD_COM0_PORT   GpioPortA
//#define MD_LCD_COM0_PIN    GpioPin9
//#define MD_LCD_COM1_PORT   GpioPortA
//#define MD_LCD_COM1_PIN    GpioPin10
//#define MD_LCD_COM2_PORT   GpioPortA
//#define MD_LCD_COM2_PIN    GpioPin11
//#define MD_LCD_COM3_PORT   GpioPortA
//#define MD_LCD_COM3_PIN    GpioPin12
//#define MD_LCD_SEG0_PORT   GpioPortA
//#define MD_LCD_SEG0_PIN    GpioPin8
//#define MD_LCD_SEG1_PORT   GpioPortC
//#define MD_LCD_SEG1_PIN    GpioPin9
//#define MD_LCD_SEG2_PORT   GpioPortC
//#define MD_LCD_SEG2_PIN    GpioPin8
//#define MD_LCD_SEG3_PORT   GpioPortC
//#define MD_LCD_SEG3_PIN    GpioPin7
//#define MD_LCD_SEG4_PORT   GpioPortC
//#define MD_LCD_SEG4_PIN    GpioPin6
//#define MD_LCD_SEG5_PORT   GpioPortB
//#define MD_LCD_SEG5_PIN    GpioPin15
//#define MD_LCD_SEG6_PORT   GpioPortB
//#define MD_LCD_SEG6_PIN    GpioPin14
//#define MD_LCD_SEG7_PORT   GpioPortB
//#define MD_LCD_SEG7_PIN    GpioPin13

//lp_uart0 RS485/irc  
#define  MD_RS485_M0P_LPUART0_TXD_PORT          GpioPortB
#define  MD_RS485_M0P_LPUART0_TXD_PIN           GpioPin0
#define  MD_RS485_M0P_LPUART0_TXD_PIN_AFN       GpioAf3
#define  MD_RS485_M0P_LPUART0_RXD_PORT          GpioPortC
#define  MD_RS485_M0P_LPUART0_RXD_PIN           GpioPin5
#define  MD_RS485_M0P_LPUART0_RXD_PIN_AFN       GpioAf1
#define  MD_RS485_DIR_PORT                      GpioPortC
#define  MD_RS485_DIR_PIN                       GpioPin4


//uart1  iot-lora/4g
#define  MD_IOT_M0P_UART1_TXD_PORT          GpioPortA
#define  MD_IOT_M0P_UART1_TXD_PIN           GpioPin2
#define  MD_IOT_M0P_UART1_TXD_PIN_AFN       GpioAf1
#define  MD_IOT_M0P_UART1_RXD_PORT          GpioPortA
#define  MD_IOT_M0P_UART1_RXD_PIN           GpioPin3
#define  MD_IOT_M0P_UART1_RXD_PIN_AFN       GpioAf1




/////< SPI0
//#define MD_SPI0_CS_PORT      GpioPortE
//#define MD_SPI0_CS_PIN       GpioPin12
//#define MD_SPI0_SCK_PORT     GpioPortE
//#define MD_SPI0_SCK_PIN      GpioPin13
//#define MD_SPI0_MISO_PORT    GpioPortE
//#define MD_SPI0_MISO_PIN     GpioPin14
//#define MD_SPI0_MOSI_PORT    GpioPortE
//#define MD_SPI0_MOSI_PIN     GpioPin15

/////< SPI1
//#define MD_SPI1_CS_PORT      GpioPortB
//#define MD_SPI1_CS_PIN       GpioPin12
//#define MD_SPI1_SCK_PORT     GpioPortB
//#define MD_SPI1_SCK_PIN      GpioPin13
//#define MD_SPI1_MISO_PORT    GpioPortB
//#define MD_SPI1_MISO_PIN     GpioPin14
//#define MD_SPI1_MOSI_PORT    GpioPortB
//#define MD_SPI1_MOSI_PIN     GpioPin15

       
///< XTH
#define SYSTEM_XTH         (8*1000*1000u)     
//#define MD_XTHI_PORT       GpioPortF
//#define MD_XTHI_PIN        GpioPin0
//#define MD_XTHO_PORT       GpioPortF
//#define MD_XTHO_PIN        GpioPin1

/////< XTL
#define SYSTEM_XTL          (32768u)          
//#define MD_XTLI_PORT       GpioPortC
//#define MD_XTLI_PIN        GpioPin14
//#define MD_XTLO_PORT       GpioPortC
//#define MD_XTLO_PIN        GpioPin15


#endif
