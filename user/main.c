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
 * @file   main.c
 *
 * @brief  Source file for LPTIMER example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "sysctrl.h"
#include "lptim.h"
#include "lpm.h"
#include "gpio.h"
#include "wdt.h"
#include "lpuart.h"
#include "uart.h"
#include "lvd.h"
#include "reset.h"
#include "rtc.h"
#include "lcd.h"
#include "assert.h"
#include "adc.h"
#include "bgr.h"
#include "time.h"

#include "system.h"
#include "device.h"
#include "elora.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
//LPTIMER0��ETR


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

//int fputc(int ch,FILE *f)
//{
//	Uart_SendDataPoll(M0P_UART0,ch);
//	return ch;
//}


static void App_LPTimer1Init(void)
{
    stc_lptim_cfg_t    stcLptCfg;    
    DDL_ZERO_STRUCT(stcLptCfg);

    ///< ʹ��LPTIM0 ����ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpTim1, TRUE);
    
    stcLptCfg.enGate   = LptimGateLow;
    stcLptCfg.enGatep  = LptimGatePLow;
    if(!systemComps.sw._bit.is_xt1_running)
    {
        stcLptCfg.enTcksel = LptimRcl;
        stcLptCfg.u16Arr   = (uint16_t)(-50*(int32_t)38400/1000); 
    }
    else
    {
        stcLptCfg.enTcksel=LptimXtl;
        stcLptCfg.u16Arr   = (uint16_t)(-50*(int32_t)32768/1000); 
    }
    stcLptCfg.enTogen  = LptimTogEnLow;
    stcLptCfg.enCt     = LptimTimerFun;           //����������
    stcLptCfg.enMd     = LptimMode2;            //����ģʽΪģʽ2���Զ���װ��16λ������/��ʱ��
                    //Ԥװ�ؼĴ���ֵ��������ֵ
    Lptim_Init(M0P_LPTIMER1, &stcLptCfg);
    
    Lptim_ClrItStatus(M0P_LPTIMER1);            //����жϱ�־λ
    Lptim_ConfIt(M0P_LPTIMER1, TRUE);           //����LPTIMER�ж�    
    EnableNvic(LPTIM_0_1_IRQn, IrqLevel3, TRUE); 
    
    Lptim_Cmd(M0P_LPTIMER1, TRUE);
   
}

static void APP_RtcInit(void)
{
    stc_rtc_initstruct_t RtcInitStruct;
    
    DDL_ZERO_STRUCT(RtcInitStruct);                      //初始值清零
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开 
    RtcInitStruct.rtcAmpm = RtcPm;                       //12小时制
    if(!systemComps.sw._bit.is_xt1_running)
    {
         RtcInitStruct.rtcClksrc = RtcClkRcl;                 //内部低速时钟
    }
    else
    {
        RtcInitStruct.rtcClksrc = RtcClkXtl;
    }
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // 使能时钟误差补偿
    RtcInitStruct.rtcCompValue = 0;                      //补偿值  根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    //RTC计数器的使能
    Rtc_Cmd(TRUE);
     
}


static void App_WdtInit(void)
{
    ///< 开启WDT外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt,TRUE);
    ///< WDT 初始化
    Wdt_Init(WdtResetEn, WdtT6s55);
    ///< 启动 WDT
    Wdt_Start();
}


static void App_LvdInit(void)
{
    stc_lvd_cfg_t stcLvdCfg;

    DDL_ZERO_STRUCT(stcLvdCfg);     //变量清0

    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);    //开LVD时钟

    stcLvdCfg.enAct        = LvdActMskReset;                ///< 配置触发产生复位
    stcLvdCfg.enInputSrc   = LvdInputSrcMskVCC;             ///< 配置LVD输入源
    stcLvdCfg.enThreshold  = LvdMskTH2_0V;                  ///< 配置LVD基准电压
    stcLvdCfg.enFilter     = LvdFilterMskEnable;            ///< 滤波使能
    stcLvdCfg.enFilterTime = LvdFilterMsk28_8ms;            ///< 滤波时间设置
    stcLvdCfg.enIrqType    = LvdIrqMskHigh;                 ///< 复位触发类型
    Lvd_Init(&stcLvdCfg);
      
    ///< LVD 模块使能
    Lvd_Enable();
}

static void App_GetLastResetSrc(void)
{

    if(TRUE == Reset_GetFlag(ResetFlagMskPor5V) || TRUE == Reset_GetFlag(ResetFlagMskPor1_5V) )
    {
         Reset_ClearFlag(ResetFlagMskPor5V);
         Reset_ClearFlag(ResetFlagMskPor1_5V);
         systemComps.rst_code=0;  //power on
    }
    else if(TRUE == Reset_GetFlag(ResetFlagMskLvd))
    {
           Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);    //开LVD时钟
           Lvd_ClearIrq();
           Reset_ClearFlag(ResetFlagMskLvd);
           systemComps.rst_code=1;
    }
    else if(TRUE == Reset_GetFlag(ResetFlagMskLockup))
    {
           Reset_ClearFlag(ResetFlagMskLockup);
           systemComps.rst_code=2;
    }
	 else if(TRUE == Reset_GetFlag(ResetFlagMskRstb))
    {
           Reset_ClearFlag(ResetFlagMskRstb);
           systemComps.rst_code=3;
    }
	  else if(TRUE == Reset_GetFlag(ResetFlagMskWdt))
    {
           Reset_ClearFlag(ResetFlagMskWdt);
           systemComps.rst_code=4;
    }
	  else if(TRUE == Reset_GetFlag(ResetFlagMskSysreq))
    {
           Reset_ClearFlag(ResetFlagMskSysreq);
           systemComps.rst_code=5;
    }
    else
    {
         systemComps.rst_code=0x0e; 
    }
		Reset_ClearFlagAll(); 
}

static void App_ClkInit(void)
{
    uint32_t u32Temp;
    volatile int16_t cnt=0;
   
   //sys clk
    Sysctrl_ClkDeInit();
    Sysctrl_SetRCHTrim(SysctrlRchFreq8MHz);
	  Sysctrl_SetRCHTrim(SysctrlRchFreq16MHz);
    Sysctrl_SetRTCAdjustClkFreq(SysctrlRTC16MHz);
    SystemCoreClockUpdate();

    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle256);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);  
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_SetRCLTrim(SysctrlRclFreq38400);                
             
    //XTL
    Sysctrl_XTLDriverCfg(SysctrlXtlAmp3, SysctrlXtalDriver3);
    Sysctrl_SetXTLStableTime(SysctrlXtlStableCycle16384);
   // Sysctrl_ClkSourceEnable(SysctrlClkXTL,TRUE);
    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
    u32Temp = M0P_SYSCTRL->PERI_CLKEN0;
    M0P_SYSCTRL->PERI_CLKEN0_f.GPIO = TRUE;
    M0P_GPIO->PCADS |= 0xC000;
    M0P_SYSCTRL->SYSCTRL0_f.XTL_EN = TRUE;
    while(cnt<600 &&  1 != M0P_SYSCTRL->XTL_CR_f.STABLE)
    {
        delay1ms(1);
        cnt++;
    }
    if(cnt<600)
    {
         systemComps.sw._bit.is_xt1_running=1;
	}
}


static void App_LcdCfg()
{
    stc_lcd_cfg_t LcdInitStruct;
    stc_lcd_segcom_t LcdSegCom;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralLcd,TRUE);   ///< 开启LCD时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 开启GPIO时钟

    Gpio_SetAnalogMode(GpioPortA, GpioPin9);  //COM0
    Gpio_SetAnalogMode(GpioPortA, GpioPin10); //COM1
    Gpio_SetAnalogMode(GpioPortA, GpioPin11); //COM2
    Gpio_SetAnalogMode(GpioPortA, GpioPin12); //COM3   

    Gpio_SetAnalogMode(GpioPortA, GpioPin8);  //SEG0
    Gpio_SetAnalogMode(GpioPortC, GpioPin9);  //SEG1
    Gpio_SetAnalogMode(GpioPortC, GpioPin8);  //SEG2
    Gpio_SetAnalogMode(GpioPortC, GpioPin7);  //SEG3
    Gpio_SetAnalogMode(GpioPortC, GpioPin6);  //SEG4
    Gpio_SetAnalogMode(GpioPortB, GpioPin15); //SEG5
    Gpio_SetAnalogMode(GpioPortB, GpioPin14); //SEG6
    Gpio_SetAnalogMode(GpioPortB, GpioPin13); //SEG7
    Gpio_SetAnalogMode(GpioPortB, GpioPin3);  //VLCDH
    Gpio_SetAnalogMode(GpioPortB, GpioPin4);  //VLCD3
    Gpio_SetAnalogMode(GpioPortB, GpioPin5);  //VLCD2
    Gpio_SetAnalogMode(GpioPortB, GpioPin6);  //VLCD1

    LcdSegCom.u32Seg0_31 = 0x00000000;                              ///< 配置LCD_POEN0寄存器 开启SEG0~SEG7
    LcdSegCom.stc_seg32_51_com0_8_t.seg32_51_com0_8 = 0xffffffff;   ///< 初始化LCD_POEN1寄存器 全部关闭输出端口
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Com0_3 = 0;          ///< 使能COM0~COM3
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Mux = 0;             ///< Mux=0,Seg32_35=0,BSEL=1表示:选择外部电容工作模式，内部电阻断路
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg32_35 = 0;
    Lcd_SetSegCom(&LcdSegCom);                                      ///< LCD COMSEG端口配置

    LcdInitStruct.LcdBiasSrc = LcdExtCap;                          ///< 电容分压模式，需要外部电路配合
    LcdInitStruct.LcdDuty = LcdDuty8;                              ///< 1/4duty
    LcdInitStruct.LcdBias = LcdBias3;                              ///< 1/3 BIAS
    LcdInitStruct.LcdCpClk = LcdClk2k;                             ///< 电压泵时钟频率选择2kHz
    LcdInitStruct.LcdScanClk = LcdClk128hz;                        ///< LCD扫描频率选择128Hz
    LcdInitStruct.LcdMode = LcdMode0;                              ///< 选择模式0
    if(!systemComps.sw._bit.is_xt1_running)
    {
        LcdInitStruct.LcdClkSrc = LcdRCL;
    }
    else
    {
       LcdInitStruct.LcdClkSrc = LcdXTL; 
    }
    LcdInitStruct.LcdEn   = LcdEnable;                             ///< 使能LCD模块
    Lcd_Init(&LcdInitStruct);
}




static void App_BoardGPIOInit(void)
{
	stc_gpio_cfg_t         GpioInitStruct;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 

/*****************************OUTPUT 0 DRH  ****************************************/
    DDL_ZERO_STRUCT(GpioInitStruct);
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    GpioInitStruct.bOutputVal  = FALSE;

    Gpio_Init(MD_BACK_LED_PORT    , MD_BACK_LED_PIN   , &GpioInitStruct);
    Gpio_Init(MD_KEY_VDD_PORT     , MD_KEY_VDD_PIN    , &GpioInitStruct);
    Gpio_Init(MD_LCD_POWER_PORT   , MD_LCD_POWER_PIN  , &GpioInitStruct);

    Gpio_Init(MD_AVDD_CTL_PORT    , MD_AVDD_CTL_PIN    , &GpioInitStruct);
    Gpio_Init(MD_MCU_EXVREF_CTL_PORT, MD_MCU_EXVREF_CTL_PIN, &GpioInitStruct);
    Gpio_Init(MD_BAT_CTL_PORT     , MD_BAT_CTL_PIN     , &GpioInitStruct);
    Gpio_Init(MD_BAT_BLUNT_CTL_PORT  , MD_BAT_BLUNT_CTL_PIN     , &GpioInitStruct);


    Gpio_Init(MD_IR_VCM_PORT     , MD_IR_VCM_PIN     , &GpioInitStruct);
    Gpio_Init(MD_COLLECTOR_VCM_PORT  , MD_COLLECTOR_VCM_PIN     , &GpioInitStruct);
    
    Gpio_Init(MD_GSM_POWER_PORT    , MD_GSM_POWER_PIN    , &GpioInitStruct);
    Gpio_Init(MD_GSM_RST_PORT      , MD_GSM_RST_PIN     , &GpioInitStruct);
    Gpio_Init(MD_GSM_ON_PORT       , MD_GSM_ON_PIN     , &GpioInitStruct);

    Gpio_Init(MD_RS485_DIR_PORT       , MD_RS485_DIR_PIN     , &GpioInitStruct);

   
   Gpio_Init(MD_4_20MA_SCI_LATCH_PORT    , MD_4_20MA_SCI_LATCH_PIN, &GpioInitStruct);
   Gpio_Init(MD_4_20MA_SCI_DATA_PORT     , MD_4_20MA_SCI_DATA_PIN     , &GpioInitStruct);
   Gpio_Init(MD_4_20MA_SCI_CLK_PORT      , MD_4_20MA_SCI_CLK_PIN     , &GpioInitStruct);

/*****************************OUTPUT 1 DRH  ****************************************/
    DDL_ZERO_STRUCT(GpioInitStruct);
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    GpioInitStruct.bOutputVal  = TRUE;
    Gpio_Init(MD_METER_FREQ_OUT_POWER_PORT,MD_METER_FREQ_OUT_POWER_PIN,&GpioInitStruct);
    Gpio_Init(MD_METER_FREQ_OUT_CTL_PORT,MD_METER_FREQ_OUT_CTL_PIN,    &GpioInitStruct);
    Gpio_Init(MD_AVDD_CTL_PORT    , MD_AVDD_CTL_PIN, &GpioInitStruct);
    

   Gpio_Init(MD_XROM_I2C_VCC_PORT    , MD_XROM_I2C_VCC_PIN    , &GpioInitStruct);
   Gpio_Init(MD_XROM_I2C_WP_PORT    , MD_XROM_I2C_WP_PIN, &GpioInitStruct);
/*********************************OUTPUT 1 DRH  OD***************************************/
    DDL_ZERO_STRUCT(GpioInitStruct);
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    GpioInitStruct.enOD  = GpioOdEnable;
    GpioInitStruct.enPu  = GpioPuEnable;
    GpioInitStruct.bOutputVal  = TRUE;
   // Gpio_Init(MD_XROM_I2C_SCL_PORT  , MD_XROM_I2C_SCL_PIN     , &GpioInitStruct);
   // Gpio_Init(MD_XROM_I2C_SDA_PORT  , MD_XROM_I2C_SDA_PIN     , &GpioInitStruct);   

/**********************************INPUT PU*******************************/
    DDL_ZERO_STRUCT(GpioInitStruct);
    GpioInitStruct.enDir  = GpioDirIn;
    GpioInitStruct.enPu  = GpioPuEnable;
    Gpio_Init(MD_S_KEY_PORT  , MD_S_KEY_PIN     , &GpioInitStruct);
    Gpio_Init(MD_M_KEY_PORT  , MD_M_KEY_PIN     , &GpioInitStruct);
    Gpio_Init(MD_J_KEY_PORT  , MD_J_KEY_PIN     , &GpioInitStruct);
    Gpio_Init(MD_LORA_AUX_PORT  , MD_LORA_AUX_PIN     , &GpioInitStruct);
    Gpio_Init(MD_SW_I2C_SCL_PORT  , MD_SW_I2C_SCL_PIN     , &GpioInitStruct);
    Gpio_Init(MD_SW_I2C_SDA_PORT  , MD_SW_I2C_SDA_PIN     , &GpioInitStruct);
    Gpio_Init(MD_SDQ_PORT  , MD_SDQ_PIN     , &GpioInitStruct);

    Gpio_Init(MD_XROM_I2C_SCL_PORT  , MD_XROM_I2C_SCL_PIN     , &GpioInitStruct);
    Gpio_Init(MD_XROM_I2C_SDA_PORT  , MD_XROM_I2C_SDA_PIN     , &GpioInitStruct);   

  
   
    
/***************************************INPUT NO PU PD***********************/
    DDL_ZERO_STRUCT(GpioInitStruct);
    GpioInitStruct.enDir  = GpioDirIn;
   
    Gpio_Init(MD_EXT_POWER_CONNECT_DETECT_PORT  , MD_EXT_POWER_CONNECT_DETECT_PIN     , &GpioInitStruct);
    Gpio_Init(MD_4_20MA_CONNECT_DETECT_PORT , MD_4_20MA_CONNECT_DETECT_PIN    , &GpioInitStruct);

 
/***************************ANLOG*****************************************/
    DDL_ZERO_STRUCT(GpioInitStruct);
    Gpio_Init(MD_MCU_EXVREF_PORT  , MD_MCU_EXVREF_PIN     , &GpioInitStruct);
    Gpio_Init(MD_BAT_AIN4_PORT  , MD_BAT_AIN4_PIN     , &GpioInitStruct);
    Gpio_Init(MD_NTC_AIN25_PORT  , MD_NTC_AIN25_PIN     , &GpioInitStruct);
    Gpio_SetAnalogMode(MD_MCU_EXVREF_PORT,MD_MCU_EXVREF_PIN);
    Gpio_SetAnalogMode(MD_BAT_AIN4_PORT,MD_BAT_AIN4_PIN);
    Gpio_SetAnalogMode(MD_NTC_AIN25_PORT,MD_NTC_AIN25_PIN);

}


static void App_All_IOs_config_dig_in_pull_dn(void)
{
   
    uint32_t tmpReg = M0P_SYSCTRL->PERI_CLKEN0;
	M0P_SYSCTRL->PERI_CLKEN0_f.GPIO = 1;
    
    M0P_GPIO->PAADS &= 0x0000;  
    M0P_GPIO->PBADS &= 0x0000;  	  
    M0P_GPIO->PCADS &= 0x0000;  
    M0P_GPIO->PDADS &= 0x0000;  
    M0P_GPIO->PEADS &= 0x0000;  
    M0P_GPIO->PFADS &= 0x0000;  

    M0P_GPIO->PADIR	|= 0xffff;  
    M0P_GPIO->PBDIR	|= 0xffff;      
    M0P_GPIO->PCDIR	|= 0xffff;  
    M0P_GPIO->PDDIR	|= 0xffff;  
    M0P_GPIO->PEDIR	|= 0xffff;  
    M0P_GPIO->PFDIR	|= 0xffff;  

    M0P_GPIO->PAPU  &= 0x0000;  
    M0P_GPIO->PBPU  &= 0x0000;  
    M0P_GPIO->PCPU  &= 0x0000;  
    M0P_GPIO->PDPU  &= 0x0000;  
    M0P_GPIO->PEPU  &= 0x0000;  
    M0P_GPIO->PFPU  &= 0x0000; 
    
    M0P_GPIO->PAPD  |= 0xffff;  
    M0P_GPIO->PBPD  |= 0xffff;  
    M0P_GPIO->PCPD  |= 0xffff;  
    M0P_GPIO->PDPD  |= 0xffff;  
    M0P_GPIO->PEPD  |= 0xffff;  
    M0P_GPIO->PFPD  |= 0xffff; 
    M0P_SYSCTRL->PERI_CLKEN0 = tmpReg;

}

static void App_unused_peripherals_init(void)
{ 
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    Adc_Disable();
    Bgr_BgrDisable();
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, FALSE);
}

static void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    App_All_IOs_config_dig_in_pull_dn();
    App_BoardGPIOInit();
	App_ClkInit();//must after IO init
   #ifndef MD_NO_LCD
   // App_LcdCfg();
   #endif

    APP_RtcInit();
    App_GetLastResetSrc();
    App_WdtInit();
    App_LvdInit();

    //device_comps.buzzer.stop();
    App_unused_peripherals_init();
    App_LPTimer1Init();
    
//    while (1)
//   {
//			 
//       Lpm_GotoDeepSleep(FALSE); 
//   }
    __enable_irq();
    
  /* End user code. Do not edit comment generated here */
}

struct tm _tm={
	.tm_sec=11,
	.tm_min=20,
	.tm_hour=10,
	.tm_mday=24,
	.tm_mon=9,
	.tm_year=2024-1900
},*ptm;
time_t timer;

int32_t  main(void)
{    
   
    timer=mktime(&_tm);
    ptm=localtime(&timer);
    #if(__sizeof_long==8)
    {
        timer=mktime(&_tm);
        ptm=localtime(&timer);
        timer=0b1110;
    }
    #endif
	 R_MAIN_UserInit();
  
    /* Start user code. Do not edit comment generated here */
    systemComps.user_init();
    do
    {
        systemComps.task_process();
    }
    while (1U);
    /* End user code. Do not edit comment generated here */
}



/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



