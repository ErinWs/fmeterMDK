#include "ddl.h"
#include "rtc.h"
#include "gpio.h"
#include "pcnt.h"
#include "lptim.h"
#include "bgr.h"
#include "adc.h"
#include "wdt.h"

#include "device.h"
#include "r_cg_sau.h"
#include "elora.h"
#include "protocol.h"
#include "net.h"
#include "irc.h"
#include "hum.h"
#include "collector.h"
#include "24cxx.h"
#include "adx.h"
#include "modbus.h"

#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"


static void enter_loar_cad_mode(void);
static void  enter_loar_work_mode(void);
static void  enter_loar_config_mode(void);
#define  MD_AS32_WORK_MODE_RECV_DELAY_MS 50// ms
/*******************************************PORTABLE***************************************/

#define MD_LORA_AUX_PORT_IRQN PORTC_E_IRQn
#define MD_LORA_INTP_ENABLE()                                              \
    do                                                                     \
    {                                                                      \
        Gpio_ClearIrq(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN);                  \
        Gpio_EnableIrq(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN, GpioIrqFalling); \
        EnableNvic(MD_LORA_AUX_PORT_IRQN, IrqLevel3, TRUE);                \
    } while (0)
#define MD_LORA_INTP_DISABLE()                                              \
    do                                                                      \
    {                                                                       \
        Gpio_DisableIrq(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN, GpioIrqFalling); \
        Gpio_ClearIrq(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN);                   \
        EnableNvic(MD_LORA_AUX_PORT_IRQN, IrqLevel3, FALSE);                \
    } while (0)

#define MD_GET_LORA_AUX_PIN GetBit(((uint32_t)&M0P_GPIO->PAIN + MD_LORA_AUX_PORT), MD_LORA_AUX_PIN)

#define MD_LORA_MD0_PIN_RESET SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_LORA_MD0_PORT), MD_LORA_MD0_PIN, FALSE)
#define MD_LORA_MD0_PIN_SET SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_LORA_MD0_PORT), MD_LORA_MD0_PIN, TRUE)
#define MD_LORA_MD1_PIN_RESET SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_LORA_MD1_PORT), MD_LORA_MD1_PIN, FALSE)
#define MD_LORA_MD1_PIN_SET SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_LORA_MD1_PORT), MD_LORA_MD1_PIN, TRUE)

static void write_lora(uint8_t *const buf, uint16_t len)
{
    // loraComps.sw._bit.busy=1;
    R_UART1_Send(buf, len);
}

static void enable_lora_com(void)
{
    enable_uart1();
}

static void disable_lora_com(void)
{
    disable_uart1();
}

static uint32_t config_lora_com(uint32_t baud, int16_t parity)
{
    App_Uart1Cfg(baud, parity);
    return baud;
}

static void lora_delay(uint16_t nNops)
{
    while (nNops--)
    {
        __NOP();
    }
}

/*************************************PORTABLE END****************************/
#define MD_DATA_ID_READ_MEASURE_INFO 0x901f
#define MD_DATA_ID_READ_BASIC_INFO 0x9020
#define MD_DATA_ID_READ_ALARM_PARAM 0x9021
#define MD_DATA_ID_READ_RPORT_PARAM 0x9022

#define MD_DATA_ID_READ_ACCESS_ADDR 0x9023
#define MD_DATA_ID_READ_HIGH_INFO 0x9026
#define MD_DATA_ID_READ_4_20MA_INFO 0x9027
#define MD_DATA_ID_READ_AUX_HIGH_H3_INFO 0x9028

#define MD_DATA_ID_SET_DEVICE_ADDR 0x9018
#define MD_DATA_ID_SET_ACCESS_ADDR 0x9005
#define MD_DATA_ID_ENTER_CAL_MODE 0xfa55     //
#define MD_DATA_ID_ENTER_NORMAL_MODE 0xfa99  //
#define MD_DATA_ID_WRITE_ALARM_PARAM 0x9001  //
#define MD_DATA_ID_WRITE_CAL_DATA 0x9002     //
#define MD_DATA_ID_WRITE_REPORT_PARAM 0x9003 //
#define MD_DATA_ID_WRITE_DEVICE_TIME 0x9004


static struct 
{ 
    uint8_t send_buf[64];
    uint8_t recv_buf[64];
    uint8_t recv_pos;
    union 
    {
           uint16_t All;
           struct{
                   volatile uint8_t  initing           :1;
                   volatile uint8_t  init_suc          :1;
                   volatile uint8_t  cmd_ack_ok        :1;
                   volatile uint8_t  data_init_once    :1;
                  
                  }_bit;
    }sw;
    uint8_t  init_state;
    int16_t  ack_delay_tmr;
    int16_t  ack_delay_init_const_tmr;
    uint8_t config_cmd;
    uint16_t readDataId;
    uint8_t  cmd;
  
    uint16_t  retry_times;
   lora_cfg_info_t  cfg_info;
    
}
loraMisc=
{
	{0},
	{0},
	 0,

	{0},
    0,
	 0,
	 0,
	 0,
     0,
	 0,

	 0,
	{0}
	 
};

static uint8_t Check_Sum(uint8_t *Data, uint8_t Len)
{
    uint8_t Sum = 0;
    uint8_t i = 0;
    for (i = 0; i < Len; i++)
    {
        Sum += Data[i];
    }
    return Sum;
}

static uint8_t Check_Sum_5A(const void *Data, uint8_t Len)
{
    uint8_t Sum = 0x5A;
    uint8_t i = 0;
    uint8_t *data = (uint8_t *)Data;
    for (i = 0; i < Len; i++)
    {
        Sum += data[i];
    }
    return Sum;
}

// static uint16_t  Check_Sum_back_2bytes(uint8_t const *Data,int16_t Len)
//{
//	uint16_t Sum=0;
//	int16_t  i=0;
//	for(i=0;i<Len;i++)
//	{
//		Sum+=Data[i];
//	}
//	return Sum;
// }

static int32_t formatData4fixDot(int32_t temp, int16_t dot)
{
    if (dot == 6)
    {
        temp /= 100;
    }
    else if (dot == 5)
    {
        temp /= 10;
    }
    else if (dot == 4)
    {
        temp /= 1;
    }
    else if (dot == 3)
    {
        temp *= 10;
    }
    else if (dot == 2)
    {
        temp *= 100;
    }
    else if (dot == 1)
    {
        temp *= 1000;
    }
    else if (dot == 0)
    {
        temp *= 10000;
    }
    return temp;
}

static int16_t read_lora_cfg_info(void *const buf, int16_t len)
{
    return _24cxx_comps.read(MD_LORA_PARAM_START_ADDR, buf, len);
}

static int16_t save_lora_cfg_info(void)
{
    loraMisc.cfg_info.cs = Check_Sum_5A(&loraMisc.cfg_info, &loraMisc.cfg_info.cs - (uint8_t *)&loraMisc.cfg_info);
    return _24cxx_comps.write(MD_LORA_PARAM_START_ADDR, &loraMisc.cfg_info, sizeof(loraMisc.cfg_info));
}

static int16_t add_measure_data(uint8_t Cmd, uint16_t DataId, uint8_t *buf)
{
    int16_t i = 3;
    uint8_t unit = 0;
    int32_t temp = 0;
    int32_t temp1 = 0;
    buf[0] = loraMisc.cfg_info.netId;
    buf[1] = 0;
    buf[2] = (loraMisc.cfg_info.freq) / 1000 - 410 + loraMisc.cfg_info.netId; // mac_end

    buf[i++] = loraMisc.cfg_info.nodeId;
    buf[i++] = (Cmd | 0x80);
    buf[i++] = 0;           // Length
    buf[i++] = DataId >> 8; // dataID
    buf[i++] = DataId;
    // add full range ...
    switch (DataId)
    {
    case MD_DATA_ID_READ_MEASURE_INFO:
        temp = formatData4fixDot(device_comps.press_cal_param.y[3], device_comps.press_cal_param.dot);
        buf[i++] = temp >> 24;
        buf[i++] = temp >> 16;
        buf[i++] = temp >> 8;
        buf[i++] = temp;
        // current p
        temp = formatData4fixDot(device_comps.current_press, device_comps.press_cal_param.dot);
        buf[i++] = temp >> 24;
        buf[i++] = temp >> 16;
        buf[i++] = temp >> 8;
        buf[i++] = temp;
        buf[i++] = (device_comps.press_cal_param.unit & 0x0f); // param unit
        break;
        //
        //      case MD_DATA_ID_READ_HIGH_INFO:
        //        	temp=formatData4fixDot(device_comps.high_cal_param.y[1],device_comps.high_cal_param.dot);
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //
        //		temp=formatData4fixDot(device_comps.current_high,device_comps.high_cal_param.dot);
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //
        //		temp=formatData4fixDot(device_comps.current_volume,device_comps.high_cal_param.dot+2); //v=S*H H/=10,S:3 fix dot
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //                break;
        //      case MD_DATA_ID_READ_4_20MA_INFO:
        //	    temp=formatData4fixDot(device_comps.current_4_20ma,3);
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //
        //		temp=formatData4fixDot(device_comps.current_4_20ma,3);
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //
        //		temp=formatData4fixDot(device_comps.current_4_20ma,3);
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //        break;
        //    case MD_DATA_ID_READ_AUX_HIGH_H3_INFO:
        //
        //	    temp=(int32_t)collectorComps.pvf;//mm 3fixdot->m
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //
        //		temp=*(float32_t *)&collectorComps.collector_data._32_data[0][0]*10000; //4fix dot Kpa
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //
        //		temp=*(float32_t *)&collectorComps.collector_data._32_data[1][0]*10000;
        //		buf[i++]=temp>>24;
        //		buf[i++]=temp>>16;
        //		buf[i++]=temp>>8;
        //		buf[i++]=temp;
        //
        //        buf[i++]=0;
        //		buf[i++]=0;
        //        break;

    default:
        break;
    }

    buf[i++] = device_comps.current_temp >> 8;
    buf[i++] = device_comps.current_temp;

    buf[i++] = device_comps.batt;

    if((loraMisc.cfg_info.nodeId&1)==0)
    { 
        buf[i++] = loraComps.packageRssi; 
    }
    else
    {
        buf[i++] = loraComps.evnRssi; 
    }
   
    // add other data
    buf[3 + 2] = i + 1 - 4 - 3;
    buf[i] = Check_Sum(buf + 3, i - 3);
    i++;

    return i;
}

static uint8_t Pro_lora(uint8_t Cmd, uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;
    uint16_t DataId = ((uint16_t)buf[3] << 8) + buf[4];
    if (Cmd == 0) // Read
    {
        switch (DataId)
        {

        case MD_DATA_ID_READ_MEASURE_INFO:     // measure info
        case MD_DATA_ID_READ_HIGH_INFO:        //
        case MD_DATA_ID_READ_4_20MA_INFO:      //
        case MD_DATA_ID_READ_AUX_HIGH_H3_INFO: // measure high_info
            if (buf[0] == 0xff)
            {
                loraMisc.ack_delay_tmr = (int16_t)loraMisc.cfg_info.nodeId * 5 + 6;
            }
            else
            {
                loraMisc.ack_delay_tmr = 6;
            }
            loraMisc.cmd = Cmd;
            loraMisc.readDataId = DataId;
            enter_loar_cad_mode();
            loraComps.op_window_time = 0;
            return len;
        default:
            return 1;
        }
        // write_lora(loraMisc.send_buf,i);
        return len;
    }
    return 1;
}

static void lora_awake_mcu_mode(void)
{

    disable_lora_com();
    config_lora_com(9600, 0);
    enable_lora_com();
    loraMisc.recv_pos = 0;
    loraComps.sw._bit.runing = 1;
    MD_LORA_INTP_DISABLE();
}

#if (MD_ELORA_VER_SEL == MD_ELORA_VER_A39C)
#define MD_BACK
static void enter_loar_config_mode(void)
{
    MD_LORA_MD0_PIN_RESET;
    MD_LORA_MD1_PIN_RESET;
    disable_lora_com();
    config_lora_com(9600, 0);
    enable_lora_com();
    loraComps.sw._bit.runing = 1;
    loraComps.work_st.mode = LORA_EM_CFG_MODE;
    MD_LORA_INTP_DISABLE();
}

static void enter_loar_cad_mode(void)
{
    MD_LORA_MD0_PIN_SET;
    MD_LORA_MD1_PIN_SET;
    disable_lora_com();
    loraComps.sw._bit.runing = 0;
    loraComps.work_st.mode = LORA_EM_CAD_MODE;
    MD_LORA_INTP_ENABLE();
}

static void enter_loar_work_mode(void)
{
    MD_LORA_MD0_PIN_SET;
    MD_LORA_MD1_PIN_RESET;
    loraComps.work_st.mode = LORA_EM_WORK_MODE;
    MD_LORA_INTP_DISABLE();
}

static uint8_t Check_lora_Com(uint8_t *Rec_Data, uint8_t Rec_Pos)
{
    uint8_t i = 0;
    if (loraComps.sw._bit.busy)
    {
        return 0;
    }
    if (Rec_Pos < 3)
    {
        return 0;
    }
    if (loraComps.work_st.mode == LORA_EM_CFG_MODE)
    {

        if (Rec_Data[0] != loraMisc.send_buf[0] || Rec_Data[1] != loraMisc.send_buf[1] || Rec_Data[2] != loraMisc.send_buf[2])
        {
            return 1;
        }
        switch (Rec_Data[1])
        {
        case 1:
            if (Rec_Pos < 7)
            {
                return 0;
            }
            else
            {
                loraComps.evnRssi = Rec_Data[4];
                loraComps.packageRssi = Rec_Data[6];
                loraMisc.ack_delay_tmr = 2;
                loraComps.dis_rssi_tmr_s = 1800;
                return 7;
            }
			break;
        case 4:
            if (Rec_Data[0] == 0)
            {
                if (Rec_Pos < 61)
                {
                    return 0;
                }
                else
                {
                    loraMisc.sw._bit.cmd_ack_ok = 1;
                    __disable_irq();
                    memcpy(loraMisc.send_buf, loraMisc.recv_buf, 61);
                    __enable_irq();
                    return 61;
                }
            }
            else if (Rec_Data[0] == 0x80)
            {
                loraMisc.sw._bit.cmd_ack_ok = 1;
                return 3;
            }
            else
            {
                return 1;
            }
            break;
        default:
            return 1;
        }
    }
    else if (loraComps.work_st.mode == LORA_EM_CAD_MODE)
    {
        if (Rec_Data[0] != loraMisc.cfg_info.nodeId && Rec_Data[0] != 0xff)
        {
            return 1;
        }
        if ((Rec_Data[1] != 0x00) && (Rec_Data[1] != 0x01))
        {
            return 1;
        }
        if (Rec_Data[2] > 32)
        {
            return 1;
        }
        if (Rec_Pos < Rec_Data[2] + 4)
        {
            return 0;
        }

        if (Rec_Data[Rec_Data[2] + 4 - 1] != Check_Sum(Rec_Data, Rec_Data[2] + 4 - 1))
        {
            return 1;
        }
        return Pro_lora(Rec_Data[1], Rec_Data, Rec_Data[2] + 4);
    }
    return 1;
}

static uint8_t load_lora_cfg_data_to_buf(uint8_t *buf)
{
    int16_t i;
    union
    {
        uint16_t param;
        struct
        {
            uint16_t rate : 3;
            uint16_t tx_power : 2;
            uint16_t channel : 7; // 410-490 {410+[0-80]}
        } param_field;

    } rf;
    rf.param = ((uint16_t)buf[8] << 8) + buf[9];
    rf.param_field.channel = (loraMisc.cfg_info.freq) / 1000 - 410 + loraMisc.cfg_info.netId;
    rf.param_field.rate = 7;
    buf[0] = 0x80;
    buf[8] = rf.param >> 8;
    buf[9] = rf.param;
    buf[10] = 0; // work_modeH
    buf[11] = 2; // work_modeL
    buf[46] = 40;
    buf[49] = 0;                       // work_option_h
    buf[50] = 0;                       ////work_option_L
    buf[51] = loraMisc.cfg_info.netId; // netid
    buf[52] = loraMisc.cfg_info.nodeId;
    ; // nodeid
    return i = 61;
}

static void init_lora(void)
{
    int i = 0;
    if (!loraMisc.sw._bit.data_init_once)
    {
        stc_gpio_cfg_t stcGpioCfg;
        Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
        DDL_ZERO_STRUCT(stcGpioCfg);
        stcGpioCfg.enDir = GpioDirOut;
        Gpio_Init(MD_LORA_MD0_PORT, MD_LORA_MD0_PIN, &stcGpioCfg);
        Gpio_Init(MD_LORA_MD1_PORT, MD_LORA_MD1_PIN, &stcGpioCfg);
        stcGpioCfg.enDir = GpioDirIn;
        stcGpioCfg.enPu = GpioPuEnable;
        Gpio_Init(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN, &stcGpioCfg);

        read_lora_cfg_info(&loraMisc.cfg_info, sizeof(loraMisc.cfg_info));
        if (loraMisc.cfg_info.cs != Check_Sum_5A(&loraMisc.cfg_info, &loraMisc.cfg_info.cs - (uint8_t *)&loraMisc.cfg_info))
        {
            loraComps.sw._bit.noParameter = 1;
            loraMisc.cfg_info.freq = 410000;
            loraMisc.cfg_info.netId = 0;  // nodeId
            loraMisc.cfg_info.nodeId = 1; // netID
        }
        loraComps.sw._bit.param_modified = 1;
        loraMisc.sw._bit.data_init_once = 1;
    }

    if (loraComps.sw._bit.param_modified)
    {
        loraComps.do_init = 1;
        loraMisc.init_state = 0;
        loraMisc.retry_times = 3;
        loraComps.sw._bit.init_ok = 0;
        loraComps.sw._bit.param_modified = 0;
    }

    if (loraComps.do_init)
    {
        switch (loraMisc.init_state)
        {
        case 0:
            if (!loraMisc.sw._bit.initing)
            {
                enter_loar_config_mode();
                loraComps.op_window_time = 4;
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }

        case 1:
            if (!loraMisc.sw._bit.initing)
            {
                i = 0;
                loraMisc.send_buf[i++] = 0;
                loraMisc.send_buf[i++] = 4;
                loraMisc.send_buf[i++] = 0x1e;
                write_lora(loraMisc.send_buf, i);
                loraComps.op_window_time = 20; // 16*50ms=800ms
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                if (loraMisc.retry_times > 0)
                {
                    loraMisc.retry_times--;
                }
                else
                {
                    loraMisc.init_state = -1;
                    loraMisc.retry_times = 3;
                }
            }
            else if (loraMisc.sw._bit.cmd_ack_ok)
            {
                loraMisc.sw._bit.cmd_ack_ok = 0;
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }
            break;
        case 2:
            if (!loraMisc.sw._bit.initing)
            {
                i = load_lora_cfg_data_to_buf(loraMisc.send_buf);
                write_lora(loraMisc.send_buf, 61);
                loraComps.op_window_time = 20; // 16*50ms=800ms
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                if (loraMisc.retry_times > 0)
                {
                    loraMisc.retry_times--;
                }
                else
                {
                    loraMisc.init_state = -1;
                    loraMisc.retry_times = 3;
                }
            }
            else if (loraMisc.sw._bit.cmd_ack_ok)
            {
                loraMisc.sw._bit.cmd_ack_ok = 0;
                loraMisc.sw._bit.init_suc = 1;
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }

            break;
        case 3:
            if (!loraMisc.sw._bit.initing)
            {
                enter_loar_work_mode();
                loraComps.op_window_time = 5;
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }
            break;

        default:
            if (loraMisc.sw._bit.init_suc)
            {
                loraComps.sw._bit.init_ok = 1;
            }
            loraComps.do_init = 0;
            enter_loar_cad_mode();
            loraComps.op_window_time = 0;
            break;
        }
    }
}

static void lora_comps_task_50ms(void)
{
    int16_t i = 0;
    if (loraComps.op_window_time > 0)
    {
        loraComps.op_window_time--;
        if (!loraComps.op_window_time)
        {
            if (!loraComps.do_init)
            {
                // loraComps.sw._bit.reading_rssi=0;
                loraComps.sw._bit.busy = 0;
                enter_loar_cad_mode();
                loraComps.op_window_time = 0;
                __NOP();
            }
        }
    }
    if (loraMisc.ack_delay_tmr > 0)
    {
        loraMisc.ack_delay_tmr--;

        if (loraMisc.ack_delay_tmr == 4)
        {
            enter_loar_config_mode();
            loraComps.op_window_time = 15;
            // lora_delay(8000);
        }
        if (loraMisc.ack_delay_tmr == 3)
        {

            loraMisc.send_buf[0] = 0;
            loraMisc.send_buf[1] = 1;
            loraMisc.send_buf[2] = 1;
            write_lora(loraMisc.send_buf, 3); // read_rssi
        }
        if (loraMisc.ack_delay_tmr == 1)
        {
            enter_loar_work_mode();
            loraComps.op_window_time = 10;
        }
        if (!loraMisc.ack_delay_tmr)
        {
            i = add_measure_data(loraMisc.cmd, loraMisc.readDataId, loraMisc.send_buf);
            write_lora(loraMisc.send_buf, i);
        }
    }
}

#elif (MD_ELORA_VER_SEL == MD_ELORA_VER_AS32)
#define  MD_BACK
 static void enter_loar_config_mode(void)
{
    MD_LORA_MD0_PIN_SET;
    MD_LORA_MD1_PIN_SET;
    disable_lora_com();
    config_lora_com(9600, 0);
    enable_lora_com();
    loraComps.sw._bit.runing = 1;
    loraComps.work_st.mode = LORA_EM_CFG_MODE;
    MD_LORA_INTP_DISABLE();
}

static void enter_loar_cad_mode(void)
{
    MD_LORA_MD0_PIN_RESET;
    MD_LORA_MD1_PIN_SET;
    disable_lora_com();
    loraComps.sw._bit.runing = 0;
    loraComps.work_st.mode = LORA_EM_CAD_MODE;
    MD_LORA_INTP_ENABLE();
}

static void enter_loar_work_mode(void)
{
    MD_LORA_MD0_PIN_RESET;
    MD_LORA_MD1_PIN_RESET;
    disable_lora_com();
    config_lora_com(9600, 0);
    enable_lora_com();
    loraComps.sw._bit.runing = 1;
    loraComps.work_st.mode = LORA_EM_WORK_MODE;
    MD_LORA_INTP_DISABLE();
}

static uint8_t Check_lora_Com(uint8_t *Rec_Data, uint8_t Rec_Pos)
{
    uint8_t i = 0;
    if (loraComps.sw._bit.busy)
    {
        return 0;
    }
    if (Rec_Pos < 1)
    {
        return 0;
    }
    if (loraComps.work_st.mode == LORA_EM_CFG_MODE)
    {
        switch (loraMisc.config_cmd)
        {
            case 0xc0:
                if (Rec_Pos < 4)
                {
                    return 0;
                }
                else
                {
                    if( Rec_Data[0] =='O' && Rec_Data[1] == 'K' && Rec_Data[2] == '\r' && Rec_Data[3] == '\n')
                    {
                        loraMisc.sw._bit.cmd_ack_ok = 1;
                        return 4;
                    }
                    else if( Rec_Data[0] =='E' && Rec_Data[1] == 'R' && Rec_Data[2] == 'R' && Rec_Data[3] == 'O')
                    {
                        if(Rec_Pos < 7)
                        {
                            return 0;
                        }
                        else 
                        {
                           return 7;
                        }
                    }
                    return 1;
                }
                break;
            case 0xc1:
                if (Rec_Data[0] != 0xc0)
                {
                    return 1;
                }
                if( Rec_Pos < 6)
                {
                    return 0;
                }
                else
                {
                        loraMisc.sw._bit.cmd_ack_ok = 1;
                       __disable_irq();
                      
                        memcpy(loraMisc.send_buf, loraMisc.recv_buf, 6);
                       __enable_irq();
                        return 6;
                }
                break;
//             case 0x73:
//                if (1)
//                {
//                    loraComps.packageRssi = Rec_Data[0];
//                    //loraMisc.ack_delay_tmr =3;
//                    loraComps.dis_rssi_tmr_s = 1800;
//                    return 1;
//                }
//                break;
//             case 0x74:
//                if (1)
//                {
//                    //loraMisc.ack_delay_tmr = 1;
//                    loraComps.dis_rssi_tmr_s = 1800;
//                    loraComps.evnRssi = Rec_Data[0];
//                    return 1;
//                }
//                break;
            default:
                return 1;
        }
    }
    else if (loraComps.work_st.mode == LORA_EM_CAD_MODE)
    {
        if (Rec_Pos < 3)
        {
            return 0;
        }
        if (Rec_Data[0] != loraMisc.cfg_info.nodeId && Rec_Data[0] != 0xff)
        {
            return 1;
        }
        if ((Rec_Data[1] != 0x00) && (Rec_Data[1] != 0x01))
        {
            return 1;
        }
        if (Rec_Data[2] > 32)
        {
            return 1;
        }
        if (Rec_Pos < Rec_Data[2] + 4)
        {
            return 0;
        }

        if (Rec_Data[Rec_Data[2] + 4 - 1] != Check_Sum(Rec_Data, Rec_Data[2] + 4 - 1))
        {
            return 1;
        }
        return Pro_lora(Rec_Data[1], Rec_Data, Rec_Data[2] + 4);
    }
	else if (loraComps.work_st.mode == LORA_EM_WORK_MODE)
	{
		switch (loraMisc.config_cmd)
        {
            case 0x73:
                if (1)
                {
                    loraComps.packageRssi = Rec_Data[0];
                    //loraMisc.ack_delay_tmr =3;
                    loraComps.dis_rssi_tmr_s = 60;
                    return 1;
                }
                break;
             case 0x74:
                if (1)
                {
                    //loraMisc.ack_delay_tmr = 1;
                    loraComps.dis_rssi_tmr_s = 60;
                    if(Rec_Data[0]!=0xff)
                    {
                        loraComps.evnRssi = Rec_Data[0];
                    }
                    return 1;
                }
                break;
            default:
                return 1;
        }
		
	}
    return 1;
}

static uint8_t load_lora_cfg_data_to_buf(uint8_t *buf)
{
    int16_t i = 0;
    union
    {
        uint8_t all;
        struct
        {
            uint8_t air_speed : 3;
            uint8_t uart_baud : 3;
            uint8_t uart_cs : 2;
        } field;

    } speed;
	 union
    {
        uint8_t all;
        struct
        {
            uint8_t send_power : 2;
            uint8_t res1 : 1;
            uint8_t wake_time : 3;
            uint8_t io_type : 1;
            uint8_t transmit_type : 1;

        } field;

    } option;
    speed.all = buf[3];
    speed.field.air_speed = 1;
   
    option.all = buf[5];
    option.field.wake_time=6;
    option.field.transmit_type = 1;
    buf[i++] = 0xc0;
    buf[i++] = loraMisc.cfg_info.netId;
    buf[i++] = loraMisc.cfg_info.nodeId;
    buf[i++] = speed.all;
    buf[i++] = ((loraMisc.cfg_info.freq) / 1000 - 410 + loraMisc.cfg_info.netId) & 0x1f;
    buf[i++] = option.all;
    return i;
}

static void init_lora(void)
{
    int i = 0;
    if (!loraMisc.sw._bit.data_init_once)
    {
       stc_gpio_cfg_t stcGpioCfg;
       Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
       DDL_ZERO_STRUCT(stcGpioCfg);
       stcGpioCfg.enDir = GpioDirOut;
       Gpio_Init(MD_LORA_MD0_PORT, MD_LORA_MD0_PIN, &stcGpioCfg);
       Gpio_Init(MD_LORA_MD1_PORT, MD_LORA_MD1_PIN, &stcGpioCfg);
       stcGpioCfg.enDir = GpioDirIn;
       stcGpioCfg.enPu = GpioPuEnable;
       Gpio_Init(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN, &stcGpioCfg);

        read_lora_cfg_info(&loraMisc.cfg_info, sizeof(loraMisc.cfg_info));
        if (loraMisc.cfg_info.cs != Check_Sum_5A(&loraMisc.cfg_info, &loraMisc.cfg_info.cs - (uint8_t *)&loraMisc.cfg_info))
        {
            loraComps.sw._bit.noParameter = 0;
            loraMisc.cfg_info.freq = 410000;
            loraMisc.cfg_info.netId = 0;  // nodeId
            loraMisc.cfg_info.nodeId = 1; // netID
        }
        loraComps.sw._bit.param_modified = 1;
        loraMisc.sw._bit.data_init_once = 1;
    }

    if (loraComps.sw._bit.param_modified)
    {
        loraComps.do_init = 1;
        loraMisc.init_state = 0;
        loraMisc.retry_times = 3;
        loraComps.sw._bit.init_ok = 0;
        loraComps.sw._bit.param_modified = 0;
    }

    if (loraComps.do_init)
    {
        switch (loraMisc.init_state)
        {
        case 0:
            if (!loraMisc.sw._bit.initing)
            {
                enter_loar_config_mode();
                loraComps.op_window_time = 4;
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }

        case 1:
            if (!loraMisc.sw._bit.initing)
            {
                i = 0;
                loraMisc.send_buf[i++] = 0xc1;
                loraMisc.send_buf[i++] = 0xc1;
                loraMisc.send_buf[i++] = 0xc1;
                loraMisc.config_cmd = 0xc1;
                write_lora(loraMisc.send_buf, i);
                loraComps.op_window_time = 20; // 16*50ms=800ms
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                if (loraMisc.retry_times > 0)
                {
                    loraMisc.retry_times--;
                }
                else
                {
                    loraMisc.init_state = -1;
                    loraMisc.retry_times = 3;
                }
            }
            else if (loraMisc.sw._bit.cmd_ack_ok)
            {
                loraMisc.sw._bit.cmd_ack_ok = 0;
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }
            break;
        case 2:
            if (!loraMisc.sw._bit.initing)
            {
                i = load_lora_cfg_data_to_buf(loraMisc.send_buf);
                loraMisc.config_cmd = 0xc0;
                write_lora(loraMisc.send_buf, i);
                loraComps.op_window_time = 20; // 16*50ms=800ms
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                if (loraMisc.retry_times > 0)
                {
                    loraMisc.retry_times--;
                }
                else
                {
                    loraMisc.init_state = -1;
                    loraMisc.retry_times = 3;
                }
            }
            else if (loraMisc.sw._bit.cmd_ack_ok)
            {
                loraMisc.sw._bit.cmd_ack_ok = 0;
                loraMisc.sw._bit.init_suc = 1;
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }

            break;
        case 3:
            if (!loraMisc.sw._bit.initing)
            {
                enter_loar_work_mode();
                loraComps.op_window_time = 3;
                loraMisc.sw._bit.initing = 1;
            }
            else if (!loraComps.op_window_time)
            {
                loraMisc.sw._bit.initing = 0;
                loraMisc.init_state++;
                loraMisc.retry_times = 3;
            }
            break;

        default:
            if (loraMisc.sw._bit.init_suc)
            {
                loraComps.sw._bit.init_ok = 1;
            }
            loraComps.do_init = 0;
            enter_loar_cad_mode();
            loraComps.op_window_time = 0;
            break;
        }
    }
}

static void lora_comps_task_50ms(void)
{
    int16_t i = 0;
    if (loraComps.op_window_time > 0)
    {
        loraComps.op_window_time--;
        if (!loraComps.op_window_time)
        {
            if (!loraComps.do_init)
            {
                // loraComps.sw._bit.reading_rssi=0;
                loraComps.sw._bit.busy = 0;
                enter_loar_cad_mode();
                loraComps.op_window_time = 0;
               // NOP();
            }
        }
    }
    if (loraMisc.ack_delay_tmr > 0)
    {
        loraMisc.ack_delay_tmr--;
        if(loraMisc.ack_delay_tmr == loraMisc.ack_delay_init_const_tmr-MD_AS32_WORK_MODE_RECV_DELAY_MS/50)
        {
             enter_loar_work_mode();
        }
        
       if (loraMisc.ack_delay_tmr == loraMisc.ack_delay_init_const_tmr-MD_AS32_WORK_MODE_RECV_DELAY_MS/50-5)
       {
            loraMisc.send_buf[i++] = 0xaf;
            loraMisc.send_buf[i++] = 0xaf;
            loraMisc.send_buf[i++] = 0x74;
            loraMisc.send_buf[i++] = 0;
            loraMisc.send_buf[i++] = 0xaf;
            loraMisc.send_buf[i++] = 0xf4;
            loraMisc.config_cmd = 0x74;
            write_lora(loraMisc.send_buf, i);
            loraComps.op_window_time = 15;
            return;
       }
        if (loraMisc.ack_delay_tmr == loraMisc.ack_delay_init_const_tmr-MD_AS32_WORK_MODE_RECV_DELAY_MS/50-7)
        {
            loraMisc.send_buf[i++] = 0xaf;
            loraMisc.send_buf[i++] = 0xaf;
            loraMisc.send_buf[i++] = 0x73;
            loraMisc.send_buf[i++] = 0;
            loraMisc.send_buf[i++] = 0xaf;
            loraMisc.send_buf[i++] = 0xf3;
            loraMisc.config_cmd = 0x73;
            write_lora(loraMisc.send_buf, i); // read_rssi
            return;
        }
        if (loraMisc.ack_delay_tmr == loraMisc.ack_delay_init_const_tmr-MD_AS32_WORK_MODE_RECV_DELAY_MS/50-9)
        {
             enter_loar_config_mode();
	        loraComps.op_window_time = 0;
            loraMisc.config_cmd=0;
        }
        if (loraMisc.ack_delay_tmr == 1)
        {
            enter_loar_work_mode();
            loraComps.op_window_time = 10;
            loraMisc.config_cmd=0;
        }
        if(loraMisc.ack_delay_tmr == 0)
        {
            i = add_measure_data(loraMisc.cmd, loraMisc.readDataId, loraMisc.send_buf);
            write_lora(loraMisc.send_buf, i);
        }
    }
}
#define MD_BACK
#endif

static void Deal_lora(void)
{
    uint8_t err = 0;
    do
    {
        err = Check_lora_Com(loraMisc.recv_buf, loraMisc.recv_pos);
        if (err > 0)
        {
            __disable_irq();

            memcpy(loraMisc.recv_buf, loraMisc.recv_buf + err, loraMisc.recv_pos - err);
            loraMisc.recv_pos -= err;
            __enable_irq();
        }
    } while (err > 0);
}

static void store_lora_buffer(uint8_t data)
{
    loraMisc.recv_buf[loraMisc.recv_pos] = data;
    loraMisc.recv_pos += 1;
    loraMisc.recv_pos &= 0x3f;
}

static void loraComps_task_handle(void)
{

    init_lora();
    if (loraComps.sw._bit.runing) // UART send data
    {
        Deal_lora();
    }
}
loraComps_t loraComps =
    {
        "", // uint8_t *desc;
        0,  // int16_t   do_init  Whether to initialize,1:init 0,no init

        &loraMisc.cfg_info,
        save_lora_cfg_info,

        {LORA_EM_CFG_MODE, LORA_EM_SEND}, // work_st
        {0},

        0, //	 int16_t  op_window_time;
        0, //    uint8_t evnRssi;
        0, //    uint8_t packageRssi;
        0, //    uint8_t dis_rssi_tmr_s;
        store_lora_buffer,
        loraComps_task_handle,
        lora_comps_task_50ms // void (*const task_send)(void);
};

void PortC_IRQHandler(void)
{
#if (MD_PRODUCT_NAME == MD_LORA)
    if (TRUE == Gpio_GetIrqStatus(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN))
    {
        if (!MD_GET_LORA_AUX_PIN)
        {
            if (loraComps.work_st.mode == LORA_EM_CAD_MODE)
            {
                lora_awake_mcu_mode();
                loraComps.op_window_time = 10;
            }
        }
        Gpio_ClearIrq(MD_LORA_AUX_PORT, MD_LORA_AUX_PIN);
    }
#endif
}
