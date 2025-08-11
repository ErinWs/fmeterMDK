#include "ddl.h"
#include "rtc.h"
#include "gpio.h"

#include "r_cg_sau.h"
#include "device.h"
#include "string.h"
#include "stdio.h"
#include "24cxx.h"
#include "hum.h"
#include "irc.h"
#include "modbus.h"
#include "adx.h"
#include "elora.h"
#include "ertc.h"

#define  MD_MODBUS_OP_WINDOW_TIME_OUT   (7200)

/*******************************************PORTABLE***************************************/

void disable_modbus_com(void)
{
   disable_LPuart0();
   modbusComps.sw._bit.busy=0;
}


void enable_modbus_com(void)
{
    enable_LPuart0();
}

static void write_modbus(uint8_t * const buf,uint16_t len)
{
    if(len>0)
    {
        MD_SET_RS_485_T_R;
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        R_LPUART0_Send(buf,len);
    }
}


static uint32_t config_modbus_com(uint32_t baud,int16_t parity)
{
    disable_modbus_com();
    uint32_t baud_num[]={1200,2400,4800,9600,19200};
    if(baud<0 || baud>4)
    {
        return 1;
    } 
    App_LpUart0Cfg(baud_num[baud],parity);
    return 0;
   
}

static uint32_t modify_modbus_baud_verify(uint32_t baud,int16_t parity)
{
    if(modbusComps.sw._bit.runing)
    {
        config_modbus_com(baud,parity);
        enable_modbus_com();
    }
    return 0;
}

static void Deconfig_modbus_com(void)
{
    App_LpUart0DeCfg();
}


/*************************************PORTABLE END*******************************/
#define  MD_MODBUS_MAX_RECV_BUF_POS          192
#define  MD_MODBUS_MAX_CFG_RECV_BUF_POS       32

static struct 
{ 
	uint8_t  send_buf[192];
	uint8_t  recv_buf[MD_MODBUS_MAX_RECV_BUF_POS];
	uint8_t  recv_pos;


	uint8_t  send_cfg_buf[32];
    uint8_t  recv_cfg_buf[MD_MODBUS_MAX_CFG_RECV_BUF_POS];
	uint8_t  recv_cfg_pos;
	modebs_param_t param;
}
modbusMisc=
{
	{0},
	{0},
	 0,
	
	{0}, 
	{0},
	 0,
	
	{0}
};

static void * const modbus_data_map[]=
{
    &device_comps.current_temp,
    &modbusMisc.param.baud,
    &device_comps.press_cal_param.dot,
    &device_comps.press_cal_param.unit,
    &device_comps.current_press,
    &device_comps.press_cal_param.y[7],
   
    &device_comps.batt,
    &device_comps.coe.press,
    &device_comps.coe.pt_temp,
};

static uint16_t generateCRC(uint8_t *buffer, uint16_t messageLength)
{
	uint16_t crc = 0xFFFF;
	int16_t i, j = 0;
	for (i = 0;i < messageLength;i++)
	{
		crc ^= buffer[i];
		for (j = 8; j != 0; j--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	
	return crc;
}


static uint8_t Check_Sum(uint8_t *Data,uint8_t Len)
{
	uint8_t Sum=0;
	uint8_t i=0;
	for(i=0;i<Len;i++)
	{
		Sum+=Data[i];
	}
	return Sum;
}

static int32_t  get4bByteFloatCode(uint8_t *buf,float32_t x)
{
    int32_t i=0;
    buf[i++] =*((uint8_t *)&x+3);
    buf[i++] =*((uint8_t *)&x+2);
    buf[i++] =*((uint8_t *)&x+1);
    buf[i++] =* (uint8_t *)&x;
    return 4;
}

static float32_t getFloatDataFrom4ByteCode(uint8_t *buf)
{
    uint32_t  dat=((unsigned long)buf[0]<<24)+((unsigned long)buf[1]<<16)+((unsigned long)buf[2]<<8)+buf[3];
    return *(float32_t *)&dat;
}

static int32_t get2ByteCodeFromFloatCodeFirst2Byte(uint8_t *buf,float32_t x)
{
    int32_t i=0;
    buf[i++] =*((uint8_t *)&x+3);
    buf[i++] =*((uint8_t *)&x+2);
    return 2;
}

static int32_t get2ByteCodeFromFloatCodeLast2Byte(uint8_t *buf,float32_t x)
{
    int32_t i=0;
    buf[i++] =*((uint8_t *)&x+1);
    buf[i++] =* (uint8_t *)&x;
    return 2;
}

static uint8_t Check_Sum_5A( const void* Data,uint8_t Len)
{
    uint8_t Sum=0x5A;
    uint8_t i=0;
	uint8_t *data=(uint8_t *)Data;
    for(i=0;i<Len;i++)
    {
        Sum+=data[i];
    }
    return Sum;
}


static int32_t pwr(int16_t n)
{
    if(n==0)
    {
        return 1;
    }
    if(n==1)
    {
        return 10;
    }
    if(n==2)
    {
        return 100;
    }
    if(n==3)
    {
        return 1000;
    }
    if(n==4)
    {
        return 10000;
    }
    if(n==5)
    {
        return 100000;
    }
    if(n==6)
    {
        return 1000000;
    }
    if(n==7)
    {
        return 10000000;
    }
    if(n==8)
    {
        return 100000000;
    }
     if(n==9)
    {
        return 1000000000;
    }
    return 1;
}

static float32_t f_mul(float32_t a,float32_t b)
{
    return a*b;
}
static float32_t f_div(float32_t a,float32_t b)
{
    return a/b;
}

static int16_t read_modbus_param(void  *buf,int16_t len )
{
     return _24cxx_comps.read(MD_MODBUS_PARAM_START_ADDR,buf,len);
}

static int16_t save_modbus_param(void)
{
     modbusMisc.param.cs=Check_Sum_5A(&modbusMisc.param, &modbusMisc.param.cs-(uint8_t *)&modbusMisc.param);
     return _24cxx_comps.write(MD_MODBUS_PARAM_START_ADDR,&modbusMisc.param,sizeof(modbusMisc.param));
}

static int16_t pro_modus_err(int16_t err,uint8_t Cmd,int16_t len)
{
    int16_t i=0;
    uint16_t crc;
    modbusMisc.send_buf[i++]=modbusMisc.param.addr;
    modbusMisc.send_buf[i++]=Cmd+0x80;
    modbusMisc.send_buf[i++]=err;
    crc=generateCRC(modbusMisc.send_buf, i);
    modbusMisc.send_buf[i++]=crc;
    modbusMisc.send_buf[i++]=crc>>8;
    write_modbus(modbusMisc.send_buf,i);
    return len;
}


static uint8_t Pro_modbus(uint8_t Cmd,uint8_t *buf,int16_t len)
{
	int16_t i=0;
    int16_t k;
	uint16_t crc;
	int32_t tmp;
    int16_t cnt=0;
    float32_t  datx;
    float32_t  daty;
    uint16_t datu;
    uint16_t cur_dis_unit;
    uint8_t u8m;
    enum
    {
       EM_NULL=0,
       EM_FLOW_METER,
       EM_MODBUS,
       EM_LORA,
       EM_ACCESS,
       EM_REPORT,
       EM_METER,
       EM_COE,
       EM_GPS,
       EM_MISC,
       EM_SYSTEM_TIME
    }save_data_type=EM_NULL;
	
	uint16_t addr=((uint16_t)buf[2]<<8)+buf[3];
	if(Cmd==0x03)//Read
	{
        uint16_t num=((uint16_t)buf[4]<<8)+buf[5];
        if((addr<0x42)&&(num+addr<0x42+1))
		{
    	    modbusMisc.send_buf[i++]=modbusMisc.param.addr;
            modbusMisc.send_buf[i++]=Cmd;
            modbusMisc.send_buf[i++]=num*2;//length
            for(k=0;k<num;k++)
            {
                switch (addr)
                {
                    case 0:  
                        datx=device_comps.current_temp/10.f;
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
//                        modbusMisc.send_buf[i++] =*((uint8_t *)&datx+1);
//                        modbusMisc.send_buf[i++] =*((uint8_t *)&datx+0);
                        addr++;
                        break;
                    case 1:
                        datx=device_comps.current_temp/10.f;
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx);
//                        modbusMisc.send_buf[i++] =*((uint8_t *)&datx+3);
//                        modbusMisc.send_buf[i++] =*((uint8_t *)&datx+2);
                        addr++;
                        break;
                    case 2: 
                        tmp=device_comps.get_cur_flow_meter_unit_display_value(device_comps.current_flow,device_comps.flow_cal_param.dot,device_comps.flow_meter.unit,&u8m,&cur_dis_unit);
                        datx=f_div(tmp,pwr(u8m));
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+1);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+0);
                        addr++;
                        break;
                    case 3:
                        tmp=device_comps.get_cur_flow_meter_unit_display_value(device_comps.current_flow,device_comps.flow_cal_param.dot,device_comps.flow_meter.unit,&u8m,&cur_dis_unit);
                        datx=f_div(tmp,pwr(u8m));
                         i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx); 
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+3);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+2);
                        addr++;
                        break;
                   case 4: 
                        tmp=device_comps.current_v;
                        datx=f_div(tmp,pwr(2));
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+1);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+0);
                        addr++;
                        break;
                   case 5:
                        tmp=device_comps.current_v;
                        datx=f_div(tmp,pwr(2));
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx); 
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+3);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+2);
                        addr++;
                        break; 
                   case 6: 
                        tmp=device_comps.flow_roll_freq_comped_cur;//0.1hz
                        datx=f_div(tmp,pwr(1));
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+1);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+0);
                        addr++;
                        break;
                   
                  case 7:
                        tmp=device_comps.flow_roll_freq_comped_cur;//0.1hz
                        datx=f_div(tmp,pwr(1));
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx); 
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+3);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+2);
                        addr++;
                        break;
                     
                  case 8: 
                        tmp=device_comps.meter.total_int;//m^3
                        modbusMisc.send_buf[i++]=tmp>>24;
                        modbusMisc.send_buf[i++]=tmp>>16;
                        addr++;
                        break;
                  case 9:
                        tmp=device_comps.meter.total_int;//m^3
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                        addr++;
                        break; 
                  case 10: 
                        tmp=f_mul(device_comps.meter.total_dec,pwr(3));
                        modbusMisc.send_buf[i++]=tmp>>24;
                        modbusMisc.send_buf[i++]=tmp>>16;
                        addr++;
                        break;
                  case 11:
                        tmp=f_mul(device_comps.meter.total_dec,pwr(3));
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                        addr++;
                        break;
                      
                  case 12: 
                        device_comps.get_cur_flow_meter_unit_display_value(device_comps.current_flow,device_comps.flow_cal_param.dot,device_comps.flow_meter.unit,&u8m,&cur_dis_unit);
                        datx=cur_dis_unit;
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+1);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+0);
                        addr++;
                        break;
                  case 13:
                        device_comps.get_cur_flow_meter_unit_display_value(device_comps.current_flow,device_comps.flow_cal_param.dot,device_comps.flow_meter.unit,&u8m,&cur_dis_unit);
                        datx=cur_dis_unit;
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx); 
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+3);
                       // modbusMisc.send_buf[i++] =*((uint8_t *)&datx+2);
                        addr++;
                        break; 

                        ////////////////////////////////////////////////////////////////
                  case 0x0e:
                        datx=f_div(device_comps.current_press,pwr(device_comps.press_cal_param.dot));
                        if(device_comps.press_cal_param.unit==0)
                        {
                            datx=f_div(device_comps.current_press*1000,pwr(device_comps.press_cal_param.dot));
                        }
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                        addr++;
                        break;
                  case 0x0f:
                        datx=f_div(device_comps.current_press,pwr(device_comps.press_cal_param.dot));
                        if(device_comps.press_cal_param.unit==0)
                        {
                            datx=f_div(device_comps.current_press*1000,pwr(device_comps.press_cal_param.dot));
                        };
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx); 
                        addr++;
                        break;
                  case 0x10:
                        datx=f_div(device_comps.current_flowN,pwr(device_comps.flow_cal_param.dot));
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                        addr++;
                        break;
                  case 0x11: 
                        datx=f_div(device_comps.current_flowN,pwr(device_comps.flow_cal_param.dot));
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx);
                        addr++;
                        break;
                  case 0x12: 
                        tmp=device_comps.meter.total_intN;//m^3
                        modbusMisc.send_buf[i++]=tmp>>24;
                        modbusMisc.send_buf[i++]=tmp>>16;
                        addr++;
                        break;
                  case 0x13:
                        tmp=device_comps.meter.total_intN;//m^3
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                        addr++;
                        break; 
                  case 0x14: 
                        tmp=f_mul(device_comps.meter.total_decN,pwr(3));
                        modbusMisc.send_buf[i++]=tmp>>24;
                        modbusMisc.send_buf[i++]=tmp>>16;
                        addr++;
                        break;
                  case 0x15:
                        tmp=f_mul(device_comps.meter.total_decN,pwr(3));
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                        addr++;
                        break;
                  case 0x16:
                        datx=f_div(device_comps.misc_param.I_o_low,pwr(device_comps.flow_cal_param.dot));
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                        addr++;
                        break;
                  case 0x17: 
                        datx=f_div(device_comps.misc_param.I_o_low,pwr(device_comps.flow_cal_param.dot));
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx);
                        addr++;
                        break;
                  case 0x18:
                        datx=f_div(device_comps.misc_param.I_o_high,pwr(device_comps.flow_cal_param.dot));
                        i+=get2ByteCodeFromFloatCodeFirst2Byte(&modbusMisc.send_buf[i],datx);
                        addr++;
                        break;
                  case 0x19: 
                        datx=f_div(device_comps.misc_param.I_o_high,pwr(device_comps.flow_cal_param.dot));
                        i+=get2ByteCodeFromFloatCodeLast2Byte(&modbusMisc.send_buf[i],datx);
                        addr++;
                        break; 
                  case 0x20: 
                        tmp=device_comps.misc_param.density;
                        modbusMisc.send_buf[i++]=tmp>>24;
                        modbusMisc.send_buf[i++]=tmp>>16;
                        addr++;
                        break;
                  case 0x21:
                        tmp=device_comps.misc_param.density;
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                        addr++;
                        break;      
                  case 0x22: 
                        tmp=device_comps.flow_meter.pluse_equ;
                        modbusMisc.send_buf[i++]=tmp>>24;
                        modbusMisc.send_buf[i++]=tmp>>16;
                        addr++;
                        break;
                  case 0x23:
                        tmp=device_comps.flow_meter.pluse_equ;
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                        addr++;
                        break;


                  case 0x24:
                  case 0x25:
                  case 0x26:
                  case 0x27:
                  case 0x28:
                  case 0x29:
                  case 0x2a:
                  case 0x2b:
                  case 0x2c:
                        modbusMisc.send_buf[i++]=0;
                        modbusMisc.send_buf[i++]=0;
                        addr++;
                        break;
                  case 0x2d:
                        datu=modbusComps.cmd_out_raw_4ma_20ma_timer;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x2e:
                        datu=device_comps.coe._4ma_raw_value;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x2f:
                        datu=device_comps.coe._20ma_raw_value;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x30:
                        modbusMisc.send_buf[i++]=0;
                        modbusMisc.send_buf[i++]=0;
                        addr++;
                        break; 
                  case 0x31:
                        device_comps.get_cur_flow_meter_unit_display_value(device_comps.current_flow,device_comps.flow_cal_param.dot,device_comps.flow_meter.unit,&u8m,&cur_dis_unit);
                        modbusMisc.send_buf[i++]=cur_dis_unit>>8;
                        modbusMisc.send_buf[i++]=cur_dis_unit;
                        addr++;
                        break;
                  case 0x32:
                        datu=device_comps.misc_param.I_o_dir;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x33:
                        datu=device_comps.flow_meter.pluse_width;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x34:
                        datu=modbusMisc.param.addr;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x35:
                        datu=modbusMisc.param.baud;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x36:
                        datu=device_comps.coe.flow;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x37:
                        datu=device_comps.coe.out_4_20ma;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x38:
                        datu=device_comps.coe.pt_temp;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x39:
                        datu=device_comps.coe.press;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x40:
                        datu=device_comps.flow_meter.sensor_low_freq_cutoff;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  case 0x41:
                        datu=device_comps.flow_meter.avg_freq_filter_timer;
                        modbusMisc.send_buf[i++]=datu>>8;
                        modbusMisc.send_buf[i++]=datu;
                        addr++;
                        break;
                  default:
                        modbusMisc.send_buf[i++]=0;
                        modbusMisc.send_buf[i++]=0;
                        addr++;
                        break;
                       
                }
            }
            crc=generateCRC(modbusMisc.send_buf, i);
            modbusMisc.send_buf[i++]=crc;
            modbusMisc.send_buf[i++]=crc>>8;
            write_modbus(modbusMisc.send_buf,i);
            return len;
      }
    
        
//          else if((addr>0x03e8)&&(addr<0x03ed)&&(num+addr<0x03ee))
//          {
                
//                modbusMisc.send_buf[i++]=modbusMisc.param.addr;
//                modbusMisc.send_buf[i++]=Cmd;
//                modbusMisc.send_buf[i++]=num*2;//length
//                for(k=0;k<num;k++)
//                {
//                        if(addr==0x03e9)
//                        {
//                            modbusMisc.send_buf[i++] =device_comps.coe.ad_gain>>8;
//                            modbusMisc.send_buf[i++] =device_comps.coe.ad_gain;
//                            addr++;
//                        }
//                        else if(addr==0x03ea)
//                        {
//                            modbusMisc.send_buf[i++] =device_comps.coe.is_auto_aligned>>8;
//                            modbusMisc.send_buf[i++] =device_comps.coe.is_auto_aligned;
//                            addr++;
//                        }
//                        else if(addr==0x03eb)
//                        {
//                            modbusMisc.send_buf[i++] =device_comps.coe.cut_off_value>>8;
//                            modbusMisc.send_buf[i++] =device_comps.coe.cut_off_value;
//                            addr++;
//                        }
//                        else if(addr==0x03ec)
//                        {
//                            modbusMisc.send_buf[i++] =device_comps.coe.press>>8;
//                            modbusMisc.send_buf[i++] =device_comps.coe.press;
//                            addr++;
//                        }
//                }
//                crc=generateCRC(modbusMisc.send_buf, i);
//                modbusMisc.send_buf[i++]=crc;
//                modbusMisc.send_buf[i++]=crc>>8;
//                write_modbus(modbusMisc.send_buf,i);
//                return len;
//         }
//         else if((addr>0x2f)&&(addr<0x32)&&(num+addr<0x33))
//         {
//            modbusMisc.send_buf[i++]=modbusMisc.param.addr;
//            modbusMisc.send_buf[i++]=Cmd;
//            modbusMisc.send_buf[i++]=num*2;//length
//            for(k=0;k<num;k++)
//            {
//                if(addr==0x30)
//                {
//                    modbusMisc.send_buf[i++] =device_comps.coe.out_4_20ma>>8;
//                    modbusMisc.send_buf[i++] =device_comps.coe.out_4_20ma;
//                    addr++;
//                }
//                else if(addr==0x31)//
//                {
                    
//                    modbusMisc.send_buf[i++] =0;//device_comps.current_4_20ma>>8;
//                    modbusMisc.send_buf[i++] =0;//device_comps.current_4_20ma;
//                    addr++;
//                }
//                else
//                {
//                    return 1;
//                }   
//            }
//            crc=generateCRC(modbusMisc.send_buf, i);
//            modbusMisc.send_buf[i++]=crc;
//            modbusMisc.send_buf[i++]=crc>>8;
//            write_modbus(modbusMisc.send_buf,i);
//            return len;
//         }
         
//         else if(( (addr==0x0064)||(addr==0x0066) )&&(num+addr<0x0069))
//         {
             
//           int16_t  param_unit;
//           int16_t  dot;
           
//           if(device_comps.flow_cal_param.is_calibrated)
//           {
//               param_unit= device_comps.flow_cal_param.unit;
//               dot=device_comps.flow_cal_param.dot;
//               daty=device_comps.flow_cal_param.y[device_comps.flow_cal_param.y_pos-1];
//           }
//           else if(device_comps.press_cal_param.is_calibrated)
//           {
//                param_unit=device_comps.press_cal_param.unit;
//                dot=device_comps.press_cal_param.dot;
//                daty=device_comps.press_cal_param.y[3];
//           }
//           if(param_unit==0)
//           {
//                datx=(float32_t)device_comps.current_press*1000;
//                daty=daty*1000;
//           }
//           else if((param_unit&0x0f)==1)//Kpa 
//           {
//                datx=(float32_t)device_comps.current_press;
                
//           }
//            datx=datx/pwr(dot);
//            daty=daty/pwr(dot);
//            modbusMisc.send_buf[i++]=modbusMisc.param.addr;
//            modbusMisc.send_buf[i++]=Cmd;
//            modbusMisc.send_buf[i++]=num*2;//length
//            for(k=0;k<num;k++)
//            {
//                if(addr==0x0064)
//                {
//                    modbusMisc.send_buf[i++] =*((uint8_t *)&datx+3);
//                    modbusMisc.send_buf[i++] =*((uint8_t *)&datx+2);
//                    addr++;
//                }
//                else if(addr==0x0065)//
//                {
//                    modbusMisc.send_buf[i++] =*((uint8_t *)&datx+1);
//                    modbusMisc.send_buf[i++] =*(uint8_t *)&datx;
//                    addr++;
//                }
//                else if(addr==0x0066)
//                {
//                    modbusMisc.send_buf[i++] =*((uint8_t *)&daty+3);
//                    modbusMisc.send_buf[i++] =*((uint8_t *)&daty+2);
//                    addr++;
//                }
//                else if(addr==0x0067)//
//                {
                    
//                    modbusMisc.send_buf[i++] =*((uint8_t *)&daty+1);
//                    modbusMisc.send_buf[i++] =*((uint8_t *)&daty);
//                    addr++;
//                }
//                else
//                {
//                    return 1;
//                }   
//            }
//            crc=generateCRC(modbusMisc.send_buf, i);
//            modbusMisc.send_buf[i++]=crc;
//            modbusMisc.send_buf[i++]=crc>>8;
//            write_modbus(modbusMisc.send_buf,i);
//            return len;
//         }
         
         else if(addr==0x2001)
         {
            modbusMisc.send_buf[i++]=modbusMisc.param.addr;
            modbusMisc.send_buf[i++]=Cmd;
            modbusMisc.send_buf[i++]=num*2;//length
            
            modbusMisc.send_buf[i++] =0;
            modbusMisc.send_buf[i++] =modbusMisc.param.addr;
            
            crc=generateCRC(modbusMisc.send_buf, i);
            modbusMisc.send_buf[i++]=crc;
            modbusMisc.send_buf[i++]=crc>>8;
            write_modbus(modbusMisc.send_buf,i);
            return len;
         }
         else if(addr>=0x3000 && addr<=0x3011)
         {
            modbusMisc.send_buf[i++]=modbusMisc.param.addr;
            modbusMisc.send_buf[i++]=Cmd;
            switch (addr)
            {
               case 0x3000:
                    modbusMisc.send_buf[i++]=2;//length
                    modbusMisc.send_buf[i++]=device_comps.flow_cal_param.unit;
                    modbusMisc.send_buf[i++]=device_comps.flow_cal_param.dot;
                    break;
                    
              case 0x3001:
                    modbusMisc.send_buf[i++]=2+MD_FLOW_MAX_CAL_POS*6;//length
                    tmp=device_comps.flow_cal_param.freq_divd_pos;
                    modbusMisc.send_buf[i++]=tmp>>8;
                    modbusMisc.send_buf[i++]=tmp;
                    for(k=0;k<MD_FLOW_MAX_CAL_POS;k++)
                    {
                        tmp=device_comps.flow_cal_param.freq_divd_value[k];
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                        daty=device_comps.flow_cal_param.freq_divd_value_meter_coe[k];
                        i+=get4bByteFloatCode(&modbusMisc.send_buf[i],daty);
                   }
                   break;
                    
              case 0x3002:
                    modbusMisc.send_buf[i++]=20;//length
                    for(k=0;k<5;k++)
                    {
                        daty=device_comps.flow_cal_param.freq_poly_coe[k];
                        i+=get4bByteFloatCode(&modbusMisc.send_buf[i],daty);
                   }
                   break;
                    
              case 0x3003:
                    modbusMisc.send_buf[i++]=4;//length
                    daty=device_comps.flow_cal_param.meter_coe;
                    i+=get4bByteFloatCode(&modbusMisc.send_buf[i],daty);
                    break;

              case 0x3004:
                    modbusMisc.send_buf[i++]=2;//length
                    tmp=device_comps.flow_cal_param.calc_mode;
                    modbusMisc.send_buf[i++]=tmp>>8;
                    modbusMisc.send_buf[i++]=tmp;
                    break; 
                    
                case 0x3005:
                    modbusMisc.send_buf[i++]=2;//length
                    tmp=device_comps.flow_cal_param.RefMaxQ;
                    modbusMisc.send_buf[i++]=tmp>>8;
                    modbusMisc.send_buf[i++]=tmp;
                    break;  
                    
               case 0x3006:
                    modbusMisc.send_buf[i++]=2;//length
                    tmp=device_comps.flow_cal_param.RefDN;
                    modbusMisc.send_buf[i++]=tmp>>8;
                    modbusMisc.send_buf[i++]=tmp;
                    break;
               case 0x3007:
                    modbusMisc.send_buf[i++]=2;//length
                    tmp=device_comps.flow_cal_param.freq_out_mode;
                    modbusMisc.send_buf[i++]=tmp>>8;
                    modbusMisc.send_buf[i++]=tmp;
                    break;     
               case 0x3010:
                    modbusMisc.send_buf[i++]=4;//length
                    tmp=device_comps.flow_total_cnts;
                    modbusMisc.send_buf[i++]=tmp>>24;
                    modbusMisc.send_buf[i++]=tmp>>16;
                    modbusMisc.send_buf[i++]=tmp>>8;
                    modbusMisc.send_buf[i++]=tmp;
                    if(num==3)
                    {
                        modbusMisc.send_buf[2]=6;//length
                        tmp=device_comps.flow_roll_freq_cur;
                        modbusMisc.send_buf[i++]=tmp>>8;
                        modbusMisc.send_buf[i++]=tmp;
                    }
                    break;
              case 0x3011:
                    modbusMisc.send_buf[i++]=2;//length
                    tmp=device_comps.flow_roll_freq_cur;
                    modbusMisc.send_buf[i++]=tmp>>8;
                    modbusMisc.send_buf[i++]=tmp;
                    break;     
                default:
                    return 1;
            }
            
            crc=generateCRC(modbusMisc.send_buf, i);
            modbusMisc.send_buf[i++]=crc;
            modbusMisc.send_buf[i++]=crc>>8;
            write_modbus(modbusMisc.send_buf,i);
            return len;
         }
         
         else 
         {
           return pro_modus_err(3,Cmd,len);
         }
    }
	else if(Cmd==6)//Write
	{
        
        if(addr==0x34)
        {
            modbusMisc.send_buf[i++]=buf[5];
        } 
        else
        {
            modbusMisc.send_buf[i++]=modbusMisc.param.addr;
        }
		modbusMisc.send_buf[i++]=Cmd;
		modbusMisc.send_buf[i++]=buf[2];
		modbusMisc.send_buf[i++]=buf[3];
		modbusMisc.send_buf[i++]=buf[4];
		modbusMisc.send_buf[i++]=buf[5];;
	    switch(addr)
	    {
//         #if( defined (MD_EXT_MEASUREMENT_MODULE) && MD_EXT_MEASUREMENT_MODULE_TYPE==MD_RAD)
//
//         #else
//           case 0:                //addr
//                 modbusMisc.param.addr=buf[5];
//                 save_modbus_param();
//     			break;
                
// 		  case 1:                //baud
// 		        tmp=buf[5];
// 		        if(tmp>0&&tmp<5)
// 		        {
//     				modbusMisc.param.baud=tmp;
//     				save_modbus_param();
//         			modbusComps.sw._bit.baud_modified=1;
//         		}
// 				break;
// //          #endif
// //		   case 5:                //level_param.coe
// //    	        tmp=((uint16_t)buf[4]<<8)+buf[5];
// //    	        if(tmp>=7000&&tmp<=13000)
// //    	        {
// //                	device_comps.level_param.coe=tmp;
// //                	device_comps.level_param.cs=Check_Sum_5A(&device_comps.level_param, & device_comps.level_param.cs-(uint8_t *)&device_comps.level_param);
// //	                device_comps.save_level_param(&device_comps.level_param,sizeof(device_comps.level_param));
// //        		}
// //              break;
// 		   case 8:                //press_coe
// 		        tmp=((uint16_t)buf[4]<<8)+buf[5];
// 		        if(tmp>=7000&&tmp<=13000)
// 		        {
//                 	device_comps.coe.press=tmp;
//                 	device_comps.coe.cs=Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs-(uint8_t *)&device_comps.coe);
//         			device_comps.save_coe(&device_comps.coe,sizeof(device_comps.coe));
//         		}
//                 break;
		  	case 0x2d: 
                datu=((uint16_t)buf[4]<<8)+buf[5];
                modbusComps.cmd_out_raw_4ma_20ma_timer=datu>60?60:datu;
                save_data_type=EM_NULL;
                break;
            case 0x2e: 
                datu=((uint16_t)buf[4]<<8)+buf[5];
                device_comps.coe._4ma_raw_value=datu;
                modbusComps.cmd_out_raw_4ma_20ma_timer=1;
                save_data_type=EM_COE;
                break;
            case 0x2f: 
                datu=((uint16_t)buf[4]<<8)+buf[5];
                device_comps.coe._20ma_raw_value=datu;
                modbusComps.cmd_out_raw_4ma_20ma_timer=1;
                save_data_type=EM_COE;
                break;
            case 0x30: 
                datu=((uint16_t)buf[4]<<8)+buf[5];
                if(datu==0)
                {
                    device_comps.meter_backup.total_int=device_comps.meter.total_int=0;
                    device_comps.meter_backup.total_intN=device_comps.meter.total_intN=0;
                    device_comps.meter_backup.total_dec=device_comps.meter.total_dec=0;
                    device_comps.meter_backup.total_decN=device_comps.meter.total_decN=0;
                    save_data_type=EM_METER;
                }
                break;
            case 0x31:
                  datu=((uint16_t)buf[4]<<8)+buf[5];
                 device_comps.flow_meter.unit=datu;
                 save_data_type=EM_FLOW_METER;
                break;
            case 0x32:
                  datu=((uint16_t)buf[4]<<8)+buf[5];
                 device_comps.misc_param.I_o_dir=datu;
                 save_data_type=EM_MISC;
                 break;
           case 0x33:
                  datu=((uint16_t)buf[4]<<8)+buf[5];
                 device_comps.flow_meter.pluse_width=datu;
                 save_data_type=EM_FLOW_METER;
                 break;
            case 0x34:
                  datu=((uint16_t)buf[4]<<8)+buf[5];
                 modbusComps.param_pt->addr=datu;
                 save_data_type=EM_MODBUS;
                 break;
            case 0x35:
                 datu=((uint16_t)buf[4]<<8)+buf[5];
                modbusComps.param_pt->baud=datu%4;
                modbusComps.modify_baud(modbusComps.param_pt->baud,0);
                save_data_type=EM_MODBUS;
                break;
            case 0x36:
                 datu=((uint16_t)buf[4]<<8)+buf[5];
                device_comps.coe.flow=datu;
                save_data_type=EM_COE;
                break;
                
            case 0x37:
                 datu=((uint16_t)buf[4]<<8)+buf[5];
                 device_comps.coe.out_4_20ma=datu;
                  save_data_type=EM_COE;
                break;
            case 0x38:
                 datu=((uint16_t)buf[4]<<8)+buf[5];
                device_comps.coe.pt_temp=datu;
                save_data_type=EM_COE;
                 break;
            case 0x39:
                 datu=((uint16_t)buf[4]<<8)+buf[5];
                device_comps.coe.press=datu;
                save_data_type=EM_COE;
                break;
           case 0x40:
                datu=((uint16_t)buf[4]<<8)+buf[5];
                device_comps.flow_meter.sensor_low_freq_cutoff=datu;
                save_data_type=EM_MISC;
                break; 
            case 0x41: 
                datu=((uint16_t)buf[4]<<8)+buf[5];
                device_comps.flow_meter.avg_freq_filter_timer=datu;
                save_data_type=EM_FLOW_METER;
                break;            
           default:
        	    return pro_modus_err(3,Cmd,len);
        }
        // if(save_data_type==EM_NULL)
        // {
        //     return pro_modus_err(3,Cmd,len);
        // }
        switch(save_data_type)
        {
            case EM_FLOW_METER:
                device_comps.flow_meter.cs=Check_Sum_5A(&device_comps.flow_meter, &device_comps.flow_meter.cs-(uint8_t *)&device_comps.flow_meter);
                device_comps.save_flow_meter(&device_comps.flow_meter,sizeof(device_comps.flow_meter));
                break;
            case EM_MODBUS:
                modbusComps.save_param();
                break;
            case EM_LORA:
                
                loraComps.save_cfg_info();
                loraComps.sw._bit.param_modified=1;
                break;
            case EM_ACCESS:
                device_comps.access_param.cs=Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs-(uint8_t *)&device_comps.access_param);
                device_comps.save_access_param(&device_comps.access_param,sizeof(device_comps.access_param));
                break;
            case EM_REPORT:
                device_comps.report_interval_timer=0;
                device_comps.report_param.cs=Check_Sum_5A(&device_comps.report_param, & device_comps.report_param.cs-(uint8_t *)&device_comps.report_param);
        	    device_comps.save_report_param(&device_comps.report_param,sizeof(device_comps.report_param));
                break;
            case EM_METER:
                device_comps.meter.cs=Check_Sum_5A(&device_comps.meter, &device_comps.meter.cs-(uint8_t *)&device_comps.meter);
                device_comps.save_meter(&device_comps.meter,sizeof(device_comps.meter));
                device_comps.meter_backup.cs=Check_Sum_5A(&device_comps.meter_backup, & device_comps.meter_backup.cs-(uint8_t *)&device_comps.meter_backup);
                device_comps.save_meter_backup(&device_comps.meter_backup,sizeof(device_comps.meter_backup));
                break;
            case EM_COE:
                device_comps.coe.cs=Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs-(uint8_t *)&device_comps.coe);
                device_comps.save_coe(&device_comps.coe,sizeof(device_comps.coe));
                break;
            case EM_GPS:
                device_comps.gps.cs=Check_Sum_5A(&device_comps.gps, &device_comps.gps.cs-(uint8_t *)&device_comps.gps);
							device_comps.save_gps_param(&device_comps.gps,sizeof(device_comps.gps));
                 break;
            case EM_MISC:
                device_comps.misc_param.cs=Check_Sum_5A(&device_comps.misc_param, &device_comps.misc_param.cs-(uint8_t *)&device_comps.misc_param);
                device_comps.save_misc_param(&device_comps.misc_param,sizeof(device_comps.misc_param));
                break;
           case EM_SYSTEM_TIME:
                device_comps.system_time.cs=Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs-(uint8_t *)&device_comps.system_time);
                device_comps.save_system_time(&device_comps.system_time,sizeof(device_comps.system_time));
                device_comps.set_system_time(&device_comps.system_time.time);
                ertc_comps.write_broken_time(&device_comps.system_time.time);    
			default:
				  break;
        }
		crc=generateCRC(modbusMisc.send_buf, i);
		modbusMisc.send_buf[i++]=crc;
		modbusMisc.send_buf[i++]=crc>>8;
		write_modbus(modbusMisc.send_buf,i);
		return len;
		
	}
	else if(Cmd==0x10)//many bytes wirte
	{
         
        switch(addr)
        {
            case 0x16:
                tmp=getFloatDataFrom4ByteCode(&buf[7])*pwr(device_comps.flow_cal_param.dot);
                device_comps.misc_param.I_o_low=tmp;
                save_data_type=EM_MISC;
                break;
            	
        	case 0x18:
                tmp=getFloatDataFrom4ByteCode(&buf[7])*pwr(device_comps.flow_cal_param.dot);
                device_comps.misc_param.I_o_high=tmp;
                save_data_type=EM_MISC;
                break;
                    
        	case 0x20:
                tmp=((uint32_t)buf[7]<<24)+((uint32_t)buf[8]<<16)+((uint32_t)buf[9]<<8)+buf[10];
                device_comps.misc_param.density=tmp;
                save_data_type=EM_MISC;
                break;
            case 0x22:
                tmp=((uint32_t)buf[7]<<24)+((uint32_t)buf[8]<<16)+((uint32_t)buf[9]<<8)+buf[10];
                device_comps.flow_meter.pluse_equ=tmp;
                save_data_type=EM_FLOW_METER;
                break;
            //case toto  
            
            case 0x2d: 
                datu=((uint16_t)buf[7]<<8)+buf[8];
                modbusComps.cmd_out_raw_4ma_20ma_timer=datu>60?60:datu;
                save_data_type=EM_NULL;
                break;
            case 0x2e: 
                datu=((uint16_t)buf[7]<<8)+buf[8];
                device_comps.coe._4ma_raw_value=datu;
                modbusComps.cmd_out_raw_4ma_20ma_timer=1;
                save_data_type=EM_COE;
                break;
            case 0x2f: 
                datu=((uint16_t)buf[7]<<8)+buf[8];
                device_comps.coe._20ma_raw_value=datu;
                modbusComps.cmd_out_raw_4ma_20ma_timer=1;
                save_data_type=EM_COE;
                break;
            case 0x30: 
                datu=((uint16_t)buf[7]<<8)+buf[8];
                if(datu==0)
                {
                    device_comps.meter_backup.total_int=device_comps.meter.total_int=0;
                    device_comps.meter_backup.total_intN=device_comps.meter.total_intN=0;
                    device_comps.meter_backup.total_dec=device_comps.meter.total_dec=0;
                    device_comps.meter_backup.total_decN=device_comps.meter.total_decN=0;
                    save_data_type=EM_METER;
                }
                break;    
            case 0x31:
                 datu=((uint16_t)buf[7]<<8)+buf[8];
                 device_comps.flow_meter.unit=datu;
                 save_data_type=EM_FLOW_METER;
                break;
            case 0x32:
                 datu=((uint16_t)buf[7]<<8)+buf[8];
                 device_comps.misc_param.I_o_dir=datu;
                 save_data_type=EM_MISC;
                 break;
           case 0x33:
                 datu=((uint16_t)buf[7]<<8)+buf[8];
                 device_comps.flow_meter.pluse_width=datu;
                 save_data_type=EM_FLOW_METER;
                 break;
            case 0x34:
                 datu=((uint16_t)buf[7]<<8)+buf[8];
                 modbusComps.param_pt->addr=datu;
                 save_data_type=EM_MODBUS;
                 break;
            case 0x35:
                datu=((uint16_t)buf[7]<<8)+buf[8];
                modbusComps.param_pt->baud=datu%4;
                modbusComps.modify_baud(modbusComps.param_pt->baud,0);
                save_data_type=EM_MODBUS;
                break;
            case 0x36:
                datu=((uint16_t)buf[7]<<8)+buf[8];
                device_comps.coe.flow=datu;
                save_data_type=EM_COE;
                break;
                
            case 0x37:
                datu=((uint16_t)buf[7]<<8)+buf[8];
                 device_comps.coe.out_4_20ma=datu;
                  save_data_type=EM_COE;
                break;
            case 0x38:
                datu=((uint16_t)buf[7]<<8)+buf[8];
                device_comps.coe.pt_temp=datu;
                save_data_type=EM_COE;
                 break;
            case 0x39:
                datu=((uint16_t)buf[7]<<8)+buf[8];
                device_comps.coe.press=datu;
                save_data_type=EM_COE;
                break;
            case 0x40:
                datu=((uint16_t)buf[7]<<8)+buf[8];
                device_comps.flow_meter.sensor_low_freq_cutoff=datu;
                save_data_type=EM_MISC;
                break; 
            case 0x41: 
                datu=((uint16_t)buf[7]<<8)+buf[8];
                device_comps.flow_meter.avg_freq_filter_timer=datu;
                save_data_type=EM_FLOW_METER;
                break;           
           default:
        	    return pro_modus_err(3,Cmd,len);
        }
        // if(save_data_type==EM_NULL)
        // {
        //     return pro_modus_err(3,Cmd,len);
        // }
        switch(save_data_type)
        {
            case EM_FLOW_METER:
                device_comps.flow_meter.cs=Check_Sum_5A(&device_comps.flow_meter, &device_comps.flow_meter.cs-(uint8_t *)&device_comps.flow_meter);
                device_comps.save_flow_meter(&device_comps.flow_meter,sizeof(device_comps.flow_meter));
                break;
            case EM_MODBUS:
                modbusComps.save_param();
                break;
            case EM_LORA:
                
                loraComps.save_cfg_info();
                loraComps.sw._bit.param_modified=1;
                break;
            case EM_ACCESS:
                device_comps.access_param.cs=Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs-(uint8_t *)&device_comps.access_param);
                device_comps.save_access_param(&device_comps.access_param,sizeof(device_comps.access_param));
                break;
            case EM_REPORT:
                device_comps.report_interval_timer=0;
                device_comps.report_param.cs=Check_Sum_5A(&device_comps.report_param, & device_comps.report_param.cs-(uint8_t *)&device_comps.report_param);
        	    device_comps.save_report_param(&device_comps.report_param,sizeof(device_comps.report_param));
                break;
            case EM_METER:
                device_comps.meter.cs=Check_Sum_5A(&device_comps.meter, &device_comps.meter.cs-(uint8_t *)&device_comps.meter);
                device_comps.save_meter(&device_comps.meter,sizeof(device_comps.meter));
                device_comps.meter_backup.cs=Check_Sum_5A(&device_comps.meter_backup, & device_comps.meter_backup.cs-(uint8_t *)&device_comps.meter_backup);
                device_comps.save_meter_backup(&device_comps.meter_backup,sizeof(device_comps.meter_backup));
                break;
            case EM_COE:
                device_comps.coe.cs=Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs-(uint8_t *)&device_comps.coe);
                device_comps.save_coe(&device_comps.coe,sizeof(device_comps.coe));
                break;
            case EM_GPS:
                device_comps.gps.cs=Check_Sum_5A(&device_comps.gps, &device_comps.gps.cs-(uint8_t *)&device_comps.gps);
							device_comps.save_gps_param(&device_comps.gps,sizeof(device_comps.gps));
                 break;
            case EM_MISC:
                device_comps.misc_param.cs=Check_Sum_5A(&device_comps.misc_param, &device_comps.misc_param.cs-(uint8_t *)&device_comps.misc_param);
                device_comps.save_misc_param(&device_comps.misc_param,sizeof(device_comps.misc_param));
                break;
           case EM_SYSTEM_TIME:
                device_comps.system_time.cs=Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs-(uint8_t *)&device_comps.system_time);
                device_comps.save_system_time(&device_comps.system_time,sizeof(device_comps.system_time));
                device_comps.set_system_time(&device_comps.system_time.time);
                ertc_comps.write_broken_time(&device_comps.system_time.time);    
			default:
				  break;
        }
        
        modbusMisc.send_buf[i++]=modbusMisc.param.addr;
        modbusMisc.send_buf[i++]=Cmd;
        modbusMisc.send_buf[i++]=buf[2];
        modbusMisc.send_buf[i++]=buf[3];
        modbusMisc.send_buf[i++]=buf[4];
        modbusMisc.send_buf[i++]=buf[5];
        crc=generateCRC(modbusMisc.send_buf, i);
		modbusMisc.send_buf[i++]=crc;
		modbusMisc.send_buf[i++]=crc>>8;
		write_modbus(modbusMisc.send_buf,i);
		return len;
	}
    else if(Cmd==0x11)//many bytes wirte CAL
	{
        int16_t isSave=1;
        switch(addr)
        {
           case 0x3000:
                tmp=((uint16_t)buf[7]<<8)+buf[8];
//                device_comps.flow_cal_param.unit=tmp>>8;
//                device_comps.flow_cal_param.dot=tmp;
//                device_comps.flow_cal_param.is_calibrated=1;
                
                break;
                
           case 0x3001:
                tmp=buf[8];
                cnt=device_comps.flow_cal_param.freq_divd_pos=tmp;
                for(k=0;k<MD_FLOW_MAX_CAL_POS;k++)
                {
                    if(cnt>0)
                    {
                         tmp=((int32_t)buf[6*k+9]<<8)+((int32_t)buf[6*k+10]);
                         device_comps.flow_cal_param.freq_divd_value[k]=tmp;
                         device_comps.flow_cal_param.freq_divd_value_meter_coe[k]=getFloatDataFrom4ByteCode(&buf[6*k+11]);
                         cnt--;
                    }
                    else
                    {
                        break;
                    }
                }
                break;
                
           case 0x3002:
                for(k=0;k<5;k++)
                {
                     device_comps.flow_cal_param.freq_poly_coe[k]=getFloatDataFrom4ByteCode(&buf[4*k+7]);
                }
               break;
                
           case 0x3003:
                device_comps.flow_cal_param.meter_coe=getFloatDataFrom4ByteCode(&buf[7]);
                break;
                
           case 0x3004:
                 device_comps.flow_cal_param.calc_mode=((uint16_t)buf[7]<<8)+buf[8];
                break;

           case 0x3005:
                device_comps.flow_cal_param.RefMaxQ=((uint16_t)buf[7]<<8)+buf[8];
                break; 
                
           case 0x3006:
                device_comps.flow_cal_param.RefDN=((uint16_t)buf[7]<<8)+buf[8];
                break;
           case 0x3007:
                device_comps.flow_cal_param.freq_out_mode=((uint16_t)buf[7]<<8)+buf[8];
                break;
           default:
                isSave=0;
                return 1;

        }
        if(isSave)
        {
            device_comps.flow_cal_param.cs =Check_Sum_5A(&device_comps.flow_cal_param, &device_comps.flow_cal_param.cs-(uint8_t *)&device_comps.flow_cal_param);
            device_comps.save_flow_cal_param(&device_comps.flow_cal_param,sizeof(device_comps.flow_cal_param));
            hum_comps.enter_default_mode(0);
           // device_comps.clr_press(0);
            isSave=0;
        }
        modbusMisc.send_buf[i++]=modbusMisc.param.addr;
        modbusMisc.send_buf[i++]=Cmd;
        modbusMisc.send_buf[i++]=buf[2];
        modbusMisc.send_buf[i++]=buf[3];
        modbusMisc.send_buf[i++]=buf[4];
        modbusMisc.send_buf[i++]=buf[5];
        crc=generateCRC(modbusMisc.send_buf, i);
		modbusMisc.send_buf[i++]=crc;
		modbusMisc.send_buf[i++]=crc>>8;
		write_modbus(modbusMisc.send_buf,i);
		return len;
	}
    else 
	{
        return pro_modus_err(1,Cmd,len);
    }
	return 1;
	
}

static uint8_t Check_modbus_Com(uint8_t *Rec_Data,uint8_t Rec_Pos)
{
    int16_t len;
//	if(modbusComps.sw._bit.busy)
//	{
//	    return 0;
//	}
	if(Rec_Pos<2)
	{
		return 0;
	}
	if(Rec_Data[0]!=modbusMisc.param.addr&&Rec_Data[0]!=0)
	{
		return 1;
	}
	if(Rec_Data[1]!=0x03&&Rec_Data[1]!=0x06 && Rec_Data[1]!=0x10 && Rec_Data[1]!=0x11)
	{
		return 1;
	}
	if(Rec_Pos<8)
	{
		return 0;
	}
	if(Rec_Data[1]==0x10)
	{
        if(Rec_Data[6]!=8 &&Rec_Data[6]!=4 && Rec_Data[6]!=2)
        {
            return 1;
        }
        if(Rec_Data[5]*2!=Rec_Data[6])
        {
            return 1;
        }
        if(Rec_Pos<Rec_Data[6]+9) 
        {
            return 0;
        }
        len=Rec_Data[6]+9;
	}
    else if(Rec_Data[1]==0x11)
    {
       
        if(Rec_Data[6]>2+MD_FLOW_MAX_CAL_POS*6)
        {
            return 1;
        }
        if(Rec_Data[5]*2!=Rec_Data[6])
        {
            return 1;
        }
        if(Rec_Pos<Rec_Data[6]+9) 
        {
            return 0;
        }
        len=Rec_Data[6]+9;
    
    }
	else 
	{
	  len=8;
	}
	if(((uint16_t)Rec_Data[len-1]<<8)+Rec_Data[len-2]!=generateCRC(Rec_Data,len-2))
	{
		return 1;
	}
	return Pro_modbus(Rec_Data[1],Rec_Data,len);
}


static uint8_t pro_modbus_config(uint8_t cmd,uint8_t *buf,int16_t len)
{
    
//    yl701_info_t yl701_info_cpy;
//    switch(cmd)
//    {
//        case 0x01://write all param
//            yl701_info_cpy.rate=buf[8]=4;
//            yl701_info_cpy.verify=buf[9]=0;
//            yl701_info_cpy.freq=((uint32_t)buf[10]<<16)+((uint32_t)buf[11]<<8)+buf[12];
//            yl701_info_cpy.sf=buf[13];
//            yl701_info_cpy.workMode=buf[14];
//            yl701_info_cpy.bw=buf[15];
//            yl701_info_cpy.NodeId=((uint32_t)buf[16]<<8)+buf[17];
//            yl701_info_cpy.netId=buf[18];
//            yl701_info_cpy.power=buf[19];
//            yl701_info_cpy.breathPeriod=buf[20];
//            yl701_info_cpy.breathTime=buf[21];
//            yl701_info_cpy.cs=Check_Sum_5A(&yl701_info_cpy, &yl701_info_cpy.cs-(uint8_t *)&yl701_info_cpy);
//            if(loraComps.save_yl701_info(&yl701_info_cpy,sizeof(yl701_info_cpy)))
//            {
                    
//            }
//            else
//            {
//                memcpy(loraComps.yl701_info_p,&yl701_info_cpy,sizeof(yl701_info_cpy));
            	
//            }
            
//        case 0x02:
//            memcpy(loraComps.send_base_pt,buf,len);
//            MD_LORA_INTP_DISABLE();
//            loraComps.work_st.mode=LORA_EM_CFG_MODE;
//            loraComps.op_window_time=10;
//            loraComps.write(loraComps.send_base_pt,len);
//            return len;

//        default:
//              break;

//     }

     return 1;
 }

 

static uint8_t Check_modbus_cfg_Com(uint8_t *Rec_Data,uint8_t Rec_Pos)
{
    if(Rec_Pos<5)
	{
		return 0;
	}
    if((Rec_Data[0]==0xaf)&&(Rec_Data[1]==0xaf)&&(Rec_Data[2]==0)&&
	(Rec_Data[3]==0)&&(Rec_Data[4]==0xaf))
	{
        if(Rec_Pos<8)
        {
            return 0;
        }
        if(Rec_Data[7]>0x0f)
        {
            return 1;
        }
        if(Rec_Pos<8+Rec_Data[7]+1+2)
        {
            return 0;
        }
        if((Rec_Data[8+Rec_Data[7]+1+2-2]!=0x0d)||(Rec_Data[8+Rec_Data[7]+1+2-1]!=0x0a))
        {
            return 1;
        }
        if(Rec_Data[8+Rec_Data[7]+1+2-3]!=Check_Sum(Rec_Data,8+Rec_Data[7]))
        {
            return 1;
        }
        return pro_modbus_config(Rec_Data[6],Rec_Data,8+Rec_Data[7]+1+2);
	}
	return 1;
}



static void Deal_modbus(void)
{
	uint8_t err=0;
	do
	{
		err=Check_modbus_Com(modbusMisc.recv_buf,modbusMisc.recv_pos); 
		if(err>0)
		{	
			__disable_irq();
			memcpy(modbusMisc.recv_buf,modbusMisc.recv_buf+err,modbusMisc.recv_pos-=err);
			__enable_irq();
    }
		
	}
    while (err>0);

    do
	{
		
		err=Check_modbus_cfg_Com(modbusMisc.recv_cfg_buf,modbusMisc.recv_cfg_pos); 
		
		if(err>0)
		{
			__disable_irq();
			memcpy(modbusMisc.recv_cfg_buf,modbusMisc.recv_cfg_buf+err,modbusMisc.recv_cfg_pos-=err);
			__enable_irq();
    }
		
	}
    while (err>0);
	
}


static void modbus_start(void)
{
   
    modbusComps.op_window_time=30;
    modbusMisc.recv_pos = 0;
   
    modbusMisc.recv_cfg_pos=0;
    config_modbus_com(modbusMisc.param.baud,0);
    enable_modbus_com();
     modbusComps.op_window_time=MD_MODBUS_OP_WINDOW_TIME_OUT;
    modbusComps.sw._bit.runing=1;
    MD_RESET_RS_485_T_R;
    MD_IR_VCM_ON;
}

static void modbus_stop(void)
{
    MD_IR_VCM_OFF;
    modbusComps.op_window_time=0;
    disable_modbus_com();
    Deconfig_modbus_com();
    modbusComps.sw._bit.runing=0;
}

static void modbus_sendend_callback(void)
{
    __NOP();__NOP();__NOP();__NOP();
    MD_RESET_RS_485_T_R;
}

static void  modbusComps_task_handle(void)
{
    static int16_t init_once=1;
    if(init_once)
    {
        read_modbus_param(&modbusMisc.param,sizeof(modbusMisc.param));
        if(modbusMisc.param.cs!=Check_Sum_5A(&modbusMisc.param, &modbusMisc.param.cs-(uint8_t *)&modbusMisc.param))
        {
            modbusMisc.param.baud=3;
            modbusMisc.param.addr=1;
        }
        if(modbusMisc.param.baud>3)
        {
            modbusMisc.param.baud=3;
        }
        init_once=0;
    }
  #if(1)
     if(device_comps.sw._bit.isExtPowerConnected || device_comps.sw._bit.com_key_en)
     {
            if(!modbusComps.sw._bit.runing)
            {
              if(!ircComps.sw._bit.runing)
              {
                 modbus_start();
              }
            }
     }
     else
     {
            if(!ircComps.sw._bit.runing)
            {
                if(modbusComps.sw._bit.runing)
                {
                   modbus_stop();
                }
            }
     }
  #else
    if(!modbusComps.sw._bit.runing)
    {
        if(!ircComps.sw._bit.runing)
        {
            modbus_start();
        }
    }
  #endif
   
   if(modbusComps.sw._bit.runing)
   {
        Deal_modbus();
   }
}

 void store_modbus_buffer(uint8_t data)
{
    if(modbusMisc.recv_pos<MD_MODBUS_MAX_RECV_BUF_POS)
    {
        modbusMisc.recv_buf[modbusMisc.recv_pos++]=data;
    }
        // modbusMisc.recv_buf[modbusMisc.recv_pos]=data;
        // modbusMisc.recv_pos+=1;
        // modbusMisc.recv_pos%=MD_MODBUS_MAX_RECV_BUF_POS;

        // modbusMisc.recv_cfg_buf[modbusMisc.recv_cfg_pos]=data;
        // modbusMisc.recv_cfg_pos+=1;
        // modbusMisc.recv_cfg_pos%=MD_MODBUS_MAX_CFG_RECV_BUF_POS;
}


modbusComps_t modbusComps=
{
     0,
     0,
    &modbusMisc.param,
    {0},


    store_modbus_buffer,
    save_modbus_param,
    modify_modbus_baud_verify,
    modbus_sendend_callback,
    modbusComps_task_handle
};




