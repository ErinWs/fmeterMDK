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
    
#define  MD_COLLECTOR_VCM_ON      Gpio_WriteOutputIO(MD_COLLECTOR_VCM_PORT, MD_COLLECTOR_VCM_PIN, TRUE)
#define  MD_COLLECTOR_VCM_OFF     Gpio_WriteOutputIO(MD_COLLECTOR_VCM_PORT, MD_COLLECTOR_VCM_PIN, FALSE)



struct 
{ 
	uint8_t send_buf[16];
	uint8_t recv_buf[32];
	uint8_t recv_pos;
	uint8_t required_ack_cmd;
	uint8_t required_ack_addr;
	uint8_t required_ack_data_bytes;
	int16_t sample_interval_timer;

	int16_t CmdWaiting;
	int16_t NoAckTimes;
	int16_t _state;
	int16_t state;
	int32_t AckTmr;
}
collectorMisc=
{0};

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

static int16_t get4bByteFloatCode(uint8_t *buf,float32_t x)
{
    int16_t i=0;
    buf[i++] =*((uint8_t *)&x+3);
    buf[i++] =*((uint8_t *)&x+2);
    buf[i++] =*((uint8_t *)&x+1);
    buf[i++] =* (uint8_t *)&x;
    return 4;
}

static float32_t getFloatDataFrom4ByteCode(uint8_t *buf)
{
    int32_t  dat=((uint32_t)buf[0]<<24)+((uint32_t)buf[1]<<16)+((uint32_t)buf[2]<<8)+buf[3];
    return *(float32_t *)&dat;
}


static void enable_collector_com(void)
{
    enable_uart2();
}

static void disable_collector_com(void)
{
    disable_uart2();
}

static int16_t config_collector_com(int16_t baud,int16_t parity)
{
     App_Uart2Cfg(baud, parity);
    return baud;
}

static void start_collector(void)
{
    
        MD_COLLECTOR_VCM_ON;
        config_collector_com(9600,0);//9600
        collectorMisc.state=0;
        collectorMisc.CmdWaiting=0;
        collectorMisc.NoAckTimes=0;
        collectorComps.sw._bit.running=1; 
   
}

static void stop_collector(void)
{
    MD_COLLECTOR_VCM_OFF;
	disable_collector_com();
    collectorMisc.state=0;
    collectorMisc.CmdWaiting=0;
    collectorMisc.NoAckTimes=0;
    collectorComps.sw._bit.running=0;  
    
}




static void write_collector(uint16_t cmd,uint8_t addr,uint16_t reg_addr,int16_t nums,int16_t *data,int16_t AckTmr)
{
    int16_t i=0;
    uint16_t crc;
    enable_collector_com();
    collectorMisc.send_buf[i++]=collectorMisc.required_ack_addr=addr;
    collectorMisc.send_buf[i++]= collectorMisc.required_ack_cmd=cmd;
    collectorMisc.send_buf[i++]=reg_addr>>8;
    collectorMisc.send_buf[i++]=reg_addr;
   
    if(cmd==3 || cmd==4)
    {
        collectorMisc.send_buf[i++]=nums>>8;
        collectorMisc.send_buf[i++]=nums;
        collectorMisc.required_ack_data_bytes=nums*2+1;
    }
    else if(cmd==6 || cmd==0x10)
    {
        //if(nums==1)
        {
            collectorMisc.send_buf[i++] = *data>>8;
            collectorMisc.send_buf[i++] = *data;
        }
        collectorMisc.required_ack_data_bytes=4;
    }
    crc=generateCRC(collectorMisc.send_buf,i);
    collectorMisc.send_buf[i++]=crc;
    collectorMisc.send_buf[i++]=crc>>8;
    collectorMisc.recv_pos=0;
    
    R_UART2_Send(collectorMisc.send_buf,i);
    collectorMisc.CmdWaiting=1;
    collectorMisc.AckTmr=AckTmr;//400ms
}




     
static void collect_ok_callback(int16_t state)
{   
//    if(state==0)
//    {
//        device_comps.DM.sw._bit.isExternPowerOnYet=((collectorMisc.recv_buf[3]<<8)+collectorMisc.recv_buf[4])==0xa1;
//        device_comps.DM.ExternPowerStatus=(collectorMisc.recv_buf[3]<<8)+collectorMisc.recv_buf[4];
//        device_comps.DM.current=          (collectorMisc.recv_buf[5]<<8)+collectorMisc.recv_buf[6];
//        device_comps.DM.voltage=          (collectorMisc.recv_buf[7]<<8)+collectorMisc.recv_buf[8];
//        device_comps.DM.sw._bit.isGetDataReq=0;
//    }
//    if(state==1)
//    {   
//        device_comps.DM.sw._bit.isExternPowerOnReq=0;
//    }
//    if(state==2)
//    {   
//        device_comps.DM.sw._bit.isExternPowerOffReq=0;
//    }
//    if(state==3)
//    {
//        collectorComps.pvf=  getFloatDataFrom4ByteCode(&collectorMisc.recv_buf[3]);
//        collectorComps.pvf=  getFloatDataFrom4ByteCode(&collectorMisc.recv_buf[7]);
//        collectorComps.sw._bit.isGetExternPressReq=0;
//    }
    
}

static void collect_time_out_callback(int16_t state)
{
//    collectorMisc.NoAckTimes++;
//    if(collectorMisc.NoAckTimes<1)
//    {
//        collectorMisc.CmdWaiting =0;
//    }
//    else 
//    {
//        if(state==0)
//        {   
//            device_comps.DM.sw._bit.isGetDataReq=0;
//        }
//        if(state==1)
//        {   
//            device_comps.DM.sw._bit.isExternPowerOnReq=0;
//        }
//        if(state==2)
//        {   
//            device_comps.DM.sw._bit.isExternPowerOffReq=0;
//        }
//        if(state==3)
//        {
//            collectorComps.sw._bit.isGetExternPressReq=0;
//        }
//       
//    }
}

static uint8_t Check_collector_Com(uint8_t *Rec_Data,uint8_t Rec_Pos)//static uint8_t Check_collector_Com(uint8_t *Rec_Data,uint8_t Rec_Pos)
{
	
    int16_t len=0;
	if(Rec_Pos<collectorMisc.required_ack_data_bytes+4)
	{
		return 0;
	}
    len=collectorMisc.required_ack_data_bytes+4;
	//if(collectorMisc.required_ack_addr!=0 && collectorMisc.required_ack_addr!=Rec_Data[0])
	if(collectorMisc.required_ack_addr!=Rec_Data[0])
	{
		return 1;
	}
	if(Rec_Data[1]!=collectorMisc.required_ack_cmd)
	{
		return 1;
	}
    if(((uint16_t)Rec_Data[len-1]<<8)+Rec_Data[len-2]!=generateCRC(Rec_Data,len-2))
	{
		return 1;
	}
	return len;
   
}


int16_t read_collect_nvm_param(void *buf,int16_t len )
{
   // return _24cxx_comps.read(MD_COLLECT_NVM_PARAM_START_ADDR,buf,len);
	    return 0;
}

int16_t save_collect_nvm_param(void const *buf,int16_t len )
{
     //return _24cxx_comps.write(MD_COLLECT_NVM_PARAM_START_ADDR,buf,len);
	    return 0;
}



static void collectorComps_task_handle(void)
{
  
   static int16_t init_once=1;
    if(init_once)
    {  
//        if(!read_collect_nvm_param(&collectorComps.nvm_param,sizeof(collectorComps.nvm_param)))
//        {
//           if(collectorComps.nvm_param.cs!=Check_Sum_5A(&collectorComps.nvm_param, &collectorComps.nvm_param.cs-(uint8_t *)&collectorComps.nvm_param))
//           {
//                collectorComps.nvm_param.refresh_freq=700;//*1s user set (4)
//                collectorComps.nvm_param.pv_coe=1000;//user set (5)
//                collectorComps.nvm_param.pv_dot=0;//user set 0-4 (6)
//                collectorComps.nvm_param.aux1_h3=250;//mm
//
//                collectorComps.nvm_param.slave_addr=1;
//                collectorComps.nvm_param.reg_addr=1;
//                collectorComps.nvm_param.cmd=3;
//                collectorComps.nvm_param.sensor_power_supply_time=400; //*50ms
//                collectorComps.nvm_param.data_type=0;//0-4 0,1 16bit_sign 16bit_unsign  2,3,4 32bit
//                collectorComps.nvm_param.data_endian=1;
//       
//           }
//        }
//        collectorComps.nvm_param.ip = &device_comps.access_param.ip;//Two screens (0 1)
//        collectorComps.nvm_param.port = &device_comps.access_param.port;//(2)
//        collectorComps.nvm_param.report_minute_interval= &device_comps.report_param.u16Minute_Interval;//(2)
//       // stop_collector();
        init_once=0;
    }
    
    
//   if(device_comps.DM.sw._bit.isPowerOnYet )
//   {
//       if(device_comps.DM.sw._bit.isGetDataReq|| device_comps.DM.sw._bit.isExternPowerOnReq 
//         || device_comps.DM.sw._bit.isExternPowerOffReq)
//       { 
//            if(!collectorComps.sw._bit.running)
//            {
//                start_collector();
//                collectorMisc.AckTmr=10000+100;
//            }

//       }
//   }
// 
//   if(collectorComps.sw._bit.isGetExternPressReq)
//   {
//      if(!collectorComps.sw._bit.running)
//      {
//          start_collector();
//          collectorMisc.AckTmr=10000+100;
//      }

//   }
   
   if(collectorComps.sw._bit.running)
   {
        uint8_t err;
        if(collectorMisc.AckTmr>10000)
        {
            return;
        }
	    
        if(collectorMisc.CmdWaiting)
        {
         	do
        	{
        	    err = Check_collector_Com(collectorMisc.recv_buf,collectorMisc.recv_pos); 
                __disable_irq();
                memcpy(collectorMisc.recv_buf,collectorMisc.recv_buf+err,collectorMisc.recv_pos-=err);
                __enable_irq();
    	    }
    	    while (err==1);
        }
        
        switch(collectorMisc.state)
        {
//            case 0:
//                if(device_comps.DM.sw._bit.isGetDataReq)
//                {
//                    if(!collectorMisc.CmdWaiting)
//                    {
//                        write_collector(3,1,2,3,(int16_t *)0,4);//4*50ms
//                    }
//                    else if(!collectorMisc.AckTmr)
//                    {
//                        collect_time_out_callback(collectorMisc.state);
//                    }
//                    else if(err>1)//receive ok
//                    {
//                         collect_ok_callback(collectorMisc.state);
//                    }
//                 }
//                 else
//                 {
//                    collectorMisc.NoAckTimes=collectorMisc.CmdWaiting=0;
//                    collectorMisc.state++;
//                 }
//                 break;
//                 
//             case 1:
//                if(device_comps.DM.sw._bit.isExternPowerOnReq)
//                {
//                    if(!collectorMisc.CmdWaiting)
//                    {
//                       int16_t data=0xa1;
//                       write_collector(6,1,2,1,&data,4);
//                    }
//                    else if(!collectorMisc.AckTmr)
//                    {
//                        collect_time_out_callback(collectorMisc.state);
//                    }
//                    else if(err>1)//receive ok
//                    {
//                         collect_ok_callback(collectorMisc.state);
//                    }
//                 }
//                 else
//                 {
//                    collectorMisc.NoAckTimes=collectorMisc.CmdWaiting=0;
//                    collectorMisc.state++;
//                 }
//                 break;

//               case 2:
//                if(device_comps.DM.sw._bit.isExternPowerOffReq)
//                {
//                    if(!collectorMisc.CmdWaiting)
//                    {
//                       int16_t data=0xa0;
//                       write_collector(6,1,2,1,&data,4);
 //                  }
//                    else if(!collectorMisc.AckTmr)
//                    {
//                        collect_time_out_callback(collectorMisc.state);
//                    }
//                    else if(err>1)//receive ok
//                    {
//                         collect_ok_callback(collectorMisc.state);
//                    }
//                 }
//                 else
//                 {
//                    collectorMisc.NoAckTimes=collectorMisc.CmdWaiting=0;
//                    collectorMisc.state++;
//                 }
//                 break; 
//                 
//              case 3:
//                if(collectorComps.sw._bit.isGetExternPressReq)
//                {
//                    if(!collectorMisc.CmdWaiting)
//                    {
//                       write_collector(3,0,100,4,(int16_t *)0,4);//
//                    }
//                    else if(!collectorMisc.AckTmr)
//                    {
//                        collect_time_out_callback(collectorMisc.state);
//                    }
//                    else if(err>1)//receive ok
//                    {
//                         collect_ok_callback(collectorMisc.state);
//                    }
//                 }
//                 else
//                 {
//                    collectorMisc.NoAckTimes=collectorMisc.CmdWaiting=0;
//                    collectorMisc.state++;
//                 }
//                 break;
            
            default:
                 stop_collector();
                break;

        }
   }

}


static void store_collector_buffer(uint8_t data)
{
        collectorMisc.recv_buf[collectorMisc.recv_pos]=data;
        collectorMisc.recv_pos+=1;
        collectorMisc.recv_pos&=0x1f;
}

static void collector_comps_task_50ms(void)
{
    if(collectorMisc.AckTmr>0)
    {
        collectorMisc.AckTmr--;
    }
}

collectorComps_t collectorComps=
{
   "",   
  {0},   //sw
  {0},   //nvm_param
         
   0,    //    int32_t pv;
   0,    //    float32_t pvf;
          
  save_collect_nvm_param,   //    int16_t (*save_nvm_param)(void const *,int16_t);
  collectorComps_task_handle,    //    void (*const task_handle)(void);
  collector_comps_task_50ms                       
};


