#include "ddl.h"
#include "rtc.h"
#include "gpio.h"

#include "r_cg_sau.h"
#include "device.h"
#include "modbus.h"
#include "hum.h"
#include "elora.h"
#include "ertc.h"
#include "net.h"
#include "irc.h"

#include "string.h"
#include "stdio.h"
    

static struct 
{ 
	 uint8_t  send_buf[128];
	 uint8_t recv_buf[128];
	 uint8_t recv_pos;
  
}
ircMisc=
{
	{0},
	{0},
	 0
    
};





//////////////////////////DealNfc

//uint8_t Leftloop(uint8_t len)
//{
//	if(len&0x80)
//	{
//		len = len * 2 + 1;
//	}
//	else
//	{
//		len = len * 2;
//	}
//	return len;
//}
//void Do_Imm(uint8_t *data)
//{
//	uint8_t i,j,k;
//	uint8_t n1;
//	uint8_t code[8]={'C','E','L','E','G','A','L','F'};
//	uint8_t cdata[8];
//	k = 0x5a;
//	for(i=0;i<8;i++)
//	{
//		cdata[i] = ~(*data);
//		*data++; 
//	}
//	for(i=0;i<8;i++)
//	{
//		for(j=0;j<8;j++)
//		{
//			n1 = cdata[j];
//			n1 = n1 - code[j];
//			k = k^n1;
//			k = Leftloop(k);
//		}
//		cdata[i]=k;
//		k ++;
//	}
//	for(i=0;i<8;i++)
//	{
//		*data--;
//		*data = ~cdata[7-i];
//	}
//}

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

static int32_t formatData4fixDot(int32_t temp,int16_t dot)
{
    if(dot==6)
    {
        temp/=100;
    }
    else if(dot==5)
    {
        temp/=10;
    }
    else if(dot==4)
    {
        temp/=1;
    }
    else if(dot==3)
    {
        temp*=10;
    }
    else if(dot==2)
    {
        temp*=100;
    }
    else if(dot==1)
    {
        temp*=1000;
    }
    else if(dot==0)
    {
        temp*=10000;
    }
    return temp;
}


int32_t format4fixDataToCalDot(int32_t temp,int16_t dot)
{
    if(dot==6)
    {
        temp*=100;
    }
    else if(dot==5)
    {
        temp*=10;
    }
    else if(dot==4)
    {
        temp/=1;
    }
    else if(dot==3)
    {
        temp/=10;
    }
    else if(dot==2)
    {
        temp/=100;
    }
    else if(dot==1)
    {
        temp/=1000;
    }
    else if(dot==0)
    {
        temp/=10000;
    }
    return temp;
}



#define  VERIFYEN                                   0
#define  MD_DATA_ID_READ_BASIC_INFO                 0x9020
#define  MD_DATA_ID_READ_ALARM_PARAM                0x9021
#define  MD_DATA_ID_READ_RPORT_PARAM                0x9022
#define  MD_DATA_ID_READ_MEASURE_INFO               0x901f
#define  MD_DATA_ID_READ_ACCESS_ADDR                0x9023
#define  MD_DATA_ID_READ_SYSTEM_TIME                 0x9024
#define  MD_DATA_ID_READ_HIGH_INFO                  0x9026
#define  MD_DATA_ID_READ_LORA_AMT_PARAM             0x9027

#define  MD_DATA_ID_READ_GPS_LOC_INFO               0x9028
#define  MD_DATA_ID_READ_MANUFACTURER_INFO          0x9029
#define  MD_DATA_ID_READ_DEVICE_TYPE_INFO           0x902a
#define  MD_DATA_ID_READ_DEVICE_ID_INFO             0x902b
#define  MD_DATA_ID_READ_SENSOR_ID_INFO             0x902c
#define  MD_DATA_ID_READ_CHANNEL_N_ADC              0x902d
#define  MD_DATA_ID_READ_LBS_TOKEN                  0x902e
#define  MD_DATA_ID_READ_LBS_DOMAIN                 0x902f





#define  MD_DATA_ID_SET_DEVICE_ADDR             0x9018
#define  MD_DATA_ID_ENTER_CAL_MODE              0xfa55//
#define  MD_DATA_ID_ENTER_NORMAL_MODE           0xfa99//
#define  MD_DATA_ID_WRITE_ALARM_PARAM           0x9001//
#define  MD_DATA_ID_WRITE_CAL_DATA              0x9002//
#define  MD_DATA_ID_WRITE_REPORT_PARAM          0x9003//
#define  MD_DATA_ID_WRITE_SYSTEM_TIME            0x9004
#define  MD_DATA_ID_SET_ACCESS_ADDR             0x9005
#define  MD_DATA_ID_WRITE_LORA_AMT_PARAM        0x9006

#define  MD_DATA_ID_WRITE_GPS_LOC_INFO          0x9007
#define  MD_DATA_ID_WRITE_MANUFACTURER_INFO     0x9008
#define  MD_DATA_ID_WRITE_DEVICE_TYPE_INFO      0x9009
#define  MD_DATA_ID_WRITE_DEVICE_ID_INFO        0x900a
#define  MD_DATA_ID_WRITE_SENSOR_INFO           0x900b
#define  MD_DATA_ID_WRITE_LBS_TOKEN             0x900c
#define  MD_DATA_ID_WRITE_LBS_DOMAIN            0x900d
#define  MD_DATA_ID_WRITE_IOT_PRODUCT_ID        0x900e
#define  MD_DATA_ID_WRITE_IOT_TENANT_ID         0x900f
#define  MD_DATA_ID_WRITE_IOT_TOKEN             0x9010
#define  MD_DATA_ID_WRITE_IOT_DEVICE_ID         0x9011






static void disable_irc_com(void)
{
    disable_LPuart0();
    
//    R_TAU0_Channel0_Stop();
//   TO0 &= ~_0010_TAU_CH4_OUTPUT_VALUE_1;

}

static void enable_irc_com(void)
{
    enable_LPuart0();
}

uint32_t config_irc_com(uint32_t baud,int16_t parity)
{
        uint32_t baud_num[]={1200,2400,4800,9600,19200};
        if(baud<0 || baud>4)
        {
            return 1;
        } 
        App_LpUart0Cfg(baud_num[baud],parity);
        return 0;
   
	
}

static void write_irc(uint8_t * const buf,uint16_t len)
{
    if(len>0)
    {
        MD_SET_RS_485_T_R;
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        R_LPUART0_Send(buf,len);
    }
}
static uint8_t Pro_irc(uint8_t Cmd,uint8_t *buf)
{
	uint32_t temp=0;

	uint8_t i=9;

	uint8_t VerifyResult;
	
	uint16_t DataId=((uint16_t)buf[11]<<8)+buf[12];
	if(Cmd==0)//Read
	{
		memcpy(ircMisc.send_buf,buf,9);
		memcpy(ircMisc.send_buf+1,device_comps.device_addr.addr,7);
		switch(DataId)
		{
			case MD_DATA_ID_READ_BASIC_INFO:                //BasicInf
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],netComps.net_info.imei,16);//ADD IMEI
					i+=16;
					memcpy(&ircMisc.send_buf[i],netComps.net_info.imsi,16);//ADD IMSI
					i+=16;
					ircMisc.send_buf[i++]=MD_FL_VER;//device hwVer
					ircMisc.send_buf[i++]=MD_FL_VER;//device swVer
					ircMisc.send_buf[i++]=MD_FL_VER;//protocol swVer
				
					memcpy(&ircMisc.send_buf[i],netComps.net_info.firmVer,20);//ADD FireWave ver
					i+=20;
					memcpy(&ircMisc.send_buf[i],netComps.net_info.iccid,20);//ADD iccid
					i+=20;
					memcpy(&ircMisc.send_buf[i],device_comps.access_param.ip,4);//New IP
					i+=4;
					ircMisc.send_buf[i++]=device_comps.access_param.port>>8;
					ircMisc.send_buf[i++]=device_comps.access_param.port;
					
					///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
                    VerifyResult++;
					break;
			case MD_DATA_ID_READ_ALARM_PARAM:                //alram param
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_upper>>24;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_upper>>16;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_upper>>8;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_upper;
					
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_lower>>24;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_lower>>16;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_lower>>8;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_high_lower;

					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_upper>>24;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_upper>>16;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_upper>>8;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_upper;

					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_lower>>24;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_lower>>16;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_lower>>8;
					ircMisc.send_buf[i++]=device_comps.alarm_param.press_low_lower;
					
					ircMisc.send_buf[i++]=device_comps.alarm_param.unit;

					ircMisc.send_buf[i++]=device_comps.alarm_param.temp_high>>8;
					ircMisc.send_buf[i++]=device_comps.alarm_param.temp_high;
					ircMisc.send_buf[i++]=device_comps.alarm_param.temp_low>>8;
					ircMisc.send_buf[i++]=device_comps.alarm_param.temp_low;
					//memcpy(&ircMisc.send_buf[i],&device_comps.alarm_param.press_high_upper,
					//i+=&device_comps.alarm_param.temp_low-&device_comps.alarm_param.press_high_upper+sizeof(device_comps.alarm_param.temp_low));//New IP
				
					///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;

			case MD_DATA_ID_READ_RPORT_PARAM:                //report param
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					
					ircMisc.send_buf[i++]=device_comps.report_param.u8Minute;
					ircMisc.send_buf[i++]=device_comps.report_param.u8Hour;
					ircMisc.send_buf[i++]=device_comps.report_param.u16Minute_Interval>>8;
					ircMisc.send_buf[i++]=device_comps.report_param.u16Minute_Interval;
					//ircMisc.send_buf[i++]=device_comps.report_param.u8Hour_Interval;
				    ircMisc.send_buf[i++]=device_comps.report_param.disFactor>>8;
				    ircMisc.send_buf[i++]=device_comps.report_param.disFactor;

                    temp=device_comps.report_param.triggerTimes;                                       
					ircMisc.send_buf[i++]=temp>>24;
					ircMisc.send_buf[i++]=temp>>16;
					ircMisc.send_buf[i++]=temp>>8;
					ircMisc.send_buf[i++]=temp;
					

				    
					///ADD Other Data
					ircMisc.send_buf[10]=i-11;
				
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;	
		     case MD_DATA_ID_READ_SYSTEM_TIME:               
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
                    ircMisc.send_buf[i++]=device_comps.system_time.time.u8Year ;
                    ircMisc.send_buf[i++]=device_comps.system_time.time.u8Month;
                    ircMisc.send_buf[i++]=device_comps.system_time.time.u8Day ; 
                    ircMisc.send_buf[i++]=device_comps.system_time.time.u8Hour; 
                    ircMisc.send_buf[i++]=device_comps.system_time.time.u8Minute;
                    ircMisc.send_buf[i++]=device_comps.system_time.time.u8Second;
					///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
                  i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;			
			
			case MD_DATA_ID_READ_MEASURE_INFO:                //measure info
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					                                        //add full range ...
					
					temp=formatData4fixDot(device_comps.press_cal_param.y[3],device_comps.press_cal_param.dot);
					ircMisc.send_buf[i++]=temp>>24;
					ircMisc.send_buf[i++]=temp>>16;
					ircMisc.send_buf[i++]=temp>>8;
					ircMisc.send_buf[i++]=temp;
					                                         //current p
					temp=formatData4fixDot(device_comps.current_press,device_comps.press_cal_param.dot);                                        
					ircMisc.send_buf[i++]=temp>>24;
					ircMisc.send_buf[i++]=temp>>16;
					ircMisc.send_buf[i++]=temp>>8;
					ircMisc.send_buf[i++]=temp;
					
		            ircMisc.send_buf[i++]=(device_comps.press_cal_param.unit&0x0f);//param unit

		            
                                    

		            ircMisc.send_buf[i++]=device_comps.current_temp>>8;
		            ircMisc.send_buf[i++]=device_comps.current_temp;

		            ircMisc.send_buf[i++]=device_comps.batt;

		            ircMisc.send_buf[i++]=0;//device status
					
					
					///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
             case MD_DATA_ID_READ_LORA_AMT_PARAM:                //lora param
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->freq>>16;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->freq>>8;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->freq;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->nodeId>>24;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->nodeId>>16;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->nodeId>>8;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->nodeId;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->netId>>8;
                    ircMisc.send_buf[i++]=loraComps.cfg_info_p->netId;
					///ADD Other Data
					ircMisc.send_buf[10]=i-11;


					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
                    break;                   
           	case MD_DATA_ID_READ_CHANNEL_N_ADC:               
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
                    
					ircMisc.send_buf[i++]=buf[13];  //    current channel                                
					
					temp=device_comps.ad1_ad2_average_result;
					ircMisc.send_buf[i++]=temp>>24;
					ircMisc.send_buf[i++]=temp>>16;
					ircMisc.send_buf[i++]=temp>>8;
					ircMisc.send_buf[i++]=temp;
				
                    ircMisc.send_buf[i++]=device_comps.current_temp>>8;
		            ircMisc.send_buf[i++]=device_comps.current_temp;

		           ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;         
			case MD_DATA_ID_READ_HIGH_INFO:                //HIGH info
//					ircMisc.send_buf[i++]=(buf[9]|0x80);
//					ircMisc.send_buf[i++]=0;//Length
//					ircMisc.send_buf[i++]=buf[11];//dataID
//					ircMisc.send_buf[i++]=buf[12];
//					                                        //add full range ...
					
//					temp=formatData4fixDot(device_comps.high_cal_param.y[1],device_comps.high_cal_param.dot);
//					ircMisc.send_buf[i++]=temp>>24;
//					ircMisc.send_buf[i++]=temp>>16;
//					ircMisc.send_buf[i++]=temp>>8;
//					ircMisc.send_buf[i++]=temp;
					                                         
//					temp=formatData4fixDot(device_comps.current_high,device_comps.high_cal_param.dot);                                        
//					ircMisc.send_buf[i++]=temp>>24;
//					ircMisc.send_buf[i++]=temp>>16;
//					ircMisc.send_buf[i++]=temp>>8;
//					ircMisc.send_buf[i++]=temp;
//					temp=formatData4fixDot(device_comps.current_volume,device_comps.high_cal_param.dot+2); //v=S*H H/=10,S:3 fix dot                                       
//					ircMisc.send_buf[i++]=temp>>24;
//					ircMisc.send_buf[i++]=temp>>16;
//					ircMisc.send_buf[i++]=temp>>8;
//					ircMisc.send_buf[i++]=temp;
					
//		             ircMisc.send_buf[i++]=device_comps.current_temp>>8;
//		            ircMisc.send_buf[i++]=device_comps.current_temp;

//		            ircMisc.send_buf[i++]=device_comps.batt;

//		            ircMisc.send_buf[i++]=0;//device status
					
					
//					///ADD Other Data
//					ircMisc.send_buf[10]=i-11;
//					  ircMisc.send_buf[i++]=Check_Sum(ircMisc.send_buf,i);
//					i++;
//					ircMisc.send_buf[i++]=0x16;
//					VerifyResult=0;
					break;	
			case MD_DATA_ID_READ_ACCESS_ADDR:                //read access addr
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],device_comps.access_param.ip,25);
					i+=25;
					ircMisc.send_buf[i++]=device_comps.access_param.port>>8;
					ircMisc.send_buf[i++]=device_comps.access_param.port;

                    memcpy(&ircMisc.send_buf[i],device_comps.access_param.ip1,25);
					i+=25;
					ircMisc.send_buf[i++]=device_comps.access_param.port1>>8;
					ircMisc.send_buf[i++]=device_comps.access_param.port1;
                    ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;

			case MD_DATA_ID_READ_GPS_LOC_INFO:                //
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					temp=device_comps.gps.glng;                                        
					ircMisc.send_buf[i++]=temp>>24;
					ircMisc.send_buf[i++]=temp>>16;
					ircMisc.send_buf[i++]=temp>>8;
					ircMisc.send_buf[i++]=temp;
					temp=device_comps.gps.glat;                                        
					ircMisc.send_buf[i++]=temp>>24;
					ircMisc.send_buf[i++]=temp>>16;
					ircMisc.send_buf[i++]=temp>>8;
					ircMisc.send_buf[i++]=temp;
                    ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
				
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
			case MD_DATA_ID_READ_MANUFACTURER_INFO:                //
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],device_comps.manufacturer_info.name,25);
					i+=25;
	                ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
				
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
			case MD_DATA_ID_READ_DEVICE_TYPE_INFO:                //
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],device_comps.device_info.type,25);
					i+=25;
	                ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
			case MD_DATA_ID_READ_DEVICE_ID_INFO:                //
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],device_comps.device_info.id,25);
					i+=25;
	                ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
				
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
			case MD_DATA_ID_READ_SENSOR_ID_INFO:                //
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],device_comps.sensor_info.id,25);
					i+=25;
	                ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
           case MD_DATA_ID_READ_LBS_TOKEN:                //
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],device_comps.lbs_param.token,sizeof(device_comps.lbs_param.token));
					i+=sizeof(device_comps.lbs_param.token);
	                ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					
					ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
          case MD_DATA_ID_READ_LBS_DOMAIN:                //
					ircMisc.send_buf[i++]=(buf[9]|0x80);
					ircMisc.send_buf[i++]=0;//Length
					ircMisc.send_buf[i++]=buf[11];//dataID
					ircMisc.send_buf[i++]=buf[12];
					memcpy(&ircMisc.send_buf[i],device_comps.lbs_param.domain,sizeof(device_comps.lbs_param.domain));
					i+=sizeof(device_comps.lbs_param.token);
	                ///ADD Other Data
					ircMisc.send_buf[10]=i-11;
					 ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
					 i++;
					ircMisc.send_buf[i++]=0x16;
					VerifyResult=0;
					break;
			default:       
					return 1;
			
		}
		write_irc(ircMisc.send_buf,i);
		return 13+buf[10];
	}
	else if(Cmd==1)//Write
	{
        if((DataId==MD_DATA_ID_SET_DEVICE_ADDR)||(DataId==MD_DATA_ID_WRITE_ALARM_PARAM)
             ||(DataId==MD_DATA_ID_SET_ACCESS_ADDR)||(DataId==MD_DATA_ID_WRITE_REPORT_PARAM)||(DataId==MD_DATA_ID_ENTER_CAL_MODE))
        {
            if(hum_comps.current_mode==EM_CAL_MODIFY_MODE)
            {
                return 1;
            }

        }
        else if(DataId==MD_DATA_ID_WRITE_CAL_DATA)
        {
            if(hum_comps.current_mode!=EM_CAL_MODIFY_MODE)
            {
                return 1;
            }
        }
	    memcpy(ircMisc.send_buf,buf,9);
		memcpy(ircMisc.send_buf+1,device_comps.device_addr.addr,7);
		switch(DataId)
	    {
		    case MD_DATA_ID_ENTER_CAL_MODE:                //enter calmode
				
                if(device_comps.sw._bit.e2prom_driver_err)
                {
                    ircMisc.send_buf[i++]=(buf[9]|0x90);    
                }
                else 
                {
				    ircMisc.send_buf[i++]=(buf[9]|0x80);
				   device_comps.cal_type=(cal_type_t)(buf[13]>>4);
				 
				   hum_comps.enter_cal_modify_mode(0);
				   if(device_comps.cal_type==EM_CAL_PRESS)
				   {
                        device_comps.press_cal_param_bak.dot=buf[13]&0x0f;
                        device_comps.press_cal_param_bak.unit=buf[14];
				   }
				   else if(device_comps.cal_type==EM_CAL_RES)
				   {
                        device_comps.res_cal_param_bak.dot=buf[13]&0x0f;
                        device_comps.res_cal_param_bak.unit=buf[14];
				   }
//				   else if(device_comps.cal_type==EM_CAL_HIGH)
//				   {
//                        device_comps.high_cal_param_bak.dot=buf[13]&0x0f;
//                        device_comps.high_cal_param_bak.unit=buf[14];
//				   }
                }
				
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];
				

				///ADD Other Data
			
				break;	
				
				
			case MD_DATA_ID_ENTER_NORMAL_MODE:                //exit cal mode
				
                ircMisc.send_buf[i++]=(buf[9]|0x80);
                hum_comps.enter_default_mode(0);

				
			    ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			
				break;
						
				
			case MD_DATA_ID_SET_DEVICE_ADDR:                //write device addr
    			if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
    				device_addr_t addr_cpy;
    				memcpy(addr_cpy.addr,&buf[13],7);
    				addr_cpy.cs=Check_Sum_5A(&addr_cpy, &addr_cpy.cs-(uint8_t *)&addr_cpy);
    				if(device_comps.save_device_addr(&addr_cpy.addr,sizeof(addr_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					memcpy(&device_comps.device_addr,&addr_cpy,sizeof(addr_cpy));
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
						mode_comps[hum_comps.current_mode].dis_option=6;
//                        hum_comps.dis_oper_mark._bit.refresh_option=1;
//						hum_comps.dis_oper_mark._bit.refresh_device_id_low=1;
				    }
    			}
    			memcpy(ircMisc.send_buf+1,device_comps.device_addr.addr,7);
    		    ircMisc.send_buf[i++]=0;
    			ircMisc.send_buf[i++]=buf[11];
    			ircMisc.send_buf[i++]=buf[12];

    			///ADD Other Data
    		
			    break;
			case MD_DATA_ID_WRITE_ALARM_PARAM:                //write alarm Param
			    if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
    				alarm_param_t alarm_param_cpy;
    				//memcpy(&alarm_param_cpy.press_high_upper,&buf[13],4*4);
    				alarm_param_cpy.press_high_upper=((uint32_t)buf[13]<<24)+((uint32_t)buf[14]<<16)+((uint32_t)buf[15]<<8)+buf[16];
    				alarm_param_cpy.press_high_lower=((uint32_t)buf[17]<<24)+((uint32_t)buf[18]<<16)+((uint32_t)buf[19]<<8)+buf[20];
    				alarm_param_cpy.press_low_upper =((uint32_t)buf[21]<<24)+((uint32_t)buf[22]<<16)+((uint32_t)buf[23]<<8)+buf[24];
    				alarm_param_cpy.press_low_lower =((uint32_t)buf[25]<<24)+((uint32_t)buf[26]<<16)+((uint32_t)buf[27]<<8)+buf[28];
    				alarm_param_cpy.unit=buf[29];
    				//memcpy(&alarm_param_cpy.temp_high,&buf[13+4*4+1],2*2);
    				alarm_param_cpy.temp_high =((uint16_t)buf[30]<<8)+buf[31];
    				alarm_param_cpy.temp_low =((uint16_t)buf[32]<<8)+buf[33];
    				alarm_param_cpy.cs=Check_Sum_5A(&alarm_param_cpy, &alarm_param_cpy.cs-(uint8_t *)&alarm_param_cpy);
    				if(device_comps.save_alarm_param(&alarm_param_cpy,sizeof(alarm_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					memcpy(&device_comps.alarm_param,&alarm_param_cpy,sizeof(alarm_param_cpy));
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
    			}	
    		    ircMisc.send_buf[i++]=0;
    			ircMisc.send_buf[i++]=buf[11];
    			ircMisc.send_buf[i++]=buf[12];

    			///ADD Other Data
    		
			    break;
			case MD_DATA_ID_SET_ACCESS_ADDR :                //write access addr
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					access_param_t access_param_cpy={0};
    				memcpy(&access_param_cpy.ip[0],&buf[13],25);
    				
    				access_param_cpy.port=((uint16_t)buf[13+25]<<8)+buf[13+26];
    				memcpy(&access_param_cpy.ip1[0],&buf[13+27],25);
    				
    				access_param_cpy.port1=((uint16_t)buf[13+27+25]<<8)+buf[13+27+26];
    				
    				access_param_cpy.cs=Check_Sum_5A(&access_param_cpy, &access_param_cpy.cs-(uint8_t *)&access_param_cpy);
    				if(device_comps.save_access_param(&access_param_cpy,sizeof(access_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					memcpy(&device_comps.access_param,&access_param_cpy,sizeof(access_param_cpy));
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			
				break;
                
			case  MD_DATA_ID_WRITE_LBS_TOKEN:              
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					lbs_param_t lbs_param_cpy=device_comps.lbs_param;
    				memcpy(&lbs_param_cpy.token[0],&buf[13],20);
    				lbs_param_cpy.cs=Check_Sum_5A(&lbs_param_cpy, &lbs_param_cpy.cs-(uint8_t *)&lbs_param_cpy);
    				if(device_comps.save_lbs_param(&lbs_param_cpy,sizeof(lbs_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					memcpy(&device_comps.lbs_param,&lbs_param_cpy,sizeof(lbs_param_cpy));
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			;
				break;

            case  MD_DATA_ID_WRITE_LBS_DOMAIN:               
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					lbs_param_t lbs_param_cpy=device_comps.lbs_param;
    				memcpy(&lbs_param_cpy.domain[0],&buf[13],32);
    				lbs_param_cpy.cs=Check_Sum_5A(&lbs_param_cpy, &lbs_param_cpy.cs-(uint8_t *)&lbs_param_cpy);
    				if(device_comps.save_lbs_param(&lbs_param_cpy,sizeof(lbs_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					memcpy(&device_comps.lbs_param,&lbs_param_cpy,sizeof(lbs_param_cpy));
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			
				break;

             case  MD_DATA_ID_WRITE_IOT_PRODUCT_ID:               
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					iot_param_t iot_param_cpy=device_comps.iot_param;
                    memset(&iot_param_cpy.productID[0],0,sizeof(iot_param_cpy.productID));
    				memcpy(&iot_param_cpy.productID[0],&buf[13],16);
    				iot_param_cpy.cs=Check_Sum_5A(&iot_param_cpy, &iot_param_cpy.cs-(uint8_t *)&iot_param_cpy);
    				if(device_comps.save_iot_param(&iot_param_cpy,sizeof(iot_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					//memcpy(&device_comps.iot_param,&iot_param_cpy,sizeof(iot_param_cpy));
                        device_comps.iot_param=iot_param_cpy;
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			    break;
             case  MD_DATA_ID_WRITE_IOT_TENANT_ID:               
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					iot_param_t iot_param_cpy=device_comps.iot_param;
                    memset(&iot_param_cpy.tenantID[0],0,sizeof(iot_param_cpy.tenantID));
    				memcpy(&iot_param_cpy.tenantID[0],&buf[13],16);
    				iot_param_cpy.cs=Check_Sum_5A(&iot_param_cpy, &iot_param_cpy.cs-(uint8_t *)&iot_param_cpy);
    				if(device_comps.save_iot_param(&iot_param_cpy,sizeof(iot_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					//memcpy(&device_comps.iot_param,&iot_param_cpy,sizeof(iot_param_cpy));
    					  device_comps.iot_param=iot_param_cpy;
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			    break;
           case  MD_DATA_ID_WRITE_IOT_TOKEN:               
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					iot_param_t iot_param_cpy=device_comps.iot_param;
                    memset(&iot_param_cpy.token[0],0,sizeof(iot_param_cpy.token));
    				memcpy(&iot_param_cpy.token[0],&buf[13],64);
    				iot_param_cpy.cs=Check_Sum_5A(&iot_param_cpy, &iot_param_cpy.cs-(uint8_t *)&iot_param_cpy);
    				if(device_comps.save_iot_param(&iot_param_cpy,sizeof(iot_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					//memcpy(&device_comps.iot_param,&iot_param_cpy,sizeof(iot_param_cpy));
    					  device_comps.iot_param=iot_param_cpy;
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			    break;    
                
             case  MD_DATA_ID_WRITE_IOT_DEVICE_ID:               
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					iot_param_t iot_param_cpy=device_comps.iot_param;
                    memset(&iot_param_cpy.deviceID[0],0,sizeof(iot_param_cpy.deviceID));
    				memcpy(&iot_param_cpy.deviceID[0],&buf[13],32);
    				iot_param_cpy.cs=Check_Sum_5A(&iot_param_cpy, &iot_param_cpy.cs-(uint8_t *)&iot_param_cpy);
    				if(device_comps.save_iot_param(&iot_param_cpy,sizeof(iot_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					//memcpy(&device_comps.iot_param,&iot_param_cpy,sizeof(iot_param_cpy));
    					  device_comps.iot_param=iot_param_cpy;
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			    break;    
                     
			case MD_DATA_ID_WRITE_GPS_LOC_INFO :                //write
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
				    device_comps.gps.glng=((uint32_t)buf[13]<<24)+((uint32_t)buf[14]<<16)+((uint32_t)buf[15]<<8)+buf[16];;
    				device_comps.gps.glat=((uint32_t)buf[17]<<24)+((uint32_t)buf[18]<<16)+((uint32_t)buf[19]<<8)+buf[20];
    	            device_comps.gps.cs=Check_Sum_5A(&device_comps.gps, &device_comps.gps.cs-(uint8_t *)&device_comps.gps);
    				if(device_comps.save_gps_param(&device_comps.gps,sizeof(device_comps.gps)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
    				
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			
				break;
			case MD_DATA_ID_WRITE_MANUFACTURER_INFO :                //write
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
    			    memcpy(device_comps.manufacturer_info.name,&buf[13],25);
    			    
		            device_comps.manufacturer_info.cs=Check_Sum_5A(&device_comps.manufacturer_info, &device_comps.manufacturer_info.cs-(uint8_t *)&device_comps.manufacturer_info);
    				if(device_comps.save_manufacturer_info(&device_comps.manufacturer_info,sizeof(device_comps.manufacturer_info)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
    				
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
		
				break;
			case MD_DATA_ID_WRITE_DEVICE_TYPE_INFO :                //write
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
    			    memcpy(device_comps.device_info.type,&buf[13],25);
    			    
		            device_comps.device_info.cs=Check_Sum_5A(&device_comps.device_info, &device_comps.device_info.cs-(uint8_t *)&device_comps.device_info);
    				if(device_comps.save_device_info(&device_comps.device_info,sizeof(device_comps.device_info)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
    				
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
		
				break;
			
			case MD_DATA_ID_WRITE_DEVICE_ID_INFO :                //write
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
    			    memcpy(device_comps.device_info.id,&buf[13],25);
    			    
		            device_comps.device_info.cs=Check_Sum_5A(&device_comps.device_info, &device_comps.device_info.cs-(uint8_t *)&device_comps.device_info);
    				if(device_comps.save_device_info(&device_comps.device_info,sizeof(device_comps.device_info)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
    				
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			
				break;
			case MD_DATA_ID_WRITE_SENSOR_INFO :                //write
			
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
    			    memcpy(device_comps.sensor_info.id,&buf[13],25);
    			    
		            device_comps.sensor_info.cs=Check_Sum_5A(&device_comps.sensor_info, &device_comps.sensor_info.cs-(uint8_t *)&device_comps.sensor_info);
    				if(device_comps.save_sensor_info(&device_comps.sensor_info,sizeof(device_comps.sensor_info)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
    				
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			
				break;	
			case MD_DATA_ID_WRITE_REPORT_PARAM:  //report param
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					report_param_t report_param_cpy;
					report_param_cpy.u8Minute=buf[13];
					report_param_cpy.u8Hour=buf[14];
					report_param_cpy.u16Minute_Interval=((uint16_t)buf[15]<<8)+buf[16];
					report_param_cpy.u8Hour_Interval=buf[15];
                    report_param_cpy.disFactor=((uint16_t)buf[17]<<8)+buf[18];
                    report_param_cpy.triggerTimes=((uint32_t)buf[19]<<24)+((uint32_t)buf[20]<<16)+((uint32_t)buf[21]<<8)+buf[22];
					report_param_cpy.cs=Check_Sum_5A(&report_param_cpy, &report_param_cpy.cs-(uint8_t *)&report_param_cpy);
    				if(device_comps.save_report_param(&report_param_cpy,sizeof(report_param_cpy)))
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
                        device_comps.report_interval_timer=0;
    					memcpy(&device_comps.report_param,&report_param_cpy,sizeof(report_param_cpy));
    					ircMisc.send_buf[i++]=(buf[9]|0x80);
    				}
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data
			
				break;
			case MD_DATA_ID_WRITE_SYSTEM_TIME:  
				if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
					device_comps.system_time.time.u8Year =buf[13];
                    device_comps.system_time.time.u8Month=buf[14];
                    device_comps.system_time.time.u8Day  =buf[15];
                    device_comps.system_time.time.u8Hour =buf[16];
                    device_comps.system_time.time.u8Minute=buf[17];
                    device_comps.system_time.time.u8Second=buf[18];
                    device_comps.system_time.cs=Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs-(uint8_t *)&device_comps.system_time);
                    device_comps.save_system_time(&device_comps.system_time,sizeof(device_comps.system_time));
                    device_comps.set_system_time(&device_comps.system_time.time);
                    ertc_comps.write_broken_time(&device_comps.system_time.time); 
                    ircMisc.send_buf[i++]=(buf[9]|0x80);
                    
				}
			
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

				///ADD Other Data

				break;	
            case MD_DATA_ID_WRITE_LORA_AMT_PARAM:
                if(device_comps.sw._bit.e2prom_driver_err)
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
			    else 
    			{
					//lora_cfg_info_t  lora_cfg_info_cpy;
					loraComps.cfg_info_p->freq=((uint32_t)buf[13]<<16)+((uint32_t)buf[14]<<8)+buf[15];
					loraComps.cfg_info_p->nodeId=((uint32_t)buf[16]<<24)+((uint32_t)buf[17]<<16)+((uint32_t)buf[18]<<8)+buf[19];
					loraComps.cfg_info_p->netId=((uint16_t)buf[20]<<8)+buf[21];
					if(loraComps.save_cfg_info())
    				{
    				    ircMisc.send_buf[i++]=(buf[9]|0x90);    
    				}
    				else
    				{
                       //  memcpy(loraComps.cfg_info_p,&lora_cfg_info_cpy,sizeof(lora_cfg_info_cpy));
			             ircMisc.send_buf[i++]=(buf[9]|0x80);
				         loraComps.sw._bit.param_modified=1;
    				}
				}
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];
                
				//ADD Other Data
				
				break;
			case MD_DATA_ID_WRITE_CAL_DATA:       //write cal data       
                if((device_comps.sw._bit.e2prom_driver_err)||(hum_comps.current_mode!=EM_CAL_MODIFY_MODE)
                  ||(mode_comps[hum_comps.current_mode].dis_option!=buf[13]))
    			{
    				ircMisc.send_buf[i++]=(buf[9]|0x90);    
    			}
    			else 
    			{
                    int32_t param=((uint32_t)buf[14]<<24)+((uint32_t)buf[15]<<16)
                                +((uint32_t)buf[16]<<8)+buf[17];
                    ircMisc.send_buf[i++]=(buf[9]|0x80); 
                    if(device_comps.cal_type==EM_CAL_PRESS)
                    {
                        switch(buf[13])
                        {
                            case 0:
                                        device_comps.press_cal_param_bak.x[0]=device_comps.ad1_ad2_average_result;
                                        device_comps.press_cal_param_bak.y[0]=param;
                                        mode_comps[hum_comps.current_mode].dis_option++;
                                       // hum_comps.dis_oper_mark._bit.refresh_option=1;
                                        hum_comps.dis_oper_mark._bit.refresh_cal_param=1;
                                        break;
                         
                            case 1:
                                        device_comps.press_cal_param_bak.x[1]=device_comps.ad1_ad2_average_result;
                                        device_comps.press_cal_param_bak.y[1]=param;
                                        mode_comps[hum_comps.current_mode].dis_option++;
                                      //  hum_comps.dis_oper_mark._bit.refresh_option=1;
                                        hum_comps.dis_oper_mark._bit.refresh_cal_param=1;
                                        break;
                            case 2:
                                        device_comps.press_cal_param_bak.x[2]=device_comps.ad1_ad2_average_result;
                                        device_comps.press_cal_param_bak.y[2]=param;
                                        mode_comps[hum_comps.current_mode].dis_option++;
                                        //hum_comps.dis_oper_mark._bit.refresh_option=1;
                                        hum_comps.dis_oper_mark._bit.refresh_cal_param=1;
                                        break;
                            case 3:     
                                        device_comps.press_cal_param_bak.x[3]=device_comps.ad1_ad2_average_result;
                                        device_comps.press_cal_param_bak.y[3]=param;
                                        device_comps.press_cal_param_bak.is_calibrated=1;
                                        device_comps.press_cal_param_bak.cs=Check_Sum_5A(&device_comps.press_cal_param_bak, & device_comps.press_cal_param_bak.cs-(uint8_t *)&device_comps.press_cal_param_bak);
                                        if(device_comps.save_press_cal_param(&device_comps.press_cal_param_bak,sizeof(device_comps.press_cal_param_bak)))
                        				{
                        				    ircMisc.send_buf[i]=(buf[9]|0x90);
                                            mode_comps[hum_comps.current_mode].dis_option=0;
                                           // hum_comps.dis_oper_mark._bit.refresh_option=1;
                                            hum_comps.dis_oper_mark._bit.refresh_cal_param=1;
                                            device_comps.press_cal_param_bak.is_calibrated=0;
                        				}
                        				else
                        				{
                        					memcpy(&device_comps.press_cal_param,&device_comps.press_cal_param_bak,sizeof(device_comps.press_cal_param_bak));
                        					ircMisc.send_buf[i]=(buf[9]|0x80);
                                            device_comps.clr_press(0);
                                            hum_comps.enter_default_mode(0);

                                         }
                                        
                                        break;
                            default:   return 1;
                            
                         }
                   }
                   
                   else if(device_comps.cal_type==EM_CAL_RES)
                   {
                        switch(buf[13])
                        {
                            case 0:
                                        device_comps.res_cal_param_bak.x[0]=device_comps.temp_p_temp_n_average_result;
                                        device_comps.res_cal_param_bak.y[0]=param;
                                        mode_comps[hum_comps.current_mode].dis_option++;
                                       // hum_comps.dis_oper_mark._bit.refresh_option=1;
                                        hum_comps.dis_oper_mark._bit.refresh_cal_param=1;
                                        break;
                         
                            case 1:
                                        device_comps.res_cal_param_bak.x[1]=device_comps.temp_p_temp_n_average_result;
                                        device_comps.res_cal_param_bak.y[1]=param;
                                        mode_comps[hum_comps.current_mode].dis_option++;
                                        //hum_comps.dis_oper_mark._bit.refresh_option=1;
                                       hum_comps.dis_oper_mark._bit.refresh_cal_param=1;
                                        device_comps.res_cal_param_bak.cs=Check_Sum_5A(&device_comps.res_cal_param_bak, & device_comps.res_cal_param_bak.cs-(uint8_t *)&device_comps.res_cal_param_bak);
                                        if(device_comps.save_res_cal_param(&device_comps.res_cal_param_bak,sizeof(device_comps.res_cal_param_bak)))
                        				{
                        				    ircMisc.send_buf[i]=(buf[9]|0x90);
                                            mode_comps[hum_comps.current_mode].dis_option=0;
                                          //  hum_comps.dis_oper_mark._bit.refresh_option=1;
                                            hum_comps.dis_oper_mark._bit.refresh_cal_param=1;
                                            device_comps.res_cal_param_bak.is_calibrated=0;
                        				}
                        				else
                        				{
                        					memcpy(&device_comps.res_cal_param,&device_comps.res_cal_param_bak,sizeof(device_comps.res_cal_param_bak));
                        					ircMisc.send_buf[i]=(buf[9]|0x80);
                                            hum_comps.enter_default_mode(0);

                                         }
                                        
                                        break;
                            default:   return 1;
                            
                         }
                   }
                  else 
                   {
                        return 1;
                   }
                }
				
				ircMisc.send_buf[i++]=0;
				ircMisc.send_buf[i++]=buf[11];
				ircMisc.send_buf[i++]=buf[12];

                ///ADD Other Data
				ircMisc.send_buf[i++]=buf[13];

				
				
				break;
				
			default:       
					return 1;	
		}
        
      
        ircMisc.send_buf[10]=i-11;
        ircMisc.send_buf[i]=Check_Sum(ircMisc.send_buf,i);
        i++;
        ircMisc.send_buf[i++]=0x16;
        write_irc(ircMisc.send_buf,i);
		return 13+buf[10];
		
	}
	
	return 1;
	
}
static uint8_t Check_irc_Com(uint8_t *Rec_Data,uint8_t Rec_Pos)
{
	uint8_t i=0;
	if(Rec_Pos<2)
	{
		return 0;
	}
	if(Rec_Data[0]!=0x68)
	{
		return 1;
	}
	if(Rec_Pos<11)
	{
		return 0;
	}
	if(Rec_Data[8]!=0x68)
    {
		return 1;
	}
    if((Rec_Data[9]!=0x00)&&(Rec_Data[9]!=0x01))
    {
		return 1;
	}
	if(Rec_Data[10]>70)
    {
		return 1;
	}
	if((Rec_Data[1]!=0x99)||(Rec_Data[2]!=0x99)||(Rec_Data[3]!=0x99)||(Rec_Data[4]!=0x99)
	&&(Rec_Data[5]!=0x99)||(Rec_Data[6]!=0x99)||(Rec_Data[7]!=0x99))
	{
		for(i=0;i<7;i++)
		{
			if(Rec_Data[1+i]!=device_comps.device_addr.addr[i])
			{
				return 1;
			}
		}
	}
	
	if(Rec_Pos<13+Rec_Data[10])
	{
		return 0;
	}
	
	if(Rec_Data[13+Rec_Data[10]-1-1]!=Check_Sum(Rec_Data,13+Rec_Data[10]-2))
	{
		return 1;
	}
	if(Rec_Data[13+Rec_Data[10]-1]!=0x16)
	{
		return 1;
	}
	
	ircComps.op_window_time=25;
    return Pro_irc(Rec_Data[9],Rec_Data);
      	
}

static void irc_start(void)
{
    if(!modbusComps.sw._bit.runing)
    {
       
        ircComps.op_window_time=30;
       
        config_irc_com(0,2);
        enable_irc_com();
        ircComps.sw._bit.runing=1;
        MD_IR_VCM_ON;
    }
    device_comps.buzzer.start(11);
  
}
 static void irc_stop(void)
 {
      if(!modbusComps.sw._bit.runing)
      {
            MD_IR_VCM_OFF;
            ircComps.op_window_time=0;
            
            disable_irc_com();
            ircComps.sw._bit.runing=0;
      }
 }

static void Deal_irc(void)
{
	uint8_t err=0;
 	do
	{
		
		err=Check_irc_Com(ircMisc.recv_buf,ircMisc.recv_pos); 
		
		if(err>0)
		{
			 __disable_irq();
			memcpy(ircMisc.recv_buf,ircMisc.recv_buf+err,ircMisc.recv_pos-=err);
			 __enable_irq();
    }
		
	}
	while (err>0);
}

static void  ircComps_task_handle(void)
{


  if(ircComps.sw._bit.runing||modbusComps.sw._bit.runing)
   {
        Deal_irc();
   }
  
}



void store_irc_buffer(uint8_t data)
{
	ircMisc.recv_buf[ircMisc.recv_pos]=data;
	ircMisc.recv_pos+=1;
	ircMisc.recv_pos&=0x7f;
}

ircComps_t ircComps=
{
    0,
    {0},
        
    store_irc_buffer,
    irc_stop,  //void (*stop)(void);
    ircComps_task_handle
};



