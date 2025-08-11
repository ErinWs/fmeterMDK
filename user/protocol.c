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



////#define static   




static struct 
{ 
	uint8_t buf[1024];
	uint8_t step;
	int16_t event_index;
	int16_t reRryTimes;
	 
	
}
protocolMisc=
{
	"",
	0 ,
	-1,
	3
};

static uint16_t generateCRC(char *buffer, uint16_t messageLength)
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

static char Check_Sum(char *Data,char Len)
{
	char Sum=0;
	char i=0;
	for(i=0;i<Len;i++)
	{
		Sum+=Data[i];
	}
	return Sum;
}

static char Check_Sum_5A( const void* Data,char Len)
{
    char Sum=0x5A;
    char i=0;
	char *data=(char *)Data;
    for(i=0;i<Len;i++)
    {
        Sum+=data[i];
    }
    return Sum;
}

static uint16_t  Check_Sum_2bytes(char const *Data,int16_t Len)
{
	uint16_t Sum=0;
	int16_t  i=0;
	for(i=0;i<Len;i++)
	{
		Sum+=Data[i];
	}
	return Sum;
}

////////return  lenght of Rcvbuf
//static int16_t crc_16(char const*puchMsg, char *Rcvbuf ,uint16_t usDataLen)
//{
//	int16_t wCRCin = 0x0000;
//	int16_t wCPoly = 0x1021;
//	int16_t wChar = 0;
//	int16_t i=0;
//
//	while (usDataLen--) 	
//	{
//	wChar = *(puchMsg++);
//	wCRCin ^= (wChar << 8);
//	for(i = 0;i < 8;i++)
//	{
//	  if(wCRCin & 0x8000)
//	    wCRCin = (wCRCin << 1) ^ wCPoly;
//	  else
//	    wCRCin = wCRCin << 1;
//	}
//	}
//	Rcvbuf[0] = (wCRCin & 0xff00)>>8;//??????
//	Rcvbuf[1] = (wCRCin& 0x00ff);  //¦Ì¨ª????
//	return 2;
//}
//
////return Return crc check result 0 ok 1,err
////static int16_t check_crc_16(char const *Array, uint16_t crc_16,uint16_t Len)
////{
//       
//static int16_t check_crc_16(char const*puchMsg, uint16_t crc_16,uint16_t usDataLen)
//{
//  int16_t wCRCin = 0x0000;
//  int16_t wCPoly = 0x1021;
//  int16_t wChar = 0;
//  int16_t i=0;
//  
//  while (usDataLen--) 	
//  {
//        wChar = *(puchMsg++);
//        wCRCin ^= (wChar << 8);
//        for(i = 0;i < 8;i++)
//        {
//          if(wCRCin & 0x8000)
//            wCRCin = (wCRCin << 1) ^ wCPoly;
//          else
//            wCRCin = wCRCin << 1;
//        }
//  }
//  return (wCRCin!=crc_16) ;
//}

static const char * const base64char = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static int16_t base64_decode( const char * base64, char * bindata )
{
    int16_t i, j;
    char k;
    char temp[4];

    for ( i = 0, j = 0; base64[i] != '\0' ; i += 4 )
    {
        memset( temp, 0xFF, sizeof(temp) );
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == base64[i] )
                temp[0] = k;
        }
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == base64[i + 1] )
                temp[1] = k;
        }
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == base64[i + 2] )
                temp[2] = k;
        }
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == base64[i + 3] )
                temp[3] = k;
        }

        bindata[j++] = ((char)(((char)(temp[0] << 2)) & 0xFC)) |
                       ((char)((char)(temp[1] >> 4) & 0x03));
        if ( base64[i + 2] == '=' )
            break;

        bindata[j++] = ((char)(((char)(temp[1] << 4)) & 0xF0)) |
                       ((char)((char)(temp[2] >> 2) & 0x0F));
        if ( base64[i + 3] == '=' )
            break;

        bindata[j++] = ((char)(((char)(temp[2] << 6)) & 0xF0)) |
                       ((char)(temp[3] & 0x3F));
    }
    return j;
}
static int16_t base64_encode( const char * bindata, char * base64, int16_t binlength )
{
    int16_t i, j;
    char current;

    for ( i = 0, j = 0 ; i < binlength ; i += 3 )
    {
        current = (bindata[i] >> 2) ;
        current &= (char)0x3F;
        base64[j++] = base64char[(int16_t)current];

        current = ( (char)(bindata[i] << 4 ) ) & ( (char)0x30 ) ;
        if ( i + 1 >= binlength )
        {
            base64[j++] = base64char[(int16_t)current];
            base64[j++] = '=';
            base64[j++] = '=';
            break;
        }
        current |= ( (char)(bindata[i + 1] >> 4) ) & ( (char) 0x0F );
        base64[j++] = base64char[(int16_t)current];

        current = ( (char)(bindata[i + 1] << 2) ) & ( (char)0x3C ) ;
        if ( i + 2 >= binlength )
        {
            base64[j++] = base64char[(int16_t)current];
            base64[j++] = '=';
            break;
        }
        current |= ( (char)(bindata[i + 2] >> 6) ) & ( (char) 0x03 );
        base64[j++] = base64char[(int16_t)current];

        current = ( (char)bindata[i + 2] ) & ( (char)0x3F ) ;
        base64[j++] = base64char[(int16_t)current];
    }
    base64[j] = '\0';
    return j;
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


static float32_t f_mul(float32_t a,float32_t b)
{
    return a*b;
}

static float32_t f_div(float32_t a,float32_t b)
{
    return a/b;
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


const char *const bcd2a_tbl="0123456789abcdef";
static char * bcd2a(void *str,uint8_t *bcd,int16_t len)
{
    int16_t i=0;
	int16_t j=0;
    char *buf=(char *)str;
    for(j=0;j<len;j++)
    {
        buf[i++]=bcd2a_tbl[ bcd[j]>>4 ];
        buf[i++]=bcd2a_tbl[ bcd[j]&0xf];
    }
    return str;
}
    
#define BK_LN
static char AddAddr( char *buf)
{
     char msg[32]="";
     int16_t i=0;
     bcd2a(msg,&device_comps.device_addr.addr[0],sizeof(device_comps.device_addr.addr));
     sprintf(&buf[i],"ID:%s;",msg);
     i+=strlen(&buf[i]);
     return i;
}
static char  AddTp( char *buf)
{
	char msg[32]="";
    int16_t i=0;
	uint8_t temp[7];
	temp[0]=0x20;
	temp[1]=device_comps.system_time.time.u8Year;
	temp[2]=device_comps.system_time.time.u8Month;
	temp[3]=device_comps.system_time.time.u8Day;
	temp[4]=device_comps.system_time.time.u8Hour;
	temp[5]=device_comps.system_time.time.u8Minute;
	temp[6]=device_comps.system_time.time.u8Second;
	bcd2a(msg,&temp[0],sizeof(temp));
    sprintf(&buf[i],"TIME:%s;",msg);
    i+=strlen(&buf[i]);
    return i;
}


static int16_t AddGpsLoc(char *buf)
{
    int16_t i=0;
    sprintf(&buf[i],"LNG:%d.%05d;",(int)device_comps.gps.glng/100000,(int)abs(device_comps.gps.glng%100000));
    i+=strlen(&buf[i]);
    sprintf(&buf[i],"LAT:%d.%05d;",(int)device_comps.gps.glat/100000,(int)abs(device_comps.gps.glat%100000));
    i+=strlen(&buf[i]);
    return i;
}

static int16_t AddPressInfo(char *buf)
 {
    
      float32_t temp;
      int16_t i=0;
      int16_t Mpa2Kpa[2]={1000,1};
      if(device_comps.press_cal_param.is_calibrated)
      {
            temp=f_div(device_comps.press_full_scale,pwr(device_comps.press_cal_param.dot));
            temp=f_mul(temp, Mpa2Kpa[device_comps.flow_cal_param.unit&1]);
            sprintf(&buf[i],"PFS:%fKPa;",temp);
            i+=strlen(&buf[i]);
            
            temp=f_div(device_comps.current_press,pwr(device_comps.press_cal_param.dot));
            temp=f_mul(temp, Mpa2Kpa[device_comps.flow_cal_param.unit&1]);
            sprintf(&buf[i],"PRES:%fKPa;",temp);
            i+=strlen(&buf[i]);
         
      }
      else
      {
             sprintf(&buf[i],"PFS:NOCAL;PRES:NOCAL;");
             i+=strlen(&buf[i]);
      }
      
//      if(collectorComps.sw._bit.isGetExternPressNoTimeOut)
//      {
//            temp=collectorComps.extern_press_span;
//            sprintf(&buf[i],"PFSE:%fKpa;",temp);
//            i+=strlen(&buf[i]);
//            temp=collectorComps.extern_press;
//            sprintf(&buf[i],"PRESE:%fKPa;",temp);
//            i+=strlen(&buf[i]);
//      }
//      else
//      {
//            sprintf(&buf[i],"PFSE:NC;PRESE:NC;");
//            i+=strlen(&buf[i]);
//      }
      return i;
 }


static uint16_t AddTimeDataSeg(char *buf)
{
//	uint16_t Addr=0;
//	uint16_t StartAddr=0;
//	uint16_t Count=0;
//	uint16_t m=0;
//	int32_t temp;
//	int32_t buffer[48];
//	Addr=device_comps.TimeSegData.store_addr;
//	if((Addr-MD_TIME_SEG_DATA_START_ADDR)/4>=48)
//	{
//		_24cxx_comps.read(Addr-(uint16_t)48*4,buffer,(uint16_t)48*4);
//	}
//	else
//	{
//		StartAddr=MD_TIME_SEG_DATA_END_ADDR-((48-(Addr-MD_TIME_SEG_DATA_START_ADDR)/4)*4);
//		Count=(48-(Addr-MD_TIME_SEG_DATA_START_ADDR)/4)*4;
//		_24cxx_comps.read(StartAddr,buffer,Count);
//		_24cxx_comps.read(MD_TIME_SEG_DATA_START_ADDR,(char *)buffer+Count,(Addr-MD_TIME_SEG_DATA_START_ADDR)/4*4);
//		
//	}
//	for(m=0;m<48;m++)
//	{
//        temp=formatData4fixDot(buffer[m],device_comps.press_cal_param.dot );
//        buf[m*4+0]=temp>>24;
//        buf[m*4+1]=temp>>16;
//        buf[m*4+2]=temp>>8;
//        buf[m*4+3]=temp;
//	}
//	return (uint16_t)48*4;

	int16_t m=0;
	int16_t k=0;
	int16_t i=0;
//	uint16_t StartAddr=0;
//	uint16_t Count=0;
//	char buffer[12*16];
//	int16_t temp;
//	uint16_t freq;
//	int16_t nums=device_comps.TimeSegData.nums;
//	uint16_t Addr=device_comps.TimeSegData.store_addr;
//	if(nums>12)
//	{
//        nums=12;
//	}
//	if((Addr-MD_TIME_SEG_DATA_START_ADDR)/16>=nums)
//	{
//		_24cxx_comps.read(Addr-(uint16_t)nums*16,buffer,(uint16_t)nums*16);
//	}
//	else
//	{
//		StartAddr=MD_TIME_SEG_DATA_END_ADDR-((nums-(Addr-MD_TIME_SEG_DATA_START_ADDR)/16)*16);
//		Count=(nums-(Addr-MD_TIME_SEG_DATA_START_ADDR)/16)*16;
//		_24cxx_comps.read(StartAddr,buffer,Count);
//		_24cxx_comps.read(MD_TIME_SEG_DATA_START_ADDR,(char *)buffer+Count,(Addr-MD_TIME_SEG_DATA_START_ADDR)/16*16);
		
//	}
//	for(m=0;m<nums;m++)
//	{
//	        for(k=0;k<4;k++)
//	        {
//	            freq=*(uint16_t *)&buffer[m*16+4*k ];
//                temp=*(int16_t *)&buffer[m*16+4*k+2];
//	            sprintf((char *)&buf[i],"F%dD%d:%.1f_%.1f;",(int)m+1,(int)k+1,freq/10.,temp/10.);
//	            i+=strlen((char *)&buf[i]);
//	        }
//	}
	return i;
}

uint16_t AddLastSamplTps(char *buf)
{
	 char msg[32]="";
     int16_t i=0;
     bcd2a(msg,&device_comps.TimeSegData.lastSampleTime[0],sizeof(device_comps.TimeSegData.lastSampleTime));
     sprintf(&buf[i],"LSTIME:%s;",msg);
     i+=strlen(&buf[i]);
     return i;
}

#define BK_LN

static char VerifyMeterId(char const *buf)
{
    char msg[32]="";
    bcd2a(msg,device_comps.device_addr.addr,sizeof(device_comps.device_addr.addr));
    return memcmp(buf,msg,14);
}


static int16_t encapsulate_zhian_pack(char *const buf,int16_t event)
{
    int16_t i=0;
    int32_t temp=0;
    char msg[32]="";
    uint16_t uitemp;
    int16_t          itemp;
    char *start_bin;
    int16_t len;
    int16_t i_cpy;

    memset(msg,0,sizeof(msg));
    memcpy(msg,&device_comps.manufacturer_info.name,sizeof(device_comps.manufacturer_info.name));
    sprintf(&buf[i],"$%s$;",msg);
    i+=strlen(&buf[i]);
    
    memset(msg,0,sizeof(msg));
    memcpy(msg,&device_comps.device_info.type,sizeof(device_comps.device_info.type));
    sprintf(&buf[i],"DTYPE:%s;",msg);
    i+=strlen(&buf[i]);
    
    memset(msg,0,sizeof(msg));
    memcpy(msg,&device_comps.device_info.id,sizeof(device_comps.device_info.id));
    sprintf(&buf[i],"DSN:%s;",msg);
    i+=strlen(&buf[i]);
    sprintf(&buf[i],"END;");
    i+=strlen(&buf[i]);


   // sprintf(&buf[i],"report event:%d;",(int)event);
   // i+=strlen(&buf[i]);

    return i;

}

static int16_t encapsulate_self_pack(char *const buf,int16_t event)
{
    int16_t i=0;
    char msg[32]="";
  #if(MD_IOT_PROTOCOL==MD_IOT_TCP)      
    memset(msg,0,sizeof(msg));
    memcpy(msg,&device_comps.manufacturer_info.name,sizeof(device_comps.manufacturer_info.name));
    sprintf(&buf[i],"$%s$;",msg);
    i+=strlen(&buf[i]);

    memset(msg,0,sizeof(msg));
    memcpy(msg,&device_comps.device_info.type,sizeof(device_comps.device_info.type));
    sprintf(&buf[i],"TYPE:%s;",msg);
    i+=strlen(&buf[i]);

    i+=AddAddr(&buf[i]);
    i+=AddGpsLoc(&buf[i]);
  
    memset(msg,0,sizeof(msg));
    memcpy(msg,netComps.net_info.iccid,20);
    sprintf(&buf[i],"ICCID:%s;",msg);
    i+=strlen(&buf[i]);

    sprintf(&buf[i],"VBATT:%d.%dV;",(int)device_comps.batt/10,(int)abs(device_comps.batt%10));
    i+=strlen(&buf[i]);
    
    sprintf(&buf[i],"CSQ:%d;",(int)netComps.net_info.Csq);
    i+=strlen(&buf[i]);
    ///////Protocol header end/////////////////

    sprintf(&buf[i],"WK_FLOW:%f;",f_div(device_comps.current_flow,pwr(device_comps.flow_cal_param.dot)));
    i+=strlen(&buf[i]);

    sprintf(&buf[i],"TOTAL_FLOW:%d.%06d;",(int)device_comps.meter.total_int,(int)(device_comps.meter.total_dec*1000));
    i+=strlen(&buf[i]);

    sprintf(&buf[i],"TEMP:%d.%d;",(int)device_comps.current_temp/10,(int)abs(device_comps.current_temp%10));
    i+=strlen(&buf[i]);

    i+=AddPressInfo(&buf[i]);

    sprintf(&buf[i],"STD_FLOW:%f;",f_div(device_comps.current_flowN,pwr(device_comps.flow_cal_param.dot)));
    i+=strlen(&buf[i]);

    sprintf(&buf[i],"TOTAL_FLOWN:%d.%06d;",(int)device_comps.meter.total_intN,(int)(device_comps.meter.total_decN*1000));
    i+=strlen(&buf[i]);

   //  sprintf(&buf[i],"MASS_FLOW:%f;",f_div(device_comps.current_flowM,pwr(device_comps.flow_cal_param.dot)));
   //  i+=strlen(&buf[i]);

   // sprintf(&buf[i],"CUR_DENS:%f;",f_div(device_comps.misc_param.density,100));
   // i+=strlen(&buf[i]);


////////////user data end///////////////////////
    i+=AddTp(&buf[i]);
   
    sprintf(&buf[i],"EVENT:%d;",(int)event);
    i+=strlen(&buf[i]);

    sprintf(&buf[i],"END;");
    i+=strlen(&buf[i]);
 #elif(MD_IOT_PROTOCOL==MD_IOT_MQTT)  

      sprintf(&buf[i],"%s","{\"tem_ala\":25,\"pre_ala\":26,\"liq_ala\":27.1,\"liqu\":28.2,\"bat\":3.6}");
      i+=strlen(&buf[i]);
 #endif
    return i;

}

static int16_t encapsulate_has_more_self_pack(char *const buf,int16_t event)
{
	int16_t i=0;
	int32_t temp=0;

	sprintf(&buf[i],"END;");
	i+=strlen(&buf[i]);
  return i;

}



//static int16_t encapsulate_sidi_info_pack(char *const buf,int16_t event,uint16_t No)
//{
//    int16_t i=0;
//    int32_t temp=0;
    
//    uint16_t crc;
//    buf[i++]=0x7b;
//    buf[i++]=0;//ser_num
//    buf[i++]=0;//Target addr h
//    buf[i++]=0;//Target addr l
//    buf[i++]=0;//source addr h
//    buf[i++]=1;//source addr l
//    buf[i++]=No>>8;//regist h
//    buf[i++]=No;//regist l
//    buf[i++]=1;//reserved 
//    buf[i++]=1;//hw ver
//    buf[i++]=1;//sf ver 
//    buf[i++]=0;//reserved 
//    buf[i++]=0;//reserved 
//    buf[i++]=0;//reserved 
//    buf[i++]=0;//reserved 
//    memcpy(&buf[i],&device_comps.device_info.id,12);
//    i+=12;
    
//    if(No==0x0301)
//    {
//        buf[1]=0;//ser_num
//        buf[i++]=0;//data len h
//        buf[i++]=7;//data len l
//        buf[i++]=0x20;
//        buf[i++]=device_comps.system_time.time.u8Year;
//        buf[i++]=device_comps.system_time.time.u8Month;
//        buf[i++]=device_comps.system_time.time.u8Day;
//        buf[i++]=device_comps.system_time.time.u8Hour;
//        buf[i++]=device_comps.system_time.time.u8Minute;
//        buf[i++]=device_comps.system_time.time.u8Second;
//    }
//    else if(No==0x0303)
//    {
//        buf[1]=1;//ser_num
//        buf[i++]=0;//data len h
//        buf[i++]=5*2+11;//data len l
//        buf[i++]=1;//packet Num
//        buf[i++]=1;//source addr
//        buf[i++]=5*2+10;//datapacket data len 
//        buf[i++]=5;//Ain nums
//        buf[i++]=2;//fix
//        buf[i++]=0;// Digital
//        buf[i++]=0;// Digital
//        buf[i++]=0;// Digital
//        buf[i++]=0;//reserved 
//        buf[i++]=0;//reserved 
//        buf[i++]=(int32_t)device_comps.current_temp*10>>8;
//        buf[i++]=(int32_t)device_comps.current_temp*10;
//        temp=formatData4fixDot(device_comps.current_press,device_comps.press_cal_param.dot);
//        if(device_comps.press_cal_param.unit&0x0f)
//        {
//            temp/=100000;
//        }
//        else
//        {
//            temp/=100;
//        }
//        buf[i++]=temp>>8;
//        buf[i++]=temp;
//        buf[i++]=(int16_t)device_comps.batt*10>>8;
//        buf[i++]=(int16_t)device_comps.batt*10;
//        buf[i++]=0;
//        buf[i++]=netComps.net_info.Csq;
//        buf[i++]=device_comps.report_param.u16Minute_Interval>>8;
//        buf[i++]=device_comps.report_param.u16Minute_Interval;
//        buf[i++]=0x0a;
//    }
    
   
//    buf[i]=Check_Sum((char *)buf+1,i-1);
//		i++;
//    buf[i++]=0x7d;
//    return i;
//}

static int16_t Encapsulate_dataPush_package(char *const buf,int16_t event)
{
    
    #if (APP_PROTOCOL_TYPE==APP_PROTOCOL_ZHIAN) 
     return encapsulate_zhian_pack(buf,event);
    #elif(APP_PROTOCOL_TYPE==APP_PROTOCOL_SELF) 
     return encapsulate_self_pack(buf,event);
    #endif
    
}



static int16_t Encapsulate_has_more_dataPush_packag(char *const buf,int16_t event)
{
    
    #if (APP_PROTOCOL_TYPE==APP_PROTOCOL_ZHIAN) 
     return encapsulate_zhian_pack(buf,event);
    #elif(APP_PROTOCOL_TYPE==APP_PROTOCOL_SELF) 
     return encapsulate_has_more_self_pack(buf,event);
    #endif
    
}




//static int16_t check_sidi_info_data(const char *buf,int16_t length)
//{
//    int16_t len;
//    if(length<27)
//    {
//         return 0;
//    }
//    if(buf[0]!=0x7b)
//    {
//        return 1;
//    }
//    if(memcmp(&buf[15],device_comps.device_info.id,12))
//    {
//        return 1;
//    }
//    if(buf[7]==1)
//    {
//       len=27+2+7+2;
//    }
//    else if(buf[7]==3)
//    {
//        len=27+2+1+2;
//    }
//    else
//    {
//        return 1;
//    }
//    if(length < len)
//    {
//        return 0;
//    }
//    if(buf[len-2]!=Check_Sum((char *)buf+1, len-3))
//    {
//        return 1;
//    }
    
//    return 2;
//}



static int16_t Analysis_downstream_package(char * buf,uint16_t length,char *address_field)
{
 #if(MD_IOT_PROTOCOL==MD_IOT_TCP)
        char *find;
        char *p;
        int16_t len;
        int16_t err;
     #if (APP_PROTOCOL_TYPE==APP_PROTOCOL_ZHIAN) 
        char  msg1[32]="";
        char   msg[32]="";
        find=strstr(buf,"DSN:");
        if(!find)
        {
            return 1;
        }
        find+=strlen("DSN:");
        p=strchr(find,';');    
        if(!p)
        {
             return 1;
        }
        len=p-find;
        if(len>sizeof(device_comps.device_info.id))
        {
            return 1;
        }
        strncpy(msg,find,len);
        memcpy(msg1,&device_comps.device_info.id,sizeof(device_comps.device_info.id));
        return strcmp(msg1,msg);
     #elif (APP_PROTOCOL_TYPE==APP_PROTOCOL_SELF) 
        find=strstr(buf,"ID:");
        if(!find)
        {
            return 1;
        }
        find+=strlen("ID:");
        p=strchr(find,';');    
        if(!p)
        {
             return 1;
        }
        len=p-find;
        if(len!=14)
        {
            return 1;
        }
        return VerifyMeterId(find);
    
//     #elif (APP_PROTOCOL_TYPE==APP_PROTOCOL_SIDI_INFO)
//       do
//       {
//            err=check_sidi_info_data((char *)buf, length);
//            if(err==0)
//            {
//                return 1;
//            }
//            if(err==2)
//            {
//                return 0;
//            }
//            memcpy(buf,buf+err,length-=err);
//       }
//       while(err>0);
//       return 1;
      #else
        return 1;
     #endif
#elif(MD_IOT_PROTOCOL==MD_IOT_MQTT)

    return 0;
#else
   return 1;
#endif


}        



static int16_t  DealDownCmd(char  * const buf,uint16_t Len)
{
 
    int16_t i=0;
    char *find;
    char  msg[32]="";
    int   msgid;
    char  topic[32]="";
    int   len;
    int   task_id;
  #if(MD_IOT_PROTOCOL==MD_IOT_TCP)   
   #if (APP_PROTOCOL_TYPE==APP_PROTOCOL_SELF)
	
     const char *sample_interval="INR:";
     const char *ip="RSA:";
     const char *rport="PORT:";
     const char *altv="ALTV:";
     const char *altm="ALTM:";
    
    find=strstr(buf,altv);
    if(find)
    {
        char  *endptr;
        int32_t   altv_x;
        int32_t   altv_y;
        int32_t   altv_z;
        find+=strlen(altv);
        if(1)
        {
            altv_x=strtof(find,&endptr)*1000;
            altv_y=strtof(endptr+1,&endptr)*1000;
            altv_z=strtof(endptr+1,&endptr)*1000;
            if(1)
            {
               
            }
        }
    }
    
    find=strstr(buf,altm);
    if(find)
    {
        char  *endptr;
        int16_t    interval;
        find+=strlen(altm);
        if(*find>='0'  && *find<='9')
        {
            interval=atol(find);
            if(1)
            {
                
            }
        }
    }

        find=strstr(buf,sample_interval);
        if(find)
        {
    	    int32_t    hcjg;	
    	    find+=strlen(sample_interval);
    	    if(*find>='0'  && *find<='9')
    	    {
                hcjg=atol(find);
                if(hcjg<65536)
                {
                       device_comps.report_interval_timer=0;
                       device_comps.report_param.u16Minute_Interval=hcjg;
        			   device_comps.report_param.u8Hour_Interval=hcjg;
        			   device_comps.report_param.cs=Check_Sum_5A(&device_comps.report_param, &device_comps.report_param.cs-(uint8_t *)&device_comps.report_param);
        			   device_comps.save_report_param(&device_comps.report_param,sizeof(device_comps.report_param));
                }
            }
       

        find=strstr(buf,rport);
        if(find)
        {
    	    uint16_t port;	
    	    find+=strlen(rport);
            port=atol(find);
            if(port>0)
            {
                    if(netComps.net_info.currentIP_No==EM_IP0)
                    {
                        device_comps.access_param.port=port;
                        device_comps.access_param.cs=Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs-(uint8_t *)&device_comps.access_param);
                        device_comps.save_access_param(&device_comps.access_param,sizeof(device_comps.access_param));
    			   }
    			   else if(netComps.net_info.currentIP_No==EM_IP1)
    			   {
    			        device_comps.access_param.port1=port;
                        device_comps.access_param.cs=Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs-(uint8_t *)&device_comps.access_param);
                        device_comps.save_access_param(&device_comps.access_param,sizeof(device_comps.access_param));
                    }
            }
        }

        find=strstr(buf,ip);
        if(find)
        {
    	    int16_t len;	
    	    find+=strlen(ip);
    	    len=strstr(find,";")- find;
            
            if( len>6&& len<26)
            {
                  if(netComps.net_info.currentIP_No==EM_IP0)
                    {
                        memset(&device_comps.access_param.ip,0,sizeof(device_comps.access_param.ip));
                        memcpy(&device_comps.access_param.ip,find,len);
                        device_comps.access_param.cs=Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs-(uint8_t *)&device_comps.access_param);
    			        device_comps.save_access_param(&device_comps.access_param,sizeof(device_comps.access_param));
                    }
                    else if(netComps.net_info.currentIP_No==EM_IP1)
                    {
                        memset(&device_comps.access_param.ip1,0,sizeof(device_comps.access_param.ip1));
                        memcpy(&device_comps.access_param.ip1,find,len);
                        device_comps.access_param.cs=Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs-(uint8_t *)&device_comps.access_param);
    			        device_comps.save_access_param(&device_comps.access_param,sizeof(device_comps.access_param));
                    }
            }
        
        }
        
    }
        
    memset(protocolMisc.buf,0,sizeof(protocolMisc.buf));
    memset(msg,0,sizeof(msg));
    memcpy(msg,&device_comps.manufacturer_info.name,sizeof(device_comps.manufacturer_info.name));
    sprintf(&buf[i],"$%s$;",msg);
    i+=strlen(&buf[i]);

  
    i+=AddAddr(&buf[i]);
   

    sprintf(&buf[i],"RCVD:%d;",(int)Len);
    i+=strlen(&buf[i]);

    sprintf(&buf[i],"END;");
    i+=strlen(&buf[i]);
  #endif
  
 #elif(MD_IOT_PROTOCOL==MD_IOT_MQTT)  //1,"alarm",35,"{"taskId":34,"payload":{"alarm":1}}"
   sscanf(buf,"%d,\"%[^\"]\",%d,\"{\"taskId\":%d",&msgid,topic,&len,&task_id);
   if(!strcmp(topic,"alarm") || !strcmp(topic,"history_switch") || !strcmp(topic,"once_report_cmd") 
      || !strcmp(topic,"pressure_alarm_switch") || !strcmp(topic,"pressure_lower_limit") || !strcmp(topic,"pressure_upper_limit"))
   {
    
        memset(protocolMisc.buf,0,sizeof(protocolMisc.buf));//{"taskId":39,"resultPayload":{"result":0}}
        sprintf(&buf[i],"{\"taskId\":%d,\"resultPayload\":{",(int)task_id);
        i+=strlen(&buf[i]);
        sprintf(&buf[i],"\"result\":%d",0);
        i+=strlen(&buf[i]);

        sprintf(&buf[i],"}}");
        i+=strlen(&buf[i]);
        sprintf(protocolComps.mqtt_pub_topic,"wzy_ack");
        
   }


   
 #endif

    if(i>0)
    {
        protocolComps.msgLen=i;
    //    protocolComps.msg;//=protocolMisc.buf;
        protocolComps.sw._bit.DataRdy=1;
    }
	return 0;
}

static void Srv_Protol(int16_t event)
{

	if(protocolMisc.step==0)//Encapsulate the registration package and send a message to esam encryption
	{
		if(netComps.St._bit.allow_data_send>0)
		{
            if(!protocolComps.sw._bit.DataRdy)
            {
                memset(protocolMisc.buf,0,sizeof(protocolMisc.buf));
    			protocolComps.msgLen=Encapsulate_dataPush_package((char *)protocolMisc.buf,event);
                sprintf(protocolComps.mqtt_pub_topic,"custome1");
   // 			protocolComps.msg;//=protocolMisc.buf;	
    		    protocolComps.sw._bit.DataRdy=1; //send a message to bc35 send
    		    protocolComps.AckTmr=10;
                protocolMisc.step++;
            }
         }
	}
    else if(protocolMisc.step==1)//Encapsulate the has more
	{
        #if(0) 
           if(device_comps.TimeSegData.nums>0) 
           {
                if(!protocolComps.sw._bit.DataRdy)
                {
                    memset(protocolMisc.buf,0,sizeof(protocolMisc.buf));
        			protocolComps.msgLen=Encapsulate_has_more_dataPush_packag((char *)protocolMisc.buf,event);
       // 			protocolComps.msg;//=protocolMisc.buf;	
        		    protocolComps.sw._bit.DataRdy=1; //send a message to bc35 send
        		    protocolComps.AckTmr=10;
                    protocolMisc.step++;
                }
           }
           else
           {
                protocolMisc.step++;
           }
         #else
            protocolMisc.step++;
         #endif
	}
	else if(protocolMisc.step==2)//
	{
		if((!protocolComps.sw._bit.DataRdy)&&(netComps.St._bit.recvData))//////////////////
		{
			char address_field;
			netComps.St._bit.recvData=0;//TODO clr buf
			memset(protocolMisc.buf,0,sizeof(protocolMisc.buf));
			memcpy(protocolMisc.buf,netComps.msg,netComps.msgLen);
			protocolComps.msgLen=netComps.msgLen;
			if(!Analysis_downstream_package((char *)protocolMisc.buf,protocolComps.msgLen,&address_field))//Remove the link layer protocol (68, len crc etc)
			{
                 //protocolComps.AckTmr+=1;
                 DealDownCmd((char *)protocolMisc.buf,protocolComps.msgLen);
                  
            }
			else
			{
				//TODO ....
			}
		}
		else
		{
			if(!protocolComps.AckTmr)
			{
                if(!netComps.St._bit.push_data_ok)
                {
    				if(protocolMisc.reRryTimes>0)
    				{
    					protocolMisc.reRryTimes--;
    					
    					if(protocolMisc.reRryTimes==0)
    					{
    						protocolComps.sw._bit.AckTmrOut=1;
    						protocolMisc.step++;
    					}
    					else
    					{
    						protocolMisc.step=0;
    					}
    				}
                    else
                    {
                        protocolComps.sw._bit.AckTmrOut=1;
                        protocolMisc.step++;
                    }   
				}
				else
				{
                    protocolComps.sw._bit.AckTmrOut=1;
                    protocolMisc.step++;
				}
				
				if(protocolComps.sw._bit.AckTmrOut)
				{
                     protocolComps.sw._bit.isAckTmrOut=1;
				}
		    }
		}
	}
	else if(protocolMisc.step==3)
	{
        
	}

    if(netComps.St._bit.noResponse||netComps.St._bit.windowTimeOut||netComps.St._bit.err||netComps.St._bit.noIP
	   ||protocolComps.sw._bit.cmd_shutDown||protocolComps.sw._bit.AckTmrOut)
	
	{
        netComps.St._bit.off=1;
        
        netComps.St._bit.noResponse=0;
        netComps.St._bit.windowTimeOut=0;
        netComps.St._bit.err=0;
        netComps.St._bit.noIP=0;
        protocolComps.sw._bit.cmd_shutDown=0;
        protocolComps.sw._bit.AckTmrOut=0;
        netComps.op_window_tmr=0;
        
     }
	
}

static void protocolComps_report_callback(void)
{
   #if(defined (MD_EXT_COLLECTOR) && MD_EXT_COLLECTOR_TYPE==MD_PRESS)
      if(!collectorComps.sw._bit.isGetExternPressReq)
       {
            collectorComps.sw._bit.isGetExternPressReq=1;
       }
   #endif 
	
}

static int16_t wait_for_signal(void)
{
  #if(defined (MD_EXT_COLLECTOR) && MD_EXT_COLLECTOR_TYPE==MD_PRESS)
    if(collectorComps.sw._bit.isGetExternPressReq)
	  {
		    return 0;
	  } 
  #endif
   return 1;
}

    
static void protocolComps_task_handle(void)
{
    int16_t wait=0;
//    if(modbusComps.sw._bit.runing && !netComps.St._bit.running)
//    {
//        return;
//    }
    if(device_comps.sw._bit.isBatBluntNow)
    {
        return;
    }
    if(protocolMisc.event_index<0)
	{
		int16_t i=0;
		for(i=0;i<sizeof(protocolComps.triggerIrq.All)*8;i++)
		{
			if((protocolComps.triggerIrq.All>>i)&1)
			{
				protocolMisc.event_index=i;
				if(!protocolMisc.event_index)
				{
                    device_comps.gps.sw._bit.isActive=1;
				}
                protocolMisc.step=0;
				protocolMisc.reRryTimes=1;
                protocolComps.sw._bit.DataRdy=0;
	            protocolComps.sw._bit.dataPushYet=0;
                protocolComps.sw._bit.dataPushYet1=0;
                protocolComps.sw._bit.isAckTmrOut=0;
                protocolComps.triggerIrq.All&=~((uint16_t)1<<protocolMisc.event_index);
              
                protocolComps_report_callback();
                netComps.disCode=EM_DIS_ACT;
                hum_comps.enter_report_mode();
                break;
			}
		}
	}
	
    if(protocolMisc.event_index>-1)
	{
		if(!netComps.St._bit.running)
		{
           
            wait=wait_for_signal();
            if(!wait)
            {
                return;
            }
            netComps.St._bit.on=1;
			adx_comps.stop();
        }
		else
		{
 
			Srv_Protol(protocolMisc.event_index);
		}
    }
}


protocolComps_t protocolComps=
{
    {0},
    {0},
    (char *)&protocolMisc.buf,
    0,
    "custome1",
    
    0,
    &protocolMisc.step,
    &protocolMisc.event_index,//uint16_t * const event_index_pt;
    protocolComps_task_handle
};

