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
#include "ertc.h"


#define GSMPWRCTL_pin	      GetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_GSM_POWER_PORT),  MD_GSM_POWER_PIN)
#define GSMPWRCTL_pin_high    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_GSM_POWER_PORT),  MD_GSM_POWER_PIN, TRUE)
#define GSMPWRCTL_pin_low     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_GSM_POWER_PORT),  MD_GSM_POWER_PIN,FALSE)
#define GSMRST_pin_high       SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_GSM_RST_PORT)  ,  MD_GSM_RST_PIN  , TRUE)
#define GSMRST_pin_low        SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_GSM_RST_PORT)  ,  MD_GSM_RST_PIN  ,FALSE)
#define GSMON_pin_high	      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_GSM_ON_PORT)   ,  MD_GSM_ON_PIN   , TRUE)
#define GSMON_pin_low         SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_GSM_ON_PORT)   ,  MD_GSM_ON_PIN   ,FALSE)

#define   MD_NET_DEVICE_RESOLVE_IP_STEP   27
#define   MD_NET_DEVICE_SEND_STEP         31
#define   MD_NET_DEVICE_DELAY_STEP        50
#define   MD_NET_DEVICE_LSB_STEP          120
#define   MD_NET_DEVICE_MQTT_STEP         150

#define   MD_NET_DEVICE_NULL_STEP         255

#define   MD_MODULE_OP_MAX_TIME          180


static void enable_net_com(void)
{
    enable_uart1();
   
}

static void disable_net_com(void)
{
    disable_uart1();
    
}

static uint32_t config_net_com(uint32_t baud, int16_t parity)
{
    disable_net_com();
    App_Uart1Cfg(baud, parity);
    return baud;
}



static struct 
{ 
    union 
    {
    	uint8_t All;
    	struct
    	{
    		uint8_t RegestCSNet     :1;
    		uint8_t RegestPSGNet    :1;
    		uint8_t RegestPSENet  	:1;
    		uint8_t res1	            :1;
    		uint8_t res2		        :1;
    		uint8_t res3	            :1;
    	   
    		
    	}_bit;
    } St;
    
    int16_t NoAckTimes;
    uint8_t  AT_Waiting;	//AT?¨¹¨¢?¡¤¡é?¨ª¡À¨º??//
    uint8_t  Flag_St;
    uint8_t _Flag_St;
    uint16_t   reStartTimes;
    uint8_t  NoRegestPSGNetCount;
    uint8_t  Reset_St ;
    uint8_t  TurnOFF_St;
    uint8_t *recv_RxSt;
    
	uint8_t ErrCode;
	uint8_t err_count;
    uint8_t TX_ATCombuffer[128]; 
	uint8_t send_buf[1024];
	uint16_t  send_len;
	uint8_t recv_buf[1024];
	uint16_t  recv_Idx;
	uint8_t recv_msg_buf[1024];//send to protocol layer
	
	
    uint8_t          pdp_id;
    uint8_t NoRegestPSENetCount;
    uint8_t  NoRegestCSNetCount;
	uint8_t  NoGetGpsLocCount;
	
	MD_STATUS  (*write)(uint8_t * const tx_buf, uint16_t tx_num);
	//uint8_t * (*check_net_device_ack)(uint8_t *FindStr);  
	
}
netMisc=
{
    {0},

	0,
	0,
	0,
	0,
	2,
	0,
	0,
	0,
	0,
	
	3,
	0,
	{0},
	{0},
	0,
	{0},
	 0,
	{0},
	
	1,
    0,
	0,
	0,	
	R_UART1_Send
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


static void init_ENV_net_device(void)
{
    netMisc.AT_Waiting=0;

    netMisc.NoRegestCSNetCount=0;
    netMisc.NoRegestPSGNetCount=0;
    netMisc.NoRegestPSENetCount=0;
    netMisc.err_count=0;

    netComps.St._bit.socket_connected=0;
   
    netComps.St._bit.ResolvedIP=0;
    netComps.St._bit.noIP=0;

    netComps.disCode=EM_DIS_ACT;
}
    
static void cold_start_net_device(void)	
{
    netMisc.Flag_St = 0;
    netMisc.Reset_St=0;//turn on moudle //
    init_ENV_net_device();
}  
   
static void hot_start_net_device(void)	
{
    netMisc.Flag_St = 0;
    netMisc.Reset_St=2;//
    init_ENV_net_device();
}    

static void stop_net_device(void)
{
    netMisc.Flag_St = 1;
    netMisc.TurnOFF_St=0;//turn on moudle //
    netMisc.AT_Waiting=0;
}

static void on_net_device_ack_ok(void)
{
       netMisc.AT_Waiting=0;
       netMisc.NoAckTimes=0;
       netMisc._Flag_St=netMisc.Flag_St;
       netMisc.Flag_St++;

       netMisc.err_count=0;
}

static void net_status_sjmp_any_step(uint8_t Flag_St)
{
        netMisc.AT_Waiting=0;
        netMisc.NoAckTimes=0;
        netMisc._Flag_St=netMisc.Flag_St;
        netMisc.Flag_St=Flag_St; 
}
 
static void reset_net_deive_rcvbuf(void)
{
    netMisc.recv_Idx = 0;
    netMisc.recv_RxSt=&netMisc.recv_buf[0];
    memset(&netMisc.recv_buf[0],0,sizeof(netMisc.recv_buf));
}


static void DealAnalyticCode(int16_t err)//err=0;err£¬err=1,recv ok £¬err=3,no data
{
    if(err==0)//
    {
        net_status_sjmp_any_step(MD_NET_DEVICE_SEND_STEP);//send data
    }

    else if(err==1)//err=1,recv data ok
    {
    	net_status_sjmp_any_step(MD_NET_DEVICE_SEND_STEP);//send data
    }
    else if(err==3)//,err=3,no data
    {
    	net_status_sjmp_any_step(MD_NET_DEVICE_SEND_STEP);//send data
    }
    else
    {
    	net_status_sjmp_any_step(MD_NET_DEVICE_SEND_STEP);//send data
    }

}
	
//void HexSrtToHex(uint8_t *des,uint8_t const *src,uint16_t Length)
//{
//	uint16_t  Count=0;
//	uint16_t  i=0;
//	uint8_t Temp_H=0;
//	uint8_t Temp_L=0;
//
//	while(Count<Length)
//	{
//		if( src[i]<=0x39)
//		{
//			//src[i]-=0x30;
//			Temp_H=src[i]-0x30;
//		}
//		else 
//		{
//			//src[i]-=0x37;
//			Temp_H=src[i]-0x37;
//		}
//
//		if( src[i+1]<=0x39)	
//		{	
//			//src[i+1]-=0x30;
//			Temp_L=src[i+1]-0x30;
//			
//		}
//		else 
//		{
//			//src[i+1]-=0x37;
//			Temp_L=src[i+1]-0x37;
//		}
//		//des[Count++]=((src[i]<<4) |src[i+1]);
//		des[Count++]=((Temp_H<<4) |Temp_L);
//		i+=2;
//	}
//
//}

static void HandleNoAck(void)
{
	if(netMisc.NoAckTimes<1)
	{
		netMisc.AT_Waiting =0;
		netMisc.NoAckTimes++;
    }
	else 
	{
        netComps.St._bit.noResponse=1;
        net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
    }
}


static void   HandleErr(void)
{
    
    if(netMisc.err_count<1)
    {
        netMisc.err_count++;
        net_status_sjmp_any_step(MD_NET_DEVICE_DELAY_STEP);//delay function
    }
    else 
    {
        if(netMisc.reStartTimes>0)
        {
            hot_start_net_device();
            netMisc.reStartTimes--;
        }
        else 
        {
            netComps.St._bit.err=1;
            net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
        }
    }
}

static void load_ip(void)
{
    memset(netComps.net_info.currentIP,0,sizeof(netComps.net_info.currentIP));
    memcpy(netComps.net_info.currentIP,device_comps.access_param.ip,sizeof(device_comps.access_param.ip));
    if(strlen(netComps.net_info.currentIP)>6)
    {
        netComps.net_info.currentIP_No=EM_IP0;
        netComps.net_info.currentPort=device_comps.access_param.port;
        
    }
    else 
    {
        memset(netComps.net_info.currentIP,0,sizeof(netComps.net_info.currentIP));
        memcpy(netComps.net_info.currentIP,device_comps.access_param.ip1,sizeof(device_comps.access_param.ip1));
       if(strlen(netComps.net_info.currentIP)>6 &&device_comps.access_param.port1>0 )
     
        {
            netComps.net_info.currentIP_No=EM_IP1;
            netComps.net_info.currentPort=device_comps.access_param.port1;
        }
        else
        {
            netComps.net_info.currentIP_No=EM_NO_IP;
        }
     }
   
}
		
static void switch_ip_push_data_to_ip1(void)
{
    memset(netComps.net_info.currentIP,0,sizeof(netComps.net_info.currentIP));
    memcpy(netComps.net_info.currentIP,device_comps.access_param.ip1,sizeof(device_comps.access_param.ip1));
    if(strlen(netComps.net_info.currentIP)>6 &&device_comps.access_param.port1>0 )
    {
        netComps.net_info.currentPort=device_comps.access_param.port1;
        netComps.net_info.currentIP_No=EM_IP1;
         netComps.St._bit.ResolvedIP=0;
        net_status_sjmp_any_step(MD_NET_DEVICE_RESOLVE_IP_STEP);
        netComps.St._bit.push_data_ok=0;
        *protocolComps.step=0;
        netComps.St._bit.allow_data_send=0;
     }
     else
     {
        netComps.St._bit.err=1;
        net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
     }
      
}

static void check_ip_switch(void)
{
    #if defined (MD_ALT_IP)||defined(MD_PUSH_DATA_TO_IP1) 

          if( netComps.net_info.currentIP_No== EM_IP0)
          { 
                if(netComps.St._bit.push_data_ok)
                {
                     protocolComps.sw._bit.dataPushYet=1;
                #if defined(MD_PUSH_DATA_TO_IP1)
                        switch_ip_push_data_to_ip1();
                #else
                       netComps.St._bit.err=1;
                       net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);

                #endif
                }
                else
                {
                       protocolComps.sw._bit.dataPushYet=0;
                       switch_ip_push_data_to_ip1();
                }     
          }
          else if(netComps.net_info.currentIP_No==EM_IP1)
          {
                if(netComps.St._bit.push_data_ok)
                {
                       protocolComps.sw._bit.dataPushYet1=1;
                }
                netComps.St._bit.err=1;
                net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
          }
          else
          {
                protocolComps.sw._bit.dataPushYet=0;
                protocolComps.sw._bit.dataPushYet1=0;
                netComps.St._bit.err=1;
                net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
          }
    #else
            if(netComps.St._bit.push_data_ok)
            {
                   if( netComps.net_info.currentIP_No==EM_IP0)
                   {
                        protocolComps.sw._bit.dataPushYet=1;
                   }
                   else if( netComps.net_info.currentIP_No==EM_IP1)
                   {
                         protocolComps.sw._bit.dataPushYet=1;
                   }
                   else
                   {
                        protocolComps.sw._bit.dataPushYet=0;
                        protocolComps.sw._bit.dataPushYet1=0;
                   }
            }
            netComps.St._bit.err=1;
            net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
         
    #endif  
}

static void HandleDataErrIP(void)
{
    if(netMisc.err_count<1)
    {
        netMisc.err_count++;
        net_status_sjmp_any_step(MD_NET_DEVICE_DELAY_STEP);//delay function
    }
    else 
    {
        check_ip_switch();  
    }

}

static void   HandleRegTimeOut(void)
{
    if(netMisc.reStartTimes>0)
    {
        hot_start_net_device();
        netMisc.reStartTimes--;
    }
    else 
    {
        netComps.St._bit.err=1;
        net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
    }
}


static void TX_ATData(uint8_t  *ptr,uint16_t WaitingTime)
{
    netComps.AckTmr = WaitingTime;
	netMisc.AT_Waiting = 1;
	netMisc.write(ptr,netMisc.send_len);
}

static void TX_ATCommand(char  *ptr,int16_t WaitingTime)
{
	uint8_t TxLen = 0;
    while(*ptr!='\0')
    {
		netMisc.TX_ATCombuffer[TxLen] = *ptr++;
		TxLen++;
	}
    netComps.AckTmr = WaitingTime;
	netMisc.AT_Waiting = 1;
	netMisc.write(&netMisc.TX_ATCombuffer[0], TxLen);
}

static char * check_net_device_ack(char const *FindStr)  
{
    uint16_t FindstrLen;
    uint8_t *StartPt,*EndPt;
    void *rslt=(void *)0;
    FindstrLen =strlen(FindStr);
    StartPt = (uint8_t *)&netMisc.recv_buf[0];
    EndPt =(uint8_t *) netMisc.recv_RxSt;
    while(EndPt-StartPt>=FindstrLen)
    {
    	if(!memcmp(FindStr,StartPt,FindstrLen))
    	 {
    		 rslt = StartPt+FindstrLen;
    		 break;
    	 } 
    	 StartPt++;
    }
    return (char *)(rslt);
}





//static void GetNuestats()
//{
//	uint8_t i=0;
//	uint8_t *pt_rssi;
//	uint8_t  buf[12]="";
//	if(!netMisc.AT_Waiting)
//	{
//		reset_net_deive_rcvbuf();
//		//TX_ATCommand("\r\nAT+COPS?\r\n",60); 
//		TX_ATCommand("\r\nAT+NUESTATS\r\n",3); 
//	}
//	else if(!netComps.AckTmr)
//	{
//		HandleNoAck();   	
//	} 
//	else if(check_net_device_ack("OK\r\n"))
//	{
//		 if(pt_rssi=check_net_device_ack("ECL:"))
//		 {
//			netComps.net_info.ecl=*pt_rssi -0x30;
//		 }
//		
//		 if(pt_rssi=check_net_device_ack("SNR:"))
//		 {
//			memset(buf,0,sizeof(buf));
//		 	for(i=0;i<sizeof(buf);i++)
//			{
//				if(*pt_rssi!='\r')
//				{
//				   buf[i]=*pt_rssi++;
//				}
//				else
//				{
//					break;
//				}
//			}
//			netComps.net_info.sina=atol(buf)/10<-128?-128:atol(buf)/10;
//		 }
//		 
//		 if(pt_rssi=check_net_device_ack("RSRQ:"))
//		 {
//			memset(buf,0,sizeof(buf));
//	        for(i=0;i<sizeof(buf);i++)
//		    {
//				if(*pt_rssi!='\r')
//				{
//				    buf[i]=*pt_rssi++;
//				}
//				else
//				{
//					break;
//				}
//	        } 
//	        netComps.net_info.rsrq=atol(buf)/10<-128?-128:atol(buf)/10;
//			
//		 }
//		 if(pt_rssi=check_net_device_ack("Signal power:"))
//		 {
//			memset(buf,0,sizeof(buf));
//		 	for(i=0;i<sizeof(buf);i++)
//			{
//				if(*pt_rssi!='\r')
//				{
//				   buf[i]=*pt_rssi++;
//				}
//				else
//				{
//					break;
//				}
//			} 
//			netComps.net_info.rsrp=atol(buf)/10<atol(buf)/10<-128?-128:atol(buf)/10;
//		 }
//		 
//		if(pt_rssi=check_net_device_ack("PCI:"))
//		{
//            memset(buf,0,sizeof(buf));
//            for(i=0;i<sizeof(buf);i++)
//            {
//                if(*pt_rssi!='\r')
//                {
//                  buf[i]=*pt_rssi++;
//                }
//                else
//                {
//                   break;
//                }
//            } 
//            netComps.net_info.pci=atol(buf);
//        }
//		 if(pt_rssi=check_net_device_ack("Cell ID:"))
//		 {
//			memset(buf,0,sizeof(buf));
//			//int16_t k=5;
//		 	//memset(netComps.net_info.cellId,0,sizeof(netComps.net_info.cellId));
//			for(i=0;i<sizeof(buf);i++)
//			{
//				if(*pt_rssi!='\r')
//				{
//                    buf[i]=*pt_rssi++;
//                }
//				else
//				{
//					break;
//				}
//			} 
//			netComps.net_info.cellId=atol(buf);
//			/*
//			while(i!=0)
//			{
//				if(i>1)
//				{
//					netComps.net_info.cellId[k]= buf[i-1]+(buf[i-2]<<4);
//					k--;
//					i-=2;
//				}
//				else if(i>0)
//				{
//					netComps.net_info.cellId[k--]= buf[i-1];
//					i--;
//				}
//			}
//			NOP();
//			*/
//			
//		 }
//		 if(pt_rssi=check_net_device_ack("EARFCN:"))
//		 {
//			memset(buf,0,sizeof(buf));
//		 	for(i=0;i<sizeof(buf);i++)
//			{
//				if(*pt_rssi!='\r')
//				{
//				   buf[i]=*pt_rssi++;
//				}
//				else
//				{
//					break;
//				}
//			} 
//			netComps.net_info.earFcn=atol(buf);
//		 }
//		 on_net_device_ack_ok();
//	}
//}

//static void DelayMs(int16_t TimeMs){
//	int16_t j;
//	uint8_t i;
//	for(j=0;j<TimeMs*10;j++)
//	{		
//		//i=7;	//10us at 8Mhz
//		i=97;	//100us at 8Mhz
//		while(i>0) i--;
//	}
//}





uint8_t PacketCoapMsg(uint8_t const *buf,int16_t len)
{
    uint8_t ret=0;
//    uint16_t  i;
//    uint8_t Var[8]="";
      memset(netMisc.send_buf,0,sizeof(netMisc.send_buf));
//    sprintf(netMisc.send_buf,"%s","\r\nAT+NMGS=");
//    i=strlen(netMisc.send_buf);
//    sprintf(Var,"%d",(int)len);
//    memcpy(&netMisc.send_buf[i],Var,strlen(Var));
//    i+=strlen(Var);
//    netMisc.send_buf[i++]=',';
   // DataAdd30H(&netMisc.send_buf[i],buf,len);
//    i+=len*2;
//    memcpy(&netMisc.send_buf[i++],"\r\n",2);//ADD \r\n 
//    ret=1;
      netMisc.send_len=len;
      memcpy(&netMisc.send_buf,buf,len); 

    return ret;
}              

//static void GetHexMsg(uint8_t *des,uint8_t const*src,uint16_t Length)
//{
//	//HexSrtToHex(des,src,Length);
//}

static uint8_t GetMsg(char const *buf,int16_t len)
{
	uint8_t err=0;
	
	memset(netMisc.recv_msg_buf,0,sizeof(netMisc.recv_msg_buf));	
	//GetHexMsg(netMisc.recv_msg_buf,buf,len);
	memcpy(netMisc.recv_msg_buf,buf,len);
//	netComps.msg;//=netMisc.recv_msg_buf;
	netComps.msgLen=len;
	netComps.St._bit.recvData=1;
	err=1;
    return err;////err=0;´íÎó£¬err=1,recv data ok;
}              

static void power_on_net_device(void)
{
   GSMPWRCTL_pin_high;// = 1;
   GSMRST_pin_high;//=1;
  delay1ms(30);
   GSMON_pin_high;// = 1;   
   delay1ms(25);
   GSMRST_pin_low;//=0;
}


static void reset_net_device(void)
{
	switch(netMisc.Reset_St)
	{
		case 0:
			power_on_net_device();	
		    reset_net_deive_rcvbuf();
			netMisc.Reset_St++;
			netComps.AckTmr=30;
			break;
			
		case 1:
		    if(check_net_device_ack("RDY\r\n"))
		    {
                GSMON_pin_low;// = 0;
                netMisc.Reset_St=0;
                net_status_sjmp_any_step(3);
            }
            else if(!netComps.AckTmr)
			{
                GSMON_pin_low;// = 0;
                netMisc.Reset_St=0;
			    net_status_sjmp_any_step(3);
			}
			break;

			
		case 2:
                //TX_ATCommand("\r\nAT+CFUN=1,1\r\n",30);
		        GSMRST_pin_high;//=1;
		        netComps.AckTmr = 3;
                netMisc.Reset_St++;
                break;
        case 3:
            if(!netComps.AckTmr)
			{
                GSMRST_pin_low;//=0;
                netMisc.Reset_St=1;
                reset_net_deive_rcvbuf();
                netComps.AckTmr=30;
            }
   }
}


void power_down_net_deivice(void)
{   
    
     switch(netMisc.TurnOFF_St)
     { 
          case 0: 
        	   	if(!netMisc.AT_Waiting)
    			{
    				reset_net_deive_rcvbuf();
                   #if(MD_IOT_PROTOCOL==MD_IOT_TCP)
    		        TX_ATCommand("\r\nAT+QICLOSE=0\r\n",10);
                   #elif(MD_IOT_PROTOCOL==MD_IOT_MQTT)
                    TX_ATCommand("\r\nAT+QMTDISC=0\r\n",2);
                   #else
                    netComps.AckTmr=1;
                   #endif
                } 
    			else if(!netComps.AckTmr)
    			{
    		         netMisc.TurnOFF_St++;
    				 netMisc.AT_Waiting=0;
    			}
    			else if(check_net_device_ack("OK\r\n") || check_net_device_ack("ERROR\r\n"))
    			{
    				 netMisc.TurnOFF_St++;
    				 netMisc.AT_Waiting=0;
    			}
                break;
    			
           case 1: 
        	   	if(!netMisc.AT_Waiting)
    			{
    				reset_net_deive_rcvbuf();
    		        TX_ATCommand("\r\nAT+QIDEACT=1\r\n",10);
                } 
    			else if(!netComps.AckTmr)
    			{
    		         netMisc.TurnOFF_St++;
    				 netMisc.AT_Waiting=0;
    			}
    			else if(check_net_device_ack("OK\r\n"))
    			{
    				 netMisc.TurnOFF_St++;
    				 netMisc.AT_Waiting=0;
    			}	
    			break;
    			
            case 2: 
        	   	if(!netMisc.AT_Waiting)
    			{
    				reset_net_deive_rcvbuf();
    		        //TX_ATCommand("\r\nAT+QPOWD\r\n",3);
			netMisc.AT_Waiting=1;
                    GSMON_pin_high;// = 1;
                    netComps.AckTmr = 3;
                } 
    			else if(!netComps.AckTmr)
    			{
                     GSMON_pin_low;// = 0;
                     netMisc.TurnOFF_St++;
    				 netMisc.AT_Waiting=0;
    			}
//    			else if(check_net_device_ack("OK\r\n"))
//    			{
//                     netMisc.TurnOFF_St++;
//    				 netMisc.AT_Waiting=0;
//    			}	
    			break; 
    			
            case 3: 
                if(!netMisc.AT_Waiting) 
                {
                    netComps.AckTmr =5;	
                    netMisc.AT_Waiting = 1;
                }
                else if(!netComps.AckTmr)
                {
                    netMisc.AT_Waiting = 0;
                    //memset(&netComps.net_info,0,sizeof(netComps.net_info));
                    netComps.net_info.Csq=99;
            		netComps.net_info.CsQ=99;
                    GSMPWRCTL_pin_low;// =0;
                    netComps.St._bit.running=0;
            		netComps.St._bit.offing=0;
            		*protocolComps.event_index_pt=-1;
            		disable_net_com();
            		hum_comps.enter_default_mode(0);
                 device_comps.net_power_off_call_back();
                    
              }	
               break;
            default: break;
     }
}


static void GetRssi(void)
{
	
	char *pt_rssi;
	if(!netMisc.AT_Waiting)
	{
		reset_net_deive_rcvbuf();
		TX_ATCommand("\r\nAT+CSQ\r\n",10);
	} 
	else if(!netComps.AckTmr)
	{
		HandleNoAck(); 
	}
	else if(check_net_device_ack("OK\r\n"))
	{
		 pt_rssi=check_net_device_ack("+CSQ: ");
	      if(pt_rssi)
	      {
                char *ptsz;
                netComps.net_info.Csq=strtol(pt_rssi,&ptsz,10);
                netComps.net_info.CsQ=strtol(ptsz+1,&ptsz,10);
    	  }
	      on_net_device_ack_ok(); 
	}  
}

//static uint8_t GMTtoLocalTime(uint8_t *tbuf)//yy mm dd hh mm ss '+' zz
//{
//	uint16_t buf[8];
//	uint16_t i=0;
//	uint8_t  monthlengths[]={31,28,31,30,31,30,31,31,30,31,30,31};
//   #define  ADJFACTOR  1
//   	for(i=0;i<8;i++)
//	{
//		if(i==0)
//		{
//			buf[i]=2000+tbuf[i];
//			if(((buf[i]%4==0) && (buf[i]%100!=0)) || (buf[i]%400==0))
//			{
//				 monthlengths[2-ADJFACTOR]=29;
//			}
//		}
//		buf[i]=tbuf[i];
//	}
//	buf[7]=(uint16_t)buf[7]*15/60;//15min
//	if(buf[6]=='+')
//	{
//		buf[3]+=buf[7];//timeZone
//		if(buf[3]<24)
//		{
//			
//		}
//		else
//		{
//			buf[3]%=24;//h
//			buf[2]++;//dd
//			if(buf[2]>monthlengths[buf[1]-ADJFACTOR])
//			{
//				buf[2]%=monthlengths[buf[1]-ADJFACTOR];
//			        buf[1]++;//mm
//				if(buf[1]>12)
//				{
//					buf[1]%=12;
//					buf[0]++;//yy++
//				}
//			}
//		}
//	}
//	else if(tbuf[6]=='-')
//	{
//		if(buf[3]>=buf[7])
//		{
//			buf[3]-=buf[7];
//		}
//		else
//		{
//			buf[3]=buf[3]+24-buf[7];//h
//		        buf[2]--;//dd
//			if(!buf[2])
//			{
//				buf[1]--;//mm
//			        buf[2]=monthlengths[buf[1]-ADJFACTOR];
//			        
//				if(!buf[1])
//				{
//				    buf[0]--;//yy--
//				    buf[1]=1;
//				}
//			}
//		}
//	}
//	for(i=0;i<6;i++)
//	{
//		tbuf[i]=((buf[i]%100/10)<<4)+(buf[i]%10);
//	}
//	return 1;
//}



static char *mqtt_config_param[]={"\r\nAT+QMTCFG=\"dataformat\",0,0,0\r\n",
                                  "\r\nAT+QMTCFG=\"recv/mode\",0,1,1\r\n",
                                //"\r\nAT+QMTCFG=\"version\",0,4\r\n",
                                  "\r\nAT+QMTCFG=\"ssl\",0,0\r\n",
                                  "\r\nAT+QMTCFG=\"pdpcid\",0,1\r\n"};
static uint8_t mqtt_config_cnt=0;

static uint32_t mqtt_msg_id=1;
union
{
       struct
      {
           uint8_t b0:         1;
           uint8_t b1:         1;
           uint8_t b2:         1;
           uint8_t b3:         1;
           uint8_t b4:         1;
      }_bit;
      uint8_t all; 
}mqtt_rcv_msg_mask={0};


static void Srv_GSM(void)
{    
    char *pt_rssi;
    uint8_t i;
   
	
	switch(netMisc.Flag_St)
	{
        case 0:
			reset_net_device();		
			break;


         case 1:
			power_down_net_deivice();		
			break;

       	
		case 3: 
			if(!netMisc.AT_Waiting) 
			{
				netComps.AckTmr =2;	
				netMisc.AT_Waiting = 1;
			}
			else if(!netComps.AckTmr)
			{
 				 on_net_device_ack_ok();
			}
			break;
			
		case 4:	
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
		       TX_ATCommand("\r\nATE0\r\n",10);
		       //TX_ATCommand("\r\nATE1\r\n",10);
            } 
			else if(!netComps.AckTmr)
			{
				  HandleNoAck(); 
			}
			else if(check_net_device_ack("OK\r\n"))
			{
               if (device_comps.gps.sel==1)
               {
    			     if(device_comps.gps.sw._bit.isActive)
    				 {
                        device_comps.gps.sw._bit.isActive=0;
                        net_status_sjmp_any_step(5);
    				 }
    				 else
    				 {
                         net_status_sjmp_any_step(11);
    				 }
               }
			   else
			   {
    				device_comps.gps.sw._bit.isActive=0;
    				net_status_sjmp_any_step(11);
               }
				
			}	
			break;

		/****************start get gps data********************/
         

         case 5:	
    		if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+CFUN=0\r\n",30);
            } 
    		else if(!netComps.AckTmr)
    		{
    			  HandleNoAck(); 
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
    			 on_net_device_ack_ok();
    		}	
    		break;

		 case 6:	
    		if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+QGPS=1\r\n",10);
            } 
    		else if(!netComps.AckTmr)
    		{
    			  HandleNoAck(); 
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
    			 on_net_device_ack_ok();
				 netMisc.NoGetGpsLocCount=0;
    			 device_comps.gps.sw._bit.isLocSuc=0;
    			 netComps.disCode=EM_DIS_GPS_STATUS;
    		}	
    		break;
    		
         case 7: 
			if(!netMisc.AT_Waiting) 
			{
				netComps.AckTmr =10;	
				netMisc.AT_Waiting = 1;
			}
			else if(!netComps.AckTmr)
			{
 				 on_net_device_ack_ok();
			}
			break;
    	 case 8:	
    		if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+QGPSLOC=2\r\n4",10);
            } 
    		else if(!netComps.AckTmr)
    		{
    			  //HandleNoAck(); 
    			    netMisc.NoGetGpsLocCount++;
                    if(netMisc.NoGetGpsLocCount>15)
                    {
                        on_net_device_ack_ok();
                        netMisc.NoGetGpsLocCount=0;
                        netComps.op_window_tmr+=100;
                        
                    }
                    else
                    {
                        net_status_sjmp_any_step(7);
                    }
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
                 pt_rssi=check_net_device_ack(",");
				 if(pt_rssi)
    			 {
    			       char loc[32]="";
   			           uint8_t num=0;
    			       for(i=0;i<sizeof(loc);i++)
                        {
                            if(*pt_rssi!=',')
                            {
                        	  loc[i]=*pt_rssi;
                              pt_rssi++;
                            }
                            else
                            {   num++;
                                if(num>=2)
                                {
                                    break;
                                }
                                loc[i]=*pt_rssi;
                                pt_rssi++;
                                
                            }
                        }
    			      if(device_comps.get_gps_info_from_net(loc))
                      {
                            netMisc.NoGetGpsLocCount++;
                            if(netMisc.NoGetGpsLocCount>30)
                            {
                                on_net_device_ack_ok();
                                netMisc.NoGetGpsLocCount=0;
                                netComps.op_window_tmr+=100;
                                
                            }
                            else
                            {
                                net_status_sjmp_any_step(7);
                            }
                       }
                      else
                      {
                            
                           device_comps.gps.sw._bit.isLocSuc=1;
                           on_net_device_ack_ok();
                           netComps.op_window_tmr=150;
                          
                      }
                 }
    			 else
    			 {
                      HandleNoAck(); 
    			 }
            }	
    		break;	
    	 case 9:	
    		if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+QGPSEND\r\n",10);
            } 
    		else if(!netComps.AckTmr)
    		{
    			  HandleNoAck(); 
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
    			 on_net_device_ack_ok();
    		}	
    		break;
    		
    	 
		case 10:	
    		if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+CFUN=1\r\n",30);
            } 
    		else if(!netComps.AckTmr)
    		{
    			  HandleNoAck(); 
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
    			  on_net_device_ack_ok();
    		}	
    		break;
            
          /****************end get gps data********************/ 
            
   case 11: 
			if(!netMisc.AT_Waiting) 
			{
				netComps.AckTmr =2;
				netMisc.AT_Waiting = 1;
			}
			else if(!netComps.AckTmr)
			{
 				 on_net_device_ack_ok();
			}
            break;
       
   case 12:	
			if(!netMisc.AT_Waiting)
			{
                reset_net_deive_rcvbuf();
                TX_ATCommand("\r\nATI\r\n",10);//Ver
			}  
			else if(!netComps.AckTmr)
			{
				HandleNoAck(); 
			}
			else if(check_net_device_ack("OK\r\n"))
			{
				 pt_rssi=check_net_device_ack("Revision: ");
				 if(pt_rssi)
				 {
                    memset(netComps.net_info.firmVer,0,sizeof(netComps.net_info.firmVer));
                    for(i=0;i<sizeof(netComps.net_info.firmVer);i++)
                    {
                        if(*pt_rssi!='\r')
                        {
                    	  netComps.net_info.firmVer[i]=*pt_rssi;
                    	  netComps.net_info.firmhdVer[i]=*pt_rssi;
                          pt_rssi++;
                        }
                        else
                        {
                           break; 	
                        }
                    }
                 }
                 on_net_device_ack_ok();
			}	
			break;
			
  case 13: 
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
			    TX_ATCommand("\r\nAT+CGSN\r\n",10);//IMEI
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck();  
			}
			else if(check_net_device_ack("OK\r\n"))
			{
				pt_rssi=check_net_device_ack("\r\n");
				if(pt_rssi)
				 {
					memset(netComps.net_info.imei,0,sizeof(netComps.net_info.imei));
			        for(i=0;i<sizeof(netComps.net_info.imei);i++)
				    {
						if(*pt_rssi!='\r')
						{
						    netComps.net_info.imei[i]=*pt_rssi++;
						}
						else
						{
							break;
						}
				    } 
				 }
				 on_net_device_ack_ok();
			}	
            break;
            
		case 14:  	
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
				TX_ATCommand("\r\nAT+ICCID\r\n",10); 
			}
			else if(!netComps.AckTmr)
			{
                HandleNoAck(); 
            } 
			else if(check_net_device_ack("ERROR"))
			{
                HandleErr();
            }
			else if(check_net_device_ack("OK\r\n"))
			{
				pt_rssi=check_net_device_ack("+ICCID: ");
				if(pt_rssi)
				 {
					 memset(netComps.net_info.iccid,0,sizeof(netComps.net_info.iccid));
				     for(i=0;i<sizeof(netComps.net_info.iccid);i++)
					 {
    						if(*pt_rssi!='\r')
    						{
    					 	    netComps.net_info.iccid[i]=*pt_rssi++;
                            }
    						else
    						{
    						   break; 	
    						}
					  }
					  netComps.disCode=EM_DIS_SEARCH_NET;
				 }
				 on_net_device_ack_ok();
			}
			break;
			
        case 15: 	
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
			    TX_ATCommand("\r\nAT+CIMI\r\n",10);//
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck();  
			}
			else if(check_net_device_ack("OK\r\n"))
			{
				 pt_rssi=check_net_device_ack("\r\n");
				 if(pt_rssi)
				 {
					 memset(netComps.net_info.imsi,0,sizeof(netComps.net_info.imsi));
				     for(i=0;i<sizeof(netComps.net_info.imsi);i++)
					 {
						if(*pt_rssi!='\r')
						{
					 	  netComps.net_info.imsi[i]=*pt_rssi++;
						}
						else
						{
						   break; 	
						}
					  }
				 }
				 on_net_device_ack_ok();
			}	
			break;	



			
/********************** Start to check registration**********************/

        case 16: 
		   if(!netMisc.AT_Waiting)                                          
    		{
    			netComps.AckTmr =2;	
    			netMisc.AT_Waiting = 1;
    		}
    		else if(!netComps.AckTmr)
    		{
    			on_net_device_ack_ok();
    		}
    		break;
    		
        case 17:
            GetRssi();
            break;
            
        case 18:  
			if(!netMisc.AT_Waiting)
			{
                reset_net_deive_rcvbuf();
				TX_ATCommand("\r\nAT+CREG?\r\n",10); 
			}
			else if(!netComps.AckTmr)
			{
				HandleNoAck();
            } 
			else if(check_net_device_ack("OK\r\n"))
			{
				if(check_net_device_ack("+CREG: 0,1"))//home 
				{
                   netMisc.St._bit.RegestCSNet=1;
                   on_net_device_ack_ok();
			    }
				else if(check_net_device_ack("+CREG: 0,5"))//Roaming
				{
				    netMisc.St._bit.RegestCSNet=1;
				    on_net_device_ack_ok();
			    }
				else//if(check_net_device_ack("+CREG: 0,2"))//search......
				{
                    netMisc.St._bit.RegestCSNet=0;
				    on_net_device_ack_ok();
                }
				
			}
			break;

         case 19:  
			if(!netMisc.AT_Waiting)
			{
                reset_net_deive_rcvbuf();
				TX_ATCommand("\r\nAT+CGREG?\r\n",10); 
			}
			else if(!netComps.AckTmr)
			{
				HandleNoAck();
            } 
			else if(check_net_device_ack("OK\r\n"))
			{
				if(check_net_device_ack("+CGREG: 0,1"))//home 
				{
				   netMisc.St._bit.RegestPSGNet=1;
                   on_net_device_ack_ok();
				}
				else if(check_net_device_ack("+CGREG: 0,5"))//Roaming
				{
					netMisc.St._bit.RegestPSGNet=1;
                    on_net_device_ack_ok();
				}
				else //if(check_net_device_ack("+CGREG: 0,2"))//search......
				{
                    netMisc.St._bit.RegestPSGNet=0;
                    on_net_device_ack_ok();

                }
            }
			break;

         case 20:  
			if(!netMisc.AT_Waiting)
			{
                reset_net_deive_rcvbuf();
				TX_ATCommand("\r\nAT+CEREG?\r\n",10); 
			}
			else if(!netComps.AckTmr)
			{
				HandleNoAck();
            } 
			else if(check_net_device_ack("OK\r\n"))
			{
				if(check_net_device_ack("+CEREG: 0,1"))//home 
				{
                    netMisc.St._bit.RegestPSENet=1;
                    on_net_device_ack_ok();
				}
				else if(check_net_device_ack("+CEREG: 0,5"))//Roaming
				{
					netMisc.St._bit.RegestPSENet=1;
                    on_net_device_ack_ok();
				}
				else //if(check_net_device_ack("+CEREG: 0,2"))//search......
				{
                    netMisc.St._bit.RegestPSENet=0;
                    on_net_device_ack_ok();
                }
            }
			break;

        case 21:
              if(!netMisc.St._bit.RegestCSNet)
              {
                    netMisc.NoRegestCSNetCount++;
                    if(netMisc.NoRegestCSNetCount>100)
                    {
                        HandleRegTimeOut();
                    	netMisc.NoRegestCSNetCount=0;
                    	break;
                    }
              }
              else if(!netMisc.St._bit.RegestPSGNet)
              {
                    netMisc.NoRegestPSGNetCount++;
                    if(netMisc.NoRegestPSGNetCount>20)
                    {
                    	netMisc.NoRegestPSGNetCount=0;
                    	netMisc.St._bit.RegestPSGNet=1;
                    }
              } 
              else if(!netMisc.St._bit.RegestPSENet)
              {
                    netMisc.NoRegestPSENetCount++;
                    if(netMisc.NoRegestPSENetCount>20)
                    {
                        netMisc.NoRegestPSENetCount=0;
                        netMisc.St._bit.RegestPSENet=1;
                    }
              }
               if(netMisc.St._bit.RegestCSNet&&netMisc.St._bit.RegestPSGNet&&netMisc.St._bit.RegestPSENet)
               {
                    //netComps.op_window_tmr=150;
                    netMisc.NoRegestPSGNetCount=0;
                    netMisc.NoRegestCSNetCount=0;
                    netMisc.NoRegestPSENetCount=0;
                    on_net_device_ack_ok();
               }
               else
               {
                    net_status_sjmp_any_step(16);//delay & read csq
               }
		       break;
                
/**************************end to check registration************************/

			
      case 22:  
                    if(!netMisc.AT_Waiting) 
    				{
    					reset_net_deive_rcvbuf();
    					TX_ATCommand("\r\nAT+QLTS=2\r\n",10);
    				}
    				else if(!netComps.AckTmr)
    				{
    					HandleNoAck(); 
    				}
    				else if(check_net_device_ack("OK\r\n"))
    				{
							pt_rssi=check_net_device_ack("+QLTS: \"20");
    					 if(pt_rssi)
    					 {
    						uint8_t Tbuf[8]=""; 
    					    uint8_t i=0;
    						//uint8_t TimeZoneEn=0;
    						for(i=0;i<6;i++)
    						{
    							Tbuf[i]=((*(pt_rssi+3*i)-0x30)*10)+(*(pt_rssi+3*i+1)-0x30);
    						}
    						Tbuf[6]=*(pt_rssi+3*i-1);//+-
    						Tbuf[7]=((*(pt_rssi+3*i)-0x30)*10)+(*(pt_rssi+3*i+1)-0x30);
                            //if(GMTtoLocalTime(Tbuf))
                            for(i=0;i<6;i++)
                        	{
                        		Tbuf[i]=((Tbuf[i]%100/10)<<4)+(Tbuf[i]%10);
                        	}
    						{
    							if((Tbuf[0]>0x16)&&(Tbuf[0]<0x50)&&(Tbuf[1]<0x13)&&(Tbuf[1]>0x00)&&(Tbuf[2]<0x32)&&
    							 (Tbuf[2]>0x00)&&(Tbuf[3]<0x24)&&(Tbuf[4]<0x60)&&(Tbuf[5]<0x60))
    							{
    								device_comps.system_time.time.u8Year=Tbuf[0];
    								device_comps.system_time.time.u8Month=Tbuf[1];
    								device_comps.system_time.time.u8Day=Tbuf[2];
    								device_comps.system_time.time.u8Hour=Tbuf[3];
    								device_comps.system_time.time.u8Minute=Tbuf[4];
    								device_comps.system_time.time.u8Second=Tbuf[5];

										device_comps.system_time.cs=Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs-(uint8_t *)&device_comps.system_time);
										device_comps.save_system_time(&device_comps.system_time,sizeof(device_comps.system_time));
										device_comps.set_system_time(&device_comps.system_time.time);
                                        ertc_comps.write_broken_time(&device_comps.system_time.time);
                                        netComps.St._bit.modifyed_time_just=1;
                                        
                  }
    						}
    					}
    					on_net_device_ack_ok();
    			    }
			        break;
         case 23:  
            if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
			    TX_ATCommand("\r\nAT+QNWINFO\r\n",10);//
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck();  
			}
			else if(check_net_device_ack("OK\r\n"))
			{
				pt_rssi=check_net_device_ack("+QNWINFO: ");
				 if(pt_rssi)
				 {
					 memset(netComps.net_info.act,0,sizeof(netComps.net_info.act));
					 memset(netComps.net_info.oper,0,sizeof(netComps.net_info.oper));
					 memset(netComps.net_info.band,0,sizeof(netComps.net_info.band));
				     for(i=0;i<sizeof(netComps.net_info.act)-1;i++)
					 {
						if(*pt_rssi!=',')
						{
					 	  netComps.net_info.act[i]=*pt_rssi++;
						}
						else
						{
						   break; 	
						}
					 }
					 pt_rssi++;//sjmp ,
					 for(i=0;i<sizeof(netComps.net_info.oper)-1;i++)
					 {
						if(*pt_rssi!=',')
						{
					 	  netComps.net_info.oper[i]=*pt_rssi++;
						}
						else
						{
						   break; 	
						}
					 }
					 pt_rssi++;//sjmp ,
					 for(i=0;i<sizeof(netComps.net_info.band)-1;i++)
					 {
						if(*pt_rssi!=',')
						{
					 	  netComps.net_info.band[i]=*pt_rssi++;
						}
						else
						{
						   break; 	
						}
					 }
				 }
                 on_net_device_ack_ok();
				 
			}	
			break;
			
        case 24: 
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
			    TX_ATCommand("\r\nAT+QICSGP=1\r\n",10);
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck();  
			}
			else if(check_net_device_ack("OK\r\n"))
			{
				pt_rssi=check_net_device_ack("+QICSGP: ");
				if(pt_rssi)
				 {
//					memset(netComps.net_info.imei,0,sizeof(netComps.net_info.imei));
//			        for(i=0;i<sizeof(netComps.net_info.imei);i++)
//				    {
//						if(*pt_rssi!='\r')
//						{
//						    netComps.net_info.imei[i]=*pt_rssi++;
//						}
//						else
//						{
//							break;
//						}
//				    } 
				 }
				 on_net_device_ack_ok();
               if(device_comps.gps.sel==2)
               {
                 net_status_sjmp_any_step(MD_NET_DEVICE_LSB_STEP-1);
               }
            }	
            break;

    
        case 25: 
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
			    TX_ATCommand("\r\nAT+QIDEACT=1\r\n",150);
			} 
			else if(!netComps.AckTmr)
			{
				HandleErr(); 
			}
			else if(check_net_device_ack("OK\r\n"))
			{
	            on_net_device_ack_ok();
			}
			else if(check_net_device_ack("ERROR"))
			{
                HandleErr();
			}
            break; 
            
       case 26: 
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
			    TX_ATCommand("\r\nAT+QIACT=1\r\n",150);
			} 
			else if(!netComps.AckTmr)
			{
				HandleErr(); 
			}
			else if(check_net_device_ack("OK\r\n"))
			{
	            on_net_device_ack_ok();
	        }
			else if(check_net_device_ack("ERROR"))
			{
                HandleErr();
			}
            break;
            
        case 27:
            if(netComps.net_info.currentIP_No<2)//device_comps.access_param.flag==2)//domain acces
            {
                if(!netMisc.AT_Waiting)
    			{
                    char msg[64]="\r\nAT+QIDNSGIP=1,";
                   // sprintf(msg+strlen(msg),"\"%s.",device_comps.iot_param.tenantID);
                    //sprintf(msg+strlen(msg),"%s\"",netComps.net_info.currentIP);
                    
                    sprintf(msg+strlen(msg),"\"%s\"",netComps.net_info.currentIP);
                    sprintf(msg+strlen(msg),"%s","\r\n");
    				reset_net_deive_rcvbuf();
                    TX_ATCommand(msg,120);
    			} 
    			else if(!netComps.AckTmr)
    			{
    				on_net_device_ack_ok();
    				netComps.disCode=EM_DIS_REGISTER_NET;
    			}
    			else if(check_net_device_ack("OK\r\n"))
    			{
                    if(check_net_device_ack("+QIURC: \"dnsgip\",0"))
                    {
    					pt_rssi=check_net_device_ack("+QIURC: \"dnsgip\",\"");
    					if(pt_rssi && strstr(pt_rssi,"\r\n"))
    					{
    							char *ptsz;
    							netComps.net_info.ResolvedIP[0]=strtol(pt_rssi,&ptsz,10);
    							netComps.net_info.ResolvedIP[1]=strtol(ptsz+1,&ptsz,10);
    							netComps.net_info.ResolvedIP[2]=strtol(ptsz+1,&ptsz,10);
    							netComps.net_info.ResolvedIP[3]=strtol(ptsz+1,&ptsz,10);
    							netComps.St._bit.ResolvedIP=1;
    							reset_net_deive_rcvbuf();
    							netComps.AckTmr=1;
    					}
                    }
                    else if(check_net_device_ack("+QIURC: \"dnsgip\",5"))
                    {
                       netComps.AckTmr=1;
                    }
                    
    			}
                else if(check_net_device_ack("ERROR"))
    			{
                    reset_net_deive_rcvbuf();
        			netComps.AckTmr=1;
                }
            }
            else 
			{
                net_status_sjmp_any_step(MD_NET_DEVICE_NULL_STEP);
                netComps.St._bit.noIP=1;
			}
            break; 

         case 28: 
              if(!netMisc.AT_Waiting)
              {
                  reset_net_deive_rcvbuf();
                  TX_ATCommand("\r\nAT+QISDE=0\r\n",10);
              } 
              else if(!netComps.AckTmr)
              {
                  HandleNoAck();  //
              }
              else if(check_net_device_ack("OK\r\n"))
              {
                  on_net_device_ack_ok();
              }
              break; 
			  
			  
        case 29: 
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
			    TX_ATCommand("\r\nAT+QICLOSE=0\r\n",5);
			} 
			else if(!netComps.AckTmr)
			{
			  #if(MD_IOT_PROTOCOL==MD_IOT_TCP)
               {
                  on_net_device_ack_ok();
	              netComps.St._bit.socket_connected=0;
               }
              #elif(MD_IOT_PROTOCOL==MD_IOT_MQTT)
               {
                 net_status_sjmp_any_step(MD_NET_DEVICE_MQTT_STEP);
                  netComps.St._bit.socket_connected=0;
                  mqtt_msg_id=1;
               }
              #endif 
			}
			else if(check_net_device_ack("OK\r\n"))
			{
              #if(MD_IOT_PROTOCOL==MD_IOT_TCP)
               {
                  on_net_device_ack_ok();
	              netComps.St._bit.socket_connected=0;
               }
              #elif(MD_IOT_PROTOCOL==MD_IOT_MQTT)
               {
                 net_status_sjmp_any_step(MD_NET_DEVICE_MQTT_STEP);
                  netComps.St._bit.socket_connected=0;
                  mqtt_msg_id=1;
               }
              #endif
			}
			break;

            


  /*********************************************TCP start***************************************/
  /*********************************************TCP start***************************************/         
        case 30: 
			if(!netMisc.AT_Waiting)
			{
                char msg[64]="\r\nAT+QIOPEN=1,0,\"TCP\",";
//                if(device_comps.access_param.flag==2)//domain acces
//                {
                    sprintf(msg+strlen(msg),"\"%s\",",netComps.net_info.currentIP);
//                }
//                else
//                {
//                    sprintf(msg+strlen(msg),"\"%d.",(int)device_comps.access_param.ip[0]);
//                    sprintf(msg+strlen(msg),"%d.",(int)device_comps.access_param.ip[1]);
//                    sprintf(msg+strlen(msg),"%d.",(int)device_comps.access_param.ip[2]);
//                    sprintf(msg+strlen(msg),"%d\",",(int)device_comps.access_param.ip[3]);
//                }
                sprintf(msg+strlen(msg),"%hu,",(unsigned short)netComps.net_info.currentPort);
                sprintf(msg+strlen(msg),"%s" ,"0,0\r\n");
				reset_net_deive_rcvbuf();
                TX_ATCommand(msg,150);
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck();  
			}
			else if(check_net_device_ack("OK\r\n"))
			{
			    if(check_net_device_ack("+QIOPEN: 0,0"))
				{
                     on_net_device_ack_ok();
					 netComps.St._bit.socket_connected=1;
					 netComps.St._bit.allow_data_send=1;
					 device_comps.sw._bit.isOnline=1;
                }
	            else if(check_net_device_ack("+QIOPEN: 0,566")||check_net_device_ack("+QIOPEN: 0,567"))
				{
					HandleDataErrIP();
	            }
	       }
			else if(check_net_device_ack("ERROR"))
			{
               HandleDataErrIP();
			}
            break;

        

/*****************************START TCP MSG LOOP*****************/

			
		case 31:
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
				netMisc.AT_Waiting=1;
				netComps.AckTmr=2; 
			}
			else if(!netComps.AckTmr)
			{
                if(protocolComps.sw._bit.DataRdy)
				{
	                PacketCoapMsg((uint8_t *)protocolComps.msg,protocolComps.msgLen);
                    on_net_device_ack_ok();
				}
				else
				{
					net_status_sjmp_any_step(37);
				}
            }
			break;
			
		case 32:
             GetRssi();
    	     break;	

		case 33:
            if(!netMisc.AT_Waiting)
            {
                char msg[32]="";
                sprintf(msg,"\r\nAT+QISEND=0,%d\r\n",(int)protocolComps.msgLen);
                reset_net_deive_rcvbuf();
                TX_ATCommand(msg,10);
            } 
            else if(!netComps.AckTmr)
            {
                HandleNoAck();  //
            }
            else if(check_net_device_ack("> "))
            {
                on_net_device_ack_ok();
            }
            else if(check_net_device_ack("ERROR"))
            {
                HandleDataErrIP();
            }
            break;

        case 34://SendData
		    if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
				TX_ATData(&netMisc.send_buf[0],10);
				netComps.disCode=EM_DIS_SEND;
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck(); 
			}
			else if(check_net_device_ack("SEND OK\r\n"))
			{
                
                 on_net_device_ack_ok();
				
			}
            else if(check_net_device_ack("ERROR"))
            {
                HandleDataErrIP();
            }

			break;
			
        case 35:
            if(!netMisc.AT_Waiting)
			{ 
                netMisc.AT_Waiting=1;
			 	netComps.AckTmr = 2;//////////////////1S
			}
			else if(!netComps.AckTmr)
			{
              on_net_device_ack_ok();
			}
            break;
                
		case 36://que SendData
    	    if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
                TX_ATCommand("\r\nAT+QISEND=0,0\r\n",10);
    		} 
    		else if(!netComps.AckTmr)
    		{
                HandleNoAck(); 
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
				pt_rssi=check_net_device_ack("+QISEND: ");
    		    if(pt_rssi)
        		{
								char *ptsz;
                    int32_t noAckBytes;
                    static int16_t query_times=0;
                    strtol(pt_rssi,&ptsz,10);
                    strtol(ptsz+1, &ptsz,10);
                    noAckBytes=strtol(ptsz+1, &ptsz,10);
                    if(!noAckBytes)
                    {
                        protocolComps.sw._bit.DataRdy=0;
                        netComps.St._bit.push_data_ok=1;
                        query_times=0;
                        on_net_device_ack_ok();
                    }
                    else
                    {
                        if(query_times<3)
                        {
                             
                             net_status_sjmp_any_step(MD_NET_DEVICE_DELAY_STEP);//
                             query_times++;
                        }
                        else
                        {
                            //net_status_sjmp_any_step(29);//close socket retry
                           // query_times=0;
                            protocolComps.sw._bit.DataRdy=0;
                           // netComps.St._bit.push_data_ok=1;
                            query_times=0;
                            on_net_device_ack_ok();
                        }
                        
                    }
    			}
    		}
            else if(check_net_device_ack("ERROR"))
            {
                HandleDataErrIP();
            }

    		break; 
			
		case 37://///////////////////Wait Rec
		  
		        if(!netMisc.AT_Waiting)
    			{ 
                    netMisc.AT_Waiting=1;
    			 	netComps.AckTmr = 2;
    			}
    			else if(!netComps.AckTmr)
    			{
                   
                    if(!netComps.St._bit.recvData)
                    {
                        on_net_device_ack_ok();
                    }
    				else
    				{
                        net_status_sjmp_any_step(MD_NET_DEVICE_SEND_STEP);//send data
    				}
    			}
                break;
         	      
		case 38:
	    	if(!netMisc.AT_Waiting)
			{
               reset_net_deive_rcvbuf();
			   TX_ATCommand("\r\nAT+QIRD=0,512\r\n",10);
            }
			else if(!netComps.AckTmr)
			{
				HandleNoAck(); 
			}
			else if(check_net_device_ack("OK\r\n"))
			{
				int16_t Length=0;
				pt_rssi=check_net_device_ack("+QIRD: ");
	       if(pt_rssi)
				{
                    
                    Length=atoi(pt_rssi);
                    if((Length<1)||(Length>512))
                    {
                    	netMisc.ErrCode=0;
                    }
					else
					{
                         while((*pt_rssi!='\n') &&(*pt_rssi!='\0'))
            			 {
            				 pt_rssi++;
            			 }
                         netMisc.ErrCode=GetMsg(pt_rssi+1,Length);
						 netComps.disCode=EM_DIS_RECV; 
					}
                }
				else
				{
					netMisc.ErrCode=3;//err=0;no data or err err=1,recv data,err=3,other
				}
				DealAnalyticCode(netMisc.ErrCode);
            }
            break; 

/*****************************END TCP MSG LOOP*****************/
/****************************TCP TCP STOP**********************/




       //////START BSL///////
        case MD_NET_DEVICE_LSB_STEP-1:
            
            netMisc.NoGetGpsLocCount=0;
            device_comps.gps.sw._bit.isLocSuc=0;
            netComps.disCode=EM_DIS_GPS_STATUS;
            netMisc.AT_Waiting=0;
            netMisc.NoAckTimes=0;
            netMisc.Flag_St++; 
    		break;
    
        case MD_NET_DEVICE_LSB_STEP:
            if(!netMisc.AT_Waiting)
    		{
                reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+QLBSCFG=\"asynch\",0\r\n",10);
            } 
    		else if(!netComps.AckTmr)
    		{
               // HandleNoAck();
                netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++; 
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
    			netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++;
             }	
    		break;

        case MD_NET_DEVICE_LSB_STEP+1:
            if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+QLBSCFG=\"timeout\",60\r\n",10);
            } 
    		else if(!netComps.AckTmr)
    		{
               // HandleNoAck();
                netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++; 
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
    			netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++;
             }	
    		break;
            
       case MD_NET_DEVICE_LSB_STEP+2:
            if(!netMisc.AT_Waiting)
            {
//                uint8_t msg[64]="";
//                memcpy(msg+strlen("\r\nAT+QLBSCFG=\"server\",\""),device_comps.lbs_param.domain,sizeof(device_comps.lbs_param.domain));
//                memcpy(msg,"\r\nAT+QLBSCFG=\"server\",\"",strlen("\r\nAT+QLBSCFG=\"server\",\""));
//                sprintf(msg+strlen(msg),"\"%s","\r\n");
//                reset_net_deive_rcvbuf();
//                TX_ATCommand(msg,10);
                  netComps.AckTmr =4;	
				  netMisc.AT_Waiting = 1;
            } 
            else if(check_net_device_ack("OK\r\n"))
            {
                netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++;
            }	
            else if(!netComps.AckTmr)
            {
            // HandleNoAck();
                netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++; 
            }
            break;
        
       case MD_NET_DEVICE_LSB_STEP+3:

            if(!netMisc.AT_Waiting)
            {
                char msg[64]="";
                memcpy(msg+strlen("r\nAT+QLBSCFG=\"token\",\""),device_comps.lbs_param.token,sizeof(device_comps.lbs_param.token));
                memcpy(msg,"r\nAT+QLBSCFG=\"token\",\"",strlen("r\nAT+QLBSCFG=\"token\",\""));
                sprintf(msg+strlen(msg),"\"%s","\r\n");
                reset_net_deive_rcvbuf();
                TX_ATCommand(msg,10);
            } 
            else if(!netComps.AckTmr)
            {
                netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++; 
            }
            else if(check_net_device_ack("OK\r\n"))
            {
                netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++; 
            }	
            break;
            
         case MD_NET_DEVICE_LSB_STEP+4:
            netMisc.AT_Waiting=0;
            netMisc.NoAckTimes=0;
            netMisc.Flag_St++;
            break;
            
         case MD_NET_DEVICE_LSB_STEP+5: 
			if(!netMisc.AT_Waiting) 
			{
				netComps.AckTmr =4;	
				netMisc.AT_Waiting = 1;
			}
			else if(!netComps.AckTmr)
			{
 				netMisc.AT_Waiting=0;
                netMisc.NoAckTimes=0;
                netMisc.Flag_St++; 
			}
			break;
            
    	 case MD_NET_DEVICE_LSB_STEP+6:	
    		if(!netMisc.AT_Waiting)
    		{
    			reset_net_deive_rcvbuf();
    	        TX_ATCommand("\r\nAT+QLBS\r\n",4);
            } 
    		else if(!netComps.AckTmr)
    		{
    			  //HandleNoAck(); 
    			    netMisc.NoGetGpsLocCount++;
                    if(netMisc.NoGetGpsLocCount>15)
                    {
                        netMisc.NoGetGpsLocCount=0;
                        netComps.op_window_tmr+=60;
                        net_status_sjmp_any_step(netMisc._Flag_St);
                        
                    }
                    else
                    {
                        netMisc.AT_Waiting=0;
                        netMisc.NoAckTimes=0;
                        netMisc.Flag_St=MD_NET_DEVICE_LSB_STEP+5; 
                    }
    		}
    		else if(check_net_device_ack("OK\r\n"))
    		{
					pt_rssi=check_net_device_ack("+QLBS: 0,");
                 if(pt_rssi)
    			 {
                    char *endptr;
                    int32_t  glat;
                    int32_t  glng;
                    glat=strtol(pt_rssi,     &endptr,10)*1000000;
                    if(glat<0)
                    {
                         glat=glat+(-strtol(endptr+1,&endptr,10));
                    }
                    else
                    {
                        glat=glat+strtol(endptr+1,&endptr,10);
                    }

                    glng=strtol(endptr+1,&endptr,10)*1000000;
                    if(glng<0)
                    {
                        glng=glng+(-strtol(endptr+1,&endptr,10));
                    }
                    else
                    {
                        glng=glng+strtol(endptr+1,&endptr,10);
                    }
                    device_comps.gps.glng=glat/10;
                    device_comps.gps.glat=glng/10;
                    device_comps.gps.cs=Check_Sum_5A(&device_comps.gps, &device_comps.gps.cs-(uint8_t *)&device_comps.gps);
                    device_comps.save_gps_param(&device_comps.gps,sizeof(device_comps.gps));  
                    device_comps.gps.sw._bit.isLocSuc=1;
                    netComps.op_window_tmr+=netMisc.NoGetGpsLocCount*4;
                  }
                  netMisc.AT_Waiting=0;
                  netMisc.NoAckTimes=0;
                  netMisc.Flag_St++; 
    			 
            }	
    		break;
          case MD_NET_DEVICE_LSB_STEP+7:
            if(!netMisc.AT_Waiting) //delay fuc
			{
				netComps.AckTmr =2;	//
				netMisc.AT_Waiting = 1;
			}
			else if(!netComps.AckTmr)
			{
                device_comps.gps.sw._bit.isLocSuc=0;
                net_status_sjmp_any_step(netMisc._Flag_St);//back to called delay function 
			}
			break;
           
       //////////////////end bsl server/////////////

         /*********************************************MQTT start***************************************/
  /*********************************************MQTT start***************************************/  
       case MD_NET_DEVICE_MQTT_STEP: 
            if(!netMisc.AT_Waiting)
    		{
                if(mqtt_config_cnt< sizeof(mqtt_config_param)/sizeof(mqtt_config_param[0]))
                {
                    reset_net_deive_rcvbuf();
    	            TX_ATCommand(mqtt_config_param[mqtt_config_cnt],10);
                }
                else
                {
                    on_net_device_ack_ok();
                }
            } 
    		else if(!netComps.AckTmr)
    		{
                HandleNoAck();
            }
    		else if(check_net_device_ack("OK\r\n"))
    		{
                mqtt_config_cnt++;
                if(mqtt_config_cnt==sizeof(mqtt_config_param)/sizeof(mqtt_config_param[0]))
                {
                    mqtt_config_cnt=0;
                    on_net_device_ack_ok();
                }
                else
                {
                    netMisc.AT_Waiting=0;
                    netMisc.NoAckTimes=0;
                }
    		}	
    		break;
        case MD_NET_DEVICE_MQTT_STEP+1:   //AT+QMTOPEN=0,"180.109.255.252",1883  "*.non-nb.ctwing.cn"
			if(!netMisc.AT_Waiting)
			{
                char msg[128]="\r\nAT+QMTOPEN=0,";
                //sprintf(msg+strlen(msg),"\"%s.",device_comps.iot_param.tenantID);
                //sprintf(msg+strlen(msg),"%s\",",netComps.net_info.currentIP);
                sprintf(msg+strlen(msg),"\"%s\",",netComps.net_info.currentIP);


                sprintf(msg+strlen(msg),"%hu",(unsigned short)netComps.net_info.currentPort);
                sprintf(msg+strlen(msg),"%s" ,"\r\n");
				reset_net_deive_rcvbuf();
                TX_ATCommand(msg,150);
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck();  
			}
			else if(check_net_device_ack("OK\r\n"))
			{
                pt_rssi=check_net_device_ack("+QMTOPEN: 0,");
                if(pt_rssi)
                {
                    
                    if(strstr(pt_rssi,"\r\n"))
                    {   
                       // int rslt;
                        //sscanf("+QIOPEN: 0,0\r\n","+QIOPEN: 0,%d\r\n",&rslt);
                        if(check_net_device_ack("+QMTOPEN: 0,0\r\n"))
        				{
                             on_net_device_ack_ok();
        					 
                        }
                        else
                        {
                            HandleDataErrIP();
                        }
                    }
                }
	       }
			else if(check_net_device_ack("ERROR"))
			{
               HandleErr();
			}
            break;

         case MD_NET_DEVICE_MQTT_STEP+2:   //AT+QMTCONN=0,"170886332","wzy","7Zzn7HSnkvZzx-ZZQJ5kFs6t0MAicI191VvAp5c_Duk"
			if(!netMisc.AT_Waiting)
			{
                char msg[128]="\r\nAT+QMTCONN=0,";
//                if(device_comps.access_param.flag==2)//domain acces
//                {
                    sprintf(msg+strlen(msg),"\"%s\",",device_comps.iot_param.deviceID);
                    sprintf(msg+strlen(msg),"\"%s\",","xianyuyi");
                    sprintf(msg+strlen(msg),"\"%s\"",device_comps.iot_param.token);
//                }
//                else
//                {
//                    sprintf(msg+strlen(msg),"\"%d.",(int)device_comps.access_param.ip[0]);
//                    sprintf(msg+strlen(msg),"%d.",(int)device_comps.access_param.ip[1]);
//                    sprintf(msg+strlen(msg),"%d.",(int)device_comps.access_param.ip[2]);
//                    sprintf(msg+strlen(msg),"%d\",",(int)device_comps.access_param.ip[3]);
//                }
               
                sprintf(msg+strlen(msg),"%s" ,"\r\n");
				reset_net_deive_rcvbuf();
                TX_ATCommand(msg,150);
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck();  
			}
			else if(check_net_device_ack("OK\r\n"))
			{
                pt_rssi=check_net_device_ack("+QMTCONN: 0,");
                if(pt_rssi)
                {
                    
                    if(strstr(pt_rssi,"\r\n"))
                    {   
                       // int rslt,ret_code;
                        //sscanf("+QMTCONN: 0,0,0\r\n","+QIOPEN: 0,%d,%d\r\n",&rslt,&ret_code);
                        if(check_net_device_ack("+QMTCONN: 0,0,0\r\n"))
        				{
                             on_net_device_ack_ok();
        					 netComps.St._bit.socket_connected=1;
        					 netComps.St._bit.allow_data_send=1;
        					 device_comps.sw._bit.isOnline=1;
                        }
                        else
                        {
                            HandleErr();
                        }
                    }
                }
	       }
			else if(check_net_device_ack("ERROR"))
			{
               HandleErr();
			}
            break;


         case MD_NET_DEVICE_MQTT_STEP+3:
            if(!netMisc.AT_Waiting) 
			{
				netComps.AckTmr =1;	//
				netMisc.AT_Waiting = 1;
			}
			else if(!netComps.AckTmr)
			{
 				on_net_device_ack_ok();
			}
			break;
            
        case MD_NET_DEVICE_MQTT_STEP+4:
           if(!netMisc.AT_Waiting)
            {
                char msg[128]="";
                mqtt_msg_id++;
                sprintf(msg,"\r\nAT+QMTSUB=0,%d,\"xianyunyi\\flowmeter\",1\r\n",(int)mqtt_msg_id);
                
                reset_net_deive_rcvbuf();
                TX_ATCommand(msg,10);
            } 
            else if(!netComps.AckTmr)
            {
                on_net_device_ack_ok();
            }
            else if(check_net_device_ack("+QMTSUB: 0,"))
            {
                char msg[32];
                sprintf(msg,"+QMTSUB: 0,%d,0,1\r\n",(int)mqtt_msg_id);
                if(check_net_device_ack(msg))
                {
                    on_net_device_ack_ok();
                }
            }
            else if(check_net_device_ack("ERROR"))
            {
                HandleDataErrIP();
            }
            break;
            

/*****************************START MQTT MSG LOOP*****************/

			
		case  MD_NET_DEVICE_MQTT_STEP+5:
			if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
				netMisc.AT_Waiting=1;
				netComps.AckTmr=1; 
			}
			else if(!netComps.AckTmr)
			{
                if(protocolComps.sw._bit.DataRdy)
				{
	                PacketCoapMsg((uint8_t *)protocolComps.msg,protocolComps.msgLen);
                    on_net_device_ack_ok();
				}
				else
				{
					net_status_sjmp_any_step(MD_NET_DEVICE_MQTT_STEP+10);
				}
            }
			break;
			
		case MD_NET_DEVICE_MQTT_STEP+6:
             GetRssi();
    	     break;	

		case MD_NET_DEVICE_MQTT_STEP+7:
            if(!netMisc.AT_Waiting)
            {
                char msg[128]="";
                mqtt_msg_id++;
                sprintf(msg,"\r\nAT+QMTPUBEX=0,%d,1,1,\"%s\",%d\r\n",(int)mqtt_msg_id,protocolComps.mqtt_pub_topic,(int)protocolComps.msgLen);
               reset_net_deive_rcvbuf();
                TX_ATCommand(msg,10);
            } 
            else if(!netComps.AckTmr)
            {
                HandleNoAck();  //
            }
            else if(check_net_device_ack("> "))
            {
                on_net_device_ack_ok();
            }
            else if(check_net_device_ack("ERROR"))
            {
                HandleDataErrIP();
            }
            break;

        case MD_NET_DEVICE_MQTT_STEP+8://SendData
		    if(!netMisc.AT_Waiting)
			{
				reset_net_deive_rcvbuf();
				TX_ATData(&netMisc.send_buf[0],10);
				netComps.disCode=EM_DIS_SEND;
			} 
			else if(!netComps.AckTmr)
			{
				HandleNoAck(); 
			}
			else if(check_net_device_ack("+QMTPUBEX: 0,"))
			{
                char msg[32];
                sprintf(msg,"+QMTPUBEX: 0,%d,0\r\n",(int)mqtt_msg_id);
                if(check_net_device_ack(msg))
                {
                    protocolComps.sw._bit.DataRdy=0;
                    netComps.St._bit.push_data_ok=1;
                    on_net_device_ack_ok();
                }
				
			}
            else if(check_net_device_ack("ERROR"))
            {
                HandleDataErrIP();
            }

			break;
			
       
                
		case MD_NET_DEVICE_MQTT_STEP+9://///////////////////Wait Rec
		  
		        if(!netMisc.AT_Waiting)
    			{ 
                    netMisc.AT_Waiting=1;
    			 	netComps.AckTmr = 1;
    			}
    			else if(!netComps.AckTmr)
    			{
                   
                    if(!netComps.St._bit.recvData)
                    {
                        on_net_device_ack_ok();
                    }
    				else
    				{
                        net_status_sjmp_any_step(MD_NET_DEVICE_MQTT_STEP+5);//send data
    				}
    			}
                break;

         case MD_NET_DEVICE_MQTT_STEP+10:
           if(!netMisc.AT_Waiting)
			{
               reset_net_deive_rcvbuf();
			   TX_ATCommand("\r\nAT+QMTRECV?\r\n",5);
            }
			else if(!netComps.AckTmr)
			{
				HandleNoAck(); 
			}
			else if(check_net_device_ack("OK\r\n"))
			{
				pt_rssi=check_net_device_ack("+QMTRECV: 0,");
				if(pt_rssi)
				{
                     int mask[5];
                     mqtt_rcv_msg_mask.all=0;
                     sscanf(pt_rssi,"%d,%d,%d,%d,%d",&mask[0],&mask[1],&mask[2],&mask[3],&mask[4]);
                     mqtt_rcv_msg_mask._bit.b0=mask[0]>0;
                     mqtt_rcv_msg_mask._bit.b1=mask[1]>0;
                     mqtt_rcv_msg_mask._bit.b2=mask[2]>0;
                     mqtt_rcv_msg_mask._bit.b3=mask[3]>0;
                     mqtt_rcv_msg_mask._bit.b4=mask[4]>0;
                     if( mqtt_rcv_msg_mask.all>0)
                     {
                        on_net_device_ack_ok();
                     }
                     else
                     {
                         net_status_sjmp_any_step(MD_NET_DEVICE_MQTT_STEP+5);//send data
                     }
				}
                else
                {
                    net_status_sjmp_any_step(MD_NET_DEVICE_MQTT_STEP+5);//send data
                }
                
			}
            break;
		case MD_NET_DEVICE_MQTT_STEP+11:
            if(1)
            {
                int8_t k;
                int msg_id=0;
                for(k=0;k<5;k++)
                {
                    if((mqtt_rcv_msg_mask.all>>k) & 1 )
                    {  
                        msg_id=k;
                        break; 
                    }
                }
    	    	if(!netMisc.AT_Waiting)
    			{
                   char msg[32]="";
                   sprintf(msg,"\r\nAT+QMTRECV=0,%d\r\n",(int)msg_id);
                   reset_net_deive_rcvbuf();
    			   TX_ATCommand(msg,5);
                }
    			else if(!netComps.AckTmr)
    			{
    				HandleNoAck(); 
    			}
    			else if(check_net_device_ack("OK\r\n"))
    			{
    				
    				pt_rssi=check_net_device_ack("+QMTRECV: 0,");
    	            if(pt_rssi)
    				{
                        GetMsg(pt_rssi,strlen(pt_rssi)-6);
    					netComps.disCode=EM_DIS_RECV; 
    					
                    }
    				net_status_sjmp_any_step(MD_NET_DEVICE_MQTT_STEP+5);//send data
                }
            }
            break; 

/*****************************END MQTT MSG LOOP*****************/
/****************************MQTT STOP**********************/









       
             
        case MD_NET_DEVICE_DELAY_STEP:
			if(!netMisc.AT_Waiting) //delay fuc
			{
				netComps.AckTmr =3;	//
				netMisc.AT_Waiting = 1;
			}
			else if(!netComps.AckTmr)
			{
 				net_status_sjmp_any_step(netMisc._Flag_St);//back to called delay function 
			}
			break;

      
        
        case MD_NET_DEVICE_NULL_STEP:
                break;

        default: 
                break;

		  
	}
}




void net_task_handle(void)
{
	if(netComps.St._bit.on)
	{
        config_net_com(115200,0);
        enable_net_com();
		if(!GSMPWRCTL_pin)
		{
			cold_start_net_device();
		}
		else
		{
			hot_start_net_device();
		}

	    load_ip();
		
	    netComps.St._bit.push_data_ok=0;
        netComps.St._bit.recvData=0;
        netComps.St._bit.allow_data_send=0;
		netMisc.reStartTimes=1;
        netComps.St._bit.running=1;
        netComps.op_window_tmr = MD_MODULE_OP_MAX_TIME;	//5¡¤??¨®//
        netComps.St._bit.on=0;
        
        device_comps.report_param.triggerTimes++;
		device_comps.report_param.cs=Check_Sum_5A((uint8_t *)&device_comps.report_param, &device_comps.report_param.cs-(uint8_t *)&device_comps.report_param);
		device_comps.save_report_param(&device_comps.report_param,sizeof(device_comps.report_param));
		
       
	}
	
	if(netComps.St._bit.off)
	{
        if(protocolComps.sw._bit.isAckTmrOut)
        {
            check_ip_switch();//is push data to ip1?
            protocolComps.sw._bit.isAckTmrOut=0;
        }
        else
        {
            netComps.disCode=EM_DIS_OFF;
    		netComps.St._bit.offing=1;
    		stop_net_device();
        }
        netComps.St._bit.off=0;
	}
	
	if(netComps.St._bit.running)
	{
		Srv_GSM();
	}
	
}

static void store_net_buffer(uint8_t data)
{
  netMisc.recv_buf[netMisc.recv_Idx] = data;
  netMisc.recv_Idx+=1;
  netMisc.recv_Idx&=0x3ff;
  netMisc.recv_RxSt=&netMisc.recv_buf[netMisc.recv_Idx];
}


netComps_t netComps=
{
	{0},//st
	EM_DIS_ACT,
	(uint8_t *)&netMisc.recv_msg_buf,//uint8_t *const msg;//Passed to the protocol layer point
	0,//uint16_t   msglen;
	
	

	0,//int16_t AckTmr;//netComps.AckTmr
	0,//int16_t op_window_tmr; 
	&netMisc.Flag_St,//uint8_t const *run_st;
	
	{0},//netinfo
	store_net_buffer,
	net_task_handle//void (*const task_handle)(void);
};

