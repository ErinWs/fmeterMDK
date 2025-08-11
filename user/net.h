#ifndef  NET_H
#define  NET_H


typedef enum 
{
      EM_DIS_ACT=0,
      EM_DIS_OFF,
      EM_DIS_GPS_STATUS,
      EM_DIS_SEARCH_NET,
      EM_DIS_REGISTER_NET,
      EM_DIS_SEND, 
      EM_DIS_RECV 
}disCode_t;


typedef struct 
{   
    union 
    {
    	uint16_t All;
    	struct
    	{
    		uint8_t offing          :1;
    		uint8_t windowTimeOut	:1;
    		uint8_t noResponse	    :1;
    		uint8_t noIP		    :1;
    		uint8_t off		        :1;
    		uint8_t err		        :1;
    	    uint8_t on		        :1;	
    		uint8_t recvData	    :1;	
    		uint8_t running		    :1;
    		uint8_t socket_connected:1;
    		uint8_t allow_data_send:1;
    		uint8_t ResolvedIP      :1;
    		uint8_t push_data_ok     :1;
            uint8_t modifyed_time_just    :1;
    		
    	}_bit;
    } St;
  
    disCode_t disCode;//0 act;1,off;2 search net;3 register net yet;4 send; 5 recv;
	uint8_t * const msg;//Passed to the protocol layer point
	uint16_t    msgLen;
	
	int16_t AckTmr;//BC35Srv_comps.AckTmr
	int16_t op_window_tmr; 
	uint8_t const * const run_st;//point to bc35 run step

	struct 
    {
      char imei[20];
    	char imsi[20];
    	char iccid[25];
    	char firmVer[25];
    	char firmhdVer[25];
    	uint32_t earFcn;
    	uint32_t cellId;
    	uint16_t  pci;
    	uint8_t          rsrp;
    	uint8_t          rsrq;
    	uint8_t ecl;
    	uint8_t          sina;
    	uint8_t Csq;
    	uint8_t CsQ;

    	char act[20];
    	char oper[20];
    	char band[20];
    	int32_t channel;
    	uint8_t ResolvedIP[4];
    	
    	
    	enum
    	{
    	    EM_IP0=0,
    	    EM_IP1,
    	    EM_NO_IP
    	}            currentIP_No;
    	char         currentIP[32];
    	uint16_t currentPort;
	
    }net_info;

    void (*store_buffer)(uint8_t data);
    void (*const task_handle)(void);
    
}netComps_t;
extern netComps_t netComps;





#endif

