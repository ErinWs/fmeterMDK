
#ifndef  ELORA_H
#define  ELORA_H


#define  MD_ELORA_VER_A39C               0
#define  MD_ELORA_VER_AS32               1
#define  MD_ELORA_VER_SEL               MD_ELORA_VER_AS32


#if(MD_ELORA_VER_SEL==MD_ELORA_VER_A39C)
    #define  MD_ELORA_RSSI_104DBM  ((uint8_t)(180))
    #define  MD_ELORA_RSSI_94DBM   ((uint8_t)(190))
    #define  MD_ELORA_RSSI_84DBM   ((uint8_t)(200))
#elif(MD_ELORA_VER_SEL==MD_ELORA_VER_AS32)
    #define  MD_ELORA_RSSI_104DBM  ((uint8_t)(-104+164))
    #define  MD_ELORA_RSSI_94DBM   ((uint8_t)(-94+164))
    #define  MD_ELORA_RSSI_84DBM   ((uint8_t)(-84+164))
#endif

typedef struct 
{   
     uint32_t freq;
     uint8_t  netId;
     uint8_t  nodeId;
     uint8_t  cs;
}lora_cfg_info_t;
	

typedef struct _LORA_COMPONENTS
{
  char *desc;  
  int16_t   do_init;//Whether to initialize,1:init 0,no init
  
	
//  yl701_info_t *yl701_info_p;
        // int16_t (*save_yl701_info)(void const *,int16_t);
        // int16_t (*read_yl701_info)(void const *,int16_t);
      
	lora_cfg_info_t  *cfg_info_p;
	int16_t (*save_cfg_info)(void );
	struct
	{
	    enum
	    {  
            LORA_EM_WORK_MODE=0,
	        LORA_EM_CFG_MODE,
	        LORA_EM_CAD_MODE
	       
	    } mode;
	    enum
	    {
	        LORA_EM_SEND,
	        LORA_EM_RECV
	    }dir;
	}work_st;

    union 
    {
    	uint16_t All;
    	struct{
                volatile uint8_t  runing		    :1;
            
                volatile uint8_t  init_ok               :1;
                volatile uint8_t  busy                  :1;
                volatile uint8_t  noParameter           :1;
                
                volatile uint8_t  param_modified        :1;
		        volatile uint8_t  test_rssi             :1;
               }_bit;
    }sw;

    int16_t  op_window_time;
    uint8_t evnRssi;
    uint8_t packageRssi;
    uint16_t dis_rssi_tmr_s;
    void (*store_buffer)(uint8_t data);
    void (*const task_handle)(void);
    void (*const task_50ms)(void);
}loraComps_t;

extern loraComps_t loraComps;

#endif 
