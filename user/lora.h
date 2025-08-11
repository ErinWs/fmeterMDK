#ifndef  LORA_H
#define  LORA_H



typedef struct 
{   
     uint32_t freq;              //Khz
     uint32_t NodeId;
     uint32_t init_value;
     uint16_t netId;
     uint8_t status;
     uint8_t mode_type;
     uint8_t mode_versor;
     uint8_t cs;
}ts_Apc340f_info;
	



typedef struct _LORA_COMPONENTS
{
  char *desc;  
  int16_t   do_init;//Whether to initialize,1:init 0,no init
  
	
//  yl701_info_t *yl701_info_p;
        // int16_t (*save_yl701_info)(void const *,int16_t);
        // int16_t (*read_yl701_info)(void const *,int16_t);
      
	ts_Apc340f_info  *Apc340f_info_p;
	int16_t (*save_Apc340f_info)(void );
	
    
	struct
	{
	    enum
	    {  LORA_EM_RUN_MODE,
	       LORA_EM_CFG_MODE,
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
                volatile uint8_t  initing		    :1;
                volatile uint8_t  init_ok               :1;
                volatile uint8_t  busy                  :1;
                volatile uint8_t  noParameter           :1;
                volatile uint8_t  init_once           :1;
                volatile uint8_t  param_modified        :1;
		        volatile uint8_t  test_rssi             :1;
               }_bit;
    }sw;

    int16_t       op_window_time;
    void (*store_buffer)(uint8_t data);
    void (*const task_handle)(void);
    void (*const task_50ms)(void);
}loraComps_t;

extern loraComps_t loraComps;

#endif