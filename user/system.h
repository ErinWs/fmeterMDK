#ifndef  SYSTEM_H
#define  SYSTEM_H

typedef void (*  pfun)(void);

typedef struct _SYSTEM_COMPONENTS
{
    union 
    {
    	uint16_t All;
    	struct
    	{
           uint8_t is_xt1_running      :1;
     }_bit;
    }sw;
    uint8_t rst_code;
    pfun const  user_init;
    pfun const  task_process;
    
}systemComps_t;


extern systemComps_t systemComps;

#endif
