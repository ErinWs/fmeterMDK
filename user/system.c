#include "ddl.h"
#include "rtc.h"
#include "gpio.h"
#include "lpm.h"
#include "lptim.h"

#include "device.h"
#include "net.h"
#include "protocol.h"
#include "hum.h"
#include "irc.h"
#include "elora.h"
#include "collector.h"
#include "modbus.h"
#include "adx.h"
#include "system.h"

typedef struct _TASK_COMPONENTS
{
    char *desc;
    int_fast8_t             run_flag;
    volatile int_fast16_t   timer;          //50ms timer counter
    const int_fast16_t      interval_time;  //*50ms timer counter
    pfun * const    		task_hook;
    boolean_t            	isRealTime;//TRUE FALSE
    //void (* *const task_hook)(void);
}task_comps_t;


static void App_EnterLowPowerModeSet(void)
{
   //swd as gpio
   // Sysctrl_SetFunc(SysctrlSWDUseIOEn, TRUE);
}

static void enter_pwd_mode(void)
{
    
    if (!netComps.St._bit.running && 
        !adx_comps.sw._bit.adc_updated && 
        !loraComps.sw._bit.runing && 
        !device_comps.sw._bit.isFreqOuting)
    {
        App_EnterLowPowerModeSet();
        Lpm_GotoDeepSleep(FALSE); 
    }
}

pfun const pwd_mode_task=enter_pwd_mode;
pfun const _hlaf_s_task_handle=_0_5s_task_handle;
pfun const delay_task_50ms=_50ms_task_handle;




static task_comps_t task_comps[]=//50ms
{
	{"", 0,1       ,1 ,     (pfun *)&hum_comps.task_handle            ,FALSE},//*50ms  hum_comps.task_handle
	{"", 0,1       ,4 ,     (pfun *)&device_comps.task_handle         ,FALSE},//delay 100*50ms
	{"", 0,1       ,10,     (pfun *)&_hlaf_s_task_handle              ,FALSE},
	{"", 0,1       ,0 ,     (pfun *)&ircComps.task_handle             ,TRUE},//fast exe
 #if(MD_PRODUCT_NAME ==MD_LORA)	
	{"", 0,1       ,0 ,     (pfun *)&loraComps.task_handle            ,TRUE},//fast exe
	{"", 0,1       ,1 ,     (pfun *)&loraComps.task_50ms              ,FALSE},
  #endif        
	{"", 0,1       ,0 ,     (pfun *)&pwd_mode_task                    ,TRUE},//fast exe
	{"", 0,1       ,0 ,     (pfun *)&modbusComps.task_handle          ,TRUE},//fast exe
 #if(MD_PRODUCT_NAME ==MD_4G)	
	{"", 0,1       ,0 ,     (pfun *)&protocolComps.task_handle        ,TRUE},//fast exe
	{"", 0,1       ,0 ,     (pfun *)&netComps.task_handle             ,TRUE},//fast exe
 #endif		
	{"", 0,1       ,1 ,     (pfun *)&delay_task_50ms                  ,FALSE},
	{"" ,0,1       ,0 ,   (pfun *)&adx_comps.task_handle              ,TRUE}//fast exe

	//...TODO......
};


#define MD_TASK_COMPS_COUNT (sizeof(task_comps) / sizeof(task_comps[0]))
static void call_back_task_remarks(void)
{
    int_fast8_t i;
    task_comps_t *task;
    for(i = 0; i < MD_TASK_COMPS_COUNT; i++)
    {
        task = &task_comps[i];
        if(task->timer > 0)
        {
            if(--task->timer == 0 )
            {
                task->timer = task->interval_time;
                task->run_flag = 1;
            }
        }
    }
}

static void task_process(void)
{
    int_fast8_t i;
    task_comps_t *task;
    for(i = 0; i < MD_TASK_COMPS_COUNT; i++)
    {
        task = &task_comps[i];
        if(task->run_flag)
        {
            (*task->task_hook)();
            if(task->isRealTime == FALSE)
            {
                task->run_flag = 0;
            }
        }
    }
}


void user_init(void)
{

    
}

systemComps_t systemComps=
{
    {._bit.is_xt1_running=0},
    0x0e,//uint8_t rst_code;
    user_init,
    task_process,
};

void LpTim1_IRQHandler(void)
{
    if (TRUE == Lptim_GetItStatus(M0P_LPTIMER1))
    {
       Lptim_ClrItStatus(M0P_LPTIMER1);
       call_back_task_remarks();
    }
}








