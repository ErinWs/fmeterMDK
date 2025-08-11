#ifndef _COL_H
#define _COL_H






typedef struct _COLLECTOR_COMPONENTS
{
	char *desc;
	union 
    {
    	uint8_t All;
    	struct{
                uint8_t running		            :1;
                uint8_t busy		                :1;
                uint8_t isGetExternPressReq       :1;
               }_bit;
    }sw;
    struct
    {
         void          *  ip;
         void          *  port;
         void          *  report_minute_interval; //*minute //user set
         uint16_t  refresh_freq;//*1s user set
         uint16_t  pv_coe;//user set 
         uint16_t  pv_dot;//user set 0-4
         int32_t          aux1_h3;
         
         uint16_t  slave_addr;
         uint16_t  reg_addr;
         uint16_t  cmd;
         uint16_t  sensor_power_supply_time;//*50ms
         uint8_t data_type;//0-4 0,1 16bit_sign 16bit_unsign  2,3,4 32bit
         uint8_t data_endian;
         uint8_t  cs;
    }nvm_param;
   
    int32_t pv;
    float32_t pvf;
    
    int16_t (*save_nvm_param)(void const *,int16_t);
    void (*const task_handle)(void);
    void (*const task_50ms)(void);
	
}collectorComps_t;
extern collectorComps_t collectorComps;
#endif





