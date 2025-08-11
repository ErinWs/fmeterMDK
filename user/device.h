#ifndef  DEVICE_H
#define  DEVICE_H

#define   MD_FL_VER                      20
#define   MD_BACK_LED_ON      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BACK_LED_PORT), MD_BACK_LED_PIN, TRUE)
#define   MD_BACK_LED_OFF     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BACK_LED_PORT), MD_BACK_LED_PIN, FALSE)


#define   MD_ADC_MAX_POS            6

#define   MD_SET_AVDD_ON                    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_AVDD_CTL_PORT),  MD_AVDD_CTL_PIN, FALSE)
#define   MD_SET_AVDD_OFF                   SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_AVDD_CTL_PORT),  MD_AVDD_CTL_PIN, TRUE)
#define   MD_SET_BAT_MEASURE_CTL_ON         SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BAT_CTL_PORT),  MD_BAT_CTL_PIN, TRUE)
#define   MD_SET_BAT_MEASURE_CTL_OFF        SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BAT_CTL_PORT),  MD_BAT_CTL_PIN, FALSE)
#define   MD_SET_MCU_VREF_CTL_ON            SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_MCU_EXVREF_CTL_PORT),  MD_MCU_EXVREF_CTL_PIN, TRUE)
#define   MD_SET_MCU_VREF_CTL_OFF           SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_MCU_EXVREF_CTL_PORT),  MD_MCU_EXVREF_CTL_PIN, FALSE)
#define   MD_BAT_BLUNT_CTL_ON               SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BAT_BLUNT_CTL_PORT),  MD_BAT_BLUNT_CTL_PIN, TRUE)   
#define   MD_BAT_BLUNT_CTL_OFF              SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BAT_BLUNT_CTL_PORT),  MD_BAT_BLUNT_CTL_PIN, FALSE)
#define   MD_GET_EXT_POWER_STATUS()         GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_EXT_POWER_CONNECT_DETECT_PORT), MD_EXT_POWER_CONNECT_DETECT_PIN)
#define   MD_GET_4_20MA_STATUS()            GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_4_20MA_CONNECT_DETECT_PORT), MD_4_20MA_CONNECT_DETECT_PIN)
          



#define  MD_3_6V   0
#define  MD_4_5V   1
#define  MD_9_0V   2
#define  MD_DEVICE_BATT           MD_3_6V

#if(MD_DEVICE_BATT==MD_3_6V)
 #define MD_BAT_RES_DIV_GAIN          30//0.1 
#elif(MD_DEVICE_BATT==MD_4_5V)
 #define MD_BAT_RES_DIV_GAIN          40//0.1
#elif(MD_DEVICE_BATT==MD_9_0V)
 #define MD_BAT_RES_DIV_GAIN          110//0.1
#else 
 #define MD_BAT_RES_DIV_GAIN          30//0.1
#endif

#if(MD_DEVICE_BATT==MD_3_6V)
 #define MD_BAT_88_PER_100  33
 #define MD_BAT_85_PER_100  31
 #define MD_BAT_80_PER_100  30
#elif(MD_DEVICE_BATT==MD_4_5V)
 #define MD_BAT_88_PER_100  40
 #define MD_BAT_85_PER_100  38
 #define MD_BAT_80_PER_100  35
#elif(MD_DEVICE_BATT==MD_9_0V) 
 #define MD_BAT_88_PER_100  76
 #define MD_BAT_85_PER_100  73
 #define MD_BAT_80_PER_100  70
#else
 #define MD_BAT_88_PER_100  33
 #define MD_BAT_85_PER_100  31
 #define MD_BAT_80_PER_100  30
#endif


#define  MD_FLOW_MAX_CAL_POS     20


#define  MD_LIQUID               0
#define  MD_GASES                1




#define  MD_4G                   0
#define  MD_LORA                 1
#define  MD_NO_IOT               2
#define  MD_PRODUCT_NAME         MD_NO_IOT //




//#define   MD_NO_LCD
//#define   MD_IGNORE_ALL_ERR
#define     MD_USE_RTC_MODULE



//#define   MD_EXT_COLLECTOR
#define   MD_PRESS       0
#define   MD_EXT_COLLECTOR_TYPE   MD_PRESS

#define   MD_IOT_TCP               0
#define   MD_IOT_MQTT              1
#define   MD_IOT_PROTOCOL          MD_IOT_TCP


#define  APP_PROTOCOL_ZHIAN            0
#define  APP_PROTOCOL_SELF             1
#define  APP_PROTOCOL_NULL             5
#define  APP_PROTOCOL_TYPE             APP_PROTOCOL_SELF

#define  PROTOCOL_VER_1               1
#define  PROTOCOL_VER_2               2
#define  PROTOCOL_VER          PROTOCOL_VER_1


#if (APP_PROTOCOL_TYPE==APP_PROTOCOL_ZHIAN)
#define  MD_PUSH_DATA_TO_IP1
#else
#define  MD_ALT_IP
#endif





#define   MD_E2PROM_DRIVER_ERR                (1<<0)

#if(MD_MEASURE_TYPE==MD_GASES)
	#define   MD_ADX_DRIVER_ERR               (0<<1)
#else
	#define   MD_ADX_DRIVER_ERR               (0<<1)
#endif

#if(MD_PRODUCT_NAME==MD_LORA)
	#define   MD_LORA_MODULE_ERR              (1<<2)
#else
	#define   MD_LORA_MODULE_ERR              (0<<2)
#endif

#if defined (MD_USE_RTC_MODULE)
	#define   MD_RTC_MODULE_ERR               (1<<4)
#else
	#define   MD_RTC_MODULE_ERR               (0<<4)
#endif



  
#define BK_LN


//#define MD_MEMORY_TEST_START_ADDR                      0x0000   //
#define MD_SYSTEM_TIME_START_ADDR                      0x0020
#define MD_PRESS_CALIBRATION_PARAM_START_ADDR          0x0030   //
#define MD_RES_CALIBRATION_PARAM_START_ADDR            0x00a0   //



#define MD_REPORT_PARAM_START_ADDR                     0x00c0   //16
#define MD_ALARM_PARAM_START_ADDR                      0x00d0   //32
#define MD_DEVICE_ADDR_START_ADDR                      0x00f0   //16
#define MD_TIME_SEG_DATA_PARAM_START_ADDR              0x0100
#define MD_MODBUS_PARAM_START_ADDR                     0x0110
#define MD_DEVICE_COE_START_ADDR                       0x0120//32


#define MD_ACCESS_PARAM_START_ADDR                     0x0180
#define MD_LORA_PARAM_START_ADDR                       0x01e0

#define MD_TIME_SEG_DATA_START_ADDR                    0x0200
#define MD_TIME_SEG_DATA_END_ADDR                      0x0400 
#define MD_LBS_PARAM_START_ADDR                        0x0410 
#define MD_FLOW_METER_START_ADDR                       0x0430
#define MD_METER_BACKUP_START_ADDR                     0x0480
#define MD_METER_START_ADDR                            0x04c0
#define MD_METER_END_ADDR                              0x04ff

#define MD_CAL_PARAM_START_ADDR                        0x0500   //
//#define MD_CAL_PARAM_START_END                         0x05d0   //
#define MD_TEMP_COMP_PARAM_START_ADDR                  0x05d0//48B

#define MD_GPS_INFO_START_ADDR                         0x0600 
#define MD_MANUFACTURER_INFO_START_ADDR                0x0620
#define MD_DEVICE_INFO_START_ADDR                      0x0640
#define MD_SENSOR_INFO_START_ADDR                      0x0680
#define MD_COLLECT_NVM_PARAM_START_ADDR                0x06c0//
#define MD_MISC_PARAM_START_ADDR                       0x06e0
#define MD_IOT_PARAM_START_ADDR                        0x0710
#define MD_MEMORY_TEST_START_ADDR                      0x07e0 



#define  MD_PRESS_CALIBRATION_PARAM_COUNT      8// ( sizeof(device_comps.press_cal_param.x)/sizeof(device_comps.press_cal_param.x[0]) )
#define  MD_RES_CALIBRATION_PARAM_COUNT    2// ( sizeof(device_comps.voltage_press_cal_param.x)/sizeof(device_comps.voltage_press_cal_param.x[0]) )  


#define  BK_LN
  typedef struct
  {
        int32_t   data_on_temp[2][3];//   T1  X_t1  Y_t1  Z_t1  
        int16_t   temp[2];//              T0  X_t0  Y_t0  Z_t0     
        union 
        {
           uint8_t All;
            struct
            {
                uint8_t is_compensated     :1;

            }_bit;
        }sw;
        uint8_t cs;
  }
  temp_comp_param_t;
  
typedef struct 
 {
    
     int32_t glng;
     int32_t glat;
     union 
     {
         uint8_t All;
         struct
         {
             uint8_t isLocSuc      :1;
             uint8_t isActive      :1;
         }_bit;
     }sw;
     uint8_t sel;
     uint8_t loc_times;
     uint8_t cs;
 }gps_t;

typedef struct
  {
      int32_t x[8];
      int32_t y[8];
      int16_t  t;//Temperature value for temperature compensation
      uint8_t dot;
      uint8_t unit;//0Mpa ,1Kpa,0x81 Kpa m
      uint8_t is_calibrated; 
      uint8_t cs;
  }
  press_cal_param_t;

  typedef struct
   {
       uint16_t  freq_divd_value[MD_FLOW_MAX_CAL_POS];//0.1hz
       float32_t     freq_divd_value_meter_coe[MD_FLOW_MAX_CAL_POS];
       float32_t     freq_poly_coe[5];
       float32_t     meter_coe;
       uint16_t  RefMaxQ;
       uint16_t RefDN;
       
       uint8_t   dot;
       uint8_t   unit;// 0：m3/h
       uint8_t   calc_mode;   //0:freq divied  1:freq calc  2:average coe
       uint8_t   freq_divd_pos;
       uint8_t   freq_out_mode;//0:comped  1:uncomped
       uint8_t   cs;
   }
   flow_cal_param_t;

 

  typedef struct
    {
        int32_t x[2];
        int32_t y[2];
        uint8_t dot;
        uint8_t unit;
        uint8_t is_calibrated; 
        uint8_t cs;
    }
    res_cal_param_t;

typedef enum  
{
	EM_CAL_PRESS=0,
	EM_CAL_RES,
	EM_CAL_FLOW=4

    //....TODO.....
	
}cal_type_t;

  typedef enum  
  {
      
      EM_PARAM_USER=0,
      EM_PARAM_USER_A,
      EM_PARAM_FACTORY
   
      //....TODO.....
      
  }param_type_t;

  typedef struct
  {
      uint8_t u8Minute;
      uint8_t u8Hour;
      int16_t  u16Minute_Interval;
      uint8_t  u8Hour_Interval;
      int16_t disFactor;
      uint32_t triggerTimes;
      uint8_t cs;
  }
  report_param_t;
 
typedef struct
{
    int32_t bottom_s;//m^2
    int32_t sample_interval_value;//s
    int32_t I_o_low;              //m3/h  auto dot =device_comps.flow_cal_param.dot
    int32_t I_o_high;             //m3/h  auto dot =device_comps.flow_cal_param.dot
    int32_t density;              //*0.01 kg/m3
    int16_t I_o_dir;
    int16_t display_tyep;
    uint8_t cs;
    
}
misc_param_t;

  typedef struct
  {
      char ip[25];
      uint16_t  port;
      char ip1[25];
      uint16_t  port1;
      uint8_t          flag;//0 null ,1 ip, 2,domain
      uint8_t cs;
  }
  access_param_t;
 
  typedef struct
  {
      char token[20];
      char domain[32];
      uint8_t cs;
  }
  lbs_param_t;
  
 typedef struct
 {
        char productID[20];
        char tenantID[20];
        char token[68];
        char deviceID[36];
        uint8_t cs;
 }
 iot_param_t;

  typedef struct 
  {
      union 
      {
     	uint16_t All;
     	struct
     	{
              uint8_t high_temp_alarm_en     :1;
              uint8_t low_temp_alarm_en		:1;
              uint8_t high_press_alarm_en	:1;
              uint8_t low_press_alarm_en	    :1;
         }_bit;
      }sw;
      int32_t press_high_upper;
      int32_t press_high_lower;
      int32_t press_low_upper;
      int32_t press_low_lower;
      int16_t  unit;

      int16_t temp_high;
      int16_t temp_low;
      
      uint8_t cs;
  }
  alarm_param_t;
  
  typedef struct
  {
        stc_rtc_time_t time;
        uint8_t cs;
  }system_time_t;
        
 

 typedef struct 
 {
     uint8_t addr[7];
     uint8_t cs;
 }
 device_addr_t;



typedef struct
{
    int16_t  store_addr;
    int16_t  nums;
    uint8_t lastSampleTime[7];
    uint8_t cs;
}
TimeSegData_t;

typedef struct
{
    int32_t   total_int;//  unit: m3
    int32_t   total_intN;//m3
    float32_t     total_dec;//  unit: L
    float32_t     total_decN;//L
    uint8_t cs;

}meter_t;

typedef struct
{
    int32_t   total_int;//  unit: m3
    int32_t   total_intN;//m3
    float32_t     total_dec;//  unit: L
    float32_t     total_decN;//L
    int16_t   store_index;
    uint8_t cs;
          
}meter_backup_t;

 typedef struct
    {
        union 
        {
        	uint8_t All;
        	struct
        	{
               uint8_t running :1;
            }_bit;
        }sw;
        int16_t timer;
        void (*start)(int16_t timer);
        void (*stop)(void);
    }buzzer_t;

    typedef  struct 
    {
        uint8_t id[25];
        uint8_t cs;
    }sensor_info_t;
	
	 typedef struct 
    {
        uint8_t type[25];
        uint8_t id[25];
        uint8_t cs;
    }device_info_t;

   typedef struct 
    {
        uint8_t name[25];
        uint8_t cs;
    }manufacturer_info_t;	
	
	 typedef struct 
    {
        int16_t  press;             // *0.0001
        int16_t  pt_temp;           // *0.0001
        int16_t  flow;              // *0.0001
        int16_t  out_4_20ma;        // *0.0001
        uint16_t _4ma_raw_value;
        uint16_t _20ma_raw_value;
        int16_t  back_led_on_time;
        int16_t  cut_off_value;     //   FS*%00 
        int16_t  ad_gain;
        int32_t  press_clr_value;
        uint8_t cs;
    }coe_t;

 typedef struct 
       {           
           uint16_t unit;//(0: m3/h, 1: L/m,  2:Kg/h, 3:L/h,   4:T/h,   5:Kg/m,   6:m3/m,  7: t/m)
           uint16_t pluse_width;          //*0.1ms    max 1000.0ms
           uint32_t pluse_equ;            //*0.001L  
           uint16_t sensor_low_freq_cutoff;//0.1 hz
           uint16_t avg_freq_filter_timer;   //s
           uint8_t cs;
       }flow_meter_t;
       
	  typedef union 
    {
    	uint64_t All;
    	struct{
                volatile uint8_t  e2prom_driver_err		            :1;
                volatile uint8_t  adx_driver_err		            :1;
                volatile uint8_t  lora_module_err                   :1;
                volatile uint8_t  res0				                :1;
                volatile uint8_t  rtc_module_err		            :1;
                volatile uint8_t  pcf857x_driver_err		        :1;
                volatile uint8_t  ad54x0_driver_err		            :1;
				volatile uint8_t  res1		            			:1;
                
                volatile uint8_t  com_key_en	                    :1;
                volatile uint8_t  isPressNoConnect	                :1;
                volatile uint8_t  isPLowRealseTriggered             :1;
                volatile uint8_t  isPLowLessTriggered               :1;
                volatile uint8_t  isPHighRealseTriggered            :1;
                volatile uint8_t  isPHighOverTriggered              :1;
                volatile uint8_t  over_range	                    :1;
                volatile uint8_t  high_over_range	                :1;
                
                volatile uint8_t  adc_busy	                        :1;
                volatile uint8_t  adc_stb	                        :1;
                volatile uint8_t  temp_adc_stb	                    :1;
                volatile uint8_t  batt_status                       :1;
                volatile uint8_t  data_mode                         :1;
                volatile uint8_t  isOnline                          :1;
                volatile uint8_t  isOutGoingPluse                   :1;
                volatile uint8_t  isFreqOuting                   :1;

                volatile uint8_t  isTempNoConnect	                :1;
                volatile uint8_t  isTHighOverTriggered	            :1;
                volatile uint8_t  isTHighRealseTriggered	         :1;
                volatile uint8_t  isTLowLessTriggered                :1;
                volatile uint8_t  isTLowRealseTriggered              :1;
                volatile uint8_t  isExtPowerConnected               :1;
                volatile uint8_t  is_4_20ma_Connected               :1;
				volatile uint8_t  isBatBluntNow                     :1;

                volatile uint8_t  cmd_out_raw__4ma_20ma            :1;//0:4ma, 1:20ma
     }_bit;
    }sw_t;

 #define  BK_LN   
typedef struct _DEVICE_COMPONENTS
{
	char *desc;
	struct _DEVICE_COMPONENTS  *const this;
	int16_t   do_init;//Whether to initialize,1:init 0,no init

	uint32_t count;            //Called counter
	uint32_t sensor_count;
	sw_t sw;
    
    int16_t PHihgOverTimer;
    int16_t PHihgRealseTimer;
    int16_t PLowLessTimer;
    int16_t PLowRealseTimer;
    int16_t THihgOverTimer;
    int16_t THihgRealseTimer;
    int16_t TLowLessTimer;
    int16_t TLowRealseTimer;
	
	uint16_t report_interval_timer;
	int16_t _0_5s_timr_acc;
	int16_t batt_blunt_timer;
    int16_t batt;//batt voltage
	
	buzzer_t buzzer;
	
    uint16_t s1_cnt_value[10];
    int16_t  s1_pos;
    uint16_t s1_pre_cnt;
    int16_t  flow_freq_cur;          //1hz
    uint32_t flow_total_cnts;          //1
    float32_t  flow_run_meter_coe; 
    int32_t  flow_roll_freq_cur;   //0.1hz
    float32_t flow_roll_freq_cur_ft;
    int32_t  flow_roll_freq_comped_cur;   //0.1hz
    float32_t flow_roll_freq_comped_cur_ft;
    float32_t  flow_freq_output_prevalue_ft;
    int32_t (*get_cur_flow_meter_unit_display_value)(int32_t cur_std_flow,uint8_t std_flow_dot,uint16_t dis_unit,uint8_t *dis_dot,uint16_t *calc_dis_unit);

    
    int32_t current_flow;    // Q m3/h
    int32_t current_flowN;   //std QN m3/h dot is the same as current_flow
    int32_t current_flowM;   //kg/h        dot is the same as current_flow
    int16_t current_v;         //0.01m/s
   
    int16_t current_i;         //0.001mA
    int16_t current_i_n_1;
    uint32_t  outed_pluse_cnt;
    float32_t this_pluse_L;
	int16_t lptimer_interrput_cnt;
    void (*const out_pluse_start)(int16_t pluse_width);
    void (* const s1_cnt_start)(void);
      
	int32_t  ad1_convert_result[MD_ADC_MAX_POS];
	uint16_t  ad1_pos;
	int32_t  ad2_convert_result[MD_ADC_MAX_POS];
	uint16_t  ad2_pos;
	int32_t          ad1_ad2_average_result;//(ad1-ad2)/ad1_pos
	int32_t  current_press;  //Yn
    int32_t  max_press;
	int32_t  current_press_n_1;//Yn-1
    int32_t  current_press_n_2;//Yn-2
	int32_t  press_full_scale;
	int16_t (*clr_press)(int16_t);
	
	int32_t  temp_p_convert_result[MD_ADC_MAX_POS];
	uint16_t  temp_p_pos;
	int32_t  temp_n_convert_result[MD_ADC_MAX_POS];
	uint16_t  temp_n_pos;
	int32_t   temp_p_temp_n_average_result;//(temp_p_convert_result-temp_n_convert_result)/temp_p_pos
	int32_t  pt_value;
    int16_t  current_pt_temp;
	int16_t  current_pt_temp_n_1;
	int16_t  current_pt_temp_n_2;
	int16_t  current_temp;

    int32_t  ad3_convert_result[MD_ADC_MAX_POS];
	uint16_t ad3_pos;
	int32_t  ad3_average_result;
	int32_t ntc_value;
	int16_t ntc_temp;
    
    cal_type_t cal_type;//0 cal press, 1,cal temp
    
    press_cal_param_t press_cal_param;
	press_cal_param_t press_cal_param_bak;
	int16_t (*save_press_cal_param)(void const *,int16_t);
    
	res_cal_param_t  res_cal_param;
    res_cal_param_t  res_cal_param_bak;
    int16_t (*save_res_cal_param)(void const *,int16_t);
   
    flow_cal_param_t flow_cal_param;
    int16_t (*save_flow_cal_param)(void const *,int16_t);
    
   
    flow_meter_t flow_meter;
    int16_t (*save_flow_meter)(void const *,int16_t);
  
    meter_t meter;    
    int16_t (*save_meter)(void const *,int16_t);
    
    meter_backup_t meter_backup;
    int16_t (*save_meter_backup)(void const *,int16_t);
    
   
    coe_t coe;
    int16_t (*save_coe)(void const *,int16_t);
   

    manufacturer_info_t manufacturer_info;
    int16_t (*save_manufacturer_info)(void const *,int16_t);

   
    device_info_t device_info;
    int16_t (*save_device_info)(void const *,int16_t);

   
	sensor_info_t sensor_info;
    int16_t (*save_sensor_info)(void const *,int16_t);

    temp_comp_param_t temp_comp_param;
	int16_t (*save_temp_comp_param)(void const *,int16_t);
    

    alarm_param_t alarm_param;
    int16_t (*save_alarm_param)(void const *,int16_t);

    device_addr_t device_addr;
    int16_t (*save_device_addr)(void const *,int16_t);

	TimeSegData_t TimeSegData;
	int16_t (*read_time_seg_data_param)(void * , int16_t);
	int16_t (*save_time_seg_data_param)(void const * , int16_t);

	system_time_t system_time;
	int16_t (*save_system_time)(void const * , int16_t);

	param_type_t param_type;
	misc_param_t  misc_param;
	int16_t (*save_misc_param)(void const * , int16_t);



    report_param_t report_param;
	int16_t (*save_report_param)(void const *,int16_t);
	
    access_param_t access_param;
    int16_t (*save_access_param)(void const *,int16_t);
	
	lbs_param_t   lbs_param;
    int16_t (*save_lbs_param)(void const * , int16_t);

    iot_param_t iot_param;
    int16_t (*save_iot_param)(void const * , int16_t);
	
	gps_t gps;
    int16_t (*save_gps_param)(void const *,int16_t);
    int16_t (*get_gps_info_from_net)(char const *);
  
	uint8_t debug_info[32];
	void  ( *const output_debug_info)(struct _DEVICE_COMPONENTS *const this);//point to sensor_comps_output_debug_info(device_comps_t const *const this)
    void  ( *const task_handle)(void);//point to device_comps_task_handle
    en_result_t (*const set_system_time)(stc_rtc_time_t* time);
    void (*const net_power_off_call_back)(void);
}device_comps_t; 

extern device_comps_t device_comps;
void _0_5s_task_handle(void);
void _50ms_task_handle(void);


#endif
