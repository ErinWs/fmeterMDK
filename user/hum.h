
#ifndef   HUM_H
#define   HUM_H





#define   MD_HIDE_DISP                  (sizeof(hum_comps.SEG_TAB)-1)
#define   MD_DIS_H                      16
#define   MD_DIS_G                      17
#define   MD_DIS_h                      18
#define   MD_DIS_c                      19
#define   MD_DIS_J                      20
#define   MD_DIS_L                      21
#define   MD_DIS_n                      22
#define   MD_DIS_N                      23
#define   MD_DIS_o                      24
#define   MD_DIS_p                      25
#define   MD_DIS_q                      26
#define   MD_DIS_r                      27
#define   MD_DIS_t                      28
#define   MD_DIS_U                      29
#define   MD_DIS__                      30



typedef enum  
{
	EM_NORMAL_MODE=0,
	EM_DEBUG_MODE,
	EM_PWD_MODE,
	EM_CAL_QUERY_MODE,   // Calibration  query mode
	EM_CAL_MODIFY_MODE,  //Calibration modify mode
	EM_PARAM_QUERY_MODE, //param query mode
	EM_PARAM_MODIFY_MODE,//param modify mode
	EM_SELF_TEST_MODE,    //test mode used to check ad54x0,e2prom,ds18b20
	EM_REPORT_MODE,
	
	//....TODO.....
	
}mode_type_t;

typedef struct _MODE_COMPONENTS//Handling of keys in different modes
{
	char *desc;
	mode_type_t type;
	void (*on_s_key)(void);
	void (*on_m_key)(void);
	void (*on_j_key)(void);
	void (*on_long_s_key)(void);
	void (*on_long_m_key)(void);
	void (*on_long_j_key)(void);
	void (*on_long_s_and_j_key)(void);
	void (*display)(uint8_t opt);
	uint8_t dis_option;
	uint8_t displayTimer;
}mode_comps_t;

typedef enum
{
	EM_NO_KEY,               //em:enum 
	EM_SHORT_KEY,
	EM_LONG_KEY
} key_type_t;

/*	  display erea
      |******0 erea******|	   |********1 erea**********|
                 |**************2 erea****************|
*/

typedef struct _HUM_COMPONENTS
{
	char *desc;
	struct _HUM_COMPONENTS *this;
	int16_t   do_init;//Whether to initialize,1:init 0,no init
	
	uint32_t count;            //task_handle Called counter
	
	/******************************lcd seg define******************************/
    const uint8_t  SEG_TAB[33];
    uint8_t dig0_0;
    uint8_t dig0_1;
    uint8_t dig0_2;
    uint8_t dig0_3;
    uint8_t dig0_4;
    uint8_t dig0_5;
    uint8_t dig0_6;
    uint8_t dig0_7;
    uint8_t dig0_8;
    uint8_t dot0_pos;
	
	uint8_t dig1_0;
	uint8_t dig1_1;
	uint8_t dig1_2;
	uint8_t dig1_3;
    uint8_t dig1_4;
    uint8_t dig1_5;
    uint8_t dot1_pos;
	
	uint8_t dig2_0;
	uint8_t dig2_1;
	uint8_t dig2_2;
	uint8_t dig2_3;
    uint8_t dig2_4;
    uint8_t dig2_5;
    uint8_t          dot2_pos;

	uint8_t dig3_0;
	uint8_t dig3_1;
	uint8_t dig3_2;
	uint8_t dig3_3;
	uint8_t dig3_4;
	uint8_t dig3_5;
	uint8_t          dot3_pos;

    uint8_t dig4_0;
	uint8_t dig4_1;
	uint8_t dig4_2;
	uint8_t dig4_3;
    uint8_t dig4_4;
    uint8_t dig4_5;
    uint8_t          dot4_pos;

    uint8_t dig5_0;
	uint8_t dig5_1;
	uint8_t dig5_2;
	uint8_t dig5_3;
	uint8_t dig5_4;
	uint8_t dig5_5;
    uint8_t dig5_6;
	uint8_t dig5_7;
    uint8_t dig5_8;
	uint8_t          dot5_pos;
    
    union 
    {
    	uint32_t All;
    	struct{
                volatile uint8_t  cur0		            :1;
                volatile uint8_t  dis0		            :1;
                volatile uint8_t  cur1		            :1;
                volatile uint8_t  dis1		            :1;
                volatile uint8_t  cur2		            :1;
                volatile uint8_t  dis2                  :1;
                volatile uint8_t  cur3                  :1;
                volatile uint8_t  dis3                   :1;
               
                
                volatile uint8_t  cur4		            :1;
                volatile uint8_t  dis4		            :1;
                volatile uint8_t  cur5                  :1;
                volatile uint8_t  dis5                  :1;
                volatile uint8_t  cur6		            :1;
                volatile uint8_t  dis6		            :1;
                volatile uint8_t  cur7      	        :1;
                volatile uint8_t  dis7      		    :1;
                
               	volatile uint8_t  refresh_normal_mode_info :1;
                volatile uint8_t  refressh_meter_data   :1;
                volatile uint8_t  refressh_broken_date   :1;
                volatile uint8_t  test_ok               :1;
                volatile uint8_t  refresh_debug_param   :1;  
                volatile uint8_t  refresh_cal_param :1;
                volatile uint8_t  refresh_param :1;
				volatile uint8_t  refresh_special_symbol :1;

    	}_bit;
    }dis_oper_mark;
	int16_t cursor_0;//0 line cursor position
	int16_t cursor_1;
	int16_t cursor_2;
	int16_t cursor_3;
    int16_t cursor_4;
    int16_t cursor_5;
	int16_t cursor_0_count;
	int16_t cursor_1_count;
	int16_t cursor_2_count;
	int16_t cursor_3_count;
    int16_t cursor_4_count;
    int16_t cursor_5_count;
	/*******************end lcd seg define **************************/ 
    uint8_t device_driver_ram[50];
    uint8_t last_display_buff[50];
    int16_t back_led_timer;
   
	mode_type_t   current_mode;
	
	int16_t up_key;
    int16_t down_key;
    
    void (*enter_report_mode)(void);
    void (*enter_default_mode)(int16_t opt);
    void (*enter_cal_modify_mode)(int16_t opt);
    void (*const task_handle)(void);
	
 
	
	
	//TODO
	
}hum_comps_t;
extern mode_comps_t  mode_comps[];
extern hum_comps_t hum_comps;

#endif
