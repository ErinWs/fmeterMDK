#ifndef  MODBUS_H
#define  MODBUS_H
#define MD_SET_RS_485_T_R      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_RS485_DIR_PORT)  ,  MD_RS485_DIR_PIN, TRUE)
#define MD_RESET_RS_485_T_R    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_RS485_DIR_PORT)  ,  MD_RS485_DIR_PIN, FALSE)

typedef	struct
{   
    uint8_t addr;
    uint8_t baud;
    uint8_t cs;
}modebs_param_t;

typedef struct 
{
    uint32_t       op_window_time; 
    uint16_t       cmd_out_raw_4ma_20ma_timer; //unit:s
    modebs_param_t *param_pt;
    union 
    {
    	uint8_t All;
    	struct{
                uint8_t runing		        :1;
                uint8_t busy		            :1;
                
                uint8_t baud_modified 		:1;
               }_bit;
    }sw;

    void (*store_buffer)(uint8_t data);
    int16_t (*save_param)(void);
    uint32_t (*modify_baud)(uint32_t,int16_t);
    void (*const sendend_callback)(void);
    void (*const task_handle)(void);
    
}modbusComps_t;

extern modbusComps_t modbusComps;


#endif