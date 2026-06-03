#ifndef _SCI_MASTER_H
#define _SCI_MASTER_H


typedef enum sci_fsm_state_t
{
    SCI_FSM_IDLE = 0,
    SCI_FSM_DM_GET_MEASURE_DATA,
    SCI_FSM_PWM_SET_DUTY,

    SCI_FSM_RADAR_SET_CAL_PARAM,
    SCI_FSM_RADAR_SET_MOUNTING_LIQUID_HEIGHT,
    SCI_FSM_RADAR_SET_MOUNTING_HEIGHT,
    SCI_FSM_RADAR_GET_MEASURE_DATA,

    SCI_FSM_ULTRASONIC_RADAR_GET_MEASURE_DATA,


    SCI_FSM_MAX_REQ_NUMS
}sci_fsm_state_t;

 typedef struct DM_PERIPH_t
{
    union
    {
        uint16_t All;
        struct
        {
            uint8_t is_get_measure_data_timeout: 1;
        } _bit;
    } sw;

    struct
    {
        uint16_t voltage;
        uint16_t current;
    } measure_data;
} DM_PERIPH_t;

typedef struct PWM_PERIPH_t
{
    union
    {
        uint16_t All;
        struct
        {
            uint8_t is_set_duty_timeout : 1;
        } _bit;
    } sw;
    
    struct
    {
        uint16_t duty;
    } setting_data;
} PWM_PERIPH_t;

typedef struct RADAR_PERIPH_t
{
    union
    {
        uint16_t All;
        struct
        {
            uint8_t is_set_cal_param_timeout : 1;
            uint8_t is_set_mounting_height_timeout : 1;
            uint8_t is_set_mounting_liquid_height_timeout : 1;
            uint8_t is_get_measure_data_timeout : 1;

        } _bit;
    } sw;
    
    struct
    {
        float32_t current_liquid_level_m;
        float32_t current_empty_height_m;
    } measure_data;
    
    struct
    {
        int16_t   cal_param_mm;
        float32_t mounting_height_m;
        float32_t mounting_liquid_height_m;
    } setting_data;
} RADAR_PERIPH_t;


typedef struct ULTRASONIC_RADAR_PERIPH_t
{
    union
    {
        uint16_t All;
        struct
        {
            uint8_t is_get_measure_data_timeout : 1;
        } _bit;
    } sw;
    
    struct
    {
        uint16_t current_empty_height_mm;
    } measure_data;
    
    struct
    {
        int16_t   cal_param_mm;
    } setting_data;
} ULTRASONIC_RADAR_PERIPH_t;

typedef struct sci_periph_t
{
    DM_PERIPH_t DM;
    PWM_PERIPH_t PWM;
    RADAR_PERIPH_t RADAR;
    ULTRASONIC_RADAR_PERIPH_t ULTRASONIC_RADAR;
    
} sci_periph_t;

typedef struct sci_comps_t
{
    char   *desc;
    uint8_t is_initialized;
    sci_periph_t periph;

    void (*const recv_1byte_callback)(uint8_t data);
    void (*const sendend_callback)(void);
    uint8_t (*const send_msg)(sci_fsm_state_t state,uint32_t wparam,uint32_t lparam);
    uint8_t (*const get_current_state_request_flag)(sci_fsm_state_t state);
    void (*const task_50ms)(void);
    void (*const task_handle)(void);
}sci_comps_t;
extern sci_comps_t sci_comps;
#endif
