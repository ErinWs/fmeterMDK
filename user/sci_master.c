#include "ddl.h"
//#include "rtc.h"
#include "gpio.h"
// #include "pcnt.h"
// #include "lptim.h"
// #include "bgr.h"
// #include "adc.h"
// #include "wdt.h"

//#include "device.h"
#include "r_cg_sau.h"
// #include "elora.h"
// #include "protocol.h"
// #include "net.h"
// #include "irc.h"
// #include "hum.h"
#include "sci_master.h"
// #include "24cxx.h"
// #include "adx.h"
// #include "modbus.h"

#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

/****************************portable ***********************************/
#define MD_SET_MASTER_RS485_T_R   // SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_MASTER_RS485_DIR_PORT)  ,  MD_MASTER_RS485_DIR_PIN, TRUE)
#define MD_RESET_MASTER_RS485_T_R // SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_MASTER_RS485_DIR_PORT)  ,  MD_MASTER_RS485_DIR_PIN, FALSE)

#define MD_SCI_MASTER_RS485_VCM_ON   Gpio_WriteOutputIO(MD_SCI_MASTER_COM_VCM_PORT, MD_SCI_MASTER_COM_VCM_PIN, TRUE)
#define MD_SCI_MASTER_RS485_VCM_OFF  Gpio_WriteOutputIO(MD_SCI_MASTER_COM_VCM_PORT, MD_SCI_MASTER_COM_VCM_PIN, FALSE)

#define MD_SCI_MASTER_PERIPH_VCM_ON
#define MD_SCI_MASTER_PERIPH_VCM_OFF

#define MD_SCI_MASTER_PERIPH_BOOST_VOLTAGE_ON
#define MD_SCI_MASTER_PERIPH_BOOST_VOLTAGE_OFF

#define MD_ENTER_CRITICAL_SECTION() __disable_irq()
#define MD_EXIT_CRITICAL_SECTION()  __enable_irq()

static void nop(void)
{
	__NOP();
}
static int16_t sci_config_com(uint32_t baud, int16_t parity)
{
    App_LpUart1Cfg(baud, parity);
    return baud;
}

static void sci_enable_com(void)
{
    enable_LPuart1();
}
static void sci_disable_com(void)
{
    disable_LPuart1();
   //App_LpUart1DeCfg();
}
static void sci_write_com(uint8_t *buf, uint16_t len)
{
    MD_SET_MASTER_RS485_T_R;
    nop();
    nop();
    nop();
    nop();
    R_LPUART1_Send(buf, len);
}

/******************************portable end*********************************/

typedef struct sci_fsm_t
{
    sci_fsm_state_t current_state;
    sci_fsm_state_t current_request_state;
    struct state_t
    {
        sci_fsm_state_t state;
        uint8_t request_flag;
        void (*send_cmd)(void);
        void (*no_ack_handler)(void);
        uint8_t *(*check_com)(void);
        void (*ack_ok_handler)(uint8_t *data);
    } state[SCI_FSM_MAX_REQ_NUMS];
    // max request number
} sci_fsm_t;

struct sci_misc_t
{
    uint8_t send_buf[64];
    uint8_t recv_buf[64];
    uint8_t recv_pos;
    uint8_t expected_ack_cmd;
    uint8_t expected_ack_addr;
    uint16_t expected_ack_total_len;
    union
    {
        uint16_t all;
        struct
        {
            uint8_t is_state_poll_disable : 1;
            uint8_t cmd_waiting : 1;
        } _bit;
    } sw;
    int16_t no_ack_times;
    int16_t ack_tmr;
    sci_fsm_t fsm;

    uint16_t periph_power_on_delay_tmr_50ms;
} sci_misc={0};

static void sci_recv_1byte_callback(uint8_t data)
{
    if (sci_misc.sw._bit.cmd_waiting)
    {
        if (sci_misc.recv_pos >= sizeof(sci_misc.recv_buf))
        {
            sci_misc.recv_pos = 0;
            memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
        }
        sci_misc.recv_buf[sci_misc.recv_pos] = data;
        sci_misc.recv_pos += 1;
    }
}

static void sci_sendend_callback(void)
{
	nop();
	nop();
	nop();
	nop();
    MD_RESET_MASTER_RS485_T_R;
}



static uint16_t generate_crc(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xffff;
    uint16_t i, j = 0;
    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 8; j != 0; j--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static int16_t get4bByteFloatCode(uint8_t *buf,float32_t x)
{
    int16_t i=0;
    buf[i++] =*((uint8_t *)&x+3);
    buf[i++] =*((uint8_t *)&x+2);
    buf[i++] =*((uint8_t *)&x+1);
    buf[i++] =* (uint8_t *)&x;
    return 4;
}

static float32_t getFloatDataFrom4ByteCode(uint8_t *buf)
{
    uint32_t  dat=((uint32_t)buf[0]<<24)+((uint32_t)buf[1]<<16)+((uint32_t)buf[2]<<8)+buf[3];
    return *(float32_t *)&dat;
}

static uint8_t sci_get_current_state_request_flag(sci_fsm_state_t state)
{
    if (state > SCI_FSM_IDLE && state < SCI_FSM_MAX_REQ_NUMS)
    {
        return sci_misc.fsm.state[state].request_flag;
    }
    return 0;
}

static uint8_t sci_send_msg(sci_fsm_state_t state, uint32_t wparam, uint32_t lparam)
{
    if(state>SCI_FSM_IDLE && state<SCI_FSM_MAX_REQ_NUMS)
    {
        sci_misc.fsm.state[state].request_flag = 1;
        if (state == SCI_FSM_PWM_SET_DUTY)
        {
            sci_comps.periph.PWM.setting_data.duty = (uint16_t)lparam;
        }
        if(state == SCI_FSM_RADAR_SET_CAL_PARAM)
        {
            sci_comps.periph.RADAR.setting_data.cal_param_mm=(uint16_t)lparam;
        }
        if(state == SCI_FSM_RADAR_SET_MOUNTING_LIQUID_HEIGHT)
        {
            sci_comps.periph.RADAR.setting_data.mounting_liquid_height_m=lparam/1000.f;
        }
        if(state == SCI_FSM_RADAR_SET_MOUNTING_HEIGHT)
        {
            sci_comps.periph.RADAR.setting_data.mounting_height_m=lparam/1000.f;
        }
        return 0;
    }
    if(state == SCI_FSM_IDLE)
    {
        MD_SCI_MASTER_PERIPH_VCM_OFF;
        MD_SCI_MASTER_PERIPH_BOOST_VOLTAGE_OFF;
        sci_disable_com();
        sci_misc.fsm.current_state = SCI_FSM_IDLE;
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.no_ack_times = 0;
        sci_misc.ack_tmr = 0;
        if (lparam == 1)
        {
            sci_misc.sw._bit.is_state_poll_disable = 1;
        }
        else
        {
            sci_misc.sw._bit.is_state_poll_disable = 0;
        }
    }
    return 1;
}

static void sci_send_modbus_cmd(uint8_t dev_addr, uint8_t func, uint16_t reg_addr, int16_t reg_cnt, uint16_t *data, int16_t ack_tmr)
{
    int16_t i = 0;
    int16_t k = 0;
    uint16_t crc;
    uint16_t expected_payload_len = 0;
    sci_misc.send_buf[i++] = sci_misc.expected_ack_addr = dev_addr;
    sci_misc.send_buf[i++] = sci_misc.expected_ack_cmd = func;
    sci_misc.send_buf[i++] = reg_addr >> 8;
    sci_misc.send_buf[i++] = reg_addr;
    if (func == 3 || func == 4)
    {
        sci_misc.send_buf[i++] = reg_cnt >> 8;
        sci_misc.send_buf[i++] = reg_cnt;
        expected_payload_len = reg_cnt * 2 + 1;
    }
    else if (func == 6)
    {
        sci_misc.send_buf[i++] = data[0] >> 8;
        sci_misc.send_buf[i++] = data[0];
        expected_payload_len = 4;
    }
    else if (func == 0x10)
    {
        sci_misc.send_buf[i++] = reg_cnt >> 8;
        sci_misc.send_buf[i++] = reg_cnt;
        sci_misc.send_buf[i++] = reg_cnt * 2;
        for (k = 0; k < reg_cnt; k++)
        {
            sci_misc.send_buf[i++] = data[k] >> 8;
            sci_misc.send_buf[i++] = data[k];
        }
        expected_payload_len = 4;
    }
    sci_misc.expected_ack_total_len = (uint16_t)expected_payload_len + 4;
    crc = generate_crc(sci_misc.send_buf, i);
    sci_misc.send_buf[i++] = crc;
    sci_misc.send_buf[i++] = crc >> 8;
    sci_misc.recv_pos = 0;
    sci_write_com(sci_misc.send_buf, i);
    sci_misc.ack_tmr = ack_tmr;
    sci_misc.sw._bit.cmd_waiting = 1;
}

static uint8_t *sci_check_com_atom(uint8_t expected_ack_addr, uint8_t expected_ack_cmd, uint16_t expected_ack_total_len)
{ 
    uint16_t expected_len = expected_ack_total_len;
    if (sci_misc.recv_pos < expected_len)
    {
        return 0;
    }
    while (sci_misc.recv_pos >= expected_len)
    {
        if (sci_misc.recv_buf[0] != expected_ack_addr)
        {
            sci_misc.recv_pos = sci_misc.recv_pos - 1;
            memmove(sci_misc.recv_buf, sci_misc.recv_buf + 1, sci_misc.recv_pos);
            continue;
        }
        if (sci_misc.recv_buf[1] != expected_ack_cmd)
        {
            sci_misc.recv_pos = sci_misc.recv_pos - 1;
            memmove(sci_misc.recv_buf, sci_misc.recv_buf + 1, sci_misc.recv_pos);
            continue;
        }
        if (((uint16_t)sci_misc.recv_buf[expected_len - 1] << 8) + sci_misc.recv_buf[expected_len - 2] != generate_crc(sci_misc.recv_buf, expected_len - 2))
        {
            sci_misc.recv_pos = sci_misc.recv_pos - 1;
            memmove(sci_misc.recv_buf, sci_misc.recv_buf + 1, sci_misc.recv_pos);
            continue;
        }
        return sci_misc.recv_buf;
    }
    return 0;
}

static uint8_t *sci_check_modbus_com(uint8_t expected_ack_addr, uint8_t expected_ack_cmd, uint16_t expected_ack_total_len)
{
    uint8_t *ret;
    MD_ENTER_CRITICAL_SECTION();
    ret=sci_check_com_atom(expected_ack_addr, expected_ack_cmd, expected_ack_total_len);
    MD_EXIT_CRITICAL_SECTION();
    return ret;
}

static uint8_t *sci_check_modbus_com_index(uint8_t expected_ack_addr, uint8_t expected_ack_cmd, uint16_t expected_ack_total_len)
{
    uint16_t index = 0;
    uint16_t index_len = sci_misc.recv_pos;
    uint16_t expected_len = expected_ack_total_len;
    if (index_len < expected_len)
    {
        return 0;
    }
    while (index_len >= expected_len)
    {
        if (sci_misc.recv_buf[index] != expected_ack_addr)
        {
            index++;
            index_len--;
            continue;
        }
        if (sci_misc.recv_buf[index + 1] != expected_ack_cmd)
        {
            index++;
            index_len--;
            continue;
        }
        if (((uint16_t)sci_misc.recv_buf[index + expected_len - 1] << 8) + sci_misc.recv_buf[index + expected_len - 2] != generate_crc(sci_misc.recv_buf + index, expected_len - 2))
        {
            index++;
            index_len--;
            continue;
        }
        return sci_misc.recv_buf + index;
    }
    return 0;
}

static void sci_fsm_finish_current_request(void)
{
    uint8_t cur = (uint8_t)sci_misc.fsm.current_state;
    sci_misc.sw._bit.cmd_waiting = 0;
    sci_misc.no_ack_times = 0;
    sci_misc.ack_tmr = 0;
    if (cur < SCI_FSM_MAX_REQ_NUMS)
    {
        sci_misc.fsm.state[cur].request_flag = 0;
    }
    // MD_SCI_MASTER_PERIPH_VCM_OFF;
    // sci_disable_com();
    sci_misc.fsm.current_state = SCI_FSM_IDLE;
}

static void sci_fsm_dm_get_measure_data_send_cmd(void)
{
    sci_misc.recv_pos = 0;
    memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
    sci_send_modbus_cmd(1, 3, 3, 2, (void *)0, 10); // 10*50ms
}

static uint8_t *sci_fsm_dm_get_measure_data_check_com(void)
{
    return sci_check_modbus_com(sci_misc.expected_ack_addr, sci_misc.expected_ack_cmd, sci_misc.expected_ack_total_len);
}

static void sci_fsm_dm_get_measure_data_no_ack_callback(void)
{
    sci_misc.no_ack_times++;
    if (sci_misc.no_ack_times >= 2)
    {
        sci_fsm_finish_current_request();
    }
    else
    {
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.ack_tmr = 0;
    }
}

static void sci_fsm_dm_get_measure_data_ack_ok_callback(uint8_t *data)
{
    sci_comps.periph.DM.measure_data.current = (((int16_t)data[3] << 8) + data[4]);
    sci_comps.periph.DM.measure_data.voltage = (((int16_t)data[5] << 8) + data[6]);
    sci_fsm_finish_current_request();
}


static void sci_fsm_pwm_set_duty_send_cmd(void)
{
    sci_misc.recv_pos = 0;
    memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
    sci_send_modbus_cmd(1,6,2,1,(uint16_t *)&sci_comps.periph.PWM.setting_data.duty,10);
}

static uint8_t *sci_fsm_pwm_set_duty_check_com(void)
{
    return sci_check_modbus_com(sci_misc.expected_ack_addr, sci_misc.expected_ack_cmd, sci_misc.expected_ack_total_len);
}

static void sci_fsm_pwm_set_duty_no_ack_callback(void)
{
    sci_misc.no_ack_times++;
    if (sci_misc.no_ack_times >= 2)
    {
        sci_fsm_finish_current_request();
    }
    else
    {
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.ack_tmr = 0;
    }
}

static void sci_fsm_pwm_set_duty_ack_ok_callback(uint8_t *data)
{
   sci_fsm_finish_current_request();
}


static void sci_fsm_radar_set_cal_param_send_cmd(void)
{
    sci_misc.recv_pos = 0;
    memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
    sci_send_modbus_cmd(0x7f,16,0x2052,1,(uint16_t *)&sci_comps.periph.RADAR.setting_data.cal_param_mm,10);
}

static uint8_t *sci_fsm_radar_set_cal_param_check_com(void)
{
    return sci_check_modbus_com(sci_misc.expected_ack_addr, sci_misc.expected_ack_cmd, sci_misc.expected_ack_total_len);
}

static void sci_fsm_radar_set_cal_param_no_ack_callback(void)
{
    sci_misc.no_ack_times++;
    if (sci_misc.no_ack_times >= 2)
    {
        sci_fsm_finish_current_request();
    }
    else
    {
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.ack_tmr = 0;
    }
}

static void sci_fsm_radar_set_cal_param_ack_ok_callback(uint8_t *data)
{
   sci_fsm_finish_current_request();
}


static void sci_fsm_radar_set_mounting_liquid_height_send_cmd(void)
{
    uint16_t data[2];
    data[0]=((uint16_t *)&sci_comps.periph.RADAR.setting_data.mounting_liquid_height_m)[0];
    data[1]=((uint16_t *)&sci_comps.periph.RADAR.setting_data.mounting_liquid_height_m)[1];
    sci_misc.recv_pos = 0;
    memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
    sci_send_modbus_cmd(0x7f,16,0x2048,2,data,10);
}

static uint8_t *sci_fsm_radar_set_mounting_liquid_height_check_com(void)
{
    return sci_check_modbus_com(sci_misc.expected_ack_addr, sci_misc.expected_ack_cmd, sci_misc.expected_ack_total_len);
}


static void sci_fsm_radar_set_mounting_liquid_height_no_ack_callback(void)
{
    sci_misc.no_ack_times++;
    if (sci_misc.no_ack_times >= 2)
    {
        sci_fsm_finish_current_request();
    }
    else
    {
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.ack_tmr = 0;
    };
}

static void sci_fsm_radar_set_mounting_liquid_height_ack_ok_callback(uint8_t *data)
{
   sci_fsm_finish_current_request();
}


static void sci_fsm_radar_set_mounting_height_send_cmd(void)
{
    uint16_t data[2];
    data[0]=((uint16_t *)&sci_comps.periph.RADAR.setting_data.mounting_height_m)[0];
    data[1]=((uint16_t *)&sci_comps.periph.RADAR.setting_data.mounting_height_m)[1];
    sci_misc.recv_pos = 0;
    memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
    sci_send_modbus_cmd(0x7f,16,0x204a,2,data,10);
}

static uint8_t *sci_fsm_radar_set_mounting_height_check_com(void)
{
    return sci_check_modbus_com(sci_misc.expected_ack_addr, sci_misc.expected_ack_cmd, sci_misc.expected_ack_total_len);
}

static void sci_fsm_radar_set_mounting_height_no_ack_callback(void)
{
    sci_misc.no_ack_times++;
    if (sci_misc.no_ack_times >= 2)
    {
        sci_fsm_finish_current_request();
    }
    else
    {
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.ack_tmr = 0;
    }
}

static void sci_fsm_radar_set_mounting_height_ack_ok_callback(uint8_t *data)
{
   sci_fsm_finish_current_request();
   sci_send_msg(SCI_FSM_RADAR_GET_MEASURE_DATA,0,0);
}


static void sci_fsm_radar_get_measure_data_send_cmd(void)
{
    sci_misc.recv_pos = 0;
    memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
    sci_send_modbus_cmd(0x7f, 4, 0x0a0b, 6, (void *)0, 10); // 10*50ms
}

static uint8_t *sci_fsm_radar_get_measure_data_check_com(void)
{
    return sci_check_modbus_com(sci_misc.expected_ack_addr, sci_misc.expected_ack_cmd, sci_misc.expected_ack_total_len);
}

static void sci_fsm_radar_get_measure_data_no_ack_callback(void)
{
    sci_misc.no_ack_times++;
    if (sci_misc.no_ack_times >= 2)
    {
        sci_fsm_finish_current_request();
        sci_comps.periph.RADAR.sw._bit.is_get_measure_data_timeout = 1;
    }
    else
    {
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.ack_tmr = 0;
    }
}

static void sci_fsm_radar_get_measure_data_ack_ok_callback(uint8_t *data)
{
    uint32_t  temp;
    float32_t ftemp;
    ((uint8_t *)&temp)[1]=data[3];
    ((uint8_t *)&temp)[0]=data[4];
    ((uint8_t *)&temp)[3]=data[5];
    ((uint8_t *)&temp)[2]=data[6];
    ftemp= *(float32_t *)&temp;
    if(ftemp<0.001)
    {
        ftemp=0;
    }
    sci_comps.periph.RADAR.measure_data.current_liquid_level_m = ftemp;

    ((uint8_t *)&temp)[1]=data[11];
    ((uint8_t *)&temp)[0]=data[12];
    ((uint8_t *)&temp)[3]=data[13];
    ((uint8_t *)&temp)[2]=data[14];
    ftemp= *(float32_t *)&temp;
    if(ftemp<0.001)
    {
        ftemp=0;
    }
    sci_comps.periph.RADAR.measure_data.current_empty_height_m = ftemp;
    sci_fsm_finish_current_request();
    sci_comps.periph.RADAR.sw._bit.is_get_measure_data_timeout = 0;
}


static void sci_fsm_ultrasonic_radar_get_measure_data_send_cmd(void)
{
    sci_misc.recv_pos = 0;
    memset(sci_misc.recv_buf, 0, sizeof(sci_misc.recv_buf));
    sci_send_modbus_cmd(0x01, 3, 0x0100, 1, (void *)0, 10); // 10*50ms
}

static uint8_t *sci_fsm_ultrasonic_radar_get_measure_data_check_com(void)
{
    return sci_check_modbus_com(sci_misc.expected_ack_addr, sci_misc.expected_ack_cmd, sci_misc.expected_ack_total_len);
}

static void sci_fsm_ultrasonic_radar_get_measure_data_no_ack_callback(void)
{
    sci_misc.no_ack_times++;
    if (sci_misc.no_ack_times >= 2)
    {
        sci_fsm_finish_current_request();
        sci_comps.periph.ULTRASONIC_RADAR.sw._bit.is_get_measure_data_timeout = 1;
    }
    else
    {
        sci_misc.sw._bit.cmd_waiting = 0;
        sci_misc.ack_tmr = 0;
    }
}

static void sci_fsm_ultrasonic_radar_get_measure_data_ack_ok_callback(uint8_t *data)
{
    sci_comps.periph.ULTRASONIC_RADAR.measure_data.current_empty_height_mm= (((uint16_t)data[3] << 8) + data[4]);
    sci_fsm_finish_current_request();
    sci_comps.periph.ULTRASONIC_RADAR.sw._bit.is_get_measure_data_timeout = 0;
}




static void sci_state_func_register(sci_fsm_t *pfsm)
{
    pfsm->state[SCI_FSM_DM_GET_MEASURE_DATA].send_cmd = sci_fsm_dm_get_measure_data_send_cmd;
    pfsm->state[SCI_FSM_DM_GET_MEASURE_DATA].no_ack_handler = sci_fsm_dm_get_measure_data_no_ack_callback;
    pfsm->state[SCI_FSM_DM_GET_MEASURE_DATA].check_com = sci_fsm_dm_get_measure_data_check_com;
    pfsm->state[SCI_FSM_DM_GET_MEASURE_DATA].ack_ok_handler = sci_fsm_dm_get_measure_data_ack_ok_callback;

    pfsm->state[SCI_FSM_PWM_SET_DUTY].send_cmd = sci_fsm_pwm_set_duty_send_cmd;
    pfsm->state[SCI_FSM_PWM_SET_DUTY].no_ack_handler = sci_fsm_pwm_set_duty_no_ack_callback;
    pfsm->state[SCI_FSM_PWM_SET_DUTY].check_com = sci_fsm_pwm_set_duty_check_com;
    pfsm->state[SCI_FSM_PWM_SET_DUTY].ack_ok_handler = sci_fsm_pwm_set_duty_ack_ok_callback;

    pfsm->state[SCI_FSM_RADAR_SET_CAL_PARAM].send_cmd = sci_fsm_radar_set_cal_param_send_cmd;
    pfsm->state[SCI_FSM_RADAR_SET_CAL_PARAM].no_ack_handler = sci_fsm_radar_set_cal_param_no_ack_callback;
    pfsm->state[SCI_FSM_RADAR_SET_CAL_PARAM].check_com = sci_fsm_radar_set_cal_param_check_com;
    pfsm->state[SCI_FSM_RADAR_SET_CAL_PARAM].ack_ok_handler = sci_fsm_radar_set_cal_param_ack_ok_callback;

    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_LIQUID_HEIGHT].send_cmd = sci_fsm_radar_set_mounting_liquid_height_send_cmd;
    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_LIQUID_HEIGHT].no_ack_handler = sci_fsm_radar_set_mounting_liquid_height_no_ack_callback;
    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_LIQUID_HEIGHT].check_com = sci_fsm_radar_set_mounting_liquid_height_check_com;
    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_LIQUID_HEIGHT].ack_ok_handler = sci_fsm_radar_set_mounting_liquid_height_ack_ok_callback;

    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_HEIGHT].send_cmd = sci_fsm_radar_set_mounting_height_send_cmd;
    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_HEIGHT].no_ack_handler = sci_fsm_radar_set_mounting_height_no_ack_callback;
    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_HEIGHT].check_com = sci_fsm_radar_set_mounting_height_check_com;
    pfsm->state[SCI_FSM_RADAR_SET_MOUNTING_HEIGHT].ack_ok_handler = sci_fsm_radar_set_mounting_height_ack_ok_callback;

    pfsm->state[SCI_FSM_RADAR_GET_MEASURE_DATA].send_cmd = sci_fsm_radar_get_measure_data_send_cmd;
    pfsm->state[SCI_FSM_RADAR_GET_MEASURE_DATA].no_ack_handler = sci_fsm_radar_get_measure_data_no_ack_callback;
    pfsm->state[SCI_FSM_RADAR_GET_MEASURE_DATA].check_com = sci_fsm_radar_get_measure_data_check_com;
    pfsm->state[SCI_FSM_RADAR_GET_MEASURE_DATA].ack_ok_handler = sci_fsm_radar_get_measure_data_ack_ok_callback;

    pfsm->state[SCI_FSM_ULTRASONIC_RADAR_GET_MEASURE_DATA].send_cmd = sci_fsm_ultrasonic_radar_get_measure_data_send_cmd;
    pfsm->state[SCI_FSM_ULTRASONIC_RADAR_GET_MEASURE_DATA].no_ack_handler = sci_fsm_ultrasonic_radar_get_measure_data_no_ack_callback;
    pfsm->state[SCI_FSM_ULTRASONIC_RADAR_GET_MEASURE_DATA].check_com = sci_fsm_ultrasonic_radar_get_measure_data_check_com;
    pfsm->state[SCI_FSM_ULTRASONIC_RADAR_GET_MEASURE_DATA].ack_ok_handler = sci_fsm_ultrasonic_radar_get_measure_data_ack_ok_callback;
}

static int try_start_state(sci_fsm_t *pfsm, uint8_t idx)
{
    if (idx >= SCI_FSM_MAX_REQ_NUMS) return 0;
    if (!pfsm->state[idx].request_flag) return 0;
    if (pfsm->state[idx].send_cmd == NULL ||
        pfsm->state[idx].no_ack_handler == NULL ||
        pfsm->state[idx].check_com == NULL ||
        pfsm->state[idx].ack_ok_handler == NULL)
    {
         pfsm->state[idx].request_flag = 0;
        return 0;
    }

    
    if(idx == SCI_FSM_RADAR_GET_MEASURE_DATA)
    {
        MD_SCI_MASTER_PERIPH_VCM_ON;
        sci_misc.periph_power_on_delay_tmr_50ms = 7; // 7*50ms
    }
    else if(idx == SCI_FSM_ULTRASONIC_RADAR_GET_MEASURE_DATA)
    {
        MD_SCI_MASTER_PERIPH_BOOST_VOLTAGE_ON;
        MD_SCI_MASTER_RS485_VCM_ON;
        sci_misc.periph_power_on_delay_tmr_50ms = 3; // 2*50ms
    }
    else
    {
        sci_misc.periph_power_on_delay_tmr_50ms = 7; // 7*50ms
    }
    
    nop(); /* small delay */
    sci_config_com(9600, 0);
    sci_enable_com();

    pfsm->current_state = (sci_fsm_state_t)idx;
    pfsm->current_request_state = (pfsm->current_state + 1) % SCI_FSM_MAX_REQ_NUMS;
    sci_misc.sw._bit.cmd_waiting = 0;
    sci_misc.ack_tmr = 0;
    return 1;
}

static sci_fsm_t sci_fsm_switch_state_by_request_state(sci_fsm_t fsm)
{
    uint16_t i;
    uint8_t start = (uint8_t)fsm.current_request_state;
    for (i = 0; i < (uint16_t)SCI_FSM_MAX_REQ_NUMS; i++)
    {
        uint8_t idx = (start + i) % SCI_FSM_MAX_REQ_NUMS;
        if (idx == (uint8_t)SCI_FSM_IDLE) continue;
        if (try_start_state(&fsm, idx))
        {
            return fsm;
        }
    }
    MD_SCI_MASTER_PERIPH_VCM_OFF;
    MD_SCI_MASTER_PERIPH_BOOST_VOLTAGE_OFF;
    MD_SCI_MASTER_RS485_VCM_OFF;
    sci_disable_com();
    fsm.current_request_state = SCI_FSM_IDLE;
    return fsm;
}

static void sci_comps_init(void)
{
    sci_misc.fsm.current_state= SCI_FSM_IDLE;
    sci_misc.fsm.current_request_state = 0;
    sci_misc.sw._bit.cmd_waiting = 0;
    sci_misc.no_ack_times = 0;
    sci_state_func_register(&sci_misc.fsm);
    sci_comps.is_initialized=1;
}

static void sci_fsm_handler(sci_fsm_t fsm)
{
    uint8_t *adu;
    if (sci_misc.periph_power_on_delay_tmr_50ms > 0)
    {
        return;
    }
    if (!sci_misc.sw._bit.cmd_waiting)
    {
        fsm.state[fsm.current_state].send_cmd();
    }
    else if (sci_misc.ack_tmr < 2)
    {
        fsm.state[fsm.current_state].no_ack_handler();
    }
    else
    {
        adu = fsm.state[fsm.current_state].check_com();
        if (adu)
        {
            fsm.state[fsm.current_state].ack_ok_handler(adu);
        }
    }
}

static void sci_task_50ms(void)
{
    if (sci_misc.periph_power_on_delay_tmr_50ms > 0)
    {
        sci_misc.periph_power_on_delay_tmr_50ms--;
    }
    if (sci_misc.ack_tmr > 0)
    {
        sci_misc.ack_tmr--;
    }
}

static void sci_task_handle(void)
{ 
    if(sci_comps.is_initialized==0)
    {
        sci_comps_init();
        return;
    }
    if(!sci_misc.sw._bit.is_state_poll_disable)
    {
        if(sci_misc.fsm.current_state==SCI_FSM_IDLE)
        {
            sci_misc.fsm=sci_fsm_switch_state_by_request_state(sci_misc.fsm);
        }
        else
        {
            sci_fsm_handler(sci_misc.fsm);
        }
    }
 }

sci_comps_t sci_comps=
{
    "",
    0,//is_initialized
    {0}, //periph

    sci_recv_1byte_callback,
    sci_sendend_callback, 
    sci_send_msg,
    sci_get_current_state_request_flag,
    sci_task_50ms,
    sci_task_handle
};