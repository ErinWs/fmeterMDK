#include "ddl.h"
#include "rtc.h"
#include "gpio.h"
#include "device.h"
#include "r_cg_sau.h"
#include "wifi.h"
#include "trans.h"
#include "stdlib.h"
#include "math.h"

#define WIFI_STARTING_DELAY     200  //200*50ms=10s
#define WIFI_TCP                1    //

static char wifi_tcp_param_argv_template[][64]= 
{
    "AT+ETH_CH1EN=ENABLE\r\n",
    "AT+ETH_CH2EN=DISABLE\r\n",
    "AT+ETH_CH1P=TCPC,\"\",0\r\n",
    "AT+ETH_CH2P=TCPC,\"\",0\r\n",
    "AT+ETH_CH1STC=0\r\n",
    "AT+ETH_CH2STC=0\r\n",
    "AT+ETH_CH1HTP=0,363534333231\r\n",
    "AT+ETH_CH2HTP=0,363534333231\r\n",
    "AT+ETH_CH1RGP=NONE,363534333231\r\n",
    "AT+ETH_CH2RGP=NONE,363534333231\r\n",
    "AT+UT_WKMODE=TCPUDP\r\n"
};

static char wifi_mqtt_param_argv_template[][64] = 
{
    "AT+MQTT_HOST=\"\",0",
    "AT+MQTT_USER=\"\"",
    "AT+MQTT_PWD=\"\"",
    "AT+MQTT_CLIENTID=\"\"",
    "AT+MQTT_OPT=FALSE,30",
    "AT+MQTT_SUB=\"\",QOS0",
    "AT+MQTT_PUB=\"\",QOS0,FALSE",
    "AT+UT_WKMODE=MQTT\r\n"
};

static struct wifi_misc_t
{
    char cmd_buf[128];
    uint8_t recv_buf[312];
    uint16_t recv_pos;
    uint8_t urc_buf[64];
    uint16_t urc_pos;
    uint8_t cfg_index;
    wifi_fsm_t fsm;
    union
    {
        uint16_t all;
        struct
        {
            uint8_t recv_1byte_data : 1;
            uint8_t cmd_waiting : 1;
        } _bit;
    } sw;
    uint8_t recv_data;
    uint8_t no_ack_times;
    int16_t ack_tmr;
} wifi_misc = {0};

/****************************portable***********************************/
#define WIFI_PWR_CTL_PIN             GetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_INVALID_PORT), MD_INVALID_PIN)
#define WIFI_PWR_CTL_PIN_SET_HIGH    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_INVALID_PORT), MD_INVALID_PIN, TRUE)
#define WIFI_PWR_CTL_PIN_SET_LOW     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_INVALID_PORT), MD_INVALID_PIN, FALSE)
#define WIFI_SET_PIN_SET_HIGH        SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_INVALID_PORT), MD_INVALID_PIN, TRUE)
#define WIFI_SET_PIN_SET_LOW         SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_INVALID_PORT), MD_INVALID_PIN, FALSE)


static uint32_t wifi_config_com(uint32_t baud, int16_t parity)
{
    App_Uart1Cfg(baud, parity);
    return baud;
}

static void wifi_enable_com(void)
{
    enable_uart1();
    wifi_misc.recv_pos = 0;
}

static void wifi_disable_com(void)
{
    disable_uart1();
}

static void wifi_write_com(uint8_t *buf, uint16_t len)
{
     R_UART1_Send(buf, len);
}

static void wifi_recv_1byte_callback(uint8_t data)
{
    if (wifi_misc.recv_pos < sizeof(wifi_misc.recv_buf))
    {
        wifi_misc.recv_buf[wifi_misc.recv_pos] = data;
        wifi_misc.recv_pos++;
    }
    else
    {
        memmove(wifi_misc.recv_buf, wifi_misc.recv_buf + 1, sizeof(wifi_misc.recv_buf) - 1);
        wifi_misc.recv_buf[sizeof(wifi_misc.recv_buf) - 1] = data;
    }

    wifi_misc.recv_data = data;
    wifi_misc.sw._bit.recv_1byte_data = 1;

    if (wifi_misc.urc_pos < sizeof(wifi_misc.urc_buf))
    {
        wifi_misc.urc_buf[wifi_misc.urc_pos] = data;
        wifi_misc.urc_pos++;
    }
    else
    {
        memmove(wifi_misc.urc_buf, wifi_misc.urc_buf + 1, sizeof(wifi_misc.urc_buf) - 1);
        wifi_misc.urc_buf[sizeof(wifi_misc.urc_buf) - 1] = data;
    }
}

// static void wifi_recv_1byte_callback(uint8_t data)
// {
//     if(wifi_misc.recv_pos>=sizeof(wifi_misc.recv_buf))
//     {
//         wifi_misc.recv_pos=0;
//         memset(wifi_misc.recv_buf,0,sizeof(wifi_misc.recv_buf));
//     }
//     wifi_misc.recv_buf[wifi_misc.recv_pos]=data;
//     wifi_misc.recv_pos+=1;
//     wifi_misc.recv_data=data;
//     wifi_misc.sw._bit.recv_1byte_data=1;
// }

static void wifi_sendend_callback(void)
{
    wifi_comps.data_msg_len=0;
}
/******************************portable end*********************************/

static char *strnstr(const char *haystack, const char *needle, size_t len)
{
    size_t needle_len;
    size_t i;

    if (haystack == NULL || needle == NULL)
        return NULL;

    needle_len = strlen(needle);
    if (needle_len == 0)
        return (char *)haystack;

    for (i = 0; i < len && haystack[i] != '\0'; i++)
    {
        if (i + needle_len > len)
            break;
        if (strncmp(&haystack[i], needle, needle_len) == 0)
            return (char *)&haystack[i];
    }

    return NULL;
}

//static size_t strnlen(const char *s, size_t maxlen)
//{
//    size_t len = 0;
//    if (s == 0)
//    {
//        return 0;
//    }
//    while (len < maxlen && s[len] != '\0')
//    {
//        len++;
//    }
//    return len;
//}

static char * wifi_check_com(char *str)
{
    if (wifi_misc.recv_pos == 0)
    {
        return 0;
    }
    return strnstr((const char *)wifi_misc.recv_buf, str, wifi_misc.recv_pos);
}

static wifi_fsm_t wifi_check_urc(wifi_fsm_t fsm, uint8_t *buf, uint16_t *pos)
{
    // if (*pos == 0)
    // {
    //     return fsm;
    // }
    // if (strnstr((const char *)buf, "WIFI DISCONNECT", *pos))
    // {
    //     *pos = 0;
    //     memset(buf, 0, sizeof(wifi_misc.urc_buf));
    //     return WIFI_FSM_OFF;
    // }
    // else if (strnstr((const char *)buf, "WIFI CONNECTED", *pos))
    // {
    //     *pos = 0;
    //     memset(buf, 0, sizeof(wifi_misc.urc_buf));
    //     return WIFI_FSM_TRANSPARENT;
    // }
    return fsm;
}

static wifi_fsm_t wifi_urc_handler(wifi_fsm_t gfsm)
{
    return wifi_check_urc(gfsm, &wifi_misc.urc_buf[0], &wifi_misc.urc_pos);
}

static wifi_fsm_t wifi_get_fsm(void)
{
    return wifi_misc.fsm;
}

static uint8_t wifi_send_msg(wifi_msg_type_t msg_type, uint8_t *msg, uint16_t len)
{
    switch (msg_type)
    {
        case WIFI_SEND_DATA:
            if (wifi_misc.fsm != WIFI_FSM_TRANSPARENT || wifi_comps.data_msg_len > 0)
            {
                return 1;
            }
            wifi_comps.data_msg = msg;
            wifi_comps.data_msg_len = len;
            wifi_comps.msg_type = WIFI_SEND_DATA;
            return 0;

        case WIFI_MSG_ON:
            if (wifi_misc.fsm != WIFI_FSM_IDLE)
            {
                return 1;
            }
            wifi_comps.msg_type = WIFI_MSG_ON;
            return 0;

        case WIFI_MSG_OFF:
            if(wifi_misc.fsm != WIFI_FSM_IDLE)
            {
                wifi_comps.msg_type = WIFI_MSG_OFF;
            }
            return 0;

        default:
            return 1;
    }
    return 1;
}

static void wifi_ack_ok_handler(wifi_fsm_t fsm)
{
    wifi_misc.ack_tmr = 0;
    wifi_misc.sw._bit.cmd_waiting = 0;
    wifi_misc.no_ack_times = 0;
}

static void wifi_no_ack_handler(wifi_fsm_t fsm)
{
    if (wifi_misc.no_ack_times < 1)
    {
        wifi_misc.no_ack_times++;
    }
    else
    {
        wifi_misc.fsm = WIFI_FSM_OFF;
        wifi_misc.no_ack_times = 0;
    }
    wifi_misc.ack_tmr = 0;
    wifi_misc.sw._bit.cmd_waiting = 0;
}

static void wifi_error_handler(wifi_fsm_t fsm, wifi_error_t error)
{
    wifi_misc.fsm = WIFI_FSM_OFF;
    wifi_misc.ack_tmr = 0;
    wifi_misc.sw._bit.cmd_waiting = 0;
    wifi_misc.no_ack_times = 0;
}

static void wifi_send_cmd(char *cmd,int16_t time_out)
{
    wifi_misc.recv_pos = 0;
    memset(wifi_misc.recv_buf, 0, sizeof(wifi_misc.recv_buf));
   // memset(wifi_misc.recv_buf, 0, wifi_misc.recv_pos);
    wifi_misc.ack_tmr = time_out;
    wifi_misc.sw._bit.cmd_waiting = 1;
    wifi_write_com((uint8_t *)cmd, strlen(cmd));
}

static void wifi_off(wifi_fsm_t fsm)
{
    if (!wifi_misc.sw._bit.cmd_waiting)
    {
        wifi_send_cmd("<->", 10);
        wifi_misc.sw._bit.cmd_waiting = 1;
    }
    else if (wifi_misc.ack_tmr <= 2)
    {
        wifi_ack_ok_handler(fsm);
        WIFI_SET_PIN_SET_LOW;
        WIFI_PWR_CTL_PIN_SET_LOW;
        wifi_disable_com();
        wifi_misc.fsm = WIFI_FSM_IDLE;
      
    }
}

static void wifi_on(wifi_fsm_t fsm)
{
    if (wifi_misc.ack_tmr == 0)
    {
        wifi_misc.ack_tmr = 10;
        WIFI_PWR_CTL_PIN_SET_HIGH;
        WIFI_SET_PIN_SET_HIGH;
    }
    else if (wifi_misc.ack_tmr <= 2)
    {
        wifi_config_com(115200, 0);
        wifi_enable_com();
        wifi_misc.cfg_index = 0;
        wifi_comps.sw.all = 0; 
        wifi_ack_ok_handler(fsm);
        memset(wifi_misc.recv_buf, 0, sizeof(wifi_misc.recv_buf));
        wifi_misc.fsm = WIFI_FSM_STARTING;
    }
}

static void wifi_starting(wifi_fsm_t fsm)
{
    if (wifi_misc.ack_tmr == 0)
    {
        wifi_misc.ack_tmr = WIFI_STARTING_DELAY;
    }
    else if (wifi_misc.ack_tmr == WIFI_STARTING_DELAY - 6)
    {
        WIFI_SET_PIN_SET_LOW;
        wifi_misc.ack_tmr = WIFI_STARTING_DELAY - 6 - 1;
    }
    else if (wifi_misc.ack_tmr == WIFI_STARTING_DELAY - 6 - 1 - 3)
    {
        WIFI_SET_PIN_SET_HIGH;
        wifi_misc.ack_tmr = WIFI_STARTING_DELAY - 6 - 1 - 3 - 1;
    }
    else if(wifi_check_com("sta ip"))
    {
         wifi_misc.ack_tmr = 6;
    }
    else if (wifi_misc.ack_tmr <= 2)
    {
        wifi_ack_ok_handler(fsm);
        if (wifi_check_com("sta ip"))
        {
            wifi_misc.fsm = WIFI_FSM_PARAM_SELECT;
        }
        else
        {
            wifi_misc.fsm = WIFI_FSM_SET_STA_MODE;
        }
	}
}

static void wifi_set_sta_mode(wifi_fsm_t fsm)
{
   if (!wifi_misc.sw._bit.cmd_waiting)
   {
        //snprintf(wifi_misc.cmd_buf, sizeof(wifi_misc.cmd_buf), "AT+WIFI=STA,\"%.20s\",\"%.20s\",WPA_WPA2_PSK\r\n", device_comps.wifi_ssid, device_comps.wifi_pwd);
        wifi_send_cmd(wifi_misc.cmd_buf, 10);
   }
   else if (wifi_check_com("OK\r\n"))
   {
      wifi_ack_ok_handler(fsm);
      wifi_misc.fsm = WIFI_FSM_PARAM_SELECT;
   }
   else if (wifi_misc.ack_tmr <= 2)
   {
       wifi_no_ack_handler(fsm);
   }
}


static wifi_error_t wifi_prepare_tcp_param(void)
{
    size_t i = 0;
    for (i = 0; i < sizeof(wifi_tcp_param_argv_template) / sizeof(wifi_tcp_param_argv_template[0]); i++)
    {
        if (strnstr(wifi_tcp_param_argv_template[i], "AT+ETH_CH1P=TCPC,", sizeof(wifi_tcp_param_argv_template[i])))
        {
            if (strnlen(device_comps.access_param.ip, sizeof(device_comps.access_param.ip)) < 7 || device_comps.access_param.port == 0)
            {
                return WIFI_ERROR_INVALID_IP;
            }
            snprintf(wifi_tcp_param_argv_template[i], sizeof(wifi_tcp_param_argv_template[i]), "AT+ETH_CH1P=TCPC,\"%s\",%hd\r\n", device_comps.access_param.ip, device_comps.access_param.port);
        }
    }
    wifi_misc.cfg_index = 0;
    wifi_misc.fsm = WIFI_FSM_WRITE_TCP_PARAM;
    return WIFI_ERROR_NONE;
}

static wifi_error_t wifi_prepare_mqtt_param(void)
{
    size_t i = 0;
    for (i = 0; i < sizeof(wifi_mqtt_param_argv_template) / sizeof(wifi_mqtt_param_argv_template[0]); i++)
    {
        if (strnstr(wifi_mqtt_param_argv_template[i], "AT+MQTT_HOST=", sizeof(wifi_mqtt_param_argv_template[i])))
        {
            snprintf(wifi_mqtt_param_argv_template[i], sizeof(wifi_mqtt_param_argv_template[i]), "AT+MQTT_HOST=\"%s\",%hd\r\n", device_comps.access_param.ip, device_comps.access_param.port);
        }
    }
    wifi_misc.cfg_index = 0;
    wifi_misc.fsm = WIFI_FSM_WRITE_MQTT_PARAM;
    return WIFI_ERROR_NONE;
}

static void wifi_write_tcp_param(wifi_fsm_t fsm)
{
    if (!wifi_misc.sw._bit.cmd_waiting)
    {
        wifi_send_cmd(wifi_tcp_param_argv_template[wifi_misc.cfg_index], 10);
    }
    else if (wifi_check_com("OK\r\n"))
    {
        wifi_misc.cfg_index++;
        if (wifi_misc.cfg_index >= sizeof(wifi_tcp_param_argv_template) / sizeof(wifi_tcp_param_argv_template[0]))
        {
            wifi_misc.fsm = WIFI_FSM_UT_RESET;
        }
        wifi_ack_ok_handler(fsm);
    }
    else if (wifi_misc.ack_tmr <= 2)
    {
        wifi_no_ack_handler(fsm);
    }
}


static void wifi_write_mqtt_param(wifi_fsm_t fsm)
{
    if (!wifi_misc.sw._bit.cmd_waiting)
    {
        wifi_send_cmd(wifi_mqtt_param_argv_template[wifi_misc.cfg_index], 10);
    }
    else if (wifi_check_com("OK\r\n"))
    {
        wifi_misc.cfg_index++;
        if (wifi_misc.cfg_index >= sizeof(wifi_mqtt_param_argv_template) / sizeof(wifi_mqtt_param_argv_template[0]))
        {
            wifi_misc.fsm = WIFI_FSM_UT_RESET;
        }
        wifi_ack_ok_handler(fsm);
    }
    else if (wifi_misc.ack_tmr <= 2)
    {
        wifi_no_ack_handler(fsm);
    }
}


static void wifi_soft_reset(wifi_fsm_t fsm)
{
    if (!wifi_misc.sw._bit.cmd_waiting)
    {
        wifi_send_cmd("AT+UT_RESET\r\n", WIFI_STARTING_DELAY);
    }
    else if(wifi_misc.ack_tmr == WIFI_STARTING_DELAY - 10)
    {
        if (wifi_check_com("OK\r\n"))
        {
            wifi_misc.ack_tmr = WIFI_STARTING_DELAY - 10 -1;
        }
        else
        {
            wifi_no_ack_handler(fsm);
        }
    }
    else if (wifi_check_com("sta ip"))
    {
        wifi_misc.ack_tmr = 6;
    }
    else if (wifi_misc.ack_tmr <= 2)
    {
        if(wifi_check_com("sta ip"))
        {
            wifi_ack_ok_handler(fsm);
            wifi_misc.fsm = WIFI_FSM_TRANSPARENT;
        }
        else
        {
            wifi_comps.last_error = WIFI_ERROR_NOT_GOT_IP;
            wifi_error_handler(fsm, wifi_comps.last_error);
        }
    }
}


static void wifi_transparent_mode_handler(void)
{
    if (wifi_comps.data_msg_len > 0 && wifi_comps.data_msg != (void *)0)
    {
        wifi_write_com(wifi_comps.data_msg, wifi_comps.data_msg_len);
        wifi_comps.data_msg = 0;
    }
    if (wifi_misc.sw._bit.recv_1byte_data)
    {
        wifi_misc.sw._bit.recv_1byte_data = 0;
        trans_comps.recv_1byte_callback(wifi_misc.recv_data);
    }
}

static void wifi_comps_init(void)
{
    wifi_misc.fsm=WIFI_FSM_IDLE;
    wifi_comps.is_initialized=1;
}


static wifi_fsm_t wifi_msgs_handler(wifi_msg_type_t msg_type,wifi_fsm_t gfsm)
{
    switch(msg_type)
    {
        case WIFI_MSG_ON:
            gfsm=WIFI_FSM_ON;
            break;
        case WIFI_MSG_OFF:
            gfsm=WIFI_FSM_OFF;
            break;
        case WIFI_SEND_DATA:
            break;
        case WIFI_MSG_NONE:
            break;
        default:
            break;
    }
    return gfsm;
}

static void wifi_fsm_handler(wifi_fsm_t gfsm)
{
    wifi_fsm_t fsm = wifi_urc_handler(gfsm);
    switch (fsm)
    {
        case WIFI_FSM_IDLE:
            break;
        case WIFI_FSM_OFF:
            wifi_off(fsm);
            break;
        case WIFI_FSM_ON:
            wifi_on(fsm);
            break;
        case WIFI_FSM_STARTING:
            wifi_starting(fsm);
            break;
        case WIFI_FSM_SET_STA_MODE:
            wifi_set_sta_mode(fsm);
            break;
        case WIFI_FSM_PARAM_SELECT:
            #if defined(WIFI_TCP)
            wifi_comps.last_error = wifi_prepare_tcp_param();
            #else
            wifi_comps.last_error = wifi_prepare_mqtt_param();
            #endif
            if (wifi_comps.last_error != WIFI_ERROR_NONE)
            {
                wifi_error_handler(fsm, wifi_comps.last_error);
            }
            break;
        case WIFI_FSM_WRITE_TCP_PARAM:
            wifi_write_tcp_param(fsm);
            break;
        case WIFI_FSM_WRITE_MQTT_PARAM:
            wifi_write_mqtt_param(fsm);
            break;
        case WIFI_FSM_UT_RESET:
            wifi_soft_reset(fsm);
            break;
        case WIFI_FSM_TRANSPARENT:
            wifi_transparent_mode_handler();
            break;
        default:
            wifi_misc.fsm = WIFI_FSM_IDLE;
            break;
    }
}

static void wifi_task_50ms(void)
{
    if (wifi_misc.ack_tmr > 0)
    {
        wifi_misc.ack_tmr--;
    }
}

static void wifi_task_handler(void)
{
    if(wifi_comps.is_initialized==0)
    {
        wifi_comps_init();
    }
    if(wifi_comps.msg_type!=WIFI_MSG_NONE)
    {
        wifi_misc.fsm = wifi_msgs_handler(wifi_comps.msg_type, wifi_misc.fsm);
        wifi_comps.msg_type=WIFI_MSG_NONE;
    }
    wifi_fsm_handler(wifi_misc.fsm);
}

wifi_comps_t wifi_comps =
{
    0,//uint8_t is_initialized;
    (void *)0,//char *data_msg;
    0,//uint16_t data_msg_len;
    WIFI_MSG_NONE,//wifi_msg_type_t msg_type;
    WIFI_ERROR_NONE,//wifi_error_t last_error;
    {0},//union

    wifi_recv_1byte_callback,//void (*const recv_1byte_callback)(uint8_t data);
    wifi_sendend_callback,//void (*const sendend_callback)(void);
    wifi_send_msg,//wifi_msg_t (*const send_msg)(wifi_msg_type_t msg_type,uint8_t *msg,uint16_t len);
    wifi_get_fsm,//wifi_fsm_t (*const get_fsm_state)(void);
    wifi_task_50ms,//void (*const task_50ms)(void);
    wifi_task_handler//void (*const task_handle)(void);
};