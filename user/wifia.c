#include "ddl.h"
#include "gpio.h"
#include "r_cg_sau.h"
#include "wifia.h"
#include "net_adapter.h"
#include "inttypes.h"
#include "stdbool.h"


/****************************portable***********************************/
#define GET_WIFI_SLEEP_PIN             GetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_WIFI_SLEEP_PORT), MD_WIFI_SLEEP_PIN)
#define WIFI_SLEEP_PIN_SET_HIGH        SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_WIFI_SLEEP_PORT), MD_WIFI_SLEEP_PIN, TRUE)
#define WIFI_SLEEP_PIN_SET_LOW         SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_WIFI_SLEEP_PORT), MD_WIFI_SLEEP_PIN, FALSE)
#define WIFI_SET_PIN_SET_HIGH          SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_WIFI_SET_PORT), MD_WIFI_SET_PIN, TRUE)
#define WIFI_SET_PIN_SET_LOW           SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_WIFI_SET_PORT), MD_WIFI_SET_PIN, FALSE)

#define WIFI_WAKEUP()         WIFI_SLEEP_PIN_SET_HIGH
#define WIFI_ENTER_SLEEP()    WIFI_SLEEP_PIN_SET_LOW

#define ENTER_CRITICAL_SECTION() do{__disable_irq();}while(0)
#define EXIT_CRITICAL_SECTION()  do{__enable_irq();}while(0)


static uint32_t config_com(uint32_t baud, int16_t parity)
{
     App_Uart1Cfg(baud, parity);
    return baud;
}
static void enable_com(void)
{
    enable_uart1();
}
static void write_com(const uint8_t *buf, uint16_t len)
{
    if (buf != NULL && len > 0)
    {
        R_UART1_Send((uint8_t *)buf, len);
    }
}
static void disable_com(void)
{
    disable_uart1();
    //App_Uart1DeInit();
}
static void init_gpio(void)
{   
    stc_gpio_cfg_t stcGpioCfg;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    DDL_ZERO_STRUCT(stcGpioCfg);
    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.bOutputVal = FALSE;
    Gpio_Init(MD_WIFI_SLEEP_PORT, MD_WIFI_SLEEP_PIN, &stcGpioCfg);
    stcGpioCfg.bOutputVal = TRUE;
    Gpio_Init(MD_WIFI_SET_PORT, MD_WIFI_SET_PIN, &stcGpioCfg);
}

/******************************portable end*********************************/
#define RX_LINE_URC_QUEUE_ITEMS 3
#define RX_LINE_URC_QUEUE_SIZE 16
#define RX_LINE_QUEUE_ITEMS 6
#define RX_LINE_QUEUE_SIZE  64
#define POP_LINE_BUF_SIZE 64



typedef enum config_items_t
{  
    UT_ENTER_CMD_MODE=-1,
    AT_ETH_CH1EN=0,
    AT_ETH_CH1P,
    AT_ETH_CH1STC,
    AT_ETH_CH1HTP,
    AT_ETH_CH1RGP,
    AT_ETH_CH2EN,
    AT_ETH_CH2P,
    AT_ETH_CH2STC,
    AT_ETH_CH2HTP,
    AT_ETH_CH2RGP,
    AT_MQTT_HOST,
    AT_MQTT_USER,
    AT_MQTT_PWD,
    AT_MQTT_CLIENTID,
    AT_MQTT_OPT,
    AT_MQTT_SUB,
    AT_MQTT_PUB,
    AT_UT_WKMODE,
    AT_WIFI_CONFIG,

    TOTAL_ITEMS
} config_items_t;

typedef enum resp_t
{
    RESP_NONE = 0,
    RESP_OK = 1,
    RESP_ERROR,
    RESP_TIMEOUT
} resp_t;

typedef union status_flag_t
{
    uint8_t All;
    struct
    {
        volatile uint8_t comm_connected : 1;
        volatile uint8_t at_test_ok : 1;
        volatile uint8_t has_param_config: 1;
        volatile uint8_t has_run_once : 1;
        volatile uint8_t is_initialized : 1;
        volatile uint8_t cmd_waiting : 1;
        volatile uint8_t in_cmd_mode : 1;
        volatile uint8_t sta_mode_got_ip : 1;
        volatile uint8_t is_tx_busy : 1;
    } _bit;
} status_flag_t;

typedef struct wifi_module_t
{
    uint8_t cmd_tx_buf[128];
    uint8_t rx_buf[RX_LINE_QUEUE_SIZE];
    uint8_t rx_idx;
    uint8_t rx_urc_buf[RX_LINE_URC_QUEUE_SIZE];
    uint8_t rx_urc_idx;
    uint8_t urc_match_idx;
    uint8_t rx_line_queue[RX_LINE_QUEUE_ITEMS][RX_LINE_QUEUE_SIZE];
    int rx_line_queue_head;
    int rx_line_queue_tail;
    uint8_t rx_line_urc_queue[RX_LINE_URC_QUEUE_ITEMS][RX_LINE_URC_QUEUE_SIZE];
    int rx_line_urc_queue_head;
    int rx_line_urc_queue_tail;
    uint8_t pop_line_buf[POP_LINE_BUF_SIZE];
    wifi_fsm_t state;
    wifi_event_t pending_event;
    const uint8_t *pending_tx_buf;
    uint16_t pending_tx_len;

    status_flag_t status_flag;
    config_items_t config_items;

    resp_t resp;
    int cmd_waiting_tmr_150ms;
    int retry_times;
    int ack_error_times;
    wifi_last_error_t last_error;
    int restart_times;

} wifi_module_t;

// static char *strnstr(const char *haystack, const char *needle, size_t len)
// {
//     size_t needle_len;
//     size_t i;

//     if (haystack == NULL || needle == NULL)
//         return NULL;

//     needle_len = strlen(needle);
//     if (needle_len == 0)
//         return (char *)haystack;

//     for (i = 0; i < len && haystack[i] != '\0'; i++)
//     {
//         if (i + needle_len > len)
//             break;
//         if (strncmp(&haystack[i], needle, needle_len) == 0)
//             return (char *)&haystack[i];
//     }

//     return NULL;
// }

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
static bool pop_line(wifi_module_t *m, uint8_t *out, int max_len)
{
    bool ret = false;
    ENTER_CRITICAL_SECTION();
    if (m->rx_line_queue_head == m->rx_line_queue_tail)
    {
        ret = false;
    }
    else
    {
        strncpy((char *)out, (const char *)m->rx_line_queue[m->rx_line_queue_tail], max_len - 1);
        out[max_len - 1] = '\0';
        m->rx_line_queue_tail = (m->rx_line_queue_tail + 1) % RX_LINE_QUEUE_ITEMS;
        ret = true;
    }
    EXIT_CRITICAL_SECTION();
    return ret;
}

static bool pop_line_urc(wifi_module_t *m, uint8_t *out, int max_len)
{
    bool ret = false;
    ENTER_CRITICAL_SECTION();
    if (m->rx_line_urc_queue_head == m->rx_line_urc_queue_tail)
    {
        ret = false;
    }
    else
    {
        strncpy((char *)out, (const char *)m->rx_line_urc_queue[m->rx_line_urc_queue_tail], max_len - 1);
        out[max_len - 1] = '\0';
        m->rx_line_urc_queue_tail = (m->rx_line_urc_queue_tail + 1) % RX_LINE_URC_QUEUE_ITEMS;
        ret = true;
    }
    EXIT_CRITICAL_SECTION();
    return ret;
}

static char *check_response(wifi_module_t *m, const char *expected)
{
    while (pop_line(m, m->pop_line_buf, sizeof(m->rx_urc_buf)))
    {
        if (strstr((const char *)m->pop_line_buf, expected))
        {
            return (char *)m->pop_line_buf;
        }
    }
    return NULL;
}

static wifi_fsm_t process_urc(wifi_module_t *m)
{
    wifi_fsm_t state = m->state;
    // while (pop_line_urc(m, m->pop_line_buf, sizeof(m->rx_urc_buf)))
    // {
    //     if (strstr((const char *)m->pop_line_buf, "URC"))
    //     {
    //        state = WIFI_FSM_UT_RESET;
    //     }
    // }
    return state;
}

static void reset_wifi_fsm_context(wifi_module_t *m)
{
    m->status_flag._bit.cmd_waiting = 0;
    m->resp = RESP_NONE;
    m->retry_times = 0;
    m->ack_error_times = 0;
    m->rx_line_queue_head = 0;
    m->rx_line_queue_tail = 0;
    m->rx_idx = 0;
}

static void process_cmd_result(wifi_module_t *m)
{
    switch (m->state)
    {
        case WIFI_FSM_CONFIG:
            m->config_items++;
            if (m->config_items >= TOTAL_ITEMS)
            {
                m->state = WIFI_FSM_UT_RESET;
                m->config_items = UT_ENTER_CMD_MODE;
            }
            break;
        case WIFI_FSM_UT_RESET:
            m->state = WIFI_FSM_TRANSPARENT;
            if (!m->status_flag._bit.sta_mode_got_ip)
            {
                m->last_error = WIFI_ERROR_NOT_GOT_IP;
            }
            break;
        case WIFI_FSM_OFF:
            WIFI_ENTER_SLEEP();
            disable_com();
            m->state = WIFI_FSM_IDLE;
            break;
        case WIFI_FSM_ON:
            if (!m->status_flag._bit.sta_mode_got_ip || !m->status_flag._bit.has_run_once || m->status_flag._bit.has_param_config)
            {
                m->status_flag._bit.has_run_once = 1;
                m->status_flag._bit.has_param_config = 0;
                m->state = WIFI_FSM_CONFIG;
            }
            else
            {
                m->state = WIFI_FSM_TRANSPARENT;
            }
            break;
        default:
             wifi_request_event(WIFI_EVENT_OFF, NULL, 0);
             break;
    }
    reset_wifi_fsm_context(m);
}
static void ack_ok_handler(wifi_module_t *m)
{
    m->resp = RESP_OK;
    process_cmd_result (m);
}

static void ack_timeout_handler(wifi_module_t *m)
{
    m->retry_times++;
    if (m->retry_times >= 2)
    {
        m->resp = RESP_TIMEOUT;
        m->last_error = WIFI_ERROR_CMD_TIMEOUT;
        process_cmd_result (m);
    }
    else
    {
        m->status_flag._bit.cmd_waiting = 0;
    }
}
static void ack_error_handler(wifi_module_t *m)
{
    m->ack_error_times++;
    if (m->ack_error_times >= 2)
    {
        m->resp = RESP_ERROR;
        m->last_error = WIFI_ERROR_ACK_ERROR;
        process_cmd_result (m);
    }
    else
    {
        m->status_flag._bit.cmd_waiting = 0;
    }
}


static void send_cmd(wifi_module_t *m, const char *cmd, int16_t cmd_timeout_150ms)
{
    m->status_flag._bit.cmd_waiting = 1;
    m->cmd_waiting_tmr_150ms = cmd_timeout_150ms;
    write_com((uint8_t *)cmd, strlen(cmd));
}

static void wifi_off(wifi_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "");
        send_cmd(m, (char *)m->cmd_tx_buf, 1 * 1000 / 150);
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        ack_ok_handler(m);
    }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}

static void wifi_on(wifi_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        WIFI_WAKEUP();
        config_com(115200, 0);
        enable_com();
        m->status_flag._bit.sta_mode_got_ip = 0;
        snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "");
        send_cmd(m, (char *)m->cmd_tx_buf, 10 * 1000 / 150);
    }
    else if(check_response(m, "sta ip"))
    {
         m->cmd_waiting_tmr_150ms = 4;
         m->status_flag._bit.sta_mode_got_ip = 1;
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        ack_ok_handler(m);
    }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}

static void reset(wifi_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        m->status_flag._bit.sta_mode_got_ip = 0;
        snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+UT_RESET\r\n");
        send_cmd(m,(char *)m->cmd_tx_buf, 10 * 1000 / 150);
    }
   else if (check_response(m, "sta ip"))
   {
        m->cmd_waiting_tmr_150ms = 4;
        m->status_flag._bit.sta_mode_got_ip = 1;
   }
   else if (m->cmd_waiting_tmr_150ms <= 0)
   {
       ack_ok_handler(m);
   }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}

static void  config_param(wifi_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        int16_t cmd_timeout_150ms = 1 * 1000 / 150;
        switch (m->config_items)
        {
        case UT_ENTER_CMD_MODE:
            cmd_timeout_150ms = 6 * 1000 / 150;
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "<->");
            break;
        case AT_ETH_CH1EN:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH1EN=ENABLE\r\n");
            break;
        case AT_ETH_CH1STC:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH1STC=0\r\n");
            break;
        case AT_ETH_CH1RGP:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH1RGP=NONE,313233\r\n");
            break;
        case AT_ETH_CH1HTP:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH1HTP=0,313233\r\n");
            break;
        case AT_ETH_CH1P:
//            if (strnlen(device_comps.access_param.ip, sizeof(device_comps.access_param.ip)) < 7 || device_comps.access_param.port == 0)
//            {
//               m->last_error = WIFI_ERROR_INVALID_IP;
//            }
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH1P=TCPC,\"%.25s\",%" PRIu16 "\r\n", "192.168.1.37", (uint16_t)32768);
            break;
        case AT_ETH_CH2EN:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH2EN=DISABLE\r\n");
            break;
        case AT_ETH_CH2STC:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH2STC=0\r\n");
            break;
        case AT_ETH_CH2RGP:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH2RGP=NONE,313233\r\n");
            break;
        case AT_ETH_CH2HTP:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH2HTP=0,313233\r\n");
            break;
        // case AT_ETH_CH2P:
        //     if (strnlen(device_comps.access_param.ip1, sizeof(device_comps.access_param.ip1)) < 7 || device_comps.access_param.port1 == 0)
        //     {
        //         m->last_error = WIFI_ERROR_INVALID_IP;
        //     }
        //     snprintf(m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+ETH_CH2P=TCPC,\"%.25s\",%" PRIu16 "\r\n", device_comps.access_param.ip1, device_comps.access_param.port1);
        //     break;
        case AT_UT_WKMODE:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+UT_WKMODE=TCPUDP\r\n");
             break;
        case AT_WIFI_CONFIG:
            snprintf((char *)m->cmd_tx_buf, sizeof(m->cmd_tx_buf), "AT+WIFI=STA,\"%.20s\",\"%.20s\",WPA_WPA2_PSK\r\n", "ChinaNet-HuE3", "79k94kch");
             break;
        default:
            ack_ok_handler(m);
            return;
        }
        send_cmd(m, (char *)m->cmd_tx_buf, cmd_timeout_150ms);
    }
    else if (m->rx_line_queue_head != m->rx_line_queue_tail)
    {
        while(pop_line(m, m->pop_line_buf, sizeof(m->pop_line_buf)))
        {
            if (strstr((char *)m->pop_line_buf, "OK"))
            {
                ack_ok_handler(m);
                return;
            }
            if (strstr((char *)m->pop_line_buf, "ERROR"))
            {
                if(m->config_items == UT_ENTER_CMD_MODE)
                {
                    ack_ok_handler(m);
                }
                else
                {
                    ack_error_handler(m);
                }
                return;
            }
        }
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        if (m->config_items == UT_ENTER_CMD_MODE)
        {
            ack_ok_handler(m);
        }
        else
        {
            ack_timeout_handler(m);
        }
    }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}


static void init(wifi_module_t *m)
{
    m->config_items = UT_ENTER_CMD_MODE;
    m->state = WIFI_FSM_IDLE;
    m->last_error = WIFI_ERROR_NONE;
    m->resp = RESP_NONE;
    m->rx_line_queue_head = 0;
    m->rx_line_queue_tail = 0;
    m->rx_line_urc_queue_head = 0;
    m->rx_line_urc_queue_tail = 0;
    m->rx_idx = 0;
    m->rx_urc_idx = 0;
    m->urc_match_idx = 0;
    m->status_flag._bit.is_initialized = 1;
    init_gpio();
}


static wifi_fsm_t calc_next_state_from_pending(wifi_module_t *m)
{ 
    switch (m->pending_event)
    {
        case WIFI_EVENT_ON:
            return WIFI_FSM_ON;
        case WIFI_EVENT_OFF:
            return WIFI_FSM_OFF;
        case WIFI_EVENT_SEND_DATA:// only trigger in transparent state and not switch state,so just ignore it
            return m->state;
        case WIFI_EVENT_SWITCH_TO_IP1:
            return WIFI_FSM_SWITCH_TO_IP1;
        default:
            return m->state;
    }
}



static void update_wifi_fsm_state(wifi_module_t *m)
{
    bool is_update_state = false;
    wifi_fsm_t final_state = m->state;
    if (m->pending_event != WIFI_EVENT_NONE)
    {
        final_state = calc_next_state_from_pending(m);
        is_update_state = true;
        m->pending_event = WIFI_EVENT_NONE;
    }
    // while (pop_line_urc(m, m->pop_line_buf, sizeof(m->rx_urc_buf)))
    // {
    //     if (strstr((const char *)m->pop_line_buf, "URC: "))
    //     {
    //        final_state = WIFI_FSM_UT_RESET;
    //        is_update_state = true;
    //     }
    // }
    if(is_update_state)
    {
        m->state = final_state;
        reset_wifi_fsm_context(m);
    }
}



static wifi_module_t wifi_module;
void wifi_task_handler(void)
{
    wifi_module_t *m = &wifi_module;
    if(m->status_flag._bit.is_initialized==0)
    {
        init(m);
        return;
    }
    update_wifi_fsm_state(m);
    switch (m->state)
    {
        case WIFI_FSM_IDLE:
            break;
        case WIFI_FSM_OFF:
            wifi_off(m);
            break;
        case WIFI_FSM_ON:
            wifi_on(m);
            break;
        case WIFI_FSM_CONFIG:
            config_param(m);
            break;
        case WIFI_FSM_UT_RESET:
            reset(m);
            break;
        case WIFI_FSM_TRANSPARENT:
            if (m->pending_tx_len > 0 && m->pending_tx_buf != NULL)
            {
                write_com(m->pending_tx_buf, m->pending_tx_len);
                m->pending_tx_buf = NULL;
            }
            break;
        default:
           m->last_error = WIFI_ERROR_UNKNOWN;
            break;
    }
}



static void push_urc_rx_line_queue_byte(wifi_module_t *m, uint8_t byte)
{
    const char urc_prefix[] = "URC:";
    const int prefix_len = sizeof(urc_prefix) - 1;

    if (byte == '\r' || byte == '\n')
    {
        if (m->rx_urc_idx > 0)
        {
            m->rx_urc_buf[m->rx_urc_idx] = '\0';
            int next_urc = (m->rx_line_urc_queue_head + 1) % RX_LINE_URC_QUEUE_ITEMS;
            if (next_urc == m->rx_line_urc_queue_tail)
            {
                m->rx_line_urc_queue_tail = (m->rx_line_urc_queue_tail + 1) % RX_LINE_URC_QUEUE_ITEMS;
            }
            strncpy((char *)m->rx_line_urc_queue[m->rx_line_urc_queue_head], (char *)m->rx_urc_buf, RX_LINE_URC_QUEUE_SIZE - 1);
            m->rx_line_urc_queue[m->rx_line_urc_queue_head][RX_LINE_URC_QUEUE_SIZE - 1] = '\0';
            m->rx_line_urc_queue_head = next_urc;
        }
        m->rx_urc_idx = 0;
        m->urc_match_idx = 0;
        return;
    }

    if (m->rx_urc_idx > 0)
    {
        if (m->rx_urc_idx < RX_LINE_URC_QUEUE_SIZE - 1)
        {
            m->rx_urc_buf[m->rx_urc_idx++] = byte;
        }
        else
        {
            m->rx_urc_idx = 0;
            m->urc_match_idx = 0;
        }
        return;
    }

    if (byte == (uint8_t)urc_prefix[m->urc_match_idx])
    {
        m->rx_urc_buf[m->urc_match_idx++] = byte;
        if (m->urc_match_idx == prefix_len)
        {
            m->rx_urc_idx = prefix_len;
            m->urc_match_idx = 0;
        }
        return;
    }

    if (byte == (uint8_t)urc_prefix[0])
    {
        m->rx_urc_buf[0] = byte;
        m->urc_match_idx = 1;
    }
    else
    {
        m->urc_match_idx = 0;
    }
}


// interface functions  start
uint8_t wifi_request_event(wifi_event_t event, void *data, uint16_t len)
{
    wifi_module_t *m = &wifi_module;
    switch (event)
    {
        case WIFI_EVENT_SEND_DATA:
            if (m->state != WIFI_FSM_TRANSPARENT || m->pending_tx_len > 0 || data == NULL || len == 0)
            {
                return 1;
            }
            m->pending_tx_buf = data; 
            m->pending_tx_len = len;
            m->pending_event = event;
            break;
         case WIFI_EVENT_SWITCH_TO_IP1:
            m->last_error = WIFI_ERROR_NONE;
            m->pending_event = event;
            break;  
        case WIFI_EVENT_ON:
            if (m->state != WIFI_FSM_IDLE)
            {
                return 1;
            }
            m->last_error = WIFI_ERROR_NONE;
            m->pending_event = event;
            break;
        case WIFI_EVENT_OFF:
            if (m->state != WIFI_FSM_IDLE && m->state != WIFI_FSM_OFF)
            {
                m->pending_event = event;
            }
            break;

        default:
            return 1;
    }
    return 0;
}


wifi_event_t get_wifi_pending_event(void)
{
    return wifi_module.pending_event;
}

wifi_fsm_t get_wifi_fsm(void)
{
    return wifi_module.state;
}

wifi_last_error_t get_wifi_last_error(void)
{
    return wifi_module.last_error;
}


void wifi_com_tx_complete_callback(void)
{
    if(wifi_module.state == WIFI_FSM_TRANSPARENT)
    {
        wifi_module.pending_tx_len = 0;
    }
} 
void wifi_com_rx_byte_isr(uint8_t byte)
{
    wifi_module_t *m= &wifi_module;
    if(m->state == WIFI_FSM_IDLE)
    {
        return;
    }
    if(m->state == WIFI_FSM_TRANSPARENT)
    {
         push_rx_byte_to_net_adapter(&byte, 1);
    }
    else
    {
        if (byte == '\r' || byte == '\n')
        {
            if (m->rx_idx > 0)
            {
                m->rx_buf[m->rx_idx] = '\0';
                int next = (m->rx_line_queue_head + 1) % RX_LINE_QUEUE_ITEMS;
                if (next == m->rx_line_queue_tail)
                {
                    m->rx_line_queue_tail = (m->rx_line_queue_tail + 1) % RX_LINE_QUEUE_ITEMS;
                }
                strncpy((char *)m->rx_line_queue[m->rx_line_queue_head], (char *)m->rx_buf, RX_LINE_QUEUE_SIZE - 1);
                m->rx_line_queue[m->rx_line_queue_head][RX_LINE_QUEUE_SIZE - 1] = '\0';
                m->rx_line_queue_head = next;
            }
            m->rx_idx = 0;
        }
        else if (m->rx_idx < RX_LINE_QUEUE_SIZE-1)
        {
            m->rx_buf[m->rx_idx++] = byte;
        }
        else
        {
            m->rx_idx = 0;
            m->rx_buf[m->rx_idx++] = byte;
        }
    }

    push_urc_rx_line_queue_byte(m, byte);
}

// interface functions  end