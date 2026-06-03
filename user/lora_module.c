#include "ddl.h"
#include "rtc.h"
#include "gpio.h"
#include "device.h"
#include "r_cg_sau.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"

/// portable

#define ENTER_CRITICAL_SECTION() do{__disable_irq();}while(0)
#define EXIT_CRITICAL_SECTION()  do{__enable_irq();}while(0)

static uint32_t config_com(uint32_t baud, int16_t parity)
{
    // App_Uart3Cfg(baud, parity);
    return baud;
}
static void enable_com(void)
{
    enable_uart3();
}
static void write_com(const uint8_t *buf, uint16_t len)
{
    if (buf != NULL && len > 0)
    {
        R_UART3_Send((uint8_t *)buf, len);
    }
}
static void disable_com(void)
{
    disable_uart3();
    //App_Uart3DeInit();
}

/// portable end

#define LINE_RX_QUEUE_ITEMS 6
#define LINE_QUEUE_ITEM_SIZE 64
#define RX_BUF_SIZE 64

typedef enum join_mode_t
{
    OTAA = 1,
    ABP = 0
} join_mode_t;

typedef enum lora_fsm_t
{
    IDLE,
    RESET,
    CONFIG,
    JOIN,
    SEND,
    RECV,

} lora_fsm_t;

typedef struct tx_config_t
{
    int port;
    int confirm;
    int payload_len;
    char payload_hex[64];
} tx_config_t;

typedef struct rx_info_t
{
    char payload[32];
    int payload_len;
    int port;
    int slot;
    int rssi;
    int snr;
    int dr;
} rx_info_t;

typedef enum resp_t
{
    RESP_NONE = 0,
    RESP_OK = 1,
    RESP_ERROR,
    RESP_TIMEOUT,
    RESP_JOINED,
    RESP_NO_NETWORK_JOINED,
    RESP_SEND_CONFIRMED
} resp_t;

typedef enum config_items_t
{
    AT_VER = 0,
    AT_DEVEUI,
    AT_APPEUI,
    AT_APPKEY,
    AT_ADR,
    AT_CLASS,
    AT_REGION,
    AT_CHANMASK,
    AT_CS,
    TOTAL_ITEMS
} config_items_t;

typedef union status_flag_t
{
    uint8_t All;
    struct
    {
        volatile uint8_t comm_connected : 1;
        volatile uint8_t at_test_ok : 1;
        volatile uint8_t is_initialized : 1;
        volatile uint8_t cmd_waiting : 1;
        volatile uint8_t has_run_once : 1;
    } _bit;
} status_flag_t;
typedef struct lora_module_t
{

    lora_fsm_t state;
    join_mode_t join_mode;
    char ver_lorawan[32];
    char ver_rp[32];
    char dev_eui[16];
    char app_eui[16];
    char app_key[36];

    config_items_t config_items;

    tx_config_t tx_param;
    rx_info_t   rx_info;

    char rx_line_queue[LINE_RX_QUEUE_ITEMS][LINE_QUEUE_ITEM_SIZE];
    int rx_line_queue_head;
    int rx_line_queue_tail;
    int rx_idx;
    char rx_buf[RX_BUF_SIZE];
    status_flag_t status_flag;
    resp_t resp;
    int retry_times;
    int ack_error_times;
    int restart_times;
    char cmd_buf[128];
    char pop_line_buf[LINE_QUEUE_ITEM_SIZE];
    int cmd_waiting_tmr_150ms;
} lora_module_t;

typedef enum app_trigger_type_t
{
    TRIGGER_MANUAL,
    TRIGGER_AUTO,
    TRIGGER_NONE
} app_trigger_type_t;

static app_trigger_type_t pack_app_data(lora_module_t *m)
{
    m->tx_param.port = 2;
    m->tx_param.confirm = 1;
    m->tx_param.payload_hex[0] = 'A';
    m->tx_param.payload_hex[1] = 'A';
    m->tx_param.payload_hex[2] = '\0';
    //    if (hum_comps.trigger_req._bit.normal_mode_on_s_key)
    //    {
    //        hum_comps.trigger_req._bit.normal_mode_on_s_key = 0;
    //        return TRIGGER_MANUAL;
    //    }
    return TRIGGER_NONE;
}

static bool pop_line(lora_module_t *m, char *out, int max_len)
{
    bool ret = false;
    ENTER_CRITICAL_SECTION();
    if (m->rx_line_queue_head == m->rx_line_queue_tail)
    {
        ret = false;
    }
    else
    {
        strncpy(out, m->rx_line_queue[m->rx_line_queue_tail], max_len - 1);
        out[max_len - 1] = '\0';
        m->rx_line_queue_tail = (m->rx_line_queue_tail + 1) % LINE_RX_QUEUE_ITEMS;
        ret = true;
    }
    EXIT_CRITICAL_SECTION();
    return ret;
}

static void init(lora_module_t *m)
{
    m->state = IDLE;
    m->join_mode = OTAA;

    m->ver_lorawan[0] = '\0';
    m->ver_rp[0] = '\0';
    m->config_items = AT_VER;

    m->rx_line_queue_head = 0;
    m->rx_line_queue_tail = 0;
    memset(m->rx_line_queue, 0, sizeof(m->rx_line_queue));

    m->status_flag._bit.is_initialized = 1;
}

static void process_cmd_result(lora_module_t *m)
{
    switch (m->state)
    {
    case CONFIG:
        // if(m->resp==RESP_OK)
        {
            m->config_items++;
            if (m->config_items >= TOTAL_ITEMS)
            {
                m->state = SEND;
                m->config_items = 0;
            }
        }
        break;
    case RESET:
        if (m->resp == RESP_OK)
        {
            m->state = JOIN;
        }
        else
        {
            m->state = IDLE;
        }
        break;

    case JOIN:
        if (m->resp == RESP_JOINED)
        {
            m->state = SEND;
        }
        else
        {
            m->state = IDLE;
        }
        break;
    case SEND:
        if (m->resp == RESP_SEND_CONFIRMED)
        {
            m->state = RECV;
        }
        else if (m->resp == RESP_ERROR)
        {
            m->state = IDLE;
        }
        else if (m->resp == RESP_TIMEOUT && !m->status_flag._bit.at_test_ok)
        {
            m->state = IDLE;
        }
        else if (m->resp == RESP_NO_NETWORK_JOINED || (m->resp == RESP_TIMEOUT && m->status_flag._bit.at_test_ok))
        {
            if (m->restart_times < 1)
            {
                m->state = RESET;
                m->restart_times++;
            }
            else
            {
                m->state = IDLE;
            }
        }
        else
        {
            m->state = IDLE;
        }
        break;
    case RECV:
        m->state = IDLE;
        break;

    default:
        m->state = IDLE;
    }
    if (m->state == IDLE)
    {
        disable_com();
        m->restart_times = 0;
        m->status_flag._bit.comm_connected = m->status_flag._bit.at_test_ok;
        m->status_flag._bit.at_test_ok = 0;
    }
    m->retry_times = 0;
    m->ack_error_times = 0;
    m->cmd_waiting_tmr_150ms = 0;
    m->status_flag._bit.cmd_waiting = 0;
    m->resp = RESP_NONE;
}

static void send_cmd(lora_module_t *m, const char *cmd, int timeout_150ms)
{

    m->status_flag._bit.cmd_waiting = 1;
    m->cmd_waiting_tmr_150ms = timeout_150ms;
    write_com((uint8_t *)cmd, strlen(cmd));
}

static char *check_response(lora_module_t *m, const char *expected)
{
    char *start;
    if (pop_line(m, m->pop_line_buf, sizeof(m->pop_line_buf)))
    {
        char *start = strstr(m->pop_line_buf, expected);
        return start;
    }
    return NULL;
}

static void ack_ok_handler(lora_module_t *m)
{
    m->resp = RESP_OK;
    process_cmd_result(m);
}

static void ack_no_network_joined_handler(lora_module_t *m)
{
    m->resp = RESP_NO_NETWORK_JOINED;
    process_cmd_result(m);
}

static void ack_send_confirmed_handler(lora_module_t *m)
{
    m->resp = RESP_SEND_CONFIRMED;
    process_cmd_result(m);
}

static void no_ack_handler(lora_module_t *m)
{

    m->retry_times++;
    if (m->retry_times >= 2)
    {
        m->resp = RESP_TIMEOUT;
        process_cmd_result(m);
    }
    else
    {
        m->status_flag._bit.cmd_waiting = 0;
    }
}

static void ack_joined_handler(lora_module_t *m)
{
    m->resp = RESP_JOINED;
    process_cmd_result(m);
}
static void ack_error_handler(lora_module_t *m)
{
    m->ack_error_times++;
    if (m->ack_error_times >= 2)
    {
        m->resp = RESP_ERROR;
        process_cmd_result(m);
    }
    else
    {
        m->status_flag._bit.cmd_waiting = 0;
    }
}
static void config_param(lora_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        switch (m->config_items)
        {
        case AT_VER:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+VER=?\r\n");
            break;
        case AT_DEVEUI:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+DEVEUI=?\r\n");
            break;
        case AT_APPEUI:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+APPEUI=?\r\n");
            break;
        case AT_APPKEY:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+APPKEY=?\r\n");
            break;
        case AT_CLASS:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+CLASS=A\r\n");
            break;
        case AT_REGION:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+REGION=2\r\n");
            break;
        case AT_CHANMASK:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+CHANMASK=0001\r\n");
            break;
        case AT_CS:
            snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+CS\r\n");
            break;
        default:
            ack_ok_handler(m);
            return;
        }
        send_cmd(m, m->cmd_buf, 1 * 1000 / 150);
    }
    else if (pop_line(m, m->pop_line_buf, sizeof(m->pop_line_buf)))
    {
        const char *start;
        switch (m->config_items)
        {
        case AT_VER:
            start = strstr(m->pop_line_buf, "LoRaWAN_SPEC_VERSION: ");
            if (start)
            {
                strncpy(m->ver_lorawan, start + 22, sizeof(m->ver_lorawan));
                return;
            }
            start = strstr(m->pop_line_buf, "RP_SPEC_VERSION: ");
            if (start)
            {
                strncpy(m->ver_rp, start + 17, sizeof(m->ver_rp));
                return;
            }
            break;
        case AT_DEVEUI:
            start = strstr(m->pop_line_buf, "DEVEUI: ");
            if (start)
            {
                strncpy(m->dev_eui, start + 8, sizeof(m->dev_eui));
                return;
            }
            break;
        case AT_APPEUI:
            start = strstr(m->pop_line_buf, "APPEUI: ");
            if (start)
            {
                strncpy(m->app_eui, start + 8, sizeof(m->app_eui));
                return;
            }
            break;
        case AT_APPKEY:
            start = strstr(m->pop_line_buf, "APPKEY: ");
            if (start)
            {
                strncpy(m->app_key, start + 8, sizeof(m->app_key));
                return;
            }
            break;
        default:
            break;
        }
        if (strstr(m->pop_line_buf, "OK"))
        {
            ack_ok_handler(m);
            return;
        }
        start = strstr(m->pop_line_buf, "ERROR");
        if (start)
        {
            ack_error_handler(m);
            return;
        }
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        no_ack_handler(m);
    }
    else
    {
        m->cmd_waiting_tmr_150ms--; // 每调用一次减1
    }
}

static void reset(lora_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+RESET\r\n");
        send_cmd(m, m->cmd_buf, 1 * 1000 / 150);
    }
    else if (check_response(m, "+READY"))
    {
        ack_ok_handler(m);
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        no_ack_handler(m);
    }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}

static void join_network(lora_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+JOIN=%d\r\n", (m->join_mode == OTAA) ? 1 : 0);
        send_cmd(m, m->cmd_buf, 12 * 1000 / 150);
    }
    else if (pop_line(m, m->pop_line_buf, sizeof(m->pop_line_buf)))
    {
        if (strstr(m->pop_line_buf, "OK"))
        {
            // m->cmd_waiting_tmr_150ms;
        }
        else if (strstr(m->pop_line_buf, "+EVT:JOINED"))
        {
            ack_joined_handler(m);
        }
        else if (strstr(m->pop_line_buf, "ERROR\r\n") || strstr(m->pop_line_buf, "+EVT:JOIN FAILED"))
        {
            ack_error_handler(m);
        }
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        no_ack_handler(m);
    }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}

static void send_app_data(lora_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        snprintf(m->cmd_buf, sizeof(m->cmd_buf), "AT+SEND=%d:%d:%s\r\n",
                 m->tx_param.port,
                 m->tx_param.confirm,
                 m->tx_param.payload_hex);
        send_cmd(m, m->cmd_buf, 10 * 1000 / 150);
    }
    else if (pop_line(m, m->pop_line_buf, sizeof(m->pop_line_buf)))
    {
        if (strstr(m->pop_line_buf, "OK"))
        {
            m->status_flag._bit.at_test_ok = 1;
        }
        else if (strstr(m->pop_line_buf, "ERROR"))
        {
            m->status_flag._bit.at_test_ok = 1;
            ack_error_handler(m);
        }
        else if (strstr(m->pop_line_buf, "NO_NETWORK_JOINED"))
        {
            m->status_flag._bit.at_test_ok = 1;
            ack_no_network_joined_handler(m);
        }
        else if (strstr(m->pop_line_buf, "+EVT:SEND_CONFIRMED"))
        {
            ack_send_confirmed_handler(m);
        }
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        no_ack_handler(m);
    }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}

static void recv_app_data(lora_module_t *m)
{
    if (!m->status_flag._bit.cmd_waiting)
    {
        m->cmd_buf[0] = '\0';
        send_cmd(m, m->cmd_buf, 2 * 1000 / 150);
    }
    else if (pop_line(m, m->pop_line_buf, sizeof(m->pop_line_buf)))
    {
        if (strstr(m->pop_line_buf, "+EVT:") && (!strstr(m->pop_line_buf, "RX")))
        {
            sscanf(m->pop_line_buf, "+EVT:%d:%d:%s", &m->rx_info.port, &m->rx_info.payload_len, m->rx_info.payload);
        }
        else if (strstr(m->pop_line_buf, "+EVT:RX"))
        {
            sscanf(m->pop_line_buf, "+EVT:RX_%d, PORT %d, DR %d, RSSI %d, SNR %d",
                   &m->rx_info.slot, &m->rx_info.port, &m->rx_info.dr, &m->rx_info.rssi, &m->rx_info.snr);
        }
    }
    else if (m->cmd_waiting_tmr_150ms <= 0)
    {
        no_ack_handler(m);
    }
    else
    {
        m->cmd_waiting_tmr_150ms--;
    }
}

static lora_module_t lora_wan;
void lorawan_task_handler(void)
{
    app_trigger_type_t app_trigger_type;
    lora_module_t *m = &lora_wan;
    if (m->status_flag._bit.is_initialized == 0)
    {
        init(m);
        return;
    }
    switch (m->state)
    {
    case RESET:
        reset(m);
        break;
    case CONFIG:
        config_param(m);
        break;
    case JOIN:
        join_network(m);
        break;
    case SEND:
        send_app_data(m);
        break;
    case IDLE:
        app_trigger_type = pack_app_data(m);
        if (app_trigger_type == TRIGGER_NONE)
        {
        }
        else
        {
            if (app_trigger_type == TRIGGER_MANUAL || !m->status_flag._bit.has_run_once)
            {
                m->state = CONFIG;
                m->status_flag._bit.has_run_once = 1;
            }
            else
            {
                m->state = SEND;
            }
            config_com(9600, 0);
            enable_com();
        }
        break;
    case RECV:
        recv_app_data(m);
        break;
    default:
        break;
    }
    
}

void lora_module_rx_byte_isr(char byte)
{
    lora_module_t *m = &lora_wan;
    if (byte == '\r' || byte == '\n')
    {
        if (m->rx_idx > 0)
        {
            m->rx_buf[m->rx_idx] = '\0';
            if (m->rx_buf[0] != '\0')
            {
                int next = (m->rx_line_queue_head + 1) % LINE_RX_QUEUE_ITEMS;
                if (next == m->rx_line_queue_tail)
                {
                    m->rx_line_queue_tail = (m->rx_line_queue_tail + 1) % LINE_RX_QUEUE_ITEMS;
                }
                strncpy(m->rx_line_queue[m->rx_line_queue_head], m->rx_buf, LINE_QUEUE_ITEM_SIZE - 1);
                m->rx_line_queue[m->rx_line_queue_head][LINE_QUEUE_ITEM_SIZE - 1] = '\0';
                m->rx_line_queue_head = next;
            }
        }
        m->rx_idx = 0;
    }
    else if (m->rx_idx < RX_BUF_SIZE - 1)
    {
        m->rx_buf[m->rx_idx++] = byte;
    }
    else
    {
        m->rx_idx = 0;
        m->rx_buf[m->rx_idx++] = byte;
    }
}