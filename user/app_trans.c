#include "ddl.h"
#include "rtc.h"
#include "device.h"
#include "hum.h"
#include "app_trans.h"
#include "wifia.h"
#include "net_adapter.h"

#include "math.h"
#include "stdlib.h"


typedef enum fsm_t
{
    IDLE = 0,
    PROCESSING,
} fsm_t;

typedef enum push_ip_mode_t
{
    PUSH_SINGLE = 0,
    PUSH_BACKUP = 1,
    PUSH_BOTH   = 2 
} push_ip_mode_t;

typedef struct app_trans_ctx_t
{
    uint8_t send_buf[256];
    uint8_t recv_buf[256];
    uint16_t recv_pos;
    uint16_t session_timeout_tmr_150ms;
    uint16_t report_data_delay_tmr;
    app_trans_status_flag_t status_flag;
    uint32_t pending_event;
    push_ip_mode_t push_ip_mode;
    uint8_t  has_history_data_retry;
    fsm_t state;

} app_trans_ctx_t;

static uint16_t generate_modubus_crc(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xffff;
    int16_t i, j = 0;
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

static uint32_t fetch_events_bitmap(void)
{
    uint32_t events_bigmap = 0;
    events_bigmap += (hum_comps.trigger_req._bit.normal_mode_on_long_s_key << (uint32_t)0);
    events_bigmap += (device_comps.trigger_req._bit.interval_time << (uint32_t)1);
    if(events_bigmap > 0)
    {
        hum_comps.trigger_req._bit.normal_mode_on_long_s_key = 0;
        device_comps.trigger_req._bit.interval_time = 0;
    }
    return events_bigmap;
}

static uint16_t pack_current_data(uint32_t events, uint8_t *buf)
{
    uint16_t i = 0;
    buf[i++] = 0x01;
    buf[i++] = (events >> 8) & 0xFF;
    buf[i++] = events & 0xFF;
    // todo add current data
    return i;
}

static uint16_t pack_history_data(uint32_t events, uint8_t *buf )
{
    uint16_t i = 0;
    buf[i++] = 0x02;
    buf[i++] = (events >> 8) & 0xFF;
    buf[i++] = events & 0xFF;
    // TODO add history data
    return i;
}


static uint16_t process_recv_buffer(app_trans_ctx_t *m)
{
    uint16_t i = 0;
    // parse data
    return i;
}

static void try_switch_to_ip1_or_power_off(app_trans_ctx_t *m)
{
//     if (m->status_flag._bit.is_using_ip1)
//     {
//         m->status_flag._bit.is_ip1_push_ok = get_net_adapter_push_restult();
//     }
//     else
//     {
//         m->status_flag._bit.is_ip0_push_ok = get_net_adapter_push_restult();
//     }
//     if (get_net_adapter_last_error() == NET_ERR_CMD_TIMEOUT)
//     {
//         goto power_off;
//     }
//     if (m->push_ip_mode == PUSH_SINGLE)
//     {
//         goto power_off;
//     }
//     if (m->status_flag._bit.is_using_ip1)
//     {
//         goto power_off;
//     }
//     if (m->push_ip_mode == PUSH_BACKUP && m->status_flag._bit.is_ip0_push_ok)
//     {
//         goto power_off;
//     }
//     m->status_flag._bit.is_using_ip1 = 1;
//     switch_net_adapter_to_ip1();
//     m->session_timeout_tmr_150ms = (30 * 1000) / 150; 
//     return;

power_off:
    request_net_adapter_event(NET_EVENT_POWER_OFF, NULL, 0);
    m->session_timeout_tmr_150ms = 0;
    m->state = IDLE;
}

static void init(app_trans_ctx_t *m)
{
    m->state = IDLE;
    m->recv_pos = 0;
    m->pending_event = 0;
    m->push_ip_mode = PUSH_SINGLE;
    m->session_timeout_tmr_150ms = 0;
    m->status_flag._bit.is_initialized = 1;
}

static void dispatch_app_data(app_trans_ctx_t *m)
{
    uint16_t len;
    if (m->pending_event)
    {
        len = pack_current_data(m->pending_event, m->send_buf);
        if(!request_net_adapter_event(NET_EVENT_SEND_DATA, m->send_buf, len))
        {
            m->pending_event = 0;
        }
    }
    else if (m->has_history_data_retry)
    {
        len = pack_history_data(m->has_history_data_retry, m->send_buf);
        if(!request_net_adapter_event(NET_EVENT_SEND_DATA, m->send_buf, len))
        {
            m->has_history_data_retry = 0;
        }
    }
    process_recv_buffer(m);
}
static app_trans_ctx_t app_trans;

void app_trans_task_handler(void)
{
    app_trans_ctx_t *m = &app_trans;
    
    if (!m->status_flag._bit.is_initialized) {
        m->state = IDLE;
        m->recv_pos = 0;
        m->pending_event = 0;
        m->session_timeout_tmr_150ms = 0;
        m->status_flag._bit.is_initialized = 1;
        return;
    }

    switch (m->state)
    {
        case IDLE:
            if (get_net_adapter_state() == NET_STATE_IDLE)
            {
                m->pending_event = fetch_events_bitmap();
                if (m->pending_event > 0)
                {
                    if(!request_net_adapter_event(NET_EVENT_POWER_ON,NULL, 0))
                    {
                        m->has_history_data_retry=1;
                        m->status_flag._bit.is_using_ip1 = 0;
                        m->status_flag._bit.is_ip0_push_ok = 0;
                        m->status_flag._bit.is_ip1_push_ok = 0;
                        m->state = PROCESSING;
                        m->session_timeout_tmr_150ms = (int32_t)30*1000/150; //
                        hum_comps.enter_net_adapter_report_mode();
                    }
                }
            }
            break;
        case PROCESSING:
            if (get_net_adapter_last_error() != NET_ERR_NONE)
            {
                try_switch_to_ip1_or_power_off(m);
                return;
            }
            if (m->session_timeout_tmr_150ms > 0)
            {
                if (--m->session_timeout_tmr_150ms == 0)
                {
                   try_switch_to_ip1_or_power_off(m);
                   return;
                }
            }
            if(get_net_adapter_state() == NET_STATE_IDLE && get_net_adapter_pending_event()== NET_EVENT_NONE)
            {
                m->state = IDLE;// net maybe self power off
                return;
            }
            if (get_net_adapter_state() == NET_STATE_READY && get_net_adapter_pending_event()== NET_EVENT_NONE)
            {
               dispatch_app_data(m);
            }
            break;  
        default:
            m->state = IDLE;
            break;
    }
}


uint16_t get_app_trans_session_timeout_sec(void)
{
    return app_trans.session_timeout_tmr_150ms * (int32_t)150 / 1000;
}
app_trans_status_flag_t get_app_trans_status_flag(void)
{
    return app_trans.status_flag;
}
uint32_t get_app_trans_pending_event(void)
{
    return app_trans.pending_event;
}
void app_trans_push_rx_data(const uint8_t *data, uint16_t len)//push rx data to app_trans
{
    app_trans_ctx_t *m = &app_trans;
    for (uint16_t i = 0; i < len; i++)
    {
        if(m->recv_pos < sizeof(m->recv_buf))
        {
            m->recv_buf[m->recv_pos++] = data[i];
        }
        else
        {
            m->recv_pos = 0;
            m->recv_buf[m->recv_pos++] = data[i];
        }
    }
}
