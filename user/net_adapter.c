#include "ddl.h"
#include "rtc.h"
#include "device.h"
#include "net_adapter.h"
#include "wifia.h"
#include "app_trans.h"

void net_adapter_task_handler(void)
{
    wifi_task_handler();
}

net_state_t get_net_adapter_state(void)
{
    switch (get_wifi_fsm())
    {
        case WIFI_FSM_IDLE:          return NET_STATE_IDLE;
        case WIFI_FSM_ON:            return NET_STATE_ON;
        case WIFI_FSM_OFF:           return NET_STATE_OFF;
        case WIFI_FSM_CONFIG:        return NET_STATE_CONFIG;
        case WIFI_FSM_SWITCH_TO_IP1: return NET_STATE_SWITCH_TO_IP1;
        case WIFI_FSM_UT_RESET:      return NET_STATE_UE_RESET;
        case WIFI_FSM_TRANSPARENT:   return NET_STATE_READY;
        default:                     return NET_STATE_IDLE;
    }
}

net_error_t get_net_adapter_last_error(void)
{
    switch (get_wifi_last_error())
    {
        case WIFI_ERROR_NONE:           return NET_ERR_NONE;
        case WIFI_ERROR_INVALID_IP:     return NET_ERR_INVALID_IP;
        case WIFI_ERROR_NOT_GOT_IP:     return NET_ERR_NOT_GOT_IP;
        case WIFI_ERROR_CMD_TIMEOUT:    return NET_ERR_CMD_TIMEOUT;
        case WIFI_ERROR_CONNECT_TIMEOUT:return NET_ERR_CONNECT_TIMEOUT;
        case WIFI_ERROR_ACK_ERROR:      return NET_ERR_NONE;
        default:                        return NET_ERR_NONE;
    }
}

net_event_t get_net_adapter_pending_event(void)
{
    switch (get_wifi_pending_event())
    {
        case WIFI_EVENT_NONE:           return NET_EVENT_NONE;
        case WIFI_EVENT_ON:             return NET_EVENT_POWER_ON;
        case WIFI_EVENT_OFF:            return NET_EVENT_POWER_OFF;
        case WIFI_EVENT_SWITCH_TO_IP1:  return NET_EVENT_SWITCH_TO_IP1;
        case WIFI_EVENT_SEND_DATA:      return NET_EVENT_SEND_DATA;
        default:                        return NET_EVENT_NONE;
    }
}

uint8_t request_net_adapter_event(net_event_t event, void *data, uint16_t len)
{
    switch (event)
    {
        case NET_EVENT_POWER_ON:
            return wifi_request_event(WIFI_EVENT_ON, NULL, 0);
        case NET_EVENT_POWER_OFF:
            return wifi_request_event(WIFI_EVENT_OFF, NULL, 0);
        case NET_EVENT_SEND_DATA:
            return wifi_request_event(WIFI_EVENT_SEND_DATA, data, len);
        case NET_EVENT_SWITCH_TO_IP1:
            return wifi_request_event(WIFI_EVENT_SWITCH_TO_IP1, NULL, 0);
        default:
            return 1;
    }
}

void push_rx_byte_to_net_adapter(const uint8_t *data, uint16_t len)//called by wifi module when receive app data 
{
    app_trans_push_rx_data(data, len);//push app data to app trans recv buffer
}