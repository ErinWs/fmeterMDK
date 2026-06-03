#ifndef NET_ADAPTER_H
#define NET_ADAPTER_H

#include <stdint.h>

typedef enum net_event_t
{
    NET_EVENT_NONE = 0,
    NET_EVENT_POWER_ON,
    NET_EVENT_POWER_OFF,
    NET_EVENT_SEND_DATA,
    NET_EVENT_SWITCH_TO_IP1

} net_event_t;

typedef enum net_state_t
{
    NET_STATE_IDLE=0,
    NET_STATE_OFF,
    NET_STATE_ON,
    NET_STATE_CONFIG,
    NET_STATE_SWITCH_TO_IP1,
    NET_STATE_UE_RESET,
    NET_STATE_READY
} net_state_t;

typedef enum net_error_t
{
    NET_ERR_NONE = 0,
    NET_ERR_INVALID_IP,
    NET_ERR_NOT_GOT_IP,
    NET_ERR_CMD_TIMEOUT,
    NET_ERR_CONNECT_TIMEOUT
} net_error_t;

net_state_t get_net_adapter_state(void);
net_event_t get_net_adapter_pending_event(void);
net_error_t get_net_adapter_last_error(void);

uint8_t request_net_adapter_event(net_event_t event, void *data, uint16_t len);
void push_rx_byte_to_net_adapter(const uint8_t *data, uint16_t len);
void net_adapter_task_handler(void);

#endif // NET_ADAPTER_H