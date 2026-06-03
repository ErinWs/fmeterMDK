#ifndef WIFI_H
#define WIFI_H

typedef enum wifi_fsm_t
{
    WIFI_FSM_IDLE=0,
    WIFI_FSM_OFF,
    WIFI_FSM_ON,
    WIFI_FSM_CONFIG,
    WIFI_FSM_SWITCH_TO_IP1,
    WIFI_FSM_UT_RESET,
    WIFI_FSM_TRANSPARENT
} wifi_fsm_t;

typedef enum wifi_last_error_t
{
    WIFI_ERROR_NONE=0,
    WIFI_ERROR_INVALID_IP,
    WIFI_ERROR_NOT_GOT_IP,
    WIFI_ERROR_CMD_TIMEOUT,
    WIFI_ERROR_CONNECT_TIMEOUT,
    WIFI_ERROR_ACK_ERROR,
    WIFI_ERROR_UNKNOWN
} wifi_last_error_t;

typedef enum wifi_event_t
{
    WIFI_EVENT_NONE = 0,
    WIFI_EVENT_SEND_DATA,
    WIFI_EVENT_ON,      
    WIFI_EVENT_OFF,
    WIFI_EVENT_SWITCH_TO_IP1
} wifi_event_t;

void wifi_task_handler(void);
uint8_t wifi_request_event(wifi_event_t event, void *data, uint16_t len);
wifi_fsm_t get_wifi_fsm(void);
wifi_event_t get_wifi_pending_event(void);
wifi_last_error_t get_wifi_last_error(void);
void wifi_com_tx_complete_callback(void);
void wifi_com_rx_byte_isr(uint8_t byte);
#endif
