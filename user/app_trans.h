#ifndef APP_TRANS_H
#define APP_TRANS_H
typedef union app_trans_status_flag_t
{
    uint8_t All;
    struct _bit
    {
        volatile uint8_t is_initialized : 1;
        volatile uint8_t is_using_ip1 : 1;
        volatile uint8_t is_ip0_push_ok : 1;
        volatile uint8_t is_ip1_push_ok : 1;

    } _bit;
} app_trans_status_flag_t;
extern app_trans_status_flag_t get_app_trans_status_flag(void);
extern uint32_t get_app_trans_pending_event(void);
uint16_t get_app_trans_session_timeout_sec(void);
extern void app_trans_task_handler(void);
extern void app_trans_push_rx_data(const uint8_t *data, uint16_t len);
#endif