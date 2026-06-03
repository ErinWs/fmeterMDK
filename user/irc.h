#ifndef  IRC_H
#define  IRC_H

typedef struct 
{
 
	int16_t       op_window_time; 
    union 
    {
    	uint8_t All;
    	struct{
                uint8_t runing		        :1;
                uint8_t on                    :1;
                uint8_t res1		            :1;
               }_bit;
    }sw;

    void (*store_buffer)(uint8_t data);
    void (*stop)(void);
    void (*const task_handle)(void);
}ircComps_t;

extern ircComps_t ircComps;

#endif