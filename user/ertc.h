#ifndef ERTC_H
#define ERTC_H


typedef struct ERTC_COMPONENTS
{
	char *desc;
    uint8_t (*read_broken_time)(stc_rtc_time_t *);
    uint8_t (*write_broken_time)(stc_rtc_time_t *);
}ertc_comps_t;

extern ertc_comps_t ertc_comps;
#endif

