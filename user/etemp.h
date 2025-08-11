#ifndef ETEMP_H
#define ETEMP_H

typedef struct _ETEMP_COMPONENTS
{
	char *desc;
	uint16_t read_reasure_relt_error_times;
	int8_t (*const start_measure)(void);
	int8_t (*const read_measure_relt) (int16_t *temp);
	
}etemp_comps_t;

extern etemp_comps_t  etemp_comps;
#endif

