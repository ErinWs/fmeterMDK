#ifndef EPRESS_H
#define EPRESS_H


typedef struct EPRESS_COMPONENTS
{
	char *desc;
	uint16_t read_reasure_relt_error_times;
	uint8_t (*start_measure)(void);
    uint8_t (*read_measure_relt)(int32_t *press, uint8_t *unit, uint8_t *dot, int32_t *full_scale, int16_t *temp);

}epress_comps_t;

extern epress_comps_t epress_comps;
#endif


