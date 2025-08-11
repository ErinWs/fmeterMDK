#ifndef SHT4X_H
#define SHT4X_H


typedef struct SHT4X_COMPONENTS
{
	char *desc;
    uint8_t (*read_serial)(uint32_t* serial);
	uint8_t (*start_measure)(void);
    uint8_t (*read_measure_relt)(int16_t *temperature, int16_t *humidity);
}sht4x_comps_t;

extern sht4x_comps_t sht4x_comps;
#endif

