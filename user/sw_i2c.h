#ifndef SW_I2C_H
#define SW_I2C_H


typedef struct SW_I2C_COMPONENTS
{
	char *desc;
	uint8_t (*const init)(void);
	uint8_t (*const write)(uint16_t addr,  void const *data, uint16_t n,int8_t ack_en);
	uint8_t (*const read )(uint16_t addr,  void       *data, uint16_t n,int8_t ack_en);
}sw_i2c_comps_t;

extern sw_i2c_comps_t sw_i2c_comps;
#endif

