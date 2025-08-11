#ifndef _24CXX_H
#define _24CXX_H


typedef struct _24CXX_COMPONENTS
{
	char *desc;
	struct _24CXX_COMPONENTS *this;
	uint8_t (*const init)(void);
	uint8_t (*const write)(uint16_t addr,  void const *buf, uint16_t n);
	uint8_t (*const read )(uint16_t addr,  void       *buf, uint16_t n);
}_24cxx_comps_t;

extern _24cxx_comps_t _24cxx_comps;
#endif