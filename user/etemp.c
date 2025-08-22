#include "ddl.h"
#include "gpio.h"
#include "etemp.h"



#define MD_DQ             GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_SDQ_PORT), MD_SDQ_PIN)
#define SET_MD_DQ_LOW     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_SDQ_PORT), MD_SDQ_PIN, FALSE)

#define MD_DQ_CFG_OUT     SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_SDQ_PORT), MD_SDQ_PIN, FALSE) 
#define MD_DQ_CFG_IN      SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_SDQ_PORT), MD_SDQ_PIN, TRUE)
#define MD_CFG_R1R0       (2)//[0--0.5   1--0.25  2--0.125  3--0.0625]

static void us_delay(uint32_t delay)//  us
{
   while(delay>0)
	{
        delay--;
		__NOP();__NOP();__NOP();
    }
}
static int8_t x18x20_reset(void)  
{  
//     int8_t retval = 0;  
//     MD_DQ_CFG_IN;
//     us_delay(4);  
//     SET_MD_DQ_LOW;
//     MD_DQ_CFG_OUT;
//    us_delay((480+960)/2);
//     MD_DQ_CFG_IN;
//     us_delay((60+75)/2);  //60us<t<75us
//     retval = MD_DQ; 
//     us_delay(500-(60+75)/2);
//     return retval;
	uint16_t cnt=0;
	int8_t retval = 0;  
	MD_DQ_CFG_IN;
	us_delay(6);  
	SET_MD_DQ_LOW;
	MD_DQ_CFG_OUT;
	us_delay(600);
	MD_DQ_CFG_IN;
	us_delay(25);//Release the bus
	while(MD_DQ)
	{
		us_delay(1);
		cnt++;
		if(cnt>400)
		{
			return 1;
		}
	}
	us_delay(30);
	retval=MD_DQ;
	us_delay(600-cnt-25-30);
	return retval;
} 

static void write_byte(uint8_t data)  
{  
    int i = 0;  
    for (i = 0; i < 8; i++)  
    {  
        MD_DQ_CFG_IN;
        us_delay(6);  
        SET_MD_DQ_LOW;
	    MD_DQ_CFG_OUT;
	    us_delay(3);
        if(data&0x01) 
	    {
		    MD_DQ_CFG_IN;
		}
	    else
	    {
            SET_MD_DQ_LOW;
            MD_DQ_CFG_OUT;
		}
        us_delay(70-3);
        data >>= 1;  
    }  
    MD_DQ_CFG_IN; 
}  

static uint8_t read_byte(void)  
{  
    int i;  
    uint8_t data = 0;  
    for (i = 0; i < 8; i++)  
    {  
        MD_DQ_CFG_IN;
        us_delay(6);  
        SET_MD_DQ_LOW;
	    MD_DQ_CFG_OUT;
        us_delay(3);  
        MD_DQ_CFG_IN;
        us_delay(3);  
        data >>= 1;  
        if(MD_DQ)
        {
            data |= 0x80; 
        }
        us_delay(70-3);  //50
    }  
	MD_DQ_CFG_IN;
    return data;
} 

static uint8_t check_crc(void const *p,uint8_t len)
{
	uint8_t crc = 0, i, j;
	uint8_t *dat=(uint8_t *)p;
	for (i = 0; i <len ; i++)
	{
		crc = crc ^ dat[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 0x01) 
			{
				crc = (crc >> 1) ^ 0x8C;
			}
			else 
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}

static int8_t read_ram(void *const buf,unsigned int len)
{
	int  i=0;
	uint8_t temp[9];
	uint8_t *Buf=(uint8_t *)buf;
	if(x18x20_reset())
	{
		return 1;
	}
	write_byte(0xcc);// Skip ROM command 
	write_byte(0xbe);// read ram command
	for(i=0;i<9;i++)
	{
		temp[i]=read_byte();
	}
	if(temp[8]==check_crc(temp,8))
	{
		for(i=0;i<len;i++)
		{
			Buf[i]=temp[i];
		}
		return 0;
	}
	else
	{
		return 1;
	}
}

static void write_ram(void const * buf, unsigned int len)
{
	int  i=0;
	uint8_t const *Buf=(uint8_t *)buf;
	x18x20_reset();
	write_byte(0xcc);// Skip ROM command 
	write_byte(0x4e);// write ram command
	for(i=0;i<len;i++)
	{
		write_byte(Buf[i]);
	}
}

static uint8_t x18x20_config(uint8_t dat)
{
	 uint8_t data[5];
    int8_t ret;
	if(!read_ram(data,sizeof(data)))
	{
		data[0]=data[2];
        data[1]=data[3];
        data[2]=data[4]&0x9f|dat;
        write_ram(data,3);
		return 0;
	}
	else
	{
		return 1;
	}
}

static int8_t x18x20_start_temp_convert(void)
{
	// if(MD_CFG_R1R0!=3)
	// {
	// 	x18x20_config(MD_CFG_R1R0<<5);
	// }
	if(!x18x20_reset())
	{
		write_byte(0xcc);// Skip ROM command 
		write_byte(0x44);// start 18b20 temp convert
		return 0;
	}
	return 1;
	
}

static int8_t x18x20_read_measure_relt(int16_t *temp)
{
    uint8_t buf[2];
    int8_t ret;
    int attempts = 0;
    
    // 尝试多次读取
    while(attempts < 3)
	 {
        if(!read_ram(buf, sizeof(buf)))
		{
            *temp = (int16_t)(((uint16_t)buf[1]<<8) + buf[0]) * 10 / 16;
            return 0;
        }
        attempts++;
    }
    *temp = 0;
    return 1;
}
	

etemp_comps_t etemp_comps=
{
    .desc="",     //char *desc;
	.read_reasure_relt_error_times=0,	  //uint16_t read_reasure_relt_error_times;
	.start_measure=x18x20_start_temp_convert,//void(*const start_measure)(void);
	.read_measure_relt=x18x20_read_measure_relt,//int8_t (*const read_measure_relt) (int16_t temp);            
};

