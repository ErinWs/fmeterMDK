
#include "ddl.h"
#include "rtc.h"
#include "ertc.h"
#include "sw_i2c.h"



//The 7-bit IIC address of the ertc is 0x51 BM8563
static uint8_t Device_Address = 0x51;
static uint8_t word_Address =0x02;



uint8_t ertc_read_broken_time(stc_rtc_time_t *broken_time)
{
    uint8_t buffer[7];
    uint8_t ret;
    buffer[0]=word_Address;
    ret=sw_i2c_comps.write(Device_Address, buffer, 1,1);
    if(!ret)
    {
        ret=sw_i2c_comps.read(Device_Address, buffer, 7,1);
        if(ret)
    	{
    		return ret;
    	}
       broken_time->u8Second =buffer[0]&0x7f;
       broken_time->u8Minute =buffer[1]&0x7f;
       broken_time->u8Hour=buffer[2]&0x3f;
       broken_time->u8Day=buffer[3]&0x3f;
       broken_time->u8DayOfWeek=buffer[4]&0x07;
       broken_time->u8Month =buffer[5]&0x1f;
       broken_time->u8Year=buffer[6];
    }
    return ret;
}
uint8_t ertc_write_broken_time( stc_rtc_time_t *broken_time)
{
    uint8_t buffer[8];
    uint8_t ret;
    buffer[0]= word_Address ;
    buffer[1]= broken_time->u8Second ;
    buffer[2]= broken_time->u8Minute ;
    buffer[3]= broken_time->u8Hour;
    buffer[4]= broken_time->u8Day;
    buffer[5]= broken_time->u8DayOfWeek;
    buffer[6]= broken_time->u8Month ;
    buffer[7]= broken_time->u8Year;
    ret=sw_i2c_comps.write(Device_Address, buffer, 8,1);
    return ret;
}

ertc_comps_t ertc_comps=
{
    .read_broken_time=ertc_read_broken_time,
    .write_broken_time=ertc_write_broken_time
};



