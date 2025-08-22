/*****************************************************************************/

#include "ddl.h"
#include  "sht4x.h"
#include  "sw_i2c.h"


/* all measurement commands return T (CRC) RH (CRC) */
#define SHT4X_CMD_MEASURE_HPM 0xFD
#define SHT4X_CMD_MEASURE_LPM 0xE0
#define SHT4X_CMD_READ_SERIAL 0x89

#define SHT4X_ADDRESS 0x44
static uint8_t sht4x_cmd_measure = SHT4X_CMD_MEASURE_HPM;

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1

static uint8_t generate_crc(const uint8_t* data, uint16_t count) 
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;
    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}


static uint8_t sht4x_start_measure(void)
{
    return sw_i2c_comps.write(SHT4X_ADDRESS, &sht4x_cmd_measure, 1,1);
}

static uint8_t sht4x_read_measure_relt(int16_t *temperature, int16_t *humidity) 
{
    int32_t  temp;
    int32_t  humiy;
    uint16_t words[2];
    uint8_t   buff[6];
    uint8_t ret=sw_i2c_comps.read(SHT4X_ADDRESS, buff, sizeof(buff),1);
    if(ret)
    {
        *temperature=0;
        *humidity=0;
        return ret;
    }
    if(generate_crc(buff, 2) ==buff[2])
    {
        words[0]=(buff[0]<<8)+buff[1];
        temp = ((21875 * (int32_t)words[0]) >> 13) - 45000;
        temp =temp/100;
        *temperature=temp;
    }
    else
    {
        ret|=2;
       *temperature =0;
    }
    if(generate_crc(buff+3, 2) ==buff[5])
    {
        words[1]=(buff[3]<<8)+buff[4];
        humiy = ((15625 * (int32_t)words[1]) >> 13) - 6000;
        humiy= humiy/100;
        if(humiy<0)
        {
            humiy=0; 
        }
        else if(humiy>1000)
        {
            humiy=1000;
        }

        *humidity=humiy;
    }
    else
    {
         ret|=4;
        *humidity=0;
    }
    
    /**
     * formulas for conversion of the sensor signals, optimized for fixed point
     * algebra:
     * Temperature = 175 * S_T / 65535 - 45
     * Relative Humidity = 125 * (S_RH / 65535) - 6
     */
    return ret;
}


static uint8_t sht4x_read_serial(uint32_t* serial) 
{
    const uint8_t cmd = SHT4X_CMD_READ_SERIAL;
    uint8_t ret;
    uint16_t words[2];
    uint8_t   buff[6];
    ret = sw_i2c_comps.write(SHT4X_ADDRESS, &cmd, 1,1);
    if(ret) return ret;
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    ret=sw_i2c_comps.read(SHT4X_ADDRESS, buff, sizeof(buff),1);
    if(ret) return ret;
    if(generate_crc(buff, 2) ==buff[2] && generate_crc(buff+3, 2) ==buff[5])
    {     
        words[0]=(buff[0]<<8)+buff[1];
        words[1]=(buff[3]<<8)+buff[4];
        *serial = ((uint32_t)words[0] << 16) | words[1];
        
    }
    else
    {
         ret|=2;
        *serial =0;
    }
    return ret;
}

sht4x_comps_t sht4x_comps=
{
	"",
	sht4x_read_serial,       
	sht4x_start_measure,      
	sht4x_read_measure_relt      
};




