/*****************************************************************************/
#include "ddl.h"
#include "gpio.h"
#include "24cxx.h"

#define   MD_I2C_VCC_HIGH     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_VCC_PORT),  MD_XROM_I2C_VCC_PIN, TRUE) //Gpio_WriteOutputIO(MD_I2C_PORT, MD_XROM_I2C_VCC_PIN, TRUE)
#define   MD_I2C_VCC_LOW      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_VCC_PORT),  MD_XROM_I2C_VCC_PIN, FALSE)
#define   MD_I2C_WP_HIGH      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_WP_PORT), MD_XROM_I2C_WP_PIN, TRUE)
#define   MD_I2C_WP_LOW       SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_WP_PORT), MD_XROM_I2C_WP_PIN, FALSE)
#define   MD_I2C_SCL_HIGH     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_SCL_PORT), MD_XROM_I2C_SCL_PIN, TRUE)
#define   MD_I2C_SCL_LOW      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_SCL_PORT), MD_XROM_I2C_SCL_PIN, FALSE)
#define   MD_I2C_SCL_IN       SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_XROM_I2C_SCL_PORT), MD_XROM_I2C_SCL_PIN, TRUE)
#define   MD_I2C_SCL_OUT      SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_XROM_I2C_SCL_PORT), MD_XROM_I2C_SCL_PIN, FALSE) 
#define   MD_I2C_GET_SCL      GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_XROM_I2C_SCL_PORT), MD_XROM_I2C_SCL_PIN)



#define   MD_I2C_SDA_HIGH     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_SDA_PORT), MD_XROM_I2C_SDA_PIN, TRUE) 
#define   MD_I2C_SDA_LOW      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_XROM_I2C_SDA_PORT), MD_XROM_I2C_SDA_PIN, FALSE)
#define   MD_I2C_SDA_IN       SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_XROM_I2C_SDA_PORT), MD_XROM_I2C_SDA_PIN, TRUE)
#define   MD_I2C_SDA_OUT      SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_XROM_I2C_SDA_PORT), MD_XROM_I2C_SDA_PIN, FALSE) 
#define   MD_I2C_GET_SDA      GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_XROM_I2C_SDA_PORT), MD_XROM_I2C_SDA_PIN)

                              
#define  MD_I2C_STATUS_TRUE     0
#define  MD_I2C_STATUS_FALSE    1
#define  MD_I2C_RETURN_FALSE    {MD_I2C_VCC_HIGH;MD_I2C_WP_LOW;MD_I2C_SCL_IN;MD_I2C_SDA_IN;return MD_I2C_STATUS_FALSE;}
#define  MD_I2C_RETURN_TRUE     {MD_I2C_VCC_HIGH;MD_I2C_WP_LOW;MD_I2C_SCL_IN;MD_I2C_SDA_IN;return MD_I2C_STATUS_TRUE;}

//static void DelayMs(int16_t TimeMs)
//{
//	volatile int16_t j;
//	int16_t  i;
//	for(j=0;j<TimeMs*10;j++)
//	{		
//		i=97;	//100us at 8Mhz
//		while(i>0) 
//		{	
//			i--;
//		}
//	}
//}

static void I2CDelay(void)	
{	
    volatile int8_t i=7;//10us at 8Mhz
	while(i>0)
	{
		i--;
	}
}

static int check_i2c_idle(void)
{
    int idle=0;
    MD_I2C_SCL_IN;
    MD_I2C_SDA_IN;
    I2CDelay();
    if(MD_I2C_GET_SCL)
    {
        idle=1;
        MD_I2C_SCL_OUT;
    }
    return idle;
}
static void I2CStart(void)
{
	
	MD_I2C_SCL_HIGH;
	MD_I2C_SDA_IN; 
	I2CDelay(); 
	MD_I2C_SDA_LOW;
	MD_I2C_SDA_OUT;  
	I2CDelay();
	MD_I2C_SCL_LOW;
}

static void I2CStop(void)
{
	MD_I2C_SDA_OUT;
	MD_I2C_SDA_LOW;
	I2CDelay(); 
	MD_I2C_SCL_HIGH; 
	I2CDelay(); 
	MD_I2C_SDA_IN;  
	I2CDelay();
	MD_I2C_SDA_IN;
}
 
static void I2CACK(void)
{
	MD_I2C_SDA_LOW;
	MD_I2C_SDA_OUT;
	I2CDelay(); 
	MD_I2C_SCL_HIGH;
	I2CDelay(); 
	MD_I2C_SCL_LOW;
	I2CDelay(); 
}

static void I2CNOACK(void)
{
	MD_I2C_SDA_IN;
	I2CDelay(); 
	MD_I2C_SCL_HIGH;
	I2CDelay(); 
	MD_I2C_SCL_LOW;
	I2CDelay(); 
}

static int8_t  I2CByteTX(int8_t byte)
{
	int8_t loop = 8;
    while(loop--)
    {
		if(byte & 0x80)  
		{
		   MD_I2C_SDA_HIGH;
		}
		else
		{
			MD_I2C_SDA_LOW;
			MD_I2C_SDA_OUT;
		}
		I2CDelay();
		MD_I2C_SCL_HIGH;
		I2CDelay();
		MD_I2C_SCL_LOW;
		byte  <<= 1;
	} 
	MD_I2C_SDA_IN; 
	I2CDelay();
	MD_I2C_SCL_HIGH;
	I2CDelay();
	if(MD_I2C_GET_SDA == 1)
	{
		MD_I2C_SCL_HIGH;
		return MD_I2C_STATUS_FALSE;
	}
	MD_I2C_SCL_LOW;
	I2CDelay();
	return MD_I2C_STATUS_TRUE;
}

static int8_t  I2CByteRX(void)
{
	int8_t byte;
	int8_t loop = 8;
	MD_I2C_SDA_IN;
	do
	{
		byte <<= 1;
		MD_I2C_SCL_HIGH;
		I2CDelay();
		if(MD_I2C_GET_SDA == 1) 
		{
		    byte |= 0x01;
		}
		MD_I2C_SCL_LOW;
		I2CDelay();
		loop--;
	}while(loop > 0);
	return(byte);
}

static uint8_t write_eeprom(uint16_t addr,  void const *buf, uint16_t n)
{
	int8_t w_wait,e2page,SlaveAddr;	
	int8_t const *src=(int8_t *)buf;
	e2page = 16;
	MD_I2C_VCC_HIGH;
	I2CDelay();
	if(!check_i2c_idle())
    {
        MD_I2C_RETURN_FALSE;
    }
	MD_I2C_WP_LOW; 
reStart:
	SlaveAddr = 0xA0 + (int8_t)((addr & 0x0700)>>7);
	for(w_wait=0; w_wait<4; w_wait++)
	{
		I2CStart();
		if(MD_I2C_STATUS_TRUE == I2CByteTX(SlaveAddr))
		{
		    break;
		}
		I2CStop();
		I2CStop();
		delay1ms(1);
    }	
	if(w_wait >= 4) 
	{
	    MD_I2C_RETURN_FALSE;
	}
	if(MD_I2C_STATUS_FALSE == I2CByteTX((int8_t)(addr%0x100))) 
	{
	    MD_I2C_RETURN_FALSE; 
	}
	while(n--)
	{
		if(MD_I2C_STATUS_FALSE == I2CByteTX(*src++)) 
		{
		    MD_I2C_RETURN_FALSE;
		}
		addr++;
		//if((addr % e2page) == 0)
		//{
		//    break;
		//}
	}	
	I2CStop();
	//delay1ms(5);
//	if(n != 0xffff)	
//	{
//	    goto reStart;
//	}
	
	I2CDelay();
		
	MD_I2C_RETURN_TRUE;
}

static uint8_t read_eeprom(uint16_t addr, void *buf, uint16_t n)
{
	int8_t w_wait,SlaveAddr;
	int8_t *dest=(int8_t *)buf;
	MD_I2C_VCC_HIGH;
	I2CDelay(); 
	if(!check_i2c_idle())
    {
        MD_I2C_RETURN_FALSE;
    }	
	SlaveAddr = 0xA0 + (int8_t)((addr & 0x0700)>>7);	
	for(w_wait=0; w_wait<4; w_wait++)
	{
		I2CStart();
		if(MD_I2C_STATUS_TRUE == I2CByteTX(SlaveAddr))
	    {
		    break;
		}
		I2CStop();
		I2CStop();
		delay1ms(1);
	}
	
	if(w_wait >= 4)
	{
	    MD_I2C_RETURN_FALSE;
	}
	if(MD_I2C_STATUS_FALSE == I2CByteTX((int8_t)(addr%0x100)))
	{
	    MD_I2C_RETURN_FALSE;
	}
	I2CStart();
	if(MD_I2C_STATUS_FALSE == I2CByteTX((SlaveAddr+1))) 
	{
	    MD_I2C_RETURN_FALSE;
	}
	while(n--)
	{
		*dest++	= I2CByteRX();
		if(n != 0)
		{
			I2CACK();
		}
		addr++;
	}
	I2CNOACK();
	I2CStop();	
	
	I2CDelay();	
	MD_I2C_RETURN_TRUE;
}

static uint8_t init_eeprom(void)
{
    int8_t status=MD_I2C_STATUS_FALSE;
    MD_I2C_VCC_LOW;           
    MD_I2C_WP_LOW;            
    MD_I2C_SCL_LOW;            
    MD_I2C_SDA_LOW;            
    MD_I2C_SDA_OUT;
    status=MD_I2C_STATUS_TRUE;
    return status;

}
_24cxx_comps_t _24cxx_comps=
{
	"", //int8_t *desc;
	&_24cxx_comps,      //struct _24CXX_COMPONENTS *this;
	init_eeprom,        // int8_t init_eeprom(void);
	write_eeprom,       //int8_t write_eeprom(uint16_t addr,  void const *buf, uint16_t n);
	read_eeprom         //int8_t read_eeprom (uint16_t addr,  void *buf, uint16_t n)
};