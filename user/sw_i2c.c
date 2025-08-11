#include "ddl.h"
#include "gpio.h"
#include  "sw_i2c.h"

#define   MD_SW_I2C_VCC_HIGH     do{__NOP();} while(0)
#define   MD_SW_I2C_VCC_LOW      do{__NOP();} while(0)


#define   MD_SW_I2C_SCL_HIGH    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_SW_I2C_SCL_PORT), MD_SW_I2C_SCL_PIN, TRUE)
#define   MD_SW_I2C_SCL_LOW     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_SW_I2C_SCL_PORT), MD_SW_I2C_SCL_PIN, FALSE)
#define   MD_SW_I2C_SCL_IN      SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_SW_I2C_SCL_PORT), MD_SW_I2C_SCL_PIN, TRUE)
#define   MD_SW_I2C_SCL_OUT     SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_SW_I2C_SCL_PORT), MD_SW_I2C_SCL_PIN, FALSE) 
#define   MD_SW_I2C_GET_SCL     GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_SW_I2C_SCL_PORT), MD_SW_I2C_SCL_PIN)


#define   MD_SW_I2C_SDA_HIGH     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_SW_I2C_SDA_PORT), MD_SW_I2C_SDA_PIN, TRUE) 
#define   MD_SW_I2C_SDA_LOW      SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_SW_I2C_SDA_PORT), MD_SW_I2C_SDA_PIN, FALSE)
#define   MD_SW_I2C_SDA_IN       SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_SW_I2C_SDA_PORT), MD_SW_I2C_SDA_PIN, TRUE)
#define   MD_SW_I2C_SDA_OUT      SetBit(((uint32_t)&M0P_GPIO->PADIR + MD_SW_I2C_SDA_PORT), MD_SW_I2C_SDA_PIN, FALSE) 
#define   MD_SW_I2C_GET_SDA      GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_SW_I2C_SDA_PORT), MD_SW_I2C_SDA_PIN)


#define  MD_SW_I2C_STATUS_TRUE     0
#define  MD_SW_I2C_STATUS_FALSE    1
#define  MD_SW_I2C_RETURN_FALSE    do {MD_SW_I2C_VCC_LOW;MD_SW_I2C_SCL_IN;MD_SW_I2C_SDA_IN;return MD_SW_I2C_STATUS_FALSE;}while(0);
#define  MD_SW_I2C_RETURN_TRUE     do {MD_SW_I2C_VCC_LOW;MD_SW_I2C_SCL_IN;MD_SW_I2C_SDA_IN;return MD_SW_I2C_STATUS_TRUE ;}while(0);

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

static void SW_I2CDelay(void)	
{	
    volatile int8_t i=8;//10us at 16Mhz
	while(i>0)
	{
		i--;
	}
}
static int check_i2c_idle(void)
{
    int idle=0;
    MD_SW_I2C_SCL_IN;
    MD_SW_I2C_SDA_IN;
    SW_I2CDelay();
    if(MD_SW_I2C_GET_SCL)
    {
        idle=1;
        MD_SW_I2C_SCL_OUT;
    }
    return idle;
}
static void SW_I2CStart(void)
{
	
	MD_SW_I2C_SCL_HIGH;
	MD_SW_I2C_SDA_IN; 
	SW_I2CDelay(); 
	MD_SW_I2C_SDA_LOW;
	MD_SW_I2C_SDA_OUT;  
	SW_I2CDelay();
	MD_SW_I2C_SCL_LOW;
}

static void SW_I2CStop(void)
{
	MD_SW_I2C_SDA_OUT;
	MD_SW_I2C_SDA_LOW;
	SW_I2CDelay(); 
	MD_SW_I2C_SCL_HIGH; 
	SW_I2CDelay(); 
	MD_SW_I2C_SDA_IN;  
	SW_I2CDelay();
	MD_SW_I2C_SDA_IN;
}
 
static void SW_I2CACK(void)
{
	MD_SW_I2C_SDA_LOW;
	MD_SW_I2C_SDA_OUT;
	SW_I2CDelay(); 
	MD_SW_I2C_SCL_HIGH;
	SW_I2CDelay(); 
	MD_SW_I2C_SCL_LOW;
	SW_I2CDelay(); 
}

static void SW_I2CNOACK(void)
{
	MD_SW_I2C_SDA_IN;
	SW_I2CDelay(); 
	MD_SW_I2C_SCL_HIGH;
	SW_I2CDelay(); 
	MD_SW_I2C_SCL_LOW;
	SW_I2CDelay(); 
}

static int8_t  SW_I2CByteTX(int8_t byte,int8_t ack_en)
{
	int8_t loop = 8;
    while(loop--)
    {
		if(byte & 0x80)  
		{
		   MD_SW_I2C_SDA_HIGH;
		}
		else
		{
			MD_SW_I2C_SDA_LOW;
			MD_SW_I2C_SDA_OUT;
		}
		SW_I2CDelay();
		MD_SW_I2C_SCL_HIGH;
		SW_I2CDelay();
		MD_SW_I2C_SCL_LOW;
		byte  <<= 1;
	} 
	MD_SW_I2C_SDA_IN; 
	SW_I2CDelay();
	MD_SW_I2C_SCL_HIGH;
	SW_I2CDelay();
	if(MD_SW_I2C_GET_SDA == 1)
	{
		if(ack_en)
		{
		    return MD_SW_I2C_STATUS_FALSE;
		}
    }
	MD_SW_I2C_SCL_LOW;
	SW_I2CDelay();
	return MD_SW_I2C_STATUS_TRUE;
}

static int8_t  SW_I2CByteRX(void)
{
	int8_t byte;
	int8_t loop = 8;
	MD_SW_I2C_SDA_IN;
	do
	{
		byte <<= 1;
		MD_SW_I2C_SCL_HIGH;
		SW_I2CDelay();
		if(MD_SW_I2C_GET_SDA == 1) 
		{
		    byte |= 0x01;
		}
		MD_SW_I2C_SCL_LOW;
		SW_I2CDelay();
		loop--;
	}while(loop > 0);
	return(byte);
}


static uint8_t i2c_write(uint16_t addr,  void const *data, uint16_t n,int8_t ack_en)
{
	int8_t w_wait,SlaveAddr;	
	int8_t const *src=(int8_t *)data;

	MD_SW_I2C_VCC_HIGH;
	SW_I2CDelay();
    if(!check_i2c_idle())
    {
        MD_SW_I2C_RETURN_FALSE;
    }
	SlaveAddr = addr<<1;
	for(w_wait=0; w_wait<2; w_wait++)
	{
		SW_I2CStart();
		if(MD_SW_I2C_STATUS_TRUE == SW_I2CByteTX(SlaveAddr,ack_en))
		{
		    break;
		}
		SW_I2CStop();
		SW_I2CDelay(); 
    }	
	if(w_wait >= 2) 
	{
	    MD_SW_I2C_RETURN_FALSE;
	}
	while(n--)
	{
		if(MD_SW_I2C_STATUS_FALSE == SW_I2CByteTX(*src++,ack_en)) 
		{
		    MD_SW_I2C_RETURN_FALSE;
		}
	}	
	SW_I2CStop();
    SW_I2CDelay();
	MD_SW_I2C_RETURN_TRUE;
}

static uint8_t i2c_read(uint16_t addr, void *data, uint16_t n,int8_t ack_en)
{
	int8_t w_wait,SlaveAddr;
	int8_t *dest=(int8_t *)data;
	MD_SW_I2C_VCC_HIGH;
	SW_I2CDelay(); 	
    if(!check_i2c_idle())
    {
        MD_SW_I2C_RETURN_FALSE;
    }
	SlaveAddr =	(addr<<1) | 1;
	for(w_wait=0; w_wait<2; w_wait++)
	{
		SW_I2CStart();
		if(MD_SW_I2C_STATUS_TRUE == SW_I2CByteTX(SlaveAddr,ack_en))
	    {
		    break;
		}
		SW_I2CStop();
		SW_I2CDelay(); 
	}
	if(w_wait >= 2)
	{
	    MD_SW_I2C_RETURN_FALSE;
	}
	
	while(n--)
	{
		*dest++	= SW_I2CByteRX();
		if(n != 0)
		{
			SW_I2CACK();
		}
		
	}
	SW_I2CNOACK();
	SW_I2CStop();	
	SW_I2CDelay();
   MD_SW_I2C_RETURN_TRUE;
}

static uint8_t i2c_init(void)
{
    int8_t status=MD_SW_I2C_STATUS_FALSE;
    MD_SW_I2C_VCC_LOW;           
    MD_SW_I2C_SCL_IN;            
    MD_SW_I2C_SDA_IN;            
    status=MD_SW_I2C_STATUS_TRUE;
    return status;

}

sw_i2c_comps_t sw_i2c_comps=
{
	"", //int8_t *desc;
	i2c_init,       
	i2c_write,      
	i2c_read       
};


