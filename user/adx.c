#include "ddl.h"
#include "rtc.h"
#include "gpio.h"
#include "pcnt.h"
#include "lptim.h"
#include "bgr.h"
#include "adc.h"
#include "wdt.h"

#include "device.h"
#include "r_cg_sau.h"
#include "elora.h"
#include "protocol.h"
#include "net.h"
#include "irc.h"
#include "hum.h"
#include "collector.h"
#include "24cxx.h"
#include "adx.h"
#include "modbus.h"

#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

#define    ADI_CS_HIGH    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_ADI_CS_PORT)  ,  MD_ADI_CS_PIN, TRUE)
#define    ADI_CS_LOW     SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_ADI_CS_PORT)  ,  MD_ADI_CS_PIN, FALSE)
#define    ADS_SCLK_HIGH  SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_ADI_SCLK_PORT),  MD_ADI_SCLK_PIN, TRUE)
#define    ADS_SCLK_LOW   SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_ADI_SCLK_PORT),  MD_ADI_SCLK_PIN, FALSE)
#define    ADS_DIN_HIGH   SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_ADI_DIN_PORT) ,  MD_ADI_DIN_PIN, TRUE)
#define    ADS_DIN_LOW    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_ADI_DIN_PORT) ,  MD_ADI_DIN_PIN, FALSE)
#define    ADS_DOUT       GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_ADI_DOUT_PORT),  MD_ADI_DOUT_PIN)

typedef uint8_t   uint8;
typedef int32_t            int32;

static int32_t formatAd(int32_t addat)
{
   if(addat&((int32_t)1<<23))
   {
     addat|=0xff000000;
   }
   addat>>=6;//24-6=18bit 2^17
   return (addat*10000/13108);
//   addat>>=3;
//   addat>>=4;
// return addat;
}

static void spi_delay(uint32_t tms)
{
  while (tms--);
}

static uint8_t SPI_COM(uint8_t m)
{
	uint8_t i,r=0;
	ADI_CS_LOW;
	for(i=0;i<8;i++)
	{
		ADS_SCLK_LOW;
		if(m & 0x80)
		{
			ADS_DIN_HIGH;
		}	  
		else
		{  
			ADS_DIN_LOW;
		}
		m = m<<1;
		spi_delay(1);
		r=r<<1;
		if(ADS_DOUT == 1)
		{ 
			r++;
		}
		ADS_SCLK_HIGH;	
		spi_delay(1); 
		
	}
	ADI_CS_HIGH;
	return r;
	
}

static  uint16_t SPI_Write(uint8_t *data,uint8_t size)
{

	uint16_t i=0;
	
	for(i=0;i<size;i++)
	{
		SPI_COM(*data++);  
	}
	
	return 0;
}


static uint16_t SPI_Read(uint8_t *data,uint8_t size)
{
	uint16_t i=0;
	
	for (i=0;i<size;i++)
	{
		*data++=SPI_COM(0);
	}
	
	return 0;
}

/*****************************  ad7793 start *********************************/

/*AD7793 Registers*/
#define AD7793_REG_COMM		0 /* Communications Register(WO, 8-bit) */
#define AD7793_REG_STAT	    0 /* Status Register	    (RO, 8-bit) */
#define AD7793_REG_MODE	    1 /* Mode Register	     	(RW, 16-bit */
#define AD7793_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
#define AD7793_REG_DATA	    3 /* Data Register	     	(RO, 16-/24-bit) */
#define AD7793_REG_ID	    4 /* ID Register	     	(RO, 8-bit) */
#define AD7793_REG_IO	    5 /* IO Register	     	(RO, 8-bit) */
#define AD7793_REG_OFFSET   6 /* Offset Register	    (RW, 24-bit */
#define AD7793_REG_FULLSALE	7 /* Full-Scale Register	(RW, 24-bit */

/* Communications Register Bit Designations (AD7793_REG_COMM) */
#define AD7793_COMM_WEN		(0 << 7) 			/* Write Enable */
#define AD7793_COMM_WRITE	(0 << 6) 			/* Write Operation */
#define AD7793_COMM_READ    (1 << 6) 			/* Read Operation */
#define AD7793_COMM_ADDR(x)	(((x) & 0x7) << 3)	/* Register Address */
#define AD7793_COMM_CREAD	(1 << 2) 			/* Continuous Read of Data Register */

/* Status Register Bit Designations (AD7793_REG_STAT) */
#define AD7793_STAT_RDY		(1 << 7) /* Ready */
#define AD7793_STAT_ERR		(1 << 6) /* Error (Overrange, Underrange) */
#define AD7793_STAT_CH3		(1 << 2) /* Channel 3 */
#define AD7793_STAT_CH2		(1 << 1) /* Channel 2 */
#define AD7793_STAT_CH1		(1 << 0) /* Channel 1 */

/* Mode Register Bit Designations (AD7793_REG_MODE) */
#define AD7793_MODE_SEL(x)		(((x) & 0x7) << 13)	/* Operation Mode Select */
#define AD7793_MODE_CLKSRC(x)	(((x) & 0x3) << 6) 	/* ADC Clock Source Select */
#define AD7793_MODE_RATE(x)		((x) & 0xF) 		/* Filter Update Rate Select */

/* AD7793_MODE_SEL(x) options */
#define AD7793_MODE_CONT		 0 /* Continuous Conversion Mode */
#define AD7793_MODE_SINGLE		 1 /* Single Conversion Mode */
#define AD7793_MODE_IDLE		 2 /* Idle Mode */
#define AD7793_MODE_PWRDN		 3 /* Power-Down Mode */
#define AD7793_MODE_CAL_INT_ZERO 4 /* Internal Zero-Scale Calibration */
#define AD7793_MODE_CAL_INT_FULL 5 /* Internal Full-Scale Calibration */
#define AD7793_MODE_CAL_SYS_ZERO 6 /* System Zero-Scale Calibration */
#define AD7793_MODE_CAL_SYS_FULL 7 /* System Full-Scale Calibration */

/* AD7793_MODE_CLKSRC(x) options */
#define AD7793_CLK_INT		0 /* Internal 64 kHz Clk not available at the CLK pin */
#define AD7793_CLK_INT_CO	1 /* Internal 64 kHz Clk available at the CLK pin */
#define AD7793_CLK_EXT		2 /* External 64 kHz Clock */
#define AD7793_CLK_EXT_DIV2	3 /* External Clock divided by 2 */

/* AD7793_MODE_RATE(x) options */
#define AD7793_MODE_RATE_4_17		 0x0f 
#define AD7793_MODE_RATE_6_25		 0x0e 
#define AD7793_MODE_RATE_10		     0x0d 
#define AD7793_MODE_RATE_33_2		 0x07
#define AD7793_MODE_RATE_500		 0x01

/* Configuration Register Bit Designations (AD7793_REG_CONF) */
#define AD7793_CONF_VBIAS(x)  (((x) & 0x3) << 14) 	/* Bias Voltage Generator Enable */
#define AD7793_CONF_BO_EN	  (1 << 13) 			/* Burnout Current Enable */
#define AD7793_CONF_POLAR(x)     (((x) & 0x1) << 12) 	/* Unipolar/Bipolar Enable */
#define AD7793_CONF_BOOST	  (1 << 11) 			/* Boost Enable */
#define AD7793_CONF_GAIN(x)	  (((x) & 0x7) << 8) 	/* Gain Select */
#define AD7793_CONF_REFSEL(x) (((x) & 0x1) << 7) 	/* INT/EXT Reference Select */
#define AD7793_CONF_BUF		  (1 << 4) 				/* Buffered Mode Enable */
#define AD7793_CONF_CHAN(x)	  ((x) & 0x7) 			/* Channel select */

/* AD7793_CONF_GAIN(x) options */
#define AD7793_GAIN_1       0
#define AD7793_GAIN_2       1
#define AD7793_GAIN_4       2
#define AD7793_GAIN_8       3
#define AD7793_GAIN_16      4
#define AD7793_GAIN_32      5
#define AD7793_GAIN_64      6
#define AD7793_GAIN_128     7

/* AD7793_CONF_REFSEL(x) options */
#define AD7793_REFSEL_INT   1	/* Internal Reference Selected. */
#define AD7793_REFSEL_EXT   0	/* External Reference Applied between REFIN(+) and REFIN(¨C). */

/* AD7793_CONF_CHAN(x) options */
#define AD7793_CH_AIN1P_AIN1M	0 /* AIN1(+) - AIN1(-) */
#define AD7793_CH_AIN2P_AIN2M	1 /* AIN2(+) - AIN2(-) */
#define AD7793_CH_AIN3P_AIN3M	2 /* AIN3(+) - AIN3(-) */
#define AD7793_CH_AIN1M_AIN1M	3 /* AIN1(-) - AIN1(-) */
#define AD7793_CH_TEMP			6 /* Temp Sensor */
#define AD7793_CH_AVDD_MONITOR	7 /* AVDD Monitor */

/* ID Register Bit Designations (AD7793_REG_ID) */
#define AD7793_ID			0xB
#define AD7793_ID_MASK		0xF

/* IO (Excitation Current Sources) Register Bit Designations (AD7793_REG_IO) */
#define AD7793_IEXCDIR(x)	(((x) & 0x3) << 2)
#define AD7793_IEXCEN(x)	(((x) & 0x3) << 0)

/* AD7793_IEXCDIR(x) options*/
#define AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2	0  /* IEXC1 connect to IOUT1, IEXC2 connect to IOUT2 */
#define AD7793_DIR_IEXC1_IOUT2_IEXC2_IOUT1	1  /* IEXC1 connect to IOUT2, IEXC2 connect to IOUT1 */
#define AD7793_DIR_IEXC1_IEXC2_IOUT1		2  /* Both current sources IEXC1,2 connect to IOUT1  */
#define AD7793_DIR_IEXC1_IEXC2_IOUT2		3  /* Both current sources IEXC1,2 connect to IOUT2 */

/* AD7793_IEXCEN(x) options*/
#define AD7793_EN_IXCEN_10uA				1  /* Excitation Current 10uA */
#define AD7793_EN_IXCEN_210uA				2  /* Excitation Current 210uA */
#define AD7793_EN_IXCEN_1mA					3  /* Excitation Current 1mA */




/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @param None.
 * 
 * @return  None.    
*******************************************************************************/
void AD7793_Reset(void)
{
	uint8_t dataToSend[4] = {0xff, 0xff, 0xff, 0xff};
	    
	SPI_Write(dataToSend,4);
		
}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
uint32_t AD7793_GetRegisterValue(uint8_t regAddress, uint8_t size)
{
	uint8_t data[4] = {0};
	uint32_t receivedData = 0x00;	
	data[0] =AD7793_COMM_WEN |  AD7793_COMM_READ |  AD7793_COMM_ADDR(regAddress);
	SPI_Write(data,1);
	SPI_Read(data,size);
	if(size == 1)
	{
		receivedData += (data[0] << 0);
	}
	if(size == 2)
	{
		receivedData += ((uint32_t)data[0]<<8);
		receivedData += (data[1] << 0);
	}
	if(size == 3)
	{
		receivedData += ((uint32_t)data[0] << 16);
		receivedData += ((uint32_t)data[1] <<8);
		receivedData += (data[2] << 0);
	}
    return receivedData;
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD7793_SetRegisterValue(uint8_t regAddress,
                             uint32_t regValue, 
                             uint8_t size)
{
	uint8_t data[4] = {0};	
	data[0] = AD7793_COMM_WEN | AD7793_COMM_WRITE |  AD7793_COMM_ADDR(regAddress);
    if(size == 1)
    {
        data[1] = (uint8_t)regValue;
    }
    if(size == 2)
    {
		data[2] = (uint8_t)((regValue & 0x0000FF) >> 0);
		data[1] = (uint8_t)((regValue & 0x00FF00) >> 8);
    }
    if(size == 3)
    {
		data[3] = (uint8_t)((regValue & 0x0000FF) >> 0);
		data[2] = (uint8_t)((regValue & 0x00FF00) >> 8);
        data[1] = (uint8_t)((regValue & 0xFF0000) >> 16);
    }
	SPI_Write(data,(1 + size));
	
}

/***************************************************************************//**
 * @brief check the AD7793 ID is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
static uint8_t AD7793_Check_ID(void)
{ 
	uint8_t status = 0x1;
	status = AD7793_GetRegisterValue(AD7793_REG_ID, 1);
	__NOP();
	__NOP();
	__NOP();
	if((status & 0x0F) != AD7793_ID)
	{
		status = 0x0;
	}
	return(status);
}
/***************************************************************************//**
 * @brief Reads /RDY bit of status reg.
 *
 * @param None.
 *
 * @return rdy	- 0 if RDY is 1.
 *              - 1 if RDY is 0.
*******************************************************************************/
uint8_t AD7793_Ready(void)
{
    uint8_t rdy = 0;
    rdy = (AD7793_GetRegisterValue( AD7793_REG_STAT,1) & 0x80);   
	return(!rdy);
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7793.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void AD7793_SetMode(uint32_t mode)
{
    uint32_t command;
    command = AD7793_GetRegisterValue(AD7793_REG_MODE,2);
    command &= ~ (uint32_t)(AD7793_MODE_SEL(0xFF));
    command |= AD7793_MODE_SEL(mode);
    AD7793_SetRegisterValue(AD7793_REG_MODE,command,2);
}
/***************************************************************************//**
 * @brief Sets the set rate of AD7793.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void AD7793_SetRate(uint32_t rate)
{
    uint32_t command;
    command = AD7793_GetRegisterValue(AD7793_REG_MODE,2);
    command &= ~ (uint32_t)(AD7793_MODE_RATE(0xFF));
    command |= AD7793_MODE_RATE(rate);
    AD7793_SetRegisterValue(AD7793_REG_MODE,command,2);
}
/***************************************************************************//**
 * @brief Selects the channel of AD7793.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
void AD7793_SetChannel(uint32_t channel)
{
    uint32_t command;
    command = AD7793_GetRegisterValue(AD7793_REG_CONF,2);
    command &= ~ (uint32_t)(AD7793_CONF_CHAN(0xFF));
    command |= AD7793_CONF_CHAN(channel);
    AD7793_SetRegisterValue( AD7793_REG_CONF,command,2);
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/
void AD7793_SetGain(uint32_t gain)
{
    uint32_t command;
    command = AD7793_GetRegisterValue(AD7793_REG_CONF,2);
    command &= ~ (uint32_t)(AD7793_CONF_GAIN(0xFF));
    command |= AD7793_CONF_GAIN(gain);
    AD7793_SetRegisterValue(AD7793_REG_CONF,command,2);
}
/***************************************************************************//**
 * @brief Ref select  function.
 *
 * @param state - State of the reference 
 *               Example: 0	- Extern Ref.
 *                        1	- Inter  Ref.
 *
 * @return None.    
*******************************************************************************/
void AD7793_SetReference(uint32_t state)
{
    uint32_t command = 0;
    command = AD7793_GetRegisterValue(AD7793_REG_CONF,2);
    command &= ~ (uint32_t)(AD7793_CONF_REFSEL(0xFF));
    command |= AD7793_CONF_REFSEL(state);
    AD7793_SetRegisterValue(AD7793_REG_CONF,command,2);
}
/***************************************************************************//**
 * @brief Polarity select  function.
 *
 * @param Polarity - Polarity 
 *               Example: 0	- Bipolar 
 *                        1	- Unipolar
 *
 * @return None.    
*******************************************************************************/

void AD7793_SetPolarity(uint32_t Polarity)
{
    uint32_t command = 0;
    command = AD7793_GetRegisterValue(AD7793_REG_CONF,2);
    command &= ~ (uint32_t)(AD7793_CONF_POLAR(0xFF));
    command |= AD7793_CONF_POLAR(Polarity);
    AD7793_SetRegisterValue(AD7793_REG_CONF,command,2);
}


void AD7793_SetIexcDir(uint32_t IexcDir)
{
    uint32_t command = 0;
    command = AD7793_GetRegisterValue(AD7793_REG_IO,1);
    command &= ~ (uint32_t)(AD7793_IEXCDIR(0xFF));
    command |= AD7793_IEXCDIR(IexcDir);
    AD7793_SetRegisterValue(AD7793_REG_IO,command,1);
}
    
void AD7793_SetIexcEn(uint32_t IexcEn)
{
    uint32_t command = 0;
    command = AD7793_GetRegisterValue(AD7793_REG_IO,1);
    command &= ~ (uint32_t)(AD7793_IEXCEN(0xFF));
    command |= AD7793_IEXCEN(IexcEn);
    AD7793_SetRegisterValue(AD7793_REG_IO,command,1);
}    
					



static uint16_t AD7793_Calibrate(uint32_t CHx,uint32_t Gain,uint32_t Rate)  
{    
	uint32_t offset=0,full=0;
	uint16_t i=0;
	AD7793_SetChannel(CHx);
	AD7793_SetGain(Gain);
	AD7793_SetRate(Rate);
	AD7793_SetMode(AD7793_MODE_CAL_INT_ZERO);//
	ADI_CS_LOW;
	for (i=0;i<5;i++)
	{
		while (ADS_DOUT);
	}
	ADI_CS_HIGH;
	offset = AD7793_GetRegisterValue( AD7793_REG_OFFSET, 3);
	
	AD7793_SetMode(AD7793_MODE_CAL_INT_FULL);//
	ADI_CS_LOW;
	for (i=0;i<5;i++)
	{
		while (ADS_DOUT);
	}
	ADI_CS_HIGH;
	full = AD7793_GetRegisterValue( AD7793_REG_FULLSALE, 3);
	return 1;  
}  




static uint8_t AD7793_Init(uint32_t Gain,uint32_t Rate) 
{  
	AD7793_Reset(); 
	
	if(!AD7793_Check_ID())
	{
		return 0;
	}
//    AD7793_SetGain(Gain);
//    AD7793_SetRate(Rate);
//    AD7793_SetReference(AD7793_REFSEL_EXT);
//    AD7793_SetPolarity(0);
    AD7793_SetIexcDir(AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2);
    AD7793_SetIexcEn (AD7793_EN_IXCEN_210uA);
    AD7793_SetMode(AD7793_MODE_PWRDN);
    return 1;  
} 



uint16_t AD7793_Start(uint32_t CHx,uint32_t Gain,uint32_t Rate)  
{      
	//AD7793_Reset(); 
//	AD7793_SetPolarity(AD7793_CONF_BIPOLAR);
	AD7793_SetChannel(CHx);
	AD7793_SetGain(Gain);
	AD7793_SetRate(Rate);
	AD7793_SetMode(AD7793_MODE_SINGLE);
	return 1;  
} 

/****************************ad7793 end**************************************/


 

static void ad779x_stop(void)
{
	
    //AD7793_SetMode(AD7793_MODE_PWRDN);
	
}

static int16_t ad779x_ReadAD(int32 *ad)
{
    int32 addat = 0;
    addat=AD7793_GetRegisterValue(AD7793_REG_DATA,3);
    addat-=0x00800000;
    *ad = formatAd(addat);
    return 1;
}

static int16_t ad779x_Init(int16_t channel_num,int16_t gain,int16_t rate)
{
	
    return (! AD7793_Init(gain,rate));
}


static void ad779x_Restart(uint32_t CHx,uint32_t Gain,uint32_t Rate)
{
   AD7793_Start(CHx,Gain, Rate);
   ADI_CS_LOW;
}


void ad779x_task_handle(void)
{
    if(adx_comps.sw._bit.adc_updated)
    {
	    
        if(adx_comps.current_channel==0)
        {
	        if(device_comps.ad1_pos<MD_ADC_MAX_POS)
            {
               ad779x_ReadAD(&device_comps.ad1_convert_result[device_comps.ad1_pos++]);
            }
        }
        else if(adx_comps.current_channel==1)
        {
	        if(device_comps.ad2_pos<MD_ADC_MAX_POS)
            {
                //ad779x_ReadAD(&device_comps.voltage_p_convert_result[device_comps.voltage_p_pos++]);//
               ad779x_ReadAD(&device_comps.temp_p_convert_result[device_comps.temp_p_pos++]);
	     
            }
            
        }
        else if(adx_comps.current_channel==2)
        {
            if(device_comps.ad1_pos<MD_ADC_MAX_POS)
            {
                
            }
        }
        
        adx_comps.current_channel++;
        adx_comps.current_channel%=2;
	    if(adx_comps.init_channel== adx_comps.current_channel)
        {
            ad779x_stop();
            //MD_SET_AVDD_OFF;
            adx_comps.sw._bit.running=0;
        }
        else
        {
            ad779x_Restart(adx_comps.current_channel,adx_comps.gain,adx_comps.rate);
            adx_comps.enable_eoc_interrupt();
	    }
	
        adx_comps.sw._bit.adc_updated=0;
    }
}

void NOP(void)
{
	__NOP();
}

//#define  MD_ADI_DOUT_PORT_IRQN PORT_X_IRQn
void enable_adx_eoc_interrutp(void)   
{ 
    Gpio_ClearIrq(MD_ADI_DOUT_PORT, MD_ADI_DOUT_PIN); 
    Gpio_EnableIrq(MD_ADI_DOUT_PORT, MD_ADI_DOUT_PIN, GpioIrqFalling);
   // EnableNvic(MD_ADI_DOUT_PORT_IRQN, IrqLevel3, TRUE);
} 

void disable_adx_eoc_interrutp(void) 
{ 
    Gpio_DisableIrq(MD_ADI_DOUT_PORT, MD_ADI_DOUT_PIN, GpioIrqFalling);
    Gpio_ClearIrq(MD_ADI_DOUT_PORT, MD_ADI_DOUT_PIN);
   // EnableNvic(MD_ADI_DOUT_PORT_IRQN, IrqLevel3, FALSE);
} 

void PortX_IRQHandler(void)
{
     if(TRUE == Gpio_GetIrqStatus(MD_ADI_DOUT_PORT, MD_ADI_DOUT_PIN))
     {            
        if(!GetBit(((uint32_t)&M0P_GPIO->PAIN  + MD_ADI_DOUT_PORT), MD_ADI_DOUT_PIN)) 
         {
            adx_comps.sw._bit.adc_updated=1;
            adx_comps.disale_eoc_interrup();
         }
       
        Gpio_ClearIrq(MD_ADI_DOUT_PORT, MD_ADI_DOUT_PIN);    
    }
}

adx_comps_t adx_comps=
{
   	 "",

	AD7793_CH_AIN1P_AIN1M,//const int16_t init_channel;
	0,//int16_t current_channel;
	AD7793_GAIN_8 ,//int16_t gain;
	AD7793_MODE_RATE_4_17,//int16_t rate;
	0,   //	union sw;
	ad779x_Init,//int16_t (*const Init )(int16_t channel_num,int16_t gain,int16_t rate)
	ad779x_stop,
	ad779x_Restart,//void (*const restart)(uint32_t CHx,uint32_t Gain,uint32_t Rate);// 
	disable_adx_eoc_interrutp,//void (*const disale_interrup(void);
	enable_adx_eoc_interrutp,//void (*const enable_interrupt(void);
	ad779x_task_handle//void  ( *const task_handle)(void);
};











