
#include "ddl.h"
#include "epress.h"
#include  "sw_i2c.h"

//// Define the upper and lower limits of the calibration pressure JHM1200 
//#define PMIN 0 //Full range pressure for example 20Kpa
//#define PMAX 700000 //Zero Point Pressure Value, for example 120Kpa
//#define DMIN 1500                           //15.00%AD  
//#define DMAX 8500                           //85.00%AD

////The 7-bit IIC address of the JHM1200 is 0x78
//static uint8_t Device_Address = 0x78;




////Delay function needs to be defined
//uint8_t JHM1200_read_stauts(uint8_t *status)
//{
//	uint8_t ret=sw_i2c_comps.read(Device_Address, status, 1,1);
//    if(ret)
//    {
//        *status=0;
//    }
//    return ret;
//}

////Read the status of IIC and judge whether IIC is busy
//uint8_t JHM1200_IsBusy(void)
//{
//	uint8_t status;
//	JHM1200_read_stauts(&status);
//	status = (status >> 5) & 0x01;
//	return status;
//}

//static uint8_t JHM1200_start_measure(void)
//{
//     uint8_t JHM1200_cmd_start = 0xac;
//    return sw_i2c_comps.write(Device_Address, &JHM1200_cmd_start, 1,1);
//}


///**
//  * @brief Using the 0xAC command to calculate the actual pressure and temperature using the JHM1200 internal algorithm
//  * @note  Send 0xAC, read IIC status until IIC is not busy
//  *	@note  The returned data is a total of six bytes, in order: status word, three-byte pressure value, two-byte temperature value
//  * @note  The returned three-byte pressure value is proportional to the 24-bit maximum value 16777216. According to this ratio, 
//           the actual pressure value is again converted according to the calibration range.
//  * @note  The returned two-byte temperature value is proportional to the 16-bit maximum value 65536. According to this ratio, 
//           the actual pressure value is again converted according to the calibration range.
//  * @note  Zero pressure point and full pressure point of calibration pressure correspond to 20kpa and 120Kpa, respectively   
//  * @note  The zero point of the calibration temperature is -40°C and the full point is 150°C
//  * @note  The pressure actual value is calculated according to the span pressure unit is Pa, temperature actual value temp unit is 0.01°C
//  */
//uint8_t JHM1200_read_measure_relt(int32_t *press, int16_t *temp)//Pa  0.1deg
//{
//	uint8_t buffer[6] = {0};
//	int32_t Dtest = 0;
//	uint16_t temp_raw = 0;
//	int32_t pressure =0 ;
//	//read the returned six-byte data
//	uint8_t ret=sw_i2c_comps.read(Device_Address, buffer, 6,1);
//	if(ret)
//	{
//			*press=0;
//			*temp=0;
//			return ret;
//	}
//	//The returned pressure and temperature values are converted into actual values according to the calibration range
//	Dtest = ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3];
//	temp_raw = ((uint16_t)buffer[4] << 8) | (buffer[5] << 0);
//	pressure = ((int64_t)Dtest*10000/16777216-DMIN)*(PMAX-PMIN)/(DMAX-DMIN)+PMIN;
//	if(pressure<0)
//	{
//			pressure=0;
//	}
//	*press=pressure;
//	*temp = (uint32_t)temp_raw*(1500-(-400)) / 65535 +(-400);
//	return ret;
//}



	#define PMIN 0 //Full range pressure for example 20Kpa
	#define PMAX 700000 //700Kpa 3dot
	#define DMIN 1000                           //10.00%AD  
	#define DMAX 9000                           //90.00%AD

	static const uint8_t _1942_Address = 0x7f;

    static const uint8_t  _1942_press_unit=1;
    static const uint8_t  _1942_press_dot=3;
    static const int32_t  _1942_press_full_scale=700000;

	static uint8_t _1942_read_stauts(uint8_t *status)
	{
		uint8_t ret=sw_i2c_comps.read(_1942_Address, status, 1,1);
		if(ret)
		{
			*status=0;
		}
		return ret;
	}

	//Read the status of IIC and judge whether IIC is busy
	uint8_t _1942_IsBusy(void)
	{
		uint8_t status;
		_1942_read_stauts(&status);
		status = (status >> 5) & 0x01;
		return status;
	}

	static uint8_t _1942_start_measure(void)
	{
		uint8_t _1942_cmd_start[2] ={0x30, 0x0a};
		return sw_i2c_comps.write(_1942_Address, &_1942_cmd_start, 2,1);
	}


	uint8_t _1942_read_measure_relt(int32_t *press, uint8_t *unit, uint8_t *dot, int32_t *full_scale, int16_t *temp)
	{
		uint8_t buffer[6] = {0};
		int32_t Dtest = 0;
		uint16_t temp_raw = 0;
		int32_t pressure =0 ;
	     uint8_t addr=0x06;
		 uint8_t ret;
		//read the returned six-byte data
		ret=sw_i2c_comps.write(_1942_Address, &addr, 1,1);
		if(ret)
		{
			*press=0;
			*temp=0;
			return ret;
		}
		ret=sw_i2c_comps.read(_1942_Address, buffer, 3,1);
		if(ret)
		{
			*press=0;
			*temp=0;
			return ret;
		}
		//The returned pressure and temperature values are converted into actual values according to the calibration range
		Dtest = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
		if(Dtest>8388607) {Dtest-=16777216;}
		//temp_raw = ((uint16_t)buffer[4] << 8) | (buffer[5] << 0);
		pressure = ((int64_t)Dtest*10000/8388608-DMIN)*(PMAX-PMIN)/(DMAX-DMIN)+PMIN;
		if(pressure<0)
		{
				pressure=0;
		}
		*press=pressure;
		*unit=_1942_press_unit;
		*dot=_1942_press_dot;
		*full_scale=_1942_press_full_scale;
		//*temp = (uint32_t)temp_raw*(1500-(-400)) / 65535 +(-400);
		return ret;
	}
	#undef  PMIN
	#undef  PMAX
	#undef  DMIN
	#undef  DMAX

	
	

	#define PMIN 0 //Full range pressure for example 20Kpa
	#define PMAX 120000 //1.2MPa 5dot
	#define DMIN 0                          //0.00%AD  
	#define DMAX 9000                           //90.00%AD

    static const uint8_t Amp6127_Address = 0x78;

    static const uint8_t amp6127_press_unit	=0;
	static const uint8_t amp6127_press_dot=5;
	static const int32_t amp6127_press_full_scale=120000;

	static uint8_t amp6127_read_stauts(uint8_t *status)
	{
		uint8_t ret=sw_i2c_comps.read(Amp6127_Address, status, 1,1);
		if(ret)
		{
			*status=0;
		}
		return ret;
	}

	//Read the status of IIC and judge whether IIC is busy
	uint8_t amp6127_IsBusy(void)
	{
		uint8_t status;
		amp6127_read_stauts(&status);
		status = (status >> 5) & 0x01;
		return status;
	}

	static uint8_t amp6127_start_measure(void)
	{
		uint8_t amp6127_cmd_start=0xac;
		return sw_i2c_comps.write(Amp6127_Address, &amp6127_cmd_start,1,1);
	}


	uint8_t amp6127_read_measure_relt(int32_t *press, uint8_t *unit, uint8_t *dot, int32_t *full_scale, int16_t *temp)
	{
		uint8_t buffer[6] = {0};
		int32_t Dtest = 0;
		uint16_t temp_raw = 0;
		int32_t pressure =0 ;
		uint8_t ret=sw_i2c_comps.read(Amp6127_Address, buffer, 6,1);
		if(ret)
		{
			*press=0;
			*temp=0;
			return ret;
		}
		Dtest = ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3];
		temp_raw = ((uint16_t)buffer[4] << 8) | (buffer[5] << 0);
		pressure = ((int64_t)Dtest*10000/0x1000000-DMIN)*(PMAX-PMIN)/(DMAX-DMIN)+PMIN;
		if(pressure<0)
		{
			pressure=0;
		}
		*press=pressure;
		*unit=amp6127_press_unit;
		*dot=amp6127_press_dot;
		*full_scale=amp6127_press_full_scale;
		*temp = (uint32_t)temp_raw*(1500-(-400)) / 0x10000 +(-400);
		return ret;
	}
	#undef  PMIN
	#undef  PMAX
	#undef  DMIN
	#undef  DMAX


  
	#define  BK_LN

	static const uint8_t cps135b_Address = 0x6d;

    static const uint8_t  cps135b_press_unit=0;
    static const uint8_t  cps135b_press_dot=5;
    static const int32_t  cps135b_press_full_scale=100000;

	static uint8_t cps135b_read_stauts(uint8_t *status)
	{
		uint8_t ret=0;
		if(ret)
		{
			*status=0;
		}
		return ret;
	}

	uint8_t cps135b_IsBusy(void)
	{
		uint8_t status;
		cps135b_read_stauts(&status);
		status = (status >> 5) & 0x01;
		return status;
	}

	static uint8_t cps135b_start_measure(void)
	{
		uint8_t cps135b_cmd_start[2] ={0x30, 0x0a};
		return sw_i2c_comps.write(cps135b_Address, &cps135b_cmd_start, 2,1);
	}


	uint8_t cps135b_read_measure_relt(int32_t *press, uint8_t *unit, uint8_t *dot, int32_t *full_scale, int16_t *temp)
	{
		uint8_t buffer[6] = {0};
		int32_t Dtest = 0;
		uint16_t temp_raw = 0;
		int32_t pressure =0 ;
	    uint8_t addr=0x06;
		uint8_t ret;
		ret=sw_i2c_comps.write(cps135b_Address, &addr, 1,1);
		if(ret)
		{
			*press=0;
			*temp=0;
			return ret;
		}
		ret=sw_i2c_comps.read(cps135b_Address, buffer, 5,1);
		if(ret)
		{
				*press=0;
				*temp=0;
				return ret;
		}
		Dtest = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
		temp_raw = ((uint16_t)buffer[3] << 8) | (buffer[4] << 0);
		pressure = (int64_t)Dtest/80;
		if(pressure<0)
		{
			pressure=0;
		}
		*press=pressure;
		*unit=cps135b_press_unit;
		*dot=cps135b_press_dot;
		*full_scale=cps135b_press_full_scale;
		*temp = (int16_t) ((int32_t)((int16_t)temp_raw)*10/256);
		 return ret;
	}

#define  BK_LN

static uint8_t measure_start(void)
{ 
	uint8_t ret=0;
	if((ret=!_1942_start_measure()))
	{

	}
    else if( ( ret=!cps135b_start_measure() ) )
	{

	}	
	// else if((ret=!amp6127_start_measure()))
	// {

	// }
	return !ret;
}

static uint8_t read_measure_relt(int32_t *press, uint8_t *unit, uint8_t *dot, int32_t *full_scale, int16_t *temp)
{
	uint8_t ret=0;
	if((ret=!_1942_read_measure_relt(press, unit, dot, full_scale, temp)))
	{

	}
	else if((ret=!cps135b_read_measure_relt(press, unit, dot, full_scale, temp)))
	{

	}
	// else if((ret=!amp6127_read_measure_relt(press, unit, dot, full_scale, temp)))
	// {

	// }
	return !ret;
}
 epress_comps_t epress_comps=
 {
	.desc="",
	.read_reasure_relt_error_times=0,
	.start_measure=measure_start,
	.read_measure_relt=read_measure_relt
};


