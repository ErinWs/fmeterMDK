#include "ddl.h"
#include "rtc.h"
#include "gpio.h"
#include "pcnt.h"
#include "lptim.h"
#include "bgr.h"
#include "adc.h"
#include "wdt.h"
#include "bt.h"

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"


#include "device.h"
#include "system.h"
#include "net.h"
#include "protocol.h"
#include "hum.h"
#include "elora.h"
#include "irc.h"
#include "24cxx.h"
#include "adx.h"
#include "sw_i2c.h"
#include "sht4x.h"
#include "epress.h"
#include "ertc.h"
#include "etemp.h"
#include "modbus.h"


#define  MD_PT100              0
#define  MD_PT1000             1
#define  MD_TEMP_MEASURE_TYPE  MD_PT1000
  
			
static const int16_t pt100_tab[]=
{   
    
    7633 ,7673 //,7713 ,7752 ,7792 ,7832 ,7872 ,7911 ,7951 ,7991, //-60....-51
//    8031 ,8070 ,8110 ,8150 ,8189 ,8229 ,8269 ,8308 ,8348 ,8388, 
//    8427 ,8467 ,8506 ,8546 ,8585 ,8625 ,8664 ,8704 ,8743 ,8783, 
//    8822 ,8862 ,8901 ,8940 ,8980 ,9019 ,9059 ,9093 ,9137 ,9177, 
//    9216 ,9255 ,9295 ,9334 ,9373 ,9412 ,9452 ,9491 ,9530 ,9569, 
//    9609 ,9648 ,9687 ,9726 ,9765 ,9804 ,9844 ,9883 ,9922 ,9961,//-10....-1
//    
//    10000,10039,10078,10117,10156,10195,10234,10273,10312,10351,//0......9
//    10390,10429,10468,10507,10546,10585,10624,10663,10702,10740,
//    10779,10818,10875,10896,10935,10973,11012,11051,11090,11128,
//    11167,11206,11245,11283,11322,11361,11499,11438,11477,11515,
//    11554,11593,11631,11670,11708,11747,11785,11824,11862,11901,
//    11940,11978,12016,12055,12093,12132,12170,12209,12247,12286,
//    12324,12362,12401,12439,12477,12516,12554,12592,12631,12669,
//    12707,12745,12784,12822,12860,12898,12937,12975,13013,13051,
//    13089,13127,13166,13204,13242,13280,13318,13356,13394,13432,
//    13470,13508,13546,13584,13622,13660,13698,13736,13774,13812,
//    13850,13888,13926,13964,14002,14039,14077,14115,14153,14191,
//    14229,14266,14304,14342,14380,14417,14455,14493,14531,14568,
//    14606,14644,14681,14719,14757,14794,14832,14870,14907,14945,
//    14982,15020,15057,15095,15133,15170,15208,15245,15283,15320,
//    15358,15395,15432,15470,15507,15545,15582,15619,15657,15694,
//    15731,15769,15806,15843,15881,15918,15955,15993,16030,16067,
//    16104,16142,16179,16216,16253,16290,16327,16365,16402,16439,
//    16476,16513,16550,16587,16624,16661,16698,16735,16772,16809,
//    16846,16883,16920,16957,16994,17031,17068,17105,17142,17179,
//    17216,17253,17290,17326,17363,17400,17437,17474,17510,17547,
//    17584,17621,17657,17694,17731,17768,17804,17841,17878,17914,
//    17951,17988,18024,18061,18097,18134,18171,18207,18244,18280,
//    18317,18353,18390,18426,18463,18499,18536,18572,18609,18645,
//    18632,18718,18754,18791,18827,18863,18900,18936,18972,19009,
//    19045,19081,19118,19154,19190,19226,19263,19299,19335,19371,
//    19407,19444,19480,19516,19552,19588,19624,19660,19696,19733,
//    19769,19805,19841,19877,19913,19949,19985,20021,20057,20093,
//    20129,20165,20201,20236,20272,20308,20344,20380,20416,20452,
//    20488,20523,20559,20595,20631,20667,20702,20738,20774,20810,
//    20845,20881,20917,20952,20988,21024,21059,21095,21131,21166,
//    21202,21237,21273,21309,21344,21380,21415,21451,21486,21522,
//    21557,21593,21628,21664,21799,21735,21770,21805,21841,21876,
//    21912,21947,21982,22018,22053,22088,22124,22159,22194,22229,
//    22265,22300,22335,22370,22406,22441,22476,22511,22546,22581,
//    22617,22652,22687,22722,22757,22792,22827,22862,22897,22932,
//    22962,23002,23042,23082,23122,23162,23202,23242,23282,23322,
//    23317,23352,23387,23422,23456,23491,23526,23561,23596,23631,
//    23665,23700,23735,23770,23804,23839,23874,23909,23945,23978,
//    24013,24047,24082,24117,24151,24186,24220,24255,24290,24324,
//    24359,24393,24428,24462,24497,24531,24566,24600,24635,24669,
//    24704,24738,24773,24807,24841,24876,24910,24945,24979,25013,
//    25048,25082,25116,25150,25185,25219,25253,25255,25322,25356,
//    25390,25424,25459,25493,25527,25561,25595,25629,25664,25698,
//    25732,25766,25800,25834,25868,25902,25936,25970,26004,26038,
//    26072,26106,26140,26174,26208,26242,26276,26310,26343,26377,
//    26411,26445,26479,26513,26547,26580,26614,26648,26682,26715,
//    26749,26783,26817,26850,26884,26918,26951,26985,27019,27052,
//    27086,27120,27153,27187,27220,27254,27288,27321,27355,27388,
//    27422,27455,27489,27522,27556,27589,27623,27656,27689,27723,
//    27756,27790,27823,27856,27890,27923,27956,27990,28023,28056,
//    28090,28123,28156,28189,28223,28256,28289,28322,28355,28389,
//    28422,28455,28488,28521,28554,28587,28621,28654,28687,28720,
//    28753,28786,28819,28852,28885,28918,28951,28984,29017,29050,
//    29083,29116,29149,29181,29214,29247,29280,29313,29346,29379,
//    29411,29444,29477,29510,29543,29575,29608,29641,29674,29706,
//    29739,29772,29804,29837,29870,29902,29935,29968,30000,30033,
//    30065,30098,30131,30163,30196,30228,30261,30293,30326,30358,
//    30391,30423,30456,30488,30520,30553,30585,30618,30650,30682,
//    30715,30747,30779,30812,30844,30876,30909,30941,30973,31005,
//    31038,31070,31102,31134,31167,31199,31231,31263,31295,31327  //590...599

};
    
static const int32_t pt1000_tab[]=
{
    
    803063 ,807033 ,811003 ,814970 ,818937 ,822902 ,826865 ,830828 ,834789 ,838748 ,//-50
    842707 ,846664 ,850619 ,854573 ,858526 ,862478 ,866428 ,870377 ,874325 ,878272 ,//-40
    882217 ,886161 ,890103 ,894044 ,897985 ,901923 ,905861 ,909798 ,913733 ,917666 ,//-30
    921599 ,925531 ,929460 ,933390 ,937317 ,941244 ,945170 ,949094 ,953016 ,956938 ,//-20
    960859 ,964779 ,968697 ,972614 ,976529 ,980444 ,984358 ,988270 ,992181 ,996091 ,//-10
    1000000,1003908,1007814,1011720,1015624,1019527,1023429,1027330,1031229,1035128,//0
    1039025,1042921,1046816,1050710,1054603,1058495,1062385,1066274,1070162,1074049,//10
    1077935,1081820,1085703,1089585,1093467,1097347,1101225,1105103,1108980,1112855,//20
    1116729,1120602,1124474,1128345,1132215,1136083,1139950,1143817,1147681,1151545,//30
    1155408,1159270,1163130,1166989,1170847,1174704,1178560,1182414,1186268,1190120,//40
    1193971,1197821,1201670,1205518,1209364,1213210,1217054,1220897,1224739,1228579,//50
    1232419,1236257,1240095,1243931,1247766,1251600,1255432,1259264,1263094,1266923,//60
    1270751,1274578,1278404,1282228,1286052,1289874,1293695,1297515,1301334,1305152,//70
    1308968,1312783,1316597,1320411,1324222,1328033,1331843,1335651,1339458,1343264,//80
    1347069,1350873,1354676,1358477,1362277,1366077,1369875,1373671,1377467,1381262,//90
    1385055,1388847,1392638,1396428,1400217,1404005,1407791,1411576,1415360,1419143,//100
    1422925,1426706,1430485,1434264,1438041,1441817,1445592,1449366,1453138,1456910,//110
    1460680,1464449,1468217,1471984,1475750,1479514,1483277,1487040,1490801,1494561,//120
    1498319,1502077,1505833,1509589,1513343,1517096,1520847,1524598,1528381,1532139,//130
    1535843,1539589,1543334,1547078,1550820,1554562,1558302,1562041,1565779,1569516,//140
    1573251,1576986,1580719,1584451,1588182,1591912,1595641,1599368,1603094,1606820,//150
    1610544,1614267,1617989,1621709,1625429,1629147,1632864,1636580,1640295,1644009,//160
    1647721,1651433,1655143,1658852,1662560,1666267,1669972,1673677,1677380,1681082,//170
    1684783,1688483,1692181,1695879,1699575,1703271,1706965,1710658,1714349,1718040,//180
    1721729,1725418,1729105,1732791,1736475,1740159,1743842,1747523,1751203,1754882,//190
    1758560,1762237,1765912,1769587,1773260,1776932,1780603,1784273,1787941,1791610,//200
    1795275,1798940,1802604,1806267,1809929,1813590,1817249,1820907,1824564,1828220,//210
    1831875,1835529,1839181,1842832,1846483,1850132,1853779,1857426,1861072,1864716,//220
    1868359,1872001,1875642,1879282,1882921,1886558,1890194,1893830,1897463,1901096,//230
    1904728,1908359,1911988,1915616,1919243,1922869,1926494,1930117,1933740,1937361,//240
    1940981,1944600,1948218,1951835,1955450,1959065,1962678,1966290,1969901,1973510,//250
    1977119,1980726,1984333,1987938,1991542,1995145,1998746,2002347,2005946,2009544,//260
    2013141,2016737,2020332,2023925,2027518,2031109,2034699,2038288,2041876,2045463,//270
    2049048,2052632,2056215,2059798,2063378,2066958,2070537,2074114,2077690,2081265,//280
    2084839,2088412,2091984,2095554,2099123,2102692,2106259,2109824,2113389,2116953,//290
    2120515									

};

static const int16_t TC_coldend_tab[]={-500,-400/*,-300,-200,-100,0,100,200,300,400,500,600,700*/ };
static const uint16_t TC_tab[]=                             // base +2000uV
{
    11   ,474 // ,844	 ,1223 ,1609 ,2000 ,                          //-50 -40      -10    0
//    2396 ,2798 ,3203 ,3611 ,4023 ,4436 ,4851 ,5266 ,5681 ,6096 ,   //10  20 30....      100
//    6509 ,6919 ,7328 ,7734 ,8138 ,8540 ,8940 ,9340 ,9739 ,10138,
//    10538,10939,11342,11747,12153,12561,12970,13382,13794,14208,
//    14623,15039,15456,15874,16293,16712,17132,17553,17975,18397,
//    18819,19243,19666,20091,20515,20940,21366,21792,22218,22644,
//    23070,23497,23923,24350,24776,25202,25628,26054,26480,26905,
//    27330,27754,28178,28602,29024,29447,29868,30289,30709,31129,
//    31547,31965,32382,32798,33213,33627,34041,34453,34864,35275,
//    35684,36093,36501,36907,37313,37717,38121,38523,38925,39325,
//    39725,40124,40521,40918,41313,41708,42101,42493,42885,43275,
//    43664,44053,44440,44826,45211,45595,45977,46359,46739,47118,
//    47496,47873,48248,48622,48995,49366,49736,50105,50472,50838,
//    51202,51565,51926,52285,52643,53000,53355,53708,54060,54410 //1210 1220...            1300
};

static void delay_us(int16_t t)
{
	while(t>0)
	{
		__NOP();__NOP();
		t--;
	}
}

static uint8_t Check_Sum_5A( const void* Data,uint8_t Len)
{
    uint8_t Sum=0x5A;
    uint8_t i=0;
	uint8_t *data=(uint8_t *)Data;
    for(i=0;i<Len;i++)
    {
        Sum+=data[i];
    }
    return Sum;
}


static int32_t pwr(int16_t n)
{
    if(n==0)
    {
        return 1;
    }
    if(n==1)
    {
        return 10;
    }
    if(n==2)
    {
        return 100;
    }
    if(n==3)
    {
        return 1000;
    }
    if(n==4)
    {
        return 10000;
    }
    if(n==5)
    {
        return 100000;
    }
    if(n==6)
    {
        return 1000000;
    }
    if(n==7)
    {
        return 10000000;
    }
    if(n==8)
    {
        return 100000000;
    }
     if(n==9)
    {
        return 1000000000;
    }
    return 1;
}


static float32_t f_mul(float32_t a,float32_t b)
{
    return a*b;
}

static float32_t f_div(float32_t a,float32_t b)
{
    return a/b;
}

#define  BK_LN

int16_t read_press_cal_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_PRESS_CALIBRATION_PARAM_START_ADDR,buf,len);
}
int16_t save_press_cal_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_PRESS_CALIBRATION_PARAM_START_ADDR,buf,len);
}

int16_t read_res_cal_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_RES_CALIBRATION_PARAM_START_ADDR,buf,len);
}
int16_t save_res_cal_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_RES_CALIBRATION_PARAM_START_ADDR,buf,len);
}

int16_t read_flow_cal_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_CAL_PARAM_START_ADDR,buf,len);
}
static int16_t save_flow_cal_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_CAL_PARAM_START_ADDR,buf,len);
}


int16_t read_flow_meter(void *buf,int16_t len)
 {
    return _24cxx_comps.read(MD_FLOW_METER_START_ADDR,buf,len);
 }
 int16_t save_flow_meter(void const *buf,int16_t len)
{
     return _24cxx_comps.write(MD_FLOW_METER_START_ADDR,buf,len);
}


int16_t read_meter(void *buf,int16_t len)
{
   // uint16_t addr=MD_METER_START_ADDR;
   // addr=addr+device_comps.meter_backup.store_index*16;
    return _24cxx_comps.read(MD_METER_START_ADDR,buf,len);
 }
 int16_t save_meter(void const *buf,int16_t len)
 {
    //uint16_t addr=MD_METER_START_ADDR;
    //addr=addr+device_comps.meter_backup.store_index*16;
    return _24cxx_comps.write(MD_METER_START_ADDR,buf,len);
 }

  int16_t read_meter_backup(void *buf,int16_t len)
  {
    return _24cxx_comps.read(MD_METER_BACKUP_START_ADDR,buf,len);
  }
  int16_t save_meter_backup(void const *buf,int16_t len)
  {
    return _24cxx_comps.write(MD_METER_BACKUP_START_ADDR,buf,len);
  }

static int16_t read_device_coe(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_DEVICE_COE_START_ADDR,buf,len);
}

int16_t save_device_coe(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_DEVICE_COE_START_ADDR,buf,len);
}

static int16_t read_manufacturer_info(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_MANUFACTURER_INFO_START_ADDR,buf,len);
}

static int16_t save_manufacturer_info(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_MANUFACTURER_INFO_START_ADDR,buf,len);
}

static int16_t read_device_info(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_DEVICE_INFO_START_ADDR,buf,len);
}

static int16_t save_device_info(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_DEVICE_INFO_START_ADDR,buf,len);
}

static int16_t read_sensor_info(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_SENSOR_INFO_START_ADDR,buf,len);
}

static int16_t save_sensor_info(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_SENSOR_INFO_START_ADDR,buf,len);
}

int16_t read_temp_comp_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_TEMP_COMP_PARAM_START_ADDR,buf,len);
}
int16_t save_temp_comp_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_TEMP_COMP_PARAM_START_ADDR,buf,len);
}

int16_t read_alarm_param(void *buf,int16_t len)
{
    return _24cxx_comps.read(MD_ALARM_PARAM_START_ADDR,buf,len);
}
 
int16_t save_alarm_param(void const *buf,int16_t len)
{
    return _24cxx_comps.write(MD_ALARM_PARAM_START_ADDR,buf,len);
}

  
int16_t read_device_addr(void *buf,int16_t len)
 {
     return _24cxx_comps.read(MD_DEVICE_ADDR_START_ADDR,buf,len);
 }
 int16_t save_device_addr(void const *buf,int16_t len)
 {
    return _24cxx_comps.write(MD_DEVICE_ADDR_START_ADDR,buf,len);
 }









int16_t read_time_seg_data_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_TIME_SEG_DATA_PARAM_START_ADDR,buf,len);
}

int16_t save_time_seg_data_param(void const *buf,int16_t len )
{
     return _24cxx_comps.write(MD_TIME_SEG_DATA_PARAM_START_ADDR,buf,len);
}

int16_t read_system_time(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_SYSTEM_TIME_START_ADDR,buf,len);
}

int16_t save_system_time(void const *buf,int16_t len )
{
     return _24cxx_comps.write(MD_SYSTEM_TIME_START_ADDR,buf,len);
}

int16_t read_misc_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_MISC_PARAM_START_ADDR,buf,len);
}

int16_t save_misc_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_MISC_PARAM_START_ADDR,buf,len);
}










static int16_t read_gps_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_GPS_INFO_START_ADDR,buf,len);
}

static int16_t save_gps_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_GPS_INFO_START_ADDR,buf,len);
}





int16_t read_lbs_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_LBS_PARAM_START_ADDR  ,buf,len);
}

int16_t save_lbs_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_LBS_PARAM_START_ADDR ,buf,len);
}

int16_t read_iot_param(void *buf,int16_t len )
{
    return _24cxx_comps.read(MD_IOT_PARAM_START_ADDR ,buf,len);
}

int16_t save_iot_param(void const *buf,int16_t len )
{
    return _24cxx_comps.write(MD_IOT_PARAM_START_ADDR,buf,len);
}



int16_t read_report_param(void *buf,int16_t len)
{
     return _24cxx_comps.read(MD_REPORT_PARAM_START_ADDR,buf,len);
}
int16_t save_report_param(void const *buf,int16_t len)
{
    return _24cxx_comps.write(MD_REPORT_PARAM_START_ADDR,buf,len);
}

     
int16_t read_access_param(void *buf,int16_t len)
{
     return _24cxx_comps.read(MD_ACCESS_PARAM_START_ADDR,buf,len);
}
int16_t save_access_param(void const *buf,int16_t len)
{
     return _24cxx_comps.write(MD_ACCESS_PARAM_START_ADDR,buf,len);
}







#define  BK_LN

 
static void start_buzzer(int16_t timer)
{
    device_comps.buzzer.timer=timer;
}
static void stop_buzzer(void)
{
//    P0 &= 0xFBU;
//    PM0 |= ~0xFBU;
//    R_PCLBUZ0_Stop();
    device_comps.buzzer.sw._bit.running=0;
}


 static int16_t get_batt(void)
 {
    stc_adc_cfg_t              stcAdcCfg;
    int16_t batt;
    uint32_t adc;
    MD_SET_BAT_MEASURE_CTL_ON;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Gpio_SetAnalogMode(MD_BAT_AIN4_PORT, MD_BAT_AIN4_PIN);        //PA00 (AIN0)
    
    DDL_ZERO_STRUCT(stcAdcCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE); 
    Bgr_BgrEnable();        ///< 开启BGR
    Adc_Enable();         ///< 开启ADC
    
    ///< ADC 初始化配置
    stcAdcCfg.enAdcMode         = AdcSglMode;               ///<采样模式-单次
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv2;            ///<采样分频-2
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle12Clk;     ///<采样周期数-12
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelInBgr1p5;  ///<参考电压选择-内部2.5V
    stcAdcCfg.enAdcOpBuf        = AdcMskBufEnable;         ///<OP BUF配置-关
    stcAdcCfg.enInRef           = AdcMskInRefEnable;        ///<内部参考电压使能-开
    stcAdcCfg.enAdcAlign        = AdcAlignRight;            ///<转换结果对齐方式-右
    Adc_Init(&stcAdcCfg);

    Adc_CfgSglChannel(AdcExInputCH4);
    Adc_EnableIrq();
    EnableNvic(ADC_DAC_IRQn, IrqLevel3, TRUE);
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    Adc_SGL_Start();
    device_comps.sw._bit.adc_busy=1;
    while(device_comps.sw._bit.adc_busy);
    adc = Adc_GetSglResult();
    
    Adc_DisableIrq();
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelExtern1;
    stcAdcCfg.enInRef           = AdcMskInRefDisable;        ///<内部参考电压关闭
    Adc_Init(&stcAdcCfg);
    Adc_Disable();
    Bgr_BgrDisable();
    MD_SET_BAT_MEASURE_CTL_OFF;
    
   batt=150*adc*MD_BAT_RES_DIV_GAIN/409500;//0.1V  20k 10k
   return batt;
   
 }



/////start ntc temp

static  int32_t calc_ad3_average(device_comps_t *const this)
{
    int16_t   i=0;
    const int16_t count=this->ad3_pos;
    uint32_t average=0;
    if(!count)
    {
        return 0;
    }
	for(i=0;i<count;i++)
	{
		average+=this-> ad3_convert_result[i];
	}
    average=average*10/count;	
   return average;
}




static int32_t calc_temp_p_temp_n_average(device_comps_t *const this)
{
    int16_t   i=0;
    const int16_t count=this->temp_p_pos;
    int32_t average=0;
	int32_t difference[10]={0};//=malloc(count*sizeof(float32_t));
	if(!count)
	{
	    return 0;
	}
	for(i=0;i<count;i++)
	{
		difference[i]=this-> temp_p_convert_result[i]-this-> temp_n_convert_result[i];
	}
    /*
    for(i=0;i<count-1;i++)
    {
        for(j=0;j<count-1-i;j++)
        {
        	if(difference[j]>difference[j+1])
        	{
        		temp_var=difference[j+1];
        		difference[j+1]=difference[j];
        		difference[j]=temp_var;
        	}
        }
    }
    */
    for(i=0;i<count;i++)
	{
		average+=difference[i];
	}	
	average=(average/(count));
	return average;

}

static int32_t calc_pt_value(device_comps_t *const this)
{
   #if(MD_TEMP_MEASURE_TYPE==MD_PT100)
    int32_t delta_y=this->res_cal_param.y[1]-this->res_cal_param.y[0];
	int32_t delta_x=this->res_cal_param.x[1]-this->res_cal_param.x[0];
    int32_t int32_t pt_value=(int32_t int32_t)delta_y*(this->temp_p_temp_n_average_result-this->res_cal_param.x[0])/delta_x+this->res_cal_param.y[0];
    if(pt_value>pt100_tab[sizeof(pt100_tab)/sizeof(pt100_tab[0])-1])
    {
        pt_value=pt100_tab[sizeof(pt100_tab)/sizeof(pt100_tab[0])-1]*2;
    }
   #elif(MD_TEMP_MEASURE_TYPE==MD_PT1000)
     int64_t pt_value=(int64_t)this->res_cal_param.y[0] *10 * this->temp_p_temp_n_average_result/this->res_cal_param.x[0];
    if(pt_value>pt1000_tab[sizeof(pt1000_tab)/sizeof(pt1000_tab[0])-1])
    {
        pt_value=pt1000_tab[sizeof(pt1000_tab)/sizeof(pt1000_tab[0])-1]*2;
    }
   #endif
	return pt_value;

}

static int16_t calc_pt_temp(device_comps_t  *const this)
{
    int32_t Tem;
    int32_t LowRValue;
    int32_t HighRValue;        
    int32_t   Tem0;
    int16_t i;
    int16_t  cBottom, cTop;
  #if(MD_TEMP_MEASURE_TYPE==MD_PT100)   
    int16_t  limit=sizeof(pt100_tab)/sizeof(pt100_tab[0])-1;
    cBottom = 0; 
    cTop    = limit;

    if (this->pt_value <pt100_tab[0])                // ����ֵС�ڱ�����ֵ�����������ޡ�
    {
     this->sw._bit.isTempNoConnect=1;    
	 return -600;
         
    }
    if (this->pt_value >pt100_tab[limit])        // ����ֵ���ڱ������ֵ�������������� ��
    {
         this->sw._bit.isTempNoConnect=1;
		return 5990;
    }
	this->sw._bit.isTempNoConnect=0;
   
     for (i=limit/2; (cTop-cBottom)!=1; )        // 2�ַ������
    {

       if (this->pt_value <pt100_tab[i])
        {
            cTop = i;
            i = (cTop + cBottom) / 2;
        }
        else if (this->pt_value > pt100_tab[i])
        {
            cBottom = i;
            i = (cTop + cBottom) / 2;
        }
        else
        {
            Tem0 = i * 10 - 600;
            Tem = Tem0;
            goto ret;
        }
    }
    Tem0 = cBottom * 10 - 600;
    LowRValue  =pt100_tab[cBottom];
    HighRValue = pt100_tab[cTop];
    Tem = ( ((this->pt_value - LowRValue)*10) / (HighRValue - LowRValue) ) + Tem0;        // ��������5��Ϊһ���ġ�
  #elif(MD_TEMP_MEASURE_TYPE==MD_PT1000)
    int16_t  limit=sizeof(pt1000_tab)/sizeof(pt1000_tab[0])-1;
    cBottom = 0; 
    cTop    = limit;

    if (this->pt_value <pt1000_tab[0])                // ����ֵС�ڱ�����ֵ�����������ޡ�
    {
     this->sw._bit.isTempNoConnect=1;    
	 return -500;
         
    }
    if (this->pt_value >pt1000_tab[limit])        // ����ֵ���ڱ������ֵ�������������� ��
    {
         this->sw._bit.isTempNoConnect=1;
		return 2910;
    }
	this->sw._bit.isTempNoConnect=0;
   
     for (i=limit/2; (cTop-cBottom)!=1; )        // 2�ַ������
    {

       if (this->pt_value <pt1000_tab[i])
        {
            cTop = i;
            i = (cTop + cBottom) / 2;
        }
        else if (this->pt_value > pt1000_tab[i])
        {
            cBottom = i;
            i = (cTop + cBottom) / 2;
        }
        else
        {
            Tem0 = i * 10 - 500;
            Tem = Tem0;
            goto ret;
        }
    }
    Tem0 = cBottom * 10 - 500;
    LowRValue  =pt1000_tab[cBottom];
    HighRValue = pt1000_tab[cTop];
    Tem = ( ((this->pt_value - LowRValue)*10) / (HighRValue - LowRValue) ) + Tem0; 
  #endif  
ret:
    Tem=Tem*(this->coe.pt_temp/10000.);
    return Tem;


}



static int16_t calc_TC_temp(device_comps_t  *const this)
{
    int16_t i;
    int32_t Tem;
    int16_t cBottom = 0; 
    int16_t cTop    = sizeof(TC_coldend_tab)/sizeof(TC_coldend_tab[0])-1;
    int32_t E_t_0;   //E(t,0)=E(t,t0)+E(t0,0)
    int32_t E_t_t0;   //
    int32_t E_t0_0;//E(t0,0)

    if (this->ntc_temp <=TC_coldend_tab[cBottom])                // ����ֵС�ڱ�����ֵ�����������ޡ�
    {
        cBottom=0;
        cTop=cBottom+1;
    }
    else if (this->ntc_temp >=TC_coldend_tab[cTop])        // ����ֵ���ڱ������ֵ�������������� ��
    { 
        cTop=cTop;
        cBottom=cTop-1;
    }   
    else
   {
        cBottom = (this->ntc_temp+500)/100; 
        cTop=cBottom+1;
   }
    E_t0_0=(int32_t)(TC_tab[cTop]-TC_tab[cBottom])*(this->ntc_temp - TC_coldend_tab[cBottom])/100+TC_tab[cBottom];   
    
    E_t_t0=(int64_t)this->ad1_ad2_average_result*1500000/199987; // this->calc_temp_p_temp_n_average
    if(E_t0_0+E_t_t0>0)
    {
          E_t_0=E_t0_0+E_t_t0;
    }
    else
    {
          
	E_t_0=0; 
    }
   

    cBottom = 0; 
    cTop    = sizeof(TC_tab)/sizeof(TC_tab[0])-1;
    if(E_t_0 <=TC_tab[0])                // ����ֵС�ڱ�����ֵ�����������ޡ�
    {
       cBottom = 0; 
       cTop =1;  
    }
    else if (E_t_0 >=TC_tab[cTop])        // ����ֵ���ڱ������ֵ�������������� ��
    {
       cBottom = 0; 
       cTop =1;  
    }
	else
	{
        for (i=cTop/2; (cTop-cBottom)!=1; )        // 2�ַ������
        {

           if (E_t_0 <TC_tab[i])
            {
                cTop = i;
                i = (cTop + cBottom) / 2;
            }
            else if (E_t_0 > TC_tab[i])
            {
                cBottom = i;
                i = (cTop + cBottom) / 2;
            }
            else
            {
                cBottom = i;
                cTop = cBottom+1;
            }
        }
	}
    
    Tem = ( E_t_0 - (int32_t)TC_tab[cBottom])*100 /(int32_t)(TC_tab[cTop]-TC_tab[cBottom]) + cBottom * 100 -500;
  //  Tem=Tem*this->level_param.coe/10000;
    return Tem;
    
}




/////////////////////////////////////////get press////////////////////////////////////

static int32_t  calc_ad1_ad2_average(device_comps_t *const this)
{
    int16_t   i=0;
    const int16_t count=this->ad1_pos;
    int32_t average=0;
	int32_t difference[8]={0};//=malloc(count*sizeof(float32_t));
	if(!count)
    {
        return 0;
    }
	for(i=0;i<count;i++)
	{
		difference[i]=this-> ad1_convert_result[i]-this-> ad2_convert_result[i];
	}
    /*for(i=0;i<count-1;i++)
    {
        for(j=0;j<count-1-i;j++)
        {
        	if(difference[j]>difference[j+1])
        	{
        		temp_var=difference[j+1];
        		difference[j+1]=difference[j];
        		difference[j]=temp_var;
        	}
        }
    }
    */
    for(i=0;i<count;i++)
	{
		average+=difference[i];
	}	
	average=(average/(count));
	//free (period);
	return average;

}


static  int32_t calc_current_press(device_comps_t *const this)
{
    int32_t delta_v= this->ad1_ad2_average_result;
	int32_t press=0;

    int32_t  LowTValue;
    int32_t  HighTValue;        
    int16_t   i;
    int16_t  Bottom=0;
    int16_t  Top=sizeof(this->press_cal_param.x)/sizeof(this->press_cal_param.x[0])-1; //cal 4 points 0-3
    if (delta_v<=device_comps.press_cal_param.x[Bottom])               
    {
       // Top=Bottom+1;
       // goto insert_calc;//jmp 2 points insert_calc code
        return press=device_comps.press_cal_param.y[Bottom];
    }
    else if (delta_v>device_comps.press_cal_param.x[Top])       
    {
        Bottom=Top-1;
        goto insert_calc;
    }
	i=Top/2;
	while(Top-Bottom>1)
	{
        if (delta_v<device_comps.press_cal_param.x[i])
        {
            Top = i;
            i = (Top + Bottom) / 2;
        }
        else if (delta_v >device_comps.press_cal_param.x[i])
        {
            Bottom = i;
            i = (Top + Bottom) / 2;
        }
        else
        {
            press = device_comps.press_cal_param.y[i];
            goto  ret;
        }
    }
insert_calc:
{
    
    LowTValue  = device_comps.press_cal_param.x[Bottom];
    HighTValue = device_comps.press_cal_param.x[Top];
    press =(int32_t)(
    ((int64_t) (delta_v-LowTValue)*(device_comps.press_cal_param.y[Top]-device_comps.press_cal_param.y[Bottom]))
    /(HighTValue - LowTValue)
    )
    +device_comps.press_cal_param.y[Bottom];
}
ret:
    press-=this->coe.press_clr_value;
	if(press>999)//this->press_cal_param.y[3]*1/100)//>fs*1/100.
	{
		//press;
	}
	else if(press<-999)//this->press_cal_param.y[3]*1/100)
	{
        //press;
    }
	else 
	{
		press=0;
	}
	return press*(this->coe.press/10000.);
}




////////////////////////////////end get press////////////////////////////////////

static  int32_t mic_cal_calc(device_comps_t *const this)
{
    //int32_t pv= this->flow_freq;
	int32_t rslut=0;
//    int32_t x[8];
//    int32_t  LowTValue;
//    int32_t  HighTValue;        
//    int16_t   i,j;
//    int16_t  Bottom;
//    int16_t  Top;
//    if(this->flow_cal_param.t_pos<2)
//    {
//        for(j=0;j<8;j++)
//        {
//            x[j]=this->flow_cal_param.x[0][j];
//        }
//    }
//    else
//    {
//        Bottom=0;
//        if( this->flow_cal_param.t_pos > sizeof(this->flow_cal_param.t)/sizeof(this->flow_cal_param.t[0]))
//        {
//           Top= sizeof(this->flow_cal_param.t)/sizeof(this->flow_cal_param.t[0]) -1;
//        }
//        else
//        {
//            Top=this->flow_cal_param.t_pos-1;
//        }
//        
//        if (this->current_temp<device_comps.flow_cal_param.t[Bottom])               
//        {
//            Top=Bottom+1;
//            //goto insert_calc;//jmp 2 points insert_calc code
//           // return rslut=device_comps.flow_cal_param.y[Bottom];
//        }
//        else if (this->current_temp>device_comps.flow_cal_param.t[Top])       
//        {
//            Bottom=Top-1;
//            //goto insert_calc;
//        }
//        else
//        {
//        	i=Top/2;
//        	while(Top-Bottom>1)
//        	{
//                if (this->current_temp<device_comps.flow_cal_param.t[i])
//                {
//                    Top = i;
//                    i = (Top + Bottom) / 2;
//                }
//                else if (this->current_temp >device_comps.flow_cal_param.t[i])
//                {
//                    Bottom = i;
//                    i = (Top + Bottom) / 2;
//                }
//                else
//                {
//    //                for(j=0;j<8;j++)
//    //                {
//    //                    x[j]=this->flow_cal_param.x[i][j];
//    //                }
//    //                goto ret;
//                    Bottom = i;
//                    Top=i+1;
//                    break;

//                }
//            }
//       }
////insert_calcx:
//        {
//            LowTValue  = device_comps.flow_cal_param.t[Bottom];
//            HighTValue = device_comps.flow_cal_param.t[Top];
//            for(j=0;j<8;j++)
//            {
//                x[j]=(int32_t)(
//                        ((int64_t) (this->current_temp-LowTValue)*(device_comps.flow_cal_param.x[Top][j]-device_comps.flow_cal_param.x[Bottom][j]))
//                        /(HighTValue - LowTValue)
//                        )
//                        +device_comps.flow_cal_param.x[Bottom][j];
//            }
//            
//        }
//   }

//    
//   
//    if(this->flow_cal_param.y_pos<2)
//    {
//        rslut=(int32_t)(int64_t)this->flow_cal_param.y[0]*pv/x[0];
//    }
//    else
//    {
//        Bottom=0;
//        if( this->flow_cal_param.y_pos > sizeof(this->flow_cal_param.y)/sizeof(this->flow_cal_param.y[0]))
//        {
//           Top= sizeof(this->flow_cal_param.y)/sizeof(this->flow_cal_param.y[0]) -1;
//        }
//        else
//        {
//            Top=this->flow_cal_param.y_pos-1;
//        }
//        
//        if (pv<x[Bottom])               
//        {
//            Top=Bottom+1;
//            //goto insert_calc;//jmp 2 points insert_calc code
//           // return rslut=device_comps.flow_cal_param.y[Bottom];
//        }
//        else if (pv>x[Top])       
//        {
//            Bottom=Top-1;
//            //goto insert_calc;
//        }
//        else
//        {
//        	i=Top/2;
//        	while(Top-Bottom>1)
//        	{
//                if (pv<x[i])
//                {
//                    Top = i;
//                    i = (Top + Bottom) / 2;
//                }
//                else if (pv >x[i])
//                {
//                    Bottom = i;
//                    i = (Top + Bottom) / 2;
//                }
//                else
//                {
//    //                for(j=0;j<8;j++)
//    //                {
//    //                    x[j]=this->flow_cal_param.x[i][j];
//    //                }
//    //                goto ret;
//                    Bottom = i;
//                    Top=i+1;
//                    break;

//                }
//            }
//       }
////insert_calcy:
//        {
//            LowTValue  = x[Bottom];
//            HighTValue = x[Top];
//            rslut=(int32_t)(
//                    ((int64_t) (pv-LowTValue)*(device_comps.flow_cal_param.y[Top]-device_comps.flow_cal_param.y[Bottom]))
//                    /(HighTValue - LowTValue)
//                    )
//                    +device_comps.flow_cal_param.y[Bottom];
//        
//            
//        }
//   
//        
//    }
//      
//    
//    rslut = (int32_t)((int64_t)rslut*this->coe.flow/10000);
    
    return rslut;
}
//Algorithm: The actual measured value is less than 4% of full scale and can be cleared. 
//The actual value = current value + cleared value. 
//Because the current value = actual value-clear value

static int16_t clr_press(int16_t cmd)
{
    int32_t actual_value;
    int16_t ret=1;
    if(device_comps.press_cal_param.is_calibrated)
    {
        if(!cmd)
        {
            actual_value=0;
            device_comps.coe.press=10000;
            device_comps.coe.cs=Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs-(uint8_t *)&device_comps.coe);
            device_comps.save_coe(&device_comps.coe,sizeof(device_comps.coe));
            
        }
        else
        {
            actual_value=device_comps.current_press+device_comps.coe.press_clr_value;
        }
        if(actual_value < 4000 && actual_value> -4000)
        {
            device_comps.coe.press_clr_value=actual_value;
            device_comps.coe.cs=Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs-(uint8_t *)&device_comps.coe);
            device_comps.save_coe(&device_comps.coe,sizeof(device_comps.coe));
            ret=0;
        }
    }
    return ret;
}


 void device_comps_output_debug_info(device_comps_t *const this)
{
//	static int16_t line_num=0;
//	int16_t tx_num=0;
//	memset(this->debug_info,0,sizeof(this->debug_info));
//	if(line_num==0)
//	{
		
//		//start output attribute name(title)
//		sprintf(this->debug_info+strlen(this->debug_info),"AD1-AD2\t\t");//
//		sprintf(this->debug_info+strlen(this->debug_info),"ad2_pos\t\t");//
//		sprintf(this->debug_info+strlen(this->debug_info),"AD3\t\t");//signal_ferq_from_timer_ch0 extern event counter 
//		sprintf(this->debug_info+strlen(this->debug_info),"ad3_pos\r\n");//signal_freq
//		//end output attribute name(title)
//	}
//	else
//	{
//		sprintf(this->debug_info+strlen(this->debug_info),"%05d\t\t",(int)this->ad1_ad2_average_result);
//		sprintf(this->debug_info+strlen(this->debug_info),"%05d\t\t",(int)this->ad2_pos);
//		sprintf(this->debug_info+strlen(this->debug_info),"%05d\t\t",(int)this->ad3_average_result);
//		sprintf(this->debug_info+strlen(this->debug_info),"%05d\r\n",(int)this->ad3_pos);
		
//	}
//	line_num++;
//	if(line_num>=10)//Output attribute name(title) every 50 lines
//	{
//		line_num=0; 
//	}
//	tx_num=strlen(this->debug_info);
//	if(tx_num>=sizeof(this->debug_info))
//	{
//		memset(this->debug_info,0,sizeof(this->debug_info));
//		sprintf(this->debug_info,"Write sensor debug output buffer overflow\r\n"); 
//		tx_num=strlen(this->debug_info);
//	}
//	ircComps.write(this->debug_info,tx_num);
	
	
}


int16_t get_gps_info_from_net(char const *loc)
{
    char *endptr;
    int32_t  glat;
    int32_t  glng;
    
    glat=strtol(loc,     &endptr,10)*100000;
    if(glat<0)
    {
       glat=glat+(-strtol(endptr+1,&endptr,10));
    }
    else
    {
        glat=glat+strtol(endptr+1,&endptr,10);
    }
    
    glng=strtol(endptr+1,&endptr,10)*100000;
    if(glng<0)
    {
        glng=glng+(-strtol(endptr+1,&endptr,10));
    }
    else
    {
        glng=glng+strtol(endptr+1,&endptr,10);
    }
    if(glat==0 && glng==0)
    {
        return 1;
    }
    device_comps.gps.loc_times++;
    if(device_comps.gps.loc_times<2)
    {
        return 1;
	}
	device_comps.gps.glat=glat;
	device_comps.gps.glng=glng;
	device_comps.gps.cs=Check_Sum_5A(&device_comps.gps, &device_comps.gps.cs-(uint8_t *)&device_comps.gps);
	device_comps.save_gps_param(&device_comps.gps,sizeof(device_comps.gps));
	device_comps.gps.loc_times=0;
	return 0;
}

static int32_t get_flow_meter_unit_display_value(int32_t cur_std_flow,uint8_t std_flow_dot,uint16_t dis_unit,uint8_t *dis_dot,uint16_t *calc_dis_unit)
{
    int32_t   num;
    float32_t f_flow=f_div(cur_std_flow,pwr(std_flow_dot));//m3/h
    *calc_dis_unit=dis_unit;
    switch(dis_unit)
    {
        case 0:  //m^3/h   (0: m3/h, 1: L/m,  2:Kg/h, 3:L/h,   4:T/h,   5:Kg/m,   6:m3/m,  7: t/m)
               
        case 1://   L/MIN
            if(dis_unit==1)
            {  
                f_flow=f_mul(f_flow,1000/60.f);
                if     (f_flow < 10   )        {num=f_mul(f_flow,1000)   ;      *dis_dot=3;}
                else if(f_flow < 100  )        {num=f_mul(f_flow,100)    ;      *dis_dot=2;}
                else if(f_flow < 1000 )        {num=f_mul(f_flow,10)     ;      *dis_dot=1;}
                else                           {num=f_flow               ;      *dis_dot=0;}
                break;
           }
           
        case 2://kg/h
           if(dis_unit==2)
           {


           }
        case 3://L/h
            if(dis_unit==3)
            {  
                f_flow=f_mul(f_flow,1000);
                if     (f_flow < 10000  )         {num=f_mul(f_flow,100)   ;     *dis_dot=2;}
                else if(f_flow < 100000 )         {num=f_mul(f_flow,10)    ;     *dis_dot=1;}
                else                              {num=f_flow              ;     *dis_dot=0;}
                break;
               
            }
        case 4:  //t/h
           if(dis_unit==4)
           {


           }
        
        case 5:  //kg/min
           if(dis_unit==5)
           {


           }
        case 6:  //m^3/min
            if(dis_unit==6)
            { 
                f_flow=f_div(f_flow,60);
                if     (f_flow < 10   )         {num=f_mul(f_flow,1000)  ;    *dis_dot=3;}
                else if(f_flow < 100  )         {num=f_mul(f_flow,100)   ;    *dis_dot=2;}
                else                            {num=f_mul(f_flow,10)    ;    *dis_dot=1;}
                break;
                
            }
        case 7:  //t/min
           if(dis_unit==7)
           {


           }
        default://m^3/h
                num=cur_std_flow/pwr(2);
                *dis_dot=std_flow_dot-2;
                 *calc_dis_unit=0;
               break;
    }
    return num;
}


static uint8_t device_comps_init(device_comps_t *const this)
{
	/**************START E2PROM TEST********************/
	if(device_comps.sw._bit.e2prom_driver_err)	
	{
		char msg[32]="";// ep2rom low 32bytes use test
		_24cxx_comps.write(MD_MEMORY_TEST_START_ADDR,"this is a e2prom driver test",strlen("this is a e2prom driver test"));
		_24cxx_comps.read (MD_MEMORY_TEST_START_ADDR,msg,strlen("this is a e2prom driver test"));
		if(!strcmp(msg,"this is a e2prom driver test"))
		{
			device_comps.sw._bit.e2prom_driver_err=0;	
		}
		else
		{
			device_comps.sw._bit.e2prom_driver_err=1;
		}
	}

    if(device_comps.sw._bit.adx_driver_err)	
	{
		MD_SET_AVDD_ON;
        adx_comps.current_channel=adx_comps.init_channel;
        if(!adx_comps.Init(adx_comps.current_channel,adx_comps.gain,adx_comps.rate)) 
        {
            device_comps.sw._bit.adx_driver_err=0;
        }
	    MD_SET_AVDD_OFF;
    }

	if(device_comps.sw._bit.lora_module_err)	
	{
	    if(loraComps.sw._bit.init_ok)	
    	{
    	    device_comps.sw._bit.lora_module_err=0;
    	}
	}

    
    
    if(device_comps.sw._bit.rtc_module_err)	
	{
	    if(systemComps.sw._bit.is_xt1_running)	
    	{
    	    device_comps.sw._bit.rtc_module_err=0;
    	}
	}

   if(!(device_comps.sw.All&0x00ff))
	{
		return 0;
	}
	else
	{
		return 1;
	}


}

static void read_all_param(struct _DEVICE_COMPONENTS  *const this)
{
    int16_t k;
    if(!device_comps.sw._bit.e2prom_driver_err)
    {
        if(ertc_comps.read_broken_time(&device_comps.system_time.time) || device_comps.system_time.time.u8Year==0)
        {
            if(!read_system_time(&device_comps.system_time,sizeof(device_comps.system_time)))
            {
                if(device_comps.system_time.cs!=Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs-(uint8_t *)&device_comps.system_time))
                {
                    device_comps.system_time.time.u8Year=0x24;
                    device_comps.system_time.time.u8Month=0x10;
                    device_comps.system_time.time.u8Day=0x24;
                    device_comps.system_time.time.u8Hour=0x17;
                    device_comps.system_time.time.u8Minute=0x20;
                    device_comps.system_time.time.u8Second=0x30;
    	        } 
                Rtc_SetTime(&device_comps.system_time.time);
                ertc_comps.write_broken_time(&device_comps.system_time.time);
    	    }
        }
       
        
        if(!read_time_seg_data_param(&device_comps.TimeSegData,sizeof(device_comps.TimeSegData)))
        {
            if(device_comps.TimeSegData.cs!=Check_Sum_5A(&device_comps.TimeSegData, &device_comps.TimeSegData.cs-(uint8_t *)&device_comps.TimeSegData))
            {
                device_comps.TimeSegData.store_addr=MD_TIME_SEG_DATA_START_ADDR;
                device_comps.TimeSegData.nums=0;
            }
        }
        
        if(!read_temp_comp_param(&device_comps.temp_comp_param,sizeof(device_comps.temp_comp_param)))
        {
            if(device_comps.temp_comp_param.cs!=Check_Sum_5A(&device_comps.temp_comp_param, & device_comps.temp_comp_param.cs-(uint8_t *)&device_comps.temp_comp_param))
            {
                memset(&device_comps.temp_comp_param,0,sizeof(device_comps.temp_comp_param));
            }
        }

        if(!read_press_cal_param(&device_comps.press_cal_param,sizeof(device_comps.press_cal_param)))
        {
            if(device_comps.press_cal_param.cs!=Check_Sum_5A(&device_comps.press_cal_param, & device_comps.press_cal_param.cs-(uint8_t *)&device_comps.press_cal_param))
            {
                memset(&device_comps.press_cal_param,0,sizeof(device_comps.press_cal_param));
                device_comps.press_cal_param.y[0]=0;     device_comps.press_cal_param.y[1]=111111;device_comps.press_cal_param.y[2]=222222;
                device_comps.press_cal_param.y[3]=333333;device_comps.press_cal_param.y[4]=444444;device_comps.press_cal_param.y[5]=555555;
                device_comps.press_cal_param.y[6]=666666;device_comps.press_cal_param.y[7]=777777;
                device_comps.press_cal_param.is_calibrated=0;
                
            }
        }
        
        if(!read_flow_cal_param(&device_comps.flow_cal_param,sizeof(device_comps.flow_cal_param)))
        {
            if(device_comps.flow_cal_param.cs!=Check_Sum_5A(&device_comps.flow_cal_param, & device_comps.flow_cal_param.cs-(uint8_t *)&device_comps.flow_cal_param))
            {
                memset(&device_comps.flow_cal_param,0,sizeof(device_comps.flow_cal_param));
                for(k=0;k<MD_FLOW_MAX_CAL_POS;k++)
                {
                    device_comps.flow_cal_param.freq_divd_value[k]=(k+1)*100;
                    device_comps.flow_cal_param.freq_divd_value_meter_coe[k]=792.f;
                }
                device_comps.flow_cal_param.freq_divd_pos=10;
                device_comps.flow_cal_param.calc_mode=2; 
                device_comps.flow_cal_param.RefMaxQ=99;
                device_comps.flow_cal_param.RefDN=150;
                 device_comps.flow_cal_param.freq_poly_coe[4]=device_comps.flow_cal_param.meter_coe=792.f;
            }
        }
        if(!read_res_cal_param(&device_comps.res_cal_param,sizeof(device_comps.res_cal_param)))
        {
            if(device_comps.res_cal_param.cs!=Check_Sum_5A(&device_comps.res_cal_param, &device_comps.res_cal_param.cs-(uint8_t *)&device_comps.res_cal_param))
            {
               memset(&device_comps.res_cal_param,0,sizeof(device_comps.res_cal_param));
            
            }
        }
       
        if(!read_lbs_param(&device_comps.lbs_param,sizeof(device_comps.lbs_param)))
        {
            if(device_comps.lbs_param.cs!=Check_Sum_5A(&device_comps.lbs_param, &device_comps.lbs_param.cs-(uint8_t *)&device_comps.lbs_param))
            {
                strcpy((char *)&device_comps.lbs_param.token,"35639zB6uQknU05I");
                strcpy((char *)&device_comps.lbs_param.domain,"www.queclocator.com:80");
            }
        }

         if(!read_iot_param(&device_comps.iot_param,sizeof(device_comps.iot_param)))
        {
            if(device_comps.iot_param.cs!=Check_Sum_5A(&device_comps.iot_param, &device_comps.iot_param.cs-(uint8_t *)&device_comps.iot_param))
            {
                strcpy((char *)&device_comps.iot_param.productID, "17088633");
                strcpy((char *)&device_comps.iot_param.tenantID,  "10506828");
                memset(&device_comps.iot_param.token,0,sizeof(device_comps.iot_param.token));
                memset(&device_comps.iot_param.deviceID,0,sizeof(device_comps.iot_param.deviceID));
            }
        }
		
		   if(!read_misc_param(&device_comps.misc_param,sizeof(device_comps.misc_param)))
        {
            if(device_comps.misc_param.cs!=Check_Sum_5A(&device_comps.misc_param, & device_comps.misc_param.cs-(uint8_t *)&device_comps.misc_param))
            {
                memset(&device_comps.misc_param,0,sizeof(device_comps.misc_param));
                device_comps.misc_param.bottom_s=1000;//1.000
                device_comps.misc_param.I_o_low=0;
                device_comps.misc_param.I_o_high=500000;
                device_comps.misc_param.density=100;//1.00
                device_comps.misc_param.sample_interval_value=10;//
            }
        }
        if(!read_flow_meter(&device_comps.flow_meter,sizeof(device_comps.flow_meter)))
        {
            if(device_comps.flow_meter.cs!=Check_Sum_5A(&device_comps.flow_meter, &device_comps.flow_meter.cs-(uint8_t *)&device_comps.flow_meter))
            {
                device_comps.flow_meter.unit=0;
                device_comps.flow_meter.pluse_width=100;
                device_comps.flow_meter.pluse_equ=10;
                device_comps.flow_meter.sensor_low_freq_cutoff=10;//  unit 0.1 hz
                device_comps.flow_meter.avg_freq_filter_timer=4;
            }
             
        }
         if(!read_meter_backup(&device_comps.meter_backup,sizeof(device_comps.meter_backup)))
        {
            if(device_comps.meter_backup.cs!=Check_Sum_5A(&device_comps.meter_backup, &device_comps.meter_backup.cs-(uint8_t *)&device_comps.meter_backup))
            {
                device_comps.meter_backup.total_int=0;
                device_comps.meter_backup.total_dec=0;
                device_comps.meter_backup.total_intN=0;
                device_comps.meter_backup.total_decN=0;
                device_comps.meter_backup.store_index=0;
            }
        }
        if(!read_meter(&device_comps.meter,sizeof(device_comps.meter)))
        {
            if(device_comps.meter.cs!=Check_Sum_5A(&device_comps.meter, &device_comps.meter.cs-(uint8_t *)&device_comps.meter))
            {
                device_comps.meter.total_int=device_comps.meter_backup.total_int;
                device_comps.meter.total_dec=device_comps.meter_backup.total_dec;
                device_comps.meter.total_intN=device_comps.meter_backup.total_intN;
                device_comps.meter.total_decN=device_comps.meter_backup.total_decN;
            }
        }
        if(!read_device_coe(&device_comps.coe,sizeof(device_comps.coe)))
        {
            if(device_comps.coe.cs!=Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs-(uint8_t *)&device_comps.coe))
            {
                device_comps.coe.press=10000;
                device_comps.coe.pt_temp=10000;
                device_comps.coe.out_4_20ma=10000;
                device_comps.coe.flow=10000;
                device_comps.coe._4ma_raw_value=4000;
                device_comps.coe._20ma_raw_value=20000;
                device_comps.coe.press_clr_value=0;
            }
            
        }
        if(!read_alarm_param(&device_comps.alarm_param,sizeof(device_comps.alarm_param)))
        {
            if(device_comps.alarm_param.cs!=Check_Sum_5A(&device_comps.alarm_param, & device_comps.alarm_param.cs-(uint8_t *)&device_comps.alarm_param))
            {
                memset(&device_comps.alarm_param,0,sizeof(device_comps.alarm_param));
            }
        }
        
        if(!read_report_param(&device_comps.report_param,sizeof(device_comps.report_param)))
        {
            if(device_comps.report_param.cs!=Check_Sum_5A(&device_comps.report_param, &device_comps.report_param.cs-(uint8_t *)&device_comps.report_param))
            {
                device_comps.report_param.u8Minute=0;
                device_comps.report_param.u8Hour=0;
                 device_comps.report_param.u16Minute_Interval=1440;
                device_comps.report_param.u8Hour_Interval=25;
                device_comps.report_param.disFactor=60;
                device_comps.report_param.triggerTimes=0;
            }
        }

        if(!read_device_addr(&device_comps.device_addr,sizeof(device_comps.device_addr)))
        {
            if(device_comps.device_addr.cs!=Check_Sum_5A(&device_comps.device_addr, & device_comps.device_addr.cs-(uint8_t *)&device_comps.device_addr))
            {
                const uint8_t default_addr[7]={0x61,0x05,0x01,0x00,0x00,0x00,0x00};
                memcpy(&device_comps.device_addr.addr[0],default_addr,sizeof(device_comps.device_addr.addr));
            }
        }
      
        if(!read_access_param(&device_comps.access_param,sizeof(device_comps.access_param)))
        {
            if(device_comps.access_param.cs!=Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs-(uint8_t *)&device_comps.access_param))
            {
                memset(&device_comps.access_param,0,sizeof(device_comps.access_param));
            }
        }
        if(!read_gps_param(&device_comps.gps,sizeof(device_comps.gps)))
        {
            if(device_comps.gps.cs!=Check_Sum_5A(&device_comps.gps, &device_comps.gps.cs-(uint8_t *)&device_comps.gps))
            {
                memset(&device_comps.gps,0,sizeof(device_comps.gps));
            }
        }
        if(!read_manufacturer_info(&device_comps.manufacturer_info,sizeof(device_comps.manufacturer_info)))
        {
            if(device_comps.manufacturer_info.cs!=Check_Sum_5A(&device_comps.manufacturer_info, &device_comps.manufacturer_info.cs-(uint8_t *)&device_comps.manufacturer_info))
            {
                memset(&device_comps.manufacturer_info,0,sizeof(device_comps.manufacturer_info));
            }
        }
        if(!read_device_info(&device_comps.device_info,sizeof(device_comps.device_info)))
        {
            if(device_comps.device_info.cs!=Check_Sum_5A(&device_comps.device_info, &device_comps.device_info.cs-(uint8_t *)&device_comps.device_info))
            {
                memset(&device_comps.device_info,0,sizeof(device_comps.device_info));
            }
        }
        if(!read_sensor_info(&device_comps.sensor_info,sizeof(device_comps.sensor_info)))
        {
            if(device_comps.sensor_info.cs!=Check_Sum_5A(&device_comps.sensor_info, &device_comps.sensor_info.cs-(uint8_t *)&device_comps.sensor_info))
            {
                memset(&device_comps.sensor_info,0,sizeof(device_comps.sensor_info));
            }
        }
         //device_comps.access_param.domain_name[48]=0;//add '\0'
    }
    else
    {
        device_comps.press_cal_param.is_calibrated=0;
        device_comps.res_cal_param.is_calibrated=0;
   }
    
}

//only use the highlow and low_upper 
static void pressOverloadReport(struct _DEVICE_COMPONENTS  *const this )
{
    int32_t temp=0;
    int32_t alram_num;
    int16_t  alram_dot;
    if(device_comps.alarm_param.unit==0x81)
    {
      //  alram_num=this->current_high;
      //  alram_dot=this->high_cal_param.dot;
    }
    else if(device_comps.alarm_param.unit==0 || device_comps.alarm_param.unit==1)
    {
         alram_num=this->current_press;
         alram_dot=this->press_cal_param.dot;
    }
    else
    {
        return ;
    }
    if((this->press_cal_param.unit&0x0f)==this->alarm_param.unit  || this->alarm_param.unit==0x81 )
    {
//       #if(APP_PROTOCOL_TYPE==APP_PROTOCOL_SHENGHUO)
//        if(this->alarm_param.sw._bit.high_press_alarm_en)
//       #endif
//        {
//            if(this->alarm_param.press_high_lower>0)
//            {
//                temp=formatData4fixDot(alram_num,alram_dot);
//                if(temp>this->alarm_param.press_high_lower)
//                {
//                    this->PHihgOverTimer++;
//                    this->PHihgRealseTimer=0;
//                    if( this->PHihgOverTimer>=40-35)
//                    {
//                        if(!this->sw._bit.isPHighOverTriggered)
//                        {
//                            if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
//                	        {
//                                if(device_comps.batt>30)
//                                {
//                                    protocolComps.triggerIrq._bit.PHighOver=1;
                                    
//                                }
//                	        }
//                	        this->sw._bit.isPHighRealseTriggered=0;
//                            this->sw._bit.isPHighOverTriggered=1;
//                        }
//                        this->PHihgOverTimer=0;
//                    }
//                }
//                else if((temp<this->alarm_param.press_high_lower-this->alarm_param.press_high_lower/10)&&(this->sw._bit.isPHighOverTriggered))//dealt 10%
//                {
//                    this->PHihgOverTimer=0;
//                    this->PHihgRealseTimer++;
//                    if( this->PHihgRealseTimer>=40-35)
//                    {
//                        if(!this->sw._bit.isPHighRealseTriggered)
//                        {
//                            if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
//                	        {
//                                if(device_comps.batt>30)
//                                {
//                                  protocolComps.triggerIrq._bit.PHighRealse=1;
                                   
//                                }
//                	        }
//                	        this->sw._bit.isPHighRealseTriggered=1;
//                            this->sw._bit.isPHighOverTriggered=0;
//                        }
                       
//                        this->PHihgRealseTimer=0;
//                    }
//                }
//            }
//       }
       
//      #if(APP_PROTOCOL_TYPE==APP_PROTOCOL_SHENGHUO)
//       if(this->alarm_param.sw._bit.low_press_alarm_en)
//      #endif
//        {
//           if(this->alarm_param.press_low_upper>0)
//           {
//               temp=formatData4fixDot(alram_num,alram_dot);
//               if(temp<this->alarm_param.press_low_upper)
//               {
//                   this->PLowLessTimer++;
//                   this->PLowRealseTimer=0;
//                   if( this->PLowLessTimer>=40-35)
//                   {
//                       if(!this->sw._bit.isPLowLessTriggered)
//                       {
//                           if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
//                           {
//                               if(device_comps.batt>30)
//                               {
//                                   protocolComps.triggerIrq._bit.PLowLess=1;
                                   
//                               }
//                           }
//                           this->sw._bit.isPLowRealseTriggered=0;
//                           this->sw._bit.isPLowLessTriggered=1;
//                       }
//                       this->PLowLessTimer=0;
//                   }
//               }
//               else if((temp>this->alarm_param.press_low_upper+this->alarm_param.press_low_upper/10)&&(this->sw._bit.isPLowLessTriggered))//dealt 10%
//               {
//                   this->PLowLessTimer=0;
//                   this->PLowRealseTimer++;
//                   if(this->PLowRealseTimer>=40-35)
//                   {
//                       if(!this->sw._bit.isPLowRealseTriggered)
//                       {
//                           if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
//                           {
//                               if(device_comps.batt>30)
//                               {
//                                  protocolComps.triggerIrq._bit.PLowRealse=1;
                                  
//                               }
//                           }
//                            this->sw._bit.isPLowRealseTriggered=1;
//                            this->sw._bit.isPLowLessTriggered=0;
//                       }
//                       this->PLowRealseTimer=0;
//                   }
//               }
//           }
//       }
        
   }

}

static void tempOverloadReport(struct _DEVICE_COMPONENTS  *const this )
{
    int32_t temp=0;
    temp=this->current_temp;
    if(this->alarm_param.sw._bit.high_temp_alarm_en)
    {
        if(temp>this->alarm_param.temp_high)
        {
            this->THihgOverTimer++;
            this->THihgRealseTimer=0;
            if( this->THihgOverTimer>=40-30)
            {
                if(!this->sw._bit.isTHighOverTriggered)
                {
                    if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
        	        {
                        if(device_comps.batt>30)
                        {
                            protocolComps.triggerIrq._bit.THighOver=1;
                            
                        }
        	        }
        	        this->sw._bit.isTHighRealseTriggered=0;
                    this->sw._bit.isTHighOverTriggered=1;
                }
                this->THihgOverTimer=0;
            }
        }
        else if((temp<this->alarm_param.temp_high-this->alarm_param.temp_high/10)&&(this->sw._bit.isTHighOverTriggered))//dealt 10%
        {
            this->THihgOverTimer=0;
            this->THihgRealseTimer++;
            if( this->THihgRealseTimer>=40-30)
            {
                if(!this->sw._bit.isTHighRealseTriggered)
                {
                    if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
        	        {
                        if(device_comps.batt>30)
                        {
                         // protocolComps.triggerIrq._bit.THighRealse=1;
                           
                        }
        	        }
        	        this->sw._bit.isTHighRealseTriggered=1;
                    this->sw._bit.isTHighOverTriggered=0;
                }
               
                this->THihgRealseTimer=0;
            }
       }
   }

   if(this->alarm_param.sw._bit.low_temp_alarm_en)
   {
       if(temp<this->alarm_param.temp_low)
       {
           this->TLowLessTimer++;
           this->TLowRealseTimer=0;
           if( this->TLowLessTimer>=40-30)
           {
               if(!this->sw._bit.isTLowLessTriggered)
               {
                   if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
                   {
                       if(device_comps.batt>30)
                       {
                           protocolComps.triggerIrq._bit.TLowLess=1;
                           
                       }
                   }
                   this->sw._bit.isTLowRealseTriggered=0;
                   this->sw._bit.isTLowLessTriggered=1;
               }
               this->TLowLessTimer=0;
           }
       }
       else if((temp>this->alarm_param.temp_low+this->alarm_param.temp_low/10)&&(this->sw._bit.isTLowLessTriggered))//dealt 10%
       {
           this->TLowLessTimer=0;
           this->TLowRealseTimer++;
           if(this->TLowRealseTimer>=40-30)
           {
               if(!this->sw._bit.isTLowRealseTriggered)
               {
                   if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
                   {
                       if(device_comps.batt>30)
                       {
                          //protocolComps.triggerIrq._bit.TLowRealse=1;
                          
                       }
                   }
                    this->sw._bit.isTLowRealseTriggered=1;
                    this->sw._bit.isTLowLessTriggered=0;
               }
               this->TLowRealseTimer=0;
           }
       }
   }
}

static void net_power_off_call_back(void)
{
    uint8_t temp[7];
    if(!protocolComps.sw._bit.dataPushYet && !protocolComps.sw._bit.dataPushYet1 )
    {
        uint16_t StoreAddr=device_comps.TimeSegData.store_addr;
	    if((StoreAddr < MD_TIME_SEG_DATA_START_ADDR )||(StoreAddr>MD_TIME_SEG_DATA_END_ADDR-16))
    	{
            StoreAddr=MD_TIME_SEG_DATA_START_ADDR;
    	}

      //  _24cxx_comps.write(StoreAddr,&vmComps.channel_0_freq,16);
        StoreAddr+=16;
		if(StoreAddr>=MD_TIME_SEG_DATA_END_ADDR)
		{
		        StoreAddr=MD_TIME_SEG_DATA_START_ADDR;
		}
        
        if(device_comps.TimeSegData.nums<12)
        {
            device_comps.TimeSegData.nums++;
        }
        else
        {
            device_comps.TimeSegData.nums=12;
        }
		device_comps.TimeSegData.store_addr=StoreAddr;
        temp[0]=0x20;
    	temp[1]=device_comps.system_time.time.u8Year;
    	temp[2]=device_comps.system_time.time.u8Month;
    	temp[3]=device_comps.system_time.time.u8Day;
    	temp[4]=device_comps.system_time.time.u8Hour;
        temp[5]=device_comps.system_time.time.u8Minute;
    	temp[6]=device_comps.system_time.time.u8Second;
        
        memcpy(device_comps.TimeSegData.lastSampleTime,&temp,sizeof(device_comps.TimeSegData.lastSampleTime));
		device_comps.TimeSegData.cs=Check_Sum_5A(&device_comps.TimeSegData, &device_comps.TimeSegData.cs-(uint8_t *)&device_comps.TimeSegData);
		device_comps.save_time_seg_data_param(&device_comps.TimeSegData,sizeof(device_comps.TimeSegData));
       
    }
    else
    {
        temp[0]=0x20;
        temp[1]=device_comps.system_time.time.u8Year;
        temp[2]=device_comps.system_time.time.u8Month;
        temp[3]=device_comps.system_time.time.u8Day;
        temp[4]=device_comps.system_time.time.u8Hour;
        temp[5]=device_comps.system_time.time.u8Minute;
        temp[6]=device_comps.system_time.time.u8Second;

        device_comps.TimeSegData.nums=0;
        memcpy(device_comps.TimeSegData.lastSampleTime,&temp,sizeof(device_comps.TimeSegData.lastSampleTime));
        device_comps.TimeSegData.cs=Check_Sum_5A(&device_comps.TimeSegData, &device_comps.TimeSegData.cs-(uint8_t *)&device_comps.TimeSegData);
        device_comps.save_time_seg_data_param(&device_comps.TimeSegData,sizeof(device_comps.TimeSegData));
    }

}

static float32_t calc_s1_freq(struct _DEVICE_COMPONENTS  *const this )
{
    int16_t   i=0;
    const int16_t count=device_comps.flow_meter.avg_freq_filter_timer%10+1;
    int32_t freq=0;
    if(!count)
    {
        return 0;
    }
	for(i=0;i<count;i++)
	{
		freq += this->s1_cnt_value[i];
	}
    return (float32_t)freq/count;	
}

static int32_t calc_s1_current_flow(struct _DEVICE_COMPONENTS  *const this )
{

    int32_t Q=0;
    int i;
    float32_t   meter_coe;
    if(device_comps.flow_cal_param.calc_mode==0 ||device_comps.flow_cal_param.calc_mode==3)
    {   
        int32_t x_freq[MD_FLOW_MAX_CAL_POS];
        float32_t   y_meter_coe[MD_FLOW_MAX_CAL_POS];
        int32_t pv= this->flow_roll_freq_cur;
        int32_t  LowTValue;
        int32_t  HighTValue;        
        int16_t   i;
        int16_t  Bottom;
        int16_t  Top;
        uint8_t  calc_pos=device_comps.flow_cal_param.freq_divd_pos;
        for(i=0;i<calc_pos;i++)
        {
             x_freq[i]=device_comps.flow_cal_param.freq_divd_value[i];
             y_meter_coe[i]=device_comps.flow_cal_param.freq_divd_value_meter_coe[i];
             if(i==MD_FLOW_MAX_CAL_POS-1)
             {
                calc_pos=MD_FLOW_MAX_CAL_POS;
                break;
             }
        }
       
        if(calc_pos<2)
        {
            meter_coe=device_comps.flow_cal_param.freq_divd_value_meter_coe[0];
        }
        else
        {   
            if(device_comps.flow_cal_param.calc_mode==0)
            {
                for(i=0;i<calc_pos;i++)
                {
                     if(pv<=x_freq[i])
                     {
                        meter_coe=y_meter_coe[i];
                        break;
                     }
                     else
                     {
                        if(i==calc_pos-1)
                        {
                            meter_coe=y_meter_coe[i];
                            break;
                        }
                    }
                }
            }
            else
            {
                Bottom=0;
                Top=calc_pos-1;
                if (pv<x_freq[Bottom])               
                {
                    Top=Bottom+1;
                    //goto insert_calc;//jmp 2 points insert_calc code
                   // return rslut=device_comps.flow_cal_param.y[Bottom];
                }
                else if (pv>x_freq[Top])       
                {
                    Bottom=Top-1;
                    //goto insert_calc;
                }
                else
                {
                	i=Top/2;
                	while(Top-Bottom>1)
                	{
                        if (pv<x_freq[i])
                        {
                            Top = i;
                            i = (Top + Bottom) / 2;
                        }
                        else if (pv >x_freq[i])
                        {
                            Bottom = i;
                            i = (Top + Bottom) / 2;
                        }
                        else
                        {
            //                for(j=0;j<8;j++)
            //                {
            //                    x[j]=this->flow_cal_param.x[i][j];
            //                }
            //                goto ret;
                            Bottom = i;
                            Top=i+1;
                            break;

                        }
                    }
               }
        //insert_calcy:
                {
                    LowTValue  = x_freq[Bottom];
                    HighTValue = x_freq[Top];
                    meter_coe=(float32_t)(
                            ((pv-LowTValue)*(y_meter_coe[Top]-y_meter_coe[Bottom]))
                            /(HighTValue - LowTValue)
                            )
                            +y_meter_coe[Bottom];
                }
            }
        }
    }
    else if(device_comps.flow_cal_param.calc_mode==1)
    {
        float32_t f= f_div(this->flow_roll_freq_cur,10.f);
        float32_t f_2=f_mul(f,f);
        float32_t f_3=f_mul(f,f_2);
        float32_t f_4=f_mul(f,f_3);
        meter_coe=this->flow_cal_param.freq_poly_coe[0]*f_4+this->flow_cal_param.freq_poly_coe[1]*f_3
                 +this->flow_cal_param.freq_poly_coe[2]*f_2+this->flow_cal_param.freq_poly_coe[3]*f+this->flow_cal_param.freq_poly_coe[4];
        
    }
    else
    {
        meter_coe=this->flow_cal_param.meter_coe;
    }
    this->flow_run_meter_coe=meter_coe;
    if(this->flow_run_meter_coe>0)
    {
        float32_t  flow=f_div(this->flow_roll_freq_cur,this->flow_run_meter_coe*10);//L/s
        flow= f_mul(flow,3.6)* this->coe.flow/10000;                      //m^3/h
        if(this->flow_cal_param.RefMaxQ<100)
        {
            Q=(int32_t)f_mul(flow, 100000);
            device_comps.flow_cal_param.dot=5;
            device_comps.flow_cal_param.unit=0;
        }
        else if(this->flow_cal_param.RefMaxQ<1000)
        {
            Q=(int32_t)f_mul(flow, 10000);
            device_comps.flow_cal_param.dot=4;
            device_comps.flow_cal_param.unit=0;
        }
        else if(this->flow_cal_param.RefMaxQ<10000)
        {
            Q=(int32_t)f_mul(flow, 1000);
            device_comps.flow_cal_param.dot=3;
            device_comps.flow_cal_param.unit=0;
        }
        else
        {
            Q=(int32_t)f_mul(flow, 100);
            device_comps.flow_cal_param.dot=2;
            device_comps.flow_cal_param.unit=0;
        }
    }
	if(Q<0)
	{
		Q=0;
	}
    
    return Q;  //m3/h
}

static int32_t calc_s1_current_flowN(struct _DEVICE_COMPONENTS  *const this )
{
    int32_t QN;
    int32_t tKelvin;
    float32_t  pKpa;
    if(!this->press_cal_param.is_calibrated || !this->res_cal_param.is_calibrated || !this->current_flow)
    {
        return QN=this->current_flow;
    }
    tKelvin=this->current_temp*10+27315;
    pKpa=f_div(this->current_press,pwr(this->press_cal_param.dot));
    if(this->press_cal_param.unit==0)
    {
        pKpa=f_div(this->current_press*1000,pwr(this->press_cal_param.dot));
    }
    #define  MD_TB_PB  (29315/101.325)
    QN =MD_TB_PB*pKpa*f_div(this->current_flow,pwr(this->flow_cal_param.dot))/tKelvin *pwr(this->flow_cal_param.dot); //m3/h
    return QN;  //m3/h
}

static int32_t calc_s1_current_flowM(struct _DEVICE_COMPONENTS  *const this )
{
    int32_t QM =f_div(this->misc_param.density,100)*f_div(this->current_flow,pwr(this->flow_cal_param.dot))*pwr(this->flow_cal_param.dot);
    return  QM;  //kg/h
}

static int16_t calc_current_v(struct _DEVICE_COMPONENTS  *const this)
{
      float32_t MD_PIPE_S =(0.0000000314*this->flow_cal_param.RefDN*this->flow_cal_param.RefDN/4); //m^2
	  float32_t v=f_div(f_div(this->current_flow,pwr(this->flow_cal_param.dot)),MD_PIPE_S);//m/h
      return (int16_t)(f_div(v,36));//0.01m/s
}

static void start_out_pluse(int16_t pluse_width)
{
    stc_gpio_cfg_t         GpioInitStruct;
    stc_lptim_cfg_t    stcLptCfg;
    
    DDL_ZERO_STRUCT(GpioInitStruct);
//使能GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    //PB06设置为LP Timer TOG
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
   Gpio_Init(MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PORT, MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PIN, &GpioInitStruct);
   Gpio_SetAfMode(MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PORT,MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PIN,MD_METER_PULSE_M0P_LPTIM0_TOG_OUT_PIN_AFN);


      
    DDL_ZERO_STRUCT(stcLptCfg);

   
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpTim0, TRUE);
    
    M0P_LPTIMER0->CR_f.TOG_EN   = LptimTogEnLow;
    Lptim_Cmd(M0P_LPTIMER0,TRUE);
    Lptim_Cmd(M0P_LPTIMER0,FALSE);

    stcLptCfg.enGate   = LptimGateLow;
    stcLptCfg.enGatep  = LptimGatePLow;
    stcLptCfg.enTcksel = LptimXtl;
    stcLptCfg.enTogen  = LptimTogEnHigh;    // 翻转输出使能
    stcLptCfg.enCt     = LptimTimerFun;     //计数器功能
    stcLptCfg.enMd     = LptimMode2;        //工作模式为模式2：自动重装载16位计数器/定时器
    stcLptCfg.u16Arr   =  (uint16_t)(-pluse_width*(int32_t)32768/10000);             //预装载寄存器值
    Lptim_Init(M0P_LPTIMER0, &stcLptCfg);
    
    Lptim_ClrItStatus(M0P_LPTIMER0);        //清除中断标志位
    Lptim_ConfIt(M0P_LPTIMER0, TRUE);       //允许LPTIMER中断    
    EnableNvic(LPTIM_0_1_IRQn, IrqLevel3, TRUE); 

    device_comps.lptimer_interrput_cnt=0;
    Lptim_Cmd(M0P_LPTIMER0, TRUE);
}

void LpTim0_IRQHandler(void)
{
    if (TRUE == Lptim_GetItStatus(M0P_LPTIMER0))
    {
       Lptim_ClrItStatus(M0P_LPTIMER0);//清除LPTIMER0的中断标志位      
       device_comps.lptimer_interrput_cnt++;
       if(device_comps.lptimer_interrput_cnt>=2)
       {
          device_comps.lptimer_interrput_cnt=0;
          if(device_comps.outed_pluse_cnt>0)
          {
            device_comps.outed_pluse_cnt--;
            if(device_comps.outed_pluse_cnt==0)
            {
                
                  Lptim_Cmd(M0P_LPTIMER0, FALSE);
                  M0P_LPTIMER0->CR_f.TOG_EN   = LptimTogEnLow;
                  device_comps.sw._bit.isOutGoingPluse=0;
            }
          }
          else
          {
              Lptim_Cmd(M0P_LPTIMER0, FALSE);
              M0P_LPTIMER0->CR_f.TOG_EN   = LptimTogEnLow;
              device_comps.sw._bit.isOutGoingPluse=0;
          }
       }
    }
}

static void calc_total_flow(struct _DEVICE_COMPONENTS  *const this )
{
    static int16_t timer=0;
	
    float32_t total_L;
    int32_t    total_int=this->meter.total_int;
    float32_t   total_dec=this->meter.total_dec;
    if(this->flow_roll_freq_cur==0 || this->flow_run_meter_coe<0.1)
    {
        return ;
    }
    total_L=f_div(this->flow_roll_freq_cur,this->flow_run_meter_coe*10);//L/S
    total_dec   +=total_L;
    
    if(1)
    {  
        uint32_t pluse_cnt;
        while(total_dec>=1000)
        {
            total_int=total_int+(int32_t)total_dec/1000;
            total_dec=total_dec-(int32_t)total_dec/1000*1000;
            if(total_int>999999999)
            {
                total_int=total_int-1000000000;
            }
        }
        device_comps.meter.total_int=total_int;
        device_comps.meter.total_dec=total_dec;
        device_comps.meter.cs=Check_Sum_5A(&device_comps.meter, & device_comps.meter.cs-(uint8_t *)&device_comps.meter);
	    save_meter(&device_comps.meter,sizeof(device_comps.meter));
//        read_meter(&meter,sizeof(meter));
//	    if(memcmp(&meter,&device_comps.meter,sizeof(meter)))
//        {
//            if(device_comps.meter_backup.store_index<7)
//            {
//               device_comps.meter_backup.store_index++;
//               device_comps.save_meter(&device_comps.meter,sizeof(device_comps.meter));
               
//               device_comps.meter_backup.total_int=total_int;
//               device_comps.meter_backup.total_dec=total_dec;
//               device_comps.meter_backup.cs=Check_Sum_5A(&device_comps.meter_backup, & device_comps.meter_backup.cs-(uint8_t *)&device_comps.meter_backup);
//               device_comps.save_meter_backup(&device_comps.meter_backup,sizeof(device_comps.meter_backup));

//            }
//	   }
     #if(MD_MEASURE_TYPE==MD_LIQUID)
        device_comps.this_pluse_L+=total_L;
        pluse_cnt=device_comps.this_pluse_L*1000/device_comps.flow_meter.pluse_equ;
        if(pluse_cnt>0)
        {
           device_comps.this_pluse_L = device_comps.this_pluse_L-pluse_cnt*(device_comps.flow_meter.pluse_equ/1000.f);
           __disable_irq();
            device_comps.outed_pluse_cnt+=pluse_cnt;
            if(!device_comps.sw._bit.isOutGoingPluse)
            {
                device_comps.sw._bit.isOutGoingPluse=1;
                start_out_pluse(this->flow_meter.pluse_width);//10.0ms
            }
            __enable_irq();
        }
      #endif
        if(++timer>5)
        {
            timer=0;
            device_comps.meter_backup.total_int=total_int;
            device_comps.meter_backup.total_dec = total_dec;
            device_comps.meter_backup.cs=Check_Sum_5A(&device_comps.meter_backup, & device_comps.meter_backup.cs-(uint8_t *)&device_comps.meter_backup);
            device_comps.save_meter_backup(&device_comps.meter_backup,sizeof(device_comps.meter_backup));
        }
        
    }
    
}

static void calc_total_flowN(struct _DEVICE_COMPONENTS  *const this )
{
    static int16_t timer=0;
	
    float32_t total_L;
    int32_t    total_int=this->meter.total_intN;
    float32_t   total_dec=this->meter.total_decN;
    if(this->current_flowN==0)
    {
        return ;
    }
    total_L=f_div(f_div(this->current_flowN,pwr(this->flow_cal_param.dot)),3.6);//L/S
    total_dec   +=total_L;
    
    if(1)
    {  
        uint32_t pluse_cnt;
        while(total_dec>=1000)
        {
            total_int=total_int+(int32_t)total_dec/1000;
            total_dec=total_dec-(int32_t)total_dec/1000*1000;
            if(total_int>99999999)
            {
                total_int=total_int-100000000;
            }
        }
        device_comps.meter.total_intN=total_int;
        device_comps.meter.total_decN=total_dec;
        device_comps.meter.cs=Check_Sum_5A(&device_comps.meter, & device_comps.meter.cs-(uint8_t *)&device_comps.meter);
	    save_meter(&device_comps.meter,sizeof(device_comps.meter));
//        read_meter(&meter,sizeof(meter));
//	    if(memcmp(&meter,&device_comps.meter,sizeof(meter)))
//        {
//            if(device_comps.meter_backup.store_index<7)
//            {
//               device_comps.meter_backup.store_index++;
//               device_comps.save_meter(&device_comps.meter,sizeof(device_comps.meter));
               
//               device_comps.meter_backup.total_int=total_int;
//               device_comps.meter_backup.total_dec=total_dec;
//               device_comps.meter_backup.cs=Check_Sum_5A(&device_comps.meter_backup, & device_comps.meter_backup.cs-(uint8_t *)&device_comps.meter_backup);
//               device_comps.save_meter_backup(&device_comps.meter_backup,sizeof(device_comps.meter_backup));

//            }
//	   }
  #if(MD_MEASURE_TYPE==MD_GASES)
        device_comps.this_pluse_L+=total_L;
        pluse_cnt=(device_comps.this_pluse_L)/device_comps.flow_meter.pluse_equ*1000;
        if(pluse_cnt>0)
        {
           device_comps.this_pluse_L = device_comps.this_pluse_L-pluse_cnt*(device_comps.flow_meter.pluse_equ/1000.f);
           __disable_irq();
            device_comps.outed_pluse_cnt+=pluse_cnt;
            if(!device_comps.sw._bit.isOutGoingPluse)
            {
                device_comps.sw._bit.isOutGoingPluse=1;
                start_out_pluse(this->flow_meter.pluse_width);//10.0ms
            }
            __enable_irq();
        }
#endif
        if(++timer>5)
        {
            timer=0;
            device_comps.meter_backup.total_intN=total_int;
            device_comps.meter_backup.total_decN = total_dec;
            device_comps.meter_backup.cs=Check_Sum_5A(&device_comps.meter_backup, & device_comps.meter_backup.cs-(uint8_t *)&device_comps.meter_backup);
            device_comps.save_meter_backup(&device_comps.meter_backup,sizeof(device_comps.meter_backup));
        }
        
    }
    
}

static void  APP_disable_freq_output(void)
{
    if(device_comps.sw._bit.isFreqOuting==0)
    {
        
    }
    else
    {   
        // stc_gpio_cfg_t         stcTIM0APort;
        // stc_bt_mode0_cfg_t     stcBtBaseCfg;
        // DDL_ZERO_STRUCT(stcBtBaseCfg);
        // DDL_ZERO_STRUCT(stcTIM0APort);
        // Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
        // stcTIM0APort.enDir  = GpioDirOut;
        // Gpio_Init(MD_METER_FREQ_M0P_TIM0_CHA_OUT_PORT, MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN, &stcTIM0APort);
        // Gpio_SetAfMode(MD_METER_FREQ_M0P_TIM0_CHA_OUT_PORT,MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN,MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN_AFN);

         Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能
        // stcBtBaseCfg.enWorkMode = BtWorkMode0;
        // stcBtBaseCfg.enCT       = BtTimer;
        // stcBtBaseCfg.enPRS      = BtPCLKDiv1;
        // stcBtBaseCfg.enCntMode  = Bt16bitArrMode;
        // stcBtBaseCfg.bEnTog     = FALSE;
        // stcBtBaseCfg.bEnGate    = FALSE;
        // stcBtBaseCfg.enGateP    = BtGatePositive;
        // Bt_Mode0_Init(TIM0, &stcBtBaseCfg);  
        Bt_M0_Enable_Output(TIM0, TRUE);
        Bt_M0_Stop(TIM0);
        device_comps.sw._bit.isFreqOuting=0;
    }
}

static void  APP_update_freq_output(float32_t output_freq)
{
    uint16_t TDR03;
    uint16_t enprs;
    stc_bt_mode0_cfg_t     stcBtBaseCfg;
    DDL_ZERO_STRUCT(stcBtBaseCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能
    if(output_freq<0.1)
    {
        APP_disable_freq_output();
    }
    else 
    {
        if(output_freq<0.5)
        {
            output_freq=0.5;
        }
        if(output_freq<32)
        {
            // Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能 
             enprs= BtPCLKDiv256;
             volatile M0P_TIM0_MODE0_TypeDef *pstcM0PBt = (M0P_TIM0_MODE0_TypeDef *)((uint32_t)M0P_TIM0_MODE0+0x100*TIM0);
             pstcM0PBt->M0CR_f.PRS = enprs;
             TDR03=  -((uint32_t) ( (SystemCoreClock/2)/(256.f*output_freq) ));
             Bt_M0_ARRSet(TIM0, TDR03);
            
        }
        else if(output_freq<160)
        {
             //Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能 
             enprs= BtPCLKDiv4;
             volatile M0P_TIM0_MODE0_TypeDef *pstcM0PBt = (M0P_TIM0_MODE0_TypeDef *)((uint32_t)M0P_TIM0_MODE0+0x100*TIM0);
             pstcM0PBt->M0CR_f.PRS = enprs;
             TDR03=  -((uint32_t) ( (SystemCoreClock/2)/(4.f*output_freq) ));
             Bt_M0_ARRSet(TIM0, TDR03);
         }
        else
        {
             //Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能 
             enprs= BtPCLKDiv1;
             volatile M0P_TIM0_MODE0_TypeDef *pstcM0PBt = (M0P_TIM0_MODE0_TypeDef *)((uint32_t)M0P_TIM0_MODE0+0x100*TIM0);
             pstcM0PBt->M0CR_f.PRS = enprs;
             TDR03=  -((uint32_t) ( (SystemCoreClock/2)/(1.f*output_freq) ));
             Bt_M0_ARRSet(TIM0, TDR03);
        }
        if(device_comps.sw._bit.isFreqOuting==0)
        {
            stc_gpio_cfg_t         stcTIM0APort;
            DDL_ZERO_STRUCT(stcTIM0APort);
            Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
            stcTIM0APort.enDir  = GpioDirOut;
            Gpio_Init(MD_METER_FREQ_M0P_TIM0_CHA_OUT_PORT, MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN, &stcTIM0APort);
            Gpio_SetAfMode(MD_METER_FREQ_M0P_TIM0_CHA_OUT_PORT,MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN,MD_METER_FREQ_M0P_TIM0_CHA_OUT_PIN_AFN);

            stc_bt_mode0_cfg_t     stcBtBaseCfg;
            DDL_ZERO_STRUCT(stcBtBaseCfg);
            Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能
            stcBtBaseCfg.enWorkMode = BtWorkMode0;
            stcBtBaseCfg.enCT       = BtTimer;
            stcBtBaseCfg.enPRS      = enprs;
            stcBtBaseCfg.enCntMode  = Bt16bitArrMode;
            stcBtBaseCfg.bEnTog     = TRUE;
            stcBtBaseCfg.bEnGate    = FALSE;
            stcBtBaseCfg.enGateP    = BtGatePositive;
            Bt_Mode0_Init(TIM0, &stcBtBaseCfg);

            Bt_M0_ARRSet(TIM0, TDR03);
            Bt_M0_Cnt16Set(TIM0,TDR03);
            Bt_M0_Enable_Output(TIM0, TRUE);
            Bt_M0_Run(TIM0);
            device_comps.sw._bit.isFreqOuting=1;
        }
        
    }

}
static void calc_freq_output(device_comps_t *this)
{
   if(device_comps.sw._bit.isExtPowerConnected)
   {
       float32_t freq;
       if(device_comps.flow_cal_param.freq_out_mode==0)
       {
            freq=this->flow_roll_freq_comped_cur_ft;
            SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_METER_FREQ_OUT_CTL_PORT)  ,  MD_METER_FREQ_OUT_CTL_PIN, TRUE);
       }
       else
       {
           freq=this->flow_roll_freq_cur_ft;
           SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_METER_FREQ_OUT_CTL_PORT)  ,  MD_METER_FREQ_OUT_CTL_PIN, FALSE);
       }
      // if(fabs(freq-this->flow_freq_output_prevalue_ft)>0.001)
       {
      //     this->flow_freq_output_prevalue_ft=freq;
           APP_update_freq_output(freq);
       }
    }
    else
    {
       APP_disable_freq_output();
       SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_METER_FREQ_OUT_CTL_PORT)  ,  MD_METER_FREQ_OUT_CTL_PIN, TRUE);
       this->flow_freq_output_prevalue_ft=0;
    }
    // boolean_t current = Gpio_ReadOutputIO(MD_METER_FREQ_OUT_CTL_PORT, MD_METER_FREQ_OUT_CTL_PIN);
    // Gpio_WriteOutputIO(MD_METER_FREQ_OUT_CTL_PORT, MD_METER_FREQ_OUT_CTL_PIN, !current);
}

static void calc_4_20ma_output( int32_t pv,int32_t lower,int32_t upper)
{
	/****START calc 4-20ma*******
	I   4000        20000	      1uA  Positive output 
	I   20000       4000          1uA  Reverse output
	     
****************************/
//    int16_t   pv_dot=device_comps.flow_cal_param.dot;
//    float32_t flow=m_div(pv,pv_dot);
//    float32_t lower_range;
//    float32_t upper_range;
//    if(device_comps.flow_meter.unit==1)  
//    {
//        lower_range=f_mul(f_div(lower,1000),60);  // m3/u8Minute -> m3/h
//        upper_range=f_mul(f_div(upper,1000),60);  // m3/u8Minute -> m3/h
//    }
//    else if(device_comps.flow_meter.unit==2)  
//    {
//        lower_range=f_div(f_div(lower,1000),1000);  //L/h -> m3/h
//        upper_range=f_div(f_div(upper,1000),1000);  //L/h -> m3/h
//    }
//    else if(device_comps.flow_meter.unit==3)  
//    {
//        lower_range=f_mul(f_div(lower,1000),0.06);  //L/u8Minute -> m3/h
//        upper_range=f_mul(f_div(upper,1000),0.06);  //L/u8Minute -> m3/h
//    }
//    else 
//    {
//        lower_range=f_div(lower,1000);
//        upper_range=f_div(upper,1000);  
//    }

    int32_t flow=pv;
    int32_t lower_range=lower;
    int32_t upper_range=upper;
	if(1)
	{
		
		uint32_t code=0;//code=(I-4)/16*65536
	    int32_t output_i=0;
        int16_t i;
		uint32_t  data=0;
        uint8_t   dat[4];
        if(upper_range==lower_range)
        {
             output_i=4000;
        }
		else if(!device_comps.misc_param.I_o_dir)
		{
		    output_i=(int64_t)(20000-4000)*(flow-lower_range)/(upper_range-lower_range)+4000;
		}
		else
		{
           output_i=(int64_t)(4000-20000)*(flow-lower_range)/(upper_range-lower_range)+20000;
		}
		output_i=output_i * device_comps.coe.out_4_20ma/10000;
        if(output_i<4000)
        {
            output_i=4000;
        }
        else if(output_i>20000)
        {
            output_i=20000;
        }
        device_comps.current_i=output_i;
        //if(!device_comps.sw._bit.isExtPowerConnected)
        //{
        //   return;
        //}
      //  if(device_comps.current_i!=device_comps.current_i_n_1)
        if(1)
        {
            int32_t _4ma_raw_value=device_comps.coe._4ma_raw_value;
            int32_t _20ma_raw_value=device_comps.coe._20ma_raw_value;
            device_comps.current_i_n_1=device_comps.current_i;
            if (_4ma_raw_value<3500)
            {
                device_comps.coe._4ma_raw_value=3500;
            }
            if(_20ma_raw_value>24000)
            {
                device_comps.coe._20ma_raw_value=24000;
            }
            
          #if(MD_MEASURE_TYPE!=MD_GASES) 
            code =(int64_t)(0x14000-0x4000)* (output_i - _4ma_raw_value) / (_20ma_raw_value - _4ma_raw_value)+0x4000;
           // code=(output_i-4000)*4096/1000+0x4000;
    		if(code<0x3800)
    		{
    		    code=0x3800;
    		}
            else if(code>0x18000)
    		{
    			code=0x18000;
    		}
    		data=code;
            if( (hum_comps.current_mode == EM_PARAM_MODIFY_MODE && device_comps.param_type==EM_PARAM_FACTORY  && mode_comps[hum_comps.current_mode].dis_option==7) 
              || ( modbusComps.cmd_out_raw_4ma_20ma_timer && !device_comps.sw._bit.cmd_out_raw__4ma_20ma) )
            {
               data = 0x4000;
            }   
            else if ((hum_comps.current_mode == EM_PARAM_MODIFY_MODE && device_comps.param_type==EM_PARAM_FACTORY && mode_comps[hum_comps.current_mode].dis_option==8) 
            || (modbusComps.cmd_out_raw_4ma_20ma_timer>0 && device_comps.sw._bit.cmd_out_raw__4ma_20ma) )
            {
                   data = 0x14000;
            }
            
            SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_CLK_PORT),  MD_4_20MA_SCI_CLK_PIN, FALSE);
            SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_LATCH_PORT),  MD_4_20MA_SCI_LATCH_PIN, FALSE);
            delay10us(2);__NOP();
            for(i = 0; i < 24; i++) //clk38-clk45
            {
                if(data & 0x800000) //MSB -> LSB
                {
                    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_DATA_PORT),  MD_4_20MA_SCI_DATA_PIN, TRUE);
                }
                else
                {
                    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_DATA_PORT),  MD_4_20MA_SCI_DATA_PIN, FALSE);
                }
                data <<= 1;
                __NOP(); delay10us(3);
                SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_CLK_PORT),  MD_4_20MA_SCI_CLK_PIN, TRUE);
                __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();delay10us(3);
                SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_CLK_PORT),  MD_4_20MA_SCI_CLK_PIN, FALSE);
                if(i==23)
                {
                    delay10us(3);__NOP();
                    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_LATCH_PORT),  MD_4_20MA_SCI_LATCH_PIN, TRUE);
                    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_DATA_PORT),  MD_4_20MA_SCI_DATA_PIN, FALSE);
                    delay10us(3);__NOP();
                    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_4_20MA_SCI_LATCH_PORT),  MD_4_20MA_SCI_LATCH_PIN, FALSE);
                    
                }
            }
            
          #else
          if(device_comps.sw._bit.is_4_20ma_Connected)
          {
             code =(int64_t)(3440-295)* (output_i - _4ma_raw_value) / (_20ma_raw_value - _4ma_raw_value)+295;
           // code=(int64_t)(output_i-2500)*24*4095/500000;
            if(code<0)
    		{
    			code=0;
    		}
    		else if(code>4095)
    		{
    		    code=4095;
    		}
    		data=code;

            if( (hum_comps.current_mode == EM_PARAM_MODIFY_MODE && device_comps.param_type==EM_PARAM_FACTORY  && mode_comps[hum_comps.current_mode].dis_option==7) 
              || ( modbusComps.cmd_out_raw_4ma_20ma_timer && !device_comps.sw._bit.cmd_out_raw__4ma_20ma) )
            {
               data = 295;
            }   
            else if ((hum_comps.current_mode == EM_PARAM_MODIFY_MODE && device_comps.param_type==EM_PARAM_FACTORY && mode_comps[hum_comps.current_mode].dis_option==8) 
            || (modbusComps.cmd_out_raw_4ma_20ma_timer>0 && device_comps.sw._bit.cmd_out_raw__4ma_20ma) )
            {
                   data =3440;
            }
            dat[0]=data>>8;
            dat[1]=data;
            sw_i2c_comps.write(0x60,dat,2,0);
          }
		 #endif
       }
    }
}

static uint32_t read_temp_adc_value(uint32_t channel)
{
    stc_adc_cfg_t              stcAdcCfg;
    uint32_t adc;
    MD_SET_MCU_VREF_CTL_ON;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Gpio_SetAnalogMode(MD_NTC_AIN25_PORT, MD_NTC_AIN25_PIN);       
    
    DDL_ZERO_STRUCT(stcAdcCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE); 
    Bgr_BgrEnable();        ///< 开启BGR

    ///< ADC 初始化配置
    stcAdcCfg.enAdcMode         = AdcSglMode;              
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv2;           
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle12Clk;    
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelExtern1;  
    stcAdcCfg.enAdcOpBuf        = AdcMskBufEnable;         
    stcAdcCfg.enInRef           = AdcMskInRefDisable;      
    stcAdcCfg.enAdcAlign        = AdcAlignRight;           
    Adc_Init(&stcAdcCfg);

    Adc_CfgSglChannel(AdcExInputCH25);
    Adc_EnableIrq();
    EnableNvic(ADC_DAC_IRQn, IrqLevel3, TRUE);
    delay10us(3);
    Adc_SGL_Start();

    device_comps.sw._bit.adc_busy=1;
    while(device_comps.sw._bit.adc_busy) ;
    adc = Adc_GetSglResult();


    Adc_DisableIrq();
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelExtern1;
    stcAdcCfg.enInRef           = AdcMskInRefDisable;        ///<内部参考电压关闭
    Adc_Init(&stcAdcCfg);
    Adc_Disable();
    Bgr_BgrDisable();
    MD_SET_MCU_VREF_CTL_OFF;
    return adc;
}



static void App_PcntInit(void)
{
    stc_pcnt_initstruct_t PcntInitStruct;
    stc_gpio_cfg_t GpioInitStruct;
    
    DDL_ZERO_STRUCT(GpioInitStruct);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirIn;
    Gpio_Init(MD_METRE_M0P_PCNT_S0_IN_PORT,MD_METRE_M0P_PCNT_S0_IN_PIN,&GpioInitStruct);
    Gpio_SetAfMode(MD_METRE_M0P_PCNT_S0_IN_PORT,MD_METRE_M0P_PCNT_S0_IN_PIN,MD_METRE_M0P_PCNT_S0_IN_PIN_AFN);              
    
    DDL_ZERO_STRUCT(PcntInitStruct);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPcnt, TRUE);
    PcntInitStruct.Pcnt_S0Sel = PcntS0PNoinvert;  //S0输入极性不取反
    PcntInitStruct.Pcnt_Clk   = PcntCLKRcl;      //采样时钟
    PcntInitStruct.Pcnt_Mode  = PcntSingleMode; //单通道脉冲计数模式
    PcntInitStruct.Pcnt_FltEn = FALSE;          //滤波使能
    PcntInitStruct.Pcnt_DebTop =5;            //滤波计数器阈值
    PcntInitStruct.Pcnt_ClkDiv = 3;            //滤波时钟分频系数
    PcntInitStruct.Pcnt_TocrEn = FALSE;        //超时控制使能
    PcntInitStruct.Pcnt_Dir    = PcntDirUp;   //
    Pcnt_Init(&PcntInitStruct);
    
    Pcnt_SetB2T(65535);
    delay10us(2);
    //Pcnt_ClrItStatus(PcntUF);
   // Pcnt_ItCfg(PcntUF, TRUE);
   // EnableNvic(PCNT_IRQn, IrqLevel3, TRUE);   

    Pcnt_Cmd(TRUE);
}

void Pcnt_IRQHandler(void)
{
    Pcnt_ClrItStatus(PcntUF);
}




static void device_comps_task_handle(void)//Execution interval is 200 ms
{
	uint16_t sensor_calc_count;
  device_comps_t *this=device_comps.this;
 (device_comps.sw._bit.isExtPowerConnected)?(sensor_calc_count=5):(sensor_calc_count=15);
	if(this->do_init==1)
	{
		if(this->count<40)
		{
			if(!device_comps_init(this))
			{
				this->do_init=0;
			}
			else
			{
				this->count++;
			}
		}
		else
		{
		    #ifdef  MD_IGNORE_ALL_ERR
			    this->do_init=0;
			#else
					__disable_irq();
			    NVIC_SystemReset();
			#endif
		}
		if(this->do_init==0)
		{
           
			this->count=0;
            this->sensor_count=0; 
			device_comps.batt=get_batt();
		   
			read_all_param(this);
            device_comps.this_pluse_L=0;
           
            this->s1_cnt_start();
            this->s1_pre_cnt=Pcnt_GetCnt();
			//TODO
		}
	}

	if(hum_comps.dis_oper_mark._bit.test_ok && this->do_init==0)
	{
    	  hum_comps.dis_oper_mark._bit.test_ok=0;
          hum_comps.enter_default_mode(0);
  	}
	
	//if((this->do_init==0)&&(!loraComps.sw._bit.runing))
	if(this->do_init==0)//&&(!netComps.St._bit.running))//&&(!loraComps.sw._bit.runing))
	{
	    if(this->count==5)//every 1s calc press and temperature
		{
            //int32_t delta_adc=0;
            uint16_t cnt;
			this->count=0;
          //  __disable_irq();
            cnt=Pcnt_GetCnt() ;
            if(cnt -this->s1_pre_cnt > this->flow_meter.sensor_low_freq_cutoff/10)
            {
                this->flow_freq_cur=this->s1_cnt_value[this->s1_pos++]=(cnt -this->s1_pre_cnt);
            }
            else
            {
                this->flow_freq_cur=this->s1_cnt_value[this->s1_pos++]=0;
            }
            this->s1_pre_cnt=cnt;
        //    __enable_irq();
            if(this->s1_pos==device_comps.flow_meter.avg_freq_filter_timer%10+1)
            {
                this->s1_pos=0;
            }
            this->flow_total_cnts+=this->flow_freq_cur;
            this->flow_roll_freq_cur_ft=calc_s1_freq(this);
            this->flow_roll_freq_cur=this->flow_roll_freq_cur_ft*10;//0.1HZ
            this->current_flow= calc_s1_current_flow(this);//M3/H
            this->flow_roll_freq_comped_cur=f_div(this->current_flow,pwr(this->flow_cal_param.dot))*device_comps.flow_cal_param.meter_coe/0.36;//0.1HZ
            if(this->flow_run_meter_coe>0)
            {
                this->flow_roll_freq_comped_cur_ft=f_mul(device_comps.flow_cal_param.meter_coe,f_div(this->flow_roll_freq_cur_ft,this->flow_run_meter_coe));
            }
            else
            {
                this->flow_roll_freq_comped_cur_ft=0;
            }
            this->current_v=calc_current_v(this);
            calc_total_flow(this);
            calc_freq_output(this);
           

         #if(MD_MEASURE_TYPE==MD_LIQUID)
          calc_4_20ma_output(this->current_flow,this->misc_param.I_o_low,this->misc_param.I_o_high);
          
         #elif(MD_MEASURE_TYPE==MD_GASES)

			this->current_flowN= calc_s1_current_flowN(this);//M3/H
           // this->current_flowM= calc_s1_current_flowM(this);//Kg/H
            calc_total_flowN(this);
           calc_4_20ma_output(this->current_flowN,this->misc_param.I_o_low,this->misc_param.I_o_high);
          
		#endif
          
//			if(this->current_press>this->max_press)
//			{
//                this->max_press=this->current_press;
//            }
           // pressOverloadReport(this);
            this->ad3_pos=0;
			this->ad1_pos=0;
			this->ad2_pos=0;
			this->temp_p_pos=0;
			this->temp_n_pos=0;
            hum_comps.dis_oper_mark._bit.refressh_meter_data=1;
            hum_comps.dis_oper_mark._bit.refresh_debug_param=1;
		}
        
        if(device_comps.sensor_count>=sensor_calc_count)
        {
            device_comps.sensor_count=0;
        }
       
	  #if(MD_MEASURE_TYPE==MD_GASES)		
		if(this->sensor_count==0)
        {
            MD_SET_AVDD_ON;
            //smp power on;
            
        }
        if(this->sensor_count==1)
        {
//            adx_comps.current_channel=adx_comps.init_channel;
//            adx_comps.restart(adx_comps.current_channel,adx_comps.gain,adx_comps.rate);
//            adx_comps.sw._bit.running=1;
//            adx_comps.enable_eoc_interrupt();
              epress_comps.start_measure();
              etemp_comps.start_measure();
        }
       
        if(this->sensor_count==4)
        {
           int32_t press;
           int16_t temp;
           if(!epress_comps.read_measure_relt(&press,&device_comps.press_cal_param.unit,
                                              &device_comps.press_cal_param.dot, &device_comps.press_full_scale,&temp))
           {
                epress_comps.read_reasure_relt_error_times=0;
                int32_t press_coed=(int64_t)press*this->coe.press/10000;
                device_comps.current_press=(press_coed+device_comps.current_press_n_1+device_comps.current_press_n_2)/3;
                device_comps.current_press_n_2=device_comps.current_press_n_1;
                device_comps.current_press_n_1=press_coed;
                device_comps.press_cal_param.is_calibrated=1;
           }
           else
           {
                epress_comps.read_reasure_relt_error_times++;
                if(epress_comps.read_reasure_relt_error_times>3)
                {  
                    epress_comps.read_reasure_relt_error_times=0;
                    device_comps.current_press=0;
                    device_comps.current_press_n_1=0;
                    device_comps.current_press_n_2=0;
                    device_comps.press_cal_param.is_calibrated=0;
                }
           }
           
           if(!etemp_comps.read_measure_relt(&temp))
           {
              etemp_comps.read_reasure_relt_error_times=0;
              device_comps.current_temp=(int64_t)temp*this->coe.pt_temp/10000;
              device_comps.res_cal_param.is_calibrated=1;
           }
           else
           {
                etemp_comps.read_reasure_relt_error_times++;
                if(etemp_comps.read_reasure_relt_error_times>3)
                {  
                    etemp_comps.read_reasure_relt_error_times=0;
                    device_comps.current_temp=0;
                    device_comps.res_cal_param.is_calibrated=0;
                }
           }
           MD_SET_AVDD_OFF;
        }
       
	  #else	
        if(this->count==3)
        {
             sht4x_comps.start_measure();
        } 
        if(this->count==4)
        {
            int16_t humi;
            int16_t temp;
            sht4x_comps.read_measure_relt(&temp,&humi);
            this->current_temp=temp*(this->coe.pt_temp/10000.);
        }
	  #endif	
        this->count++;
        this->sensor_count++;
	}
}

static void NOP(void)
{
	__NOP();
}

#define  BK_LN
device_comps_t device_comps=
{
  "",                           //	uint8_t *desc;
  &device_comps,                //	struct _DEVICE_COMPONENTS  *const this;
  1,                                  //	int16_t   do_init;//Whether to initialize,1:init 0,no init

  0,//	uint32_t count;            //Called counter
  0,//uint32_t    sensor_count
  {MD_E2PROM_DRIVER_ERR|MD_ADX_DRIVER_ERR|MD_LORA_MODULE_ERR|MD_RTC_MODULE_ERR}, //	sw_t sw;
                                        
  0,        //    int16_t PHihgOverTimer;
  0,       //    int16_t PHihgRealseTimer;
  0,       //    int16_t PLowLessTimer;
  0,        //    int16_t PLowRealseTimer;
  0,        //    int16_t THihgOverTimer;
  0,        //    int16_t THihgRealseTimer;
  0,        //    int16_t TLowLessTimer;
  0,        //    int16_t TLowRealseTimer;
          	
  0,         //	uint16_t report_interval_timer;
  0,        //	int16_t _0_5s_timr_acc;
  0,        //	int16_t batt_blunt_timer;
  0,                                  //	uint8_t batt;//batt voltage 
  
  {  {0},0,start_buzzer,stop_buzzer},    //	buzzer_t buzzer;
 

   {0}, // uint16_t s1_cnt_value[10];
    0, //     int16_t  s1_pos;
    0,//     uint16_t s1_pre_cnt;
    0,//     int16_t  flow_freq_cur;          //1hz
    0,//     uint32_t flow_total_cnts;          //1
    0,//     float32_t  flow_run_meter_coe; 
    0,// int32_t  flow_roll_freq_cur;   //0.1hz
    0,//     float32_t flow_roll_freq_cur_ft
    0,//     int32_t  flow_roll_freq_comped_cur;   //0.1hz
    0,//     float32_t flow_roll_freq_comped_cur_ft;
    0,//float32_t  flow_freq_output_prevalue_ft;
    get_flow_meter_unit_display_value,
  
  0,     //int32_t current_flow;    // Q m3/h
  0,     //int32_t current_flowN;   //std QN m3/h
  0,     //int32_t current_flowM;   //kg/h
  0,     //int16_t current_v;         //0.01m/s


  0,          //    int16_t current_i;         //0.001mA
  0,          //   int16_t current_i_n_1;
  0,         //    int16_t  outed_pluse_cnt;
  0,         //    float32_t this_pluse_L;
	0,                //    int16_t lptimer_interrput_cnt;
 start_out_pluse,   //    void (*const out_pluse_start)(int16_t pluse_width);   static void start_out_pluse(int16_t pluse_width)
 App_PcntInit,       //    void (* const s1_cnt_start)(void);




                                       
 {0},       //	int32_t  ad1_convert_result[MD_ADC_MAX_POS];
  0,       //	uint16_t  ad1_pos;
 {0},       //	int32_t  ad2_convert_result[MD_ADC_MAX_POS];
  0,       //	uint16_t  ad2_pos;
  0,       //	int32_t          ad1_ad2_average_result;//(ad1-ad2)/ad1_pos
  0,       //	int32_t  current_press;  //Yn
  0,       //    int32_t  max_press;
  0,       //	int32_t  current_press_n_1;//Yn-1
    0,       //	int32_t  current_press_n_2;//Yn-2
  0,       //	int32_t  press_full_scale;
  clr_press, //    int16_t (*clr_press)(int16_t);
 
  {0},     //	int32_t  temp_p_convert_result[MD_ADC_MAX_POS];
   0,      //	uint16_t  temp_p_pos;
  {0},      //	int32_t  temp_n_convert_result[MD_ADC_MAX_POS];
   0,      //	uint16_t  temp_n_pos;
   0,      //	int32_t          temp_p_temp_n_average_result;//(temp_p_convert_result-temp_n_convert_result)/temp_p_pos
   0,       //	int32_t pt_value;
   0,      //    int16_t  pt_temp;
   0,      //	int16_t  pt_temp_n_1;
   0,      //	int16_t  pt_temp_n_2;
   0,      //	int16_t  current_temp;

  {0},     //    int32_t  ad3_convert_result[MD_ADC_MAX_POS];
   0,      //	uint16_t  ad3_pos;
   0,       //	int32_t          ad3_average_result;
   0,      //	int32_t ntc_value;
   0,      //	int16_t  ntc_temp;
                                        
   EM_CAL_PRESS,                      //    cal_type_t cal_type;//0 cal press, 1,cal temp
                                        
   {0},                                 //    press_cal_param_t press_cal_param;
   {0},                                //	press_cal_param_t press_cal_param_bak;
   save_press_cal_param,        //	int16_t (*save_press_cal_param)(void const *,int16_t);
   
   {0},                              //	res_cal_param_t  res_cal_param;
   {0},        //    res_cal_param_t  res_cal_param_bak;
   save_res_cal_param,         //    int16_t (*save_res_cal_param)(void const *,int16_t);
                                   
   {0},                            //    flow_cal_param_t flow_cal_param;
   save_flow_cal_param,                  //    int16_t (*save_flow_cal_param)(void const *,int16_t);
                                    
   {0},                               
   save_flow_meter,                  //    flow_meter_t flow_meter;
                                //    int16_t (*save_flow_meter)(void const *,int16_t);
                                  
   {0},                           //    meter_t meter;    
   save_meter,                           //    int16_t (*save_meter)(void const *,int16_t);
                                    
   {0},                            //    meter_backup_t meter_backup;
   save_meter_backup,              //    int16_t (*save_meter_backup)(void const *,int16_t);
                                    
                                   
   {0},                             //    coe_t coe;
   save_device_coe,                  //    int16_t (*save_coe)(void const *,int16_t);
                                 
 
   {0},                             //    manufacturer_info_t manufacturer_info;
   save_manufacturer_info,          //    int16_t (*save_manufacturer_info)(void const *,int16_t);

                                       
   {0},                             //    device_info_t device_info;
   save_device_info,               //    int16_t (*save_device_info)(void const *,int16_t);

                                       
  {0},                                   //	sensor_info_t sensor_info;
  save_sensor_info,                 //    int16_t (*save_sensor_info)(void const *,int16_t);

   {0},                                 //    temp_comp_param_t temp_comp_param;
   save_temp_comp_param,                //	int16_t (*save_temp_comp_param)(void const *,int16_t);
                                        

 
    {0},                //    alarm_param_t alarm_param;
   save_alarm_param,     //    int16_t (*save_alarm_param)(void const *,int16_t);

   {0},                               //    device_addr_t device_addr;
   save_device_addr,                  //    int16_t (*save_device_addr)(void const *,int16_t);

   {0},                              //	TimeSegData_t TimeSegData;
   read_time_seg_data_param,         //	int16_t (*read_time_seg_data_param)(void * , int16_t);
   save_time_seg_data_param,         //	int16_t (*save_time_seg_data_param)(void const * , int16_t);

   {0},                               //	system_time_t system_time;
   save_system_time,                 //	int16_t (*save_system_time)(void const * , int16_t);

   EM_PARAM_USER,                                //	param_type_t param_type;
   {0},                              //	misc_param_t  misc_param;
   save_misc_param,                  //	int16_t (*save_misc_param)(void const * , int16_t);


   
   {0},              //    report_param_t report_param;
   save_report_param,       //	int16_t (*save_report_param)(void const *,int16_t);
                            	
   {0},                         //    access_param_t access_param;
   save_access_param,            //    int16_t (*save_access_param)(void const *,int16_t);
                            	
   {0},                         //	lbs_param_t   lbs_param;
   save_lbs_param,              //    int16_t (*save_lbs_param)(void const * , int16_t);

   {0},             //   iot_param_t iot_param;
   save_iot_param,             //    int16_t (*save_iot_param)(void const * , int16_t);
                                    	
   {0},                  //	gps_t gps;
   save_gps_param,             //    int16_t (*save_gps_param)(void const *,int16_t);
   get_gps_info_from_net,     //    int16_t (*get_gps_info_from_net)(uint8_t const *);
                             
   "",                        //	uint8_t debug_info[32];
   device_comps_output_debug_info,          //	void  ( *const output_debug_info)(struct _DEVICE_COMPONENTS const  *const);
   device_comps_task_handle,    //    void  ( *const task_handle)(void);//point to device_comps_task_handle
   Rtc_SetTime,                 //en_result_t Rtc_SetTime(stc_rtc_time_t* time)
   net_power_off_call_back    //    void (*const net_power_off_call_back)(void);
};




void CalcReportTime(uint8_t *hur,uint8_t *u8Minute,uint8_t *u8Second)
{
	uint32_t  DisPerTimeTotalSecond=(uint32_t)((device_comps.device_addr.addr[6]>>4)*10+(device_comps.device_addr.addr[6]&0x0f))* device_comps.report_param.disFactor;
	uint32_t  Min=DisPerTimeTotalSecond/60;
	uint32_t  DisPerTimehur=Min/60;
	uint32_t  DisPerTimeMin=Min%60;//u8Minute
	
	
	*u8Second=DisPerTimeTotalSecond%60;
	*u8Minute=(device_comps.report_param.u8Minute+DisPerTimeMin)%60;
	*hur=(device_comps.report_param.u8Hour+DisPerTimehur+(device_comps.report_param.u8Minute+DisPerTimeMin)/60)%24;
	
	*u8Second=(*u8Second/10<<4)+(*u8Second%10);
	*u8Minute=(*u8Minute/10<<4)+(*u8Minute%10);
	*hur=(*hur/10<<4)+(*hur%10);
}

void GSMReturnTimeChk(uint8_t RHur,uint8_t RMin,uint8_t RSec)
{
	//uint16_t cmp=device_comps.report_param.u8Hour_Interval;
	uint16_t cmp=device_comps.report_param.u16Minute_Interval;
	if(!cmp)
	{
        device_comps.report_interval_timer=0;
	    return ;
	}
    if(cmp<5)
	{
		cmp=5;
	}
	//if(device_comps.report_interval_timer>=(uint32_t)cmp*60)
	if(device_comps.report_interval_timer>=cmp)
    {
//		uint8_t reltSec=((RSec>>4)*10+(RSec&0x0f)) +60-((device_comps.system_time.time.u8Second>>4)*10+(device_comps.system_time.time.u8Second&0x0f));
//		uint8_t reltMin=((RMin>>4)*10+(RMin&0x0f)) +60-1-((device_comps.system_time.time.u8Minute>>4)*10+(device_comps.system_time.time.u8Minute&0x0f));
//		uint8_t reltHur=((RHur>>4)*10+(RHur&0x0f)) +24-1-((device_comps.system_time.time.u8Hour>>4)*10+(device_comps.system_time.time.u8Hour&0x0f));
//		uint8_t equSec= reltSec%60;
//		uint8_t equMin=(reltMin+(reltSec/60))%60;
//		uint8_t equHur=(reltHur+((reltMin+(reltSec/60))/60))%24;
//		uint32_t InterTimer=(uint32_t)equHur*3600+   (uint32_t)equMin*60    +(uint32_t)equSec;
//	     if(InterTimer>(uint32_t)5*60) 
        if(1)
		{
			 if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
	        {
                if(device_comps.batt>30)
                {
                    protocolComps.triggerIrq._bit.intervalTime=1;
                }
	        }
	    }
	    device_comps.report_interval_timer=0;
	}
}

void  Timing_interval_report(void)
{
    uint8_t ReportHur;
	uint8_t ReportMin;
	uint8_t ReportSec;
	uint8_t ReportMin_30;
	uint8_t temp[7];
	
    CalcReportTime(&ReportHur,&ReportMin,&ReportSec);
    if(device_comps.system_time.time.u8Second == ReportSec)
	{	
		device_comps.report_interval_timer++;
        if((ReportHur==device_comps.system_time.time.u8Hour)&&(ReportMin==device_comps.system_time.time.u8Minute))
        {
	        if((device_comps.device_addr.addr[4]!=0)&&(device_comps.device_addr.addr[3]!=0))
	        {
                if(device_comps.batt>30)
                {
                    if(device_comps.report_param.u8Hour || device_comps.report_param.u8Minute )
                    {
                        protocolComps.triggerIrq._bit.timeAuto=1;
                       // device_comps.report_interval_timer=0;
                    }
                }
                
	        }
	        if(!device_comps.report_param.u16Minute_Interval && !device_comps.report_param.u8Hour && !device_comps.report_param.u8Minute )
	        {
               if(device_comps.system_time.time.u8Day==1 || device_comps.system_time.time.u8Day==0x11 || device_comps.system_time.time.u8Day==0x21)
               {
                    protocolComps.triggerIrq._bit.batteryBlunt=1;
               }
	        }
            
			if(device_comps.system_time.time.u8Day==1 || device_comps.system_time.time.u8Day==0x11 || device_comps.system_time.time.u8Day==0x21)
            {
                  device_comps.sw._bit.isBatBluntNow=1;
                  device_comps.batt_blunt_timer=10;
            }
	     }
		GSMReturnTimeChk(ReportHur,ReportMin,ReportSec); 

        ReportMin_30=ReportMin+0x30;
        if(ReportMin_30>=0x60)
        {
            ReportMin_30-=0x60;
        }
		if(device_comps.system_time.time.u8Minute == ReportMin  || device_comps.system_time.time.u8Minute == ReportMin_30)
		{
			if(device_comps.press_cal_param.is_calibrated)
			{
//				uint16_t StoreAddr=device_comps.TimeSegData.store_addr;
//				
//				if((StoreAddr>= MD_TIME_SEG_DATA_START_ADDR )&&(StoreAddr<=MD_TIME_SEG_DATA_END_ADDR-4))
//				{
//                    _24cxx_comps.write(StoreAddr,&device_comps.current_press,4);
//					
//					StoreAddr+=4;
//					if(StoreAddr>=MD_TIME_SEG_DATA_END_ADDR)
//					{
//					        StoreAddr=MD_TIME_SEG_DATA_START_ADDR;
//					}
//					device_comps.TimeSegData.store_addr=StoreAddr;
//
//					
//                	temp[0]=0x20;
//                	temp[1]=device_comps.system_time.time.u8Year;
//                	temp[2]=device_comps.system_time.time.u8Month;
//                	temp[3]=device_comps.system_time.time.u8Day;
//                	temp[4]=device_comps.system_time.time.u8Hour;
//                    temp[5]=device_comps.system_time.time.u8Minute;
//                	temp[6]=device_comps.system_time.time.u8Second;
//					
//					memcpy(device_comps.TimeSegData.lastSampleTime,&temp,sizeof(device_comps.TimeSegData.lastSampleTime));
//					device_comps.TimeSegData.cs=Check_Sum_5A(&device_comps.TimeSegData, &device_comps.TimeSegData.cs-(uint8_t *)&device_comps.TimeSegData);
//					device_comps.save_time_seg_data_param(&device_comps.TimeSegData,sizeof(device_comps.TimeSegData));
//				}
//				else
//				{
//					StoreAddr=MD_TIME_SEG_DATA_START_ADDR;
//					device_comps.TimeSegData.store_addr=StoreAddr;
//					device_comps.TimeSegData.cs=Check_Sum_5A(&device_comps.TimeSegData, &device_comps.TimeSegData.cs-(uint8_t *)&device_comps.TimeSegData);
//					device_comps.save_time_seg_data_param(&device_comps.TimeSegData,sizeof(device_comps.TimeSegData));
//				}
			}
		}
	}
}



void _0_5s_task_handle(void)
{
    Wdt_Feed();
    if(hum_comps.current_mode!=EM_SELF_TEST_MODE)
    {
        device_comps._0_5s_timr_acc++;
    	if(device_comps._0_5s_timr_acc>1)
    	{
            if( (ircComps.op_window_time>0)&&(hum_comps.current_mode!=EM_CAL_MODIFY_MODE))
    		{
    			ircComps.op_window_time--;
    			if(ircComps.op_window_time==0)
    			{
    				ircComps.stop();
    			}
    		}
            if(modbusComps.cmd_out_raw_4ma_20ma_timer>0)
            {
               if(modbusComps.cmd_out_raw_4ma_20ma_timer%4==0)
                {
                   device_comps.sw._bit.cmd_out_raw__4ma_20ma=!device_comps.sw._bit.cmd_out_raw__4ma_20ma;
                }
                modbusComps.cmd_out_raw_4ma_20ma_timer--;
                if(modbusComps.cmd_out_raw_4ma_20ma_timer==0)
                {
                    device_comps.sw._bit.cmd_out_raw__4ma_20ma=0;
                }
    }   
          
           if( (modbusComps.op_window_time>0)&&(hum_comps.current_mode!=EM_CAL_MODIFY_MODE))
           {
               modbusComps.op_window_time--;
               if(modbusComps.op_window_time==0)
               {
                  device_comps.sw._bit.com_key_en=0;
               }
           }
          
       	    if(hum_comps.current_mode==EM_DEBUG_MODE || hum_comps.current_mode==EM_NORMAL_MODE)
    	    {
    	        if(mode_comps[hum_comps.current_mode].displayTimer>0)
    	        {
    	            mode_comps[hum_comps.current_mode].displayTimer--;
    	            if(!mode_comps[hum_comps.current_mode].displayTimer)
    	            {
                       // hum_comps.enter_default_mode(0);
    	            }
    	        }
    	    }

	     
				if(hum_comps.back_led_timer>0)
				{
					hum_comps.back_led_timer--;
					if(hum_comps.back_led_timer==0)
					{
						MD_BACK_LED_OFF;
					}
				}
            device_comps.batt=get_batt();
			hum_comps.dis_oper_mark._bit.refresh_special_symbol=1;			 
            if(device_comps.batt_blunt_timer>0)
            {
                 if(!netComps.St._bit.running) 
                 {
                    MD_BAT_BLUNT_CTL_ON;
                    device_comps.batt_blunt_timer--;
                    if(!device_comps.batt_blunt_timer)
                    {
                        MD_BAT_BLUNT_CTL_OFF;
                        device_comps.sw._bit.isBatBluntNow=0;
                    }
                 }
           }
			if(!netComps.St._bit.modifyed_time_just)
			{   
                if(!device_comps.sw._bit.rtc_module_err)
    		    {			
                     Rtc_ReadDateTime(&device_comps.system_time.time);
    	        }
                ertc_comps.read_broken_time(&device_comps.system_time.time);
			}
            else
            {
                netComps.St._bit.modifyed_time_just=0;
            }
            hum_comps.dis_oper_mark._bit.refressh_broken_date=1;
        #if(MD_PRODUCT_NAME==MD_4G)
              if(netComps.op_window_tmr>0)
            {
                netComps.op_window_tmr--;
                if(!netComps.op_window_tmr)
                {
                    netComps.St._bit.windowTimeOut=1;
                }
            }
            if(protocolComps.AckTmr>0)
            {
                protocolComps.AckTmr--;
            }
			
           Timing_interval_report();
        #endif
            if(device_comps.system_time.time.u8Hour==0x21&&device_comps.system_time.time.u8Minute==0x21&&device_comps.system_time.time.u8Second==0x21)
            {
                device_comps.gps.sw._bit.isActive=1;
            }

           if(device_comps.system_time.time.u8Second==0x30)
           {
                device_comps.system_time.cs=Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs-(uint8_t *)&device_comps.system_time);
                device_comps.save_system_time(&device_comps.system_time,sizeof(device_comps.system_time));
           }
           device_comps.sw._bit.isExtPowerConnected=MD_GET_EXT_POWER_STATUS();
           device_comps.sw._bit.is_4_20ma_Connected=!MD_GET_4_20MA_STATUS();
            device_comps._0_5s_timr_acc=0;
    	}//end 1s

        if(netComps.AckTmr>0)
        {
            netComps.AckTmr--;
        }
    }
}

void _50ms_task_handle(void)
{
    
   if(device_comps.buzzer.timer>0)
    {
       if(!device_comps.buzzer.sw._bit.running)
       {
          // R_PCLBUZ0_Start();
          // P0 &= 0xFBU;
          // PM0 &= 0xFBU;
           device_comps.buzzer.sw._bit.running=1;
       }
       else
       {
           device_comps.buzzer.timer--;
           if(!device_comps.buzzer.timer)
           {
                device_comps.buzzer.stop();
           }
       }
    }
     
}

void Adc_IRQHandler(void)
{    
    if(TRUE == Adc_GetIrqStatus(AdcMskIrqSgl))
    {
        Adc_ClrIrqStatus(AdcMskIrqSgl);       ///< 清除中断标志位
        Adc_SGL_Stop();                       ///< ADC 单次转换停止
        device_comps.sw._bit.adc_busy=0;
    }
}

