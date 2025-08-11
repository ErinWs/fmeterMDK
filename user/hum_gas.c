#include "ddl.h"
#include "i2c.h"
#include "gpio.h"
#include "rtc.h"
#include "lcd.h"

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#include "system.h"
#include "hum.h"
#include "irc.h"
#include "elora.h"
#include "net.h"
#include "protocol.h"
#include "device.h"
#include "modbus.h"
#include "ertc.h"



#define MD_KEY_VDD_ENABLE SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_KEY_VDD_PORT), MD_KEY_VDD_PIN, TRUE);
#define MD_KEY_VDD_DISENABLE SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_KEY_VDD_PORT), MD_KEY_VDD_PIN, FALSE);

#define MD_KEY_MASK (uint16_t)0x07
#define MD_KEY_DATA (GetBit(((uint32_t)&M0P_GPIO->PAIN + MD_S_KEY_PORT), MD_S_KEY_PIN) << 2) + (GetBit(((uint32_t)&M0P_GPIO->PAIN + MD_M_KEY_PORT), MD_M_KEY_PIN) << 1) + (GetBit(((uint32_t)&M0P_GPIO->PAIN + MD_J_KEY_PORT), MD_J_KEY_PIN) << 0)

#define MD_S_KEY 0x03 // 011
#define MD_M_KEY 0x05 // 101
#define MD_J_KEY 0x06 // 110

#define MD_LCD_POWER_PIN_ENABLE SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_LCD_POWER_PORT), MD_LCD_POWER_PIN, TRUE)
#define MD_LCD_POWER_PIN_DISABLE SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_LCD_POWER_PORT), MD_LCD_POWER_PIN, FALSE)

static uint8_t reverse_4b_t[] = {0x00, 0x08, 0x04, 0x0c, 0x02, 0x0a, 0x06, 0x0e, 0x01, 0x09, 0x05, 0x0d, 0x03, 0x0b, 0x07, 0x0f};
static uint8_t reverse_L(uint8_t c)
{
    return reverse_4b_t[c & 0x0f] & 0x0f;
}

static uint8_t reverse_H(uint8_t c)
{
    return reverse_4b_t[c >> 4] << 4;
}

static en_result_t App_Lcd_I2cCfg(M0P_I2C_TypeDef *I2CX)
{
    stc_i2c_cfg_t stcI2cCfg;
    stc_gpio_cfg_t stcGpioCfg;

    DDL_ZERO_STRUCT(stcI2cCfg); ///< ��ʼ���ṹ�������ֵΪ0
    if (I2CX == M0P_I2C0)
    {
        Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c0, TRUE); ///< ����I2C0ʱ���ſ�
    }
    else if (I2CX == M0P_I2C1)
    {
        Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c1, TRUE);
    }
    else
    {
        return ErrorInvalidParameter;
    }

    stcI2cCfg.u32Pclk = Sysctrl_GetPClkFreq(); ///< ��ȡPCLKʱ��
    stcI2cCfg.u32Baud = 400000;                ///< 1MHz
    stcI2cCfg.enMode = I2cMasterMode;          ///< ����ģʽ
    stcI2cCfg.u8SlaveAddr = 0x55;              ///< �ӵ�ַ����ģʽ��Ч
    stcI2cCfg.bGc = FALSE;                     ///< �㲥��ַӦ��ʹ�ܹر�
    I2C_Init(I2CX, &stcI2cCfg);                ///< ģ���ʼ��

    DDL_ZERO_STRUCT(stcGpioCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enOD = GpioOdEnable;
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.bOutputVal = TRUE;
    Gpio_Init(MD_LCD_I2C0_SDA_PORT, MD_LCD_I2C0_SDA_PIN, &stcGpioCfg);
    Gpio_Init(MD_LCD_I2C0_SCL_PORT, MD_LCD_I2C0_SCL_PIN, &stcGpioCfg);
    // Gpio_SetAfMode_Lite(PF00_I2C0_SDA);
    //  Gpio_SetAfMode_Lite(PF01_I2C0_SCL);
    Gpio_SetAfMode(MD_LCD_I2C0_SDA_PORT, MD_LCD_I2C0_SDA_PIN, MD_LCD_I2C0_SDA_PIN_AFN);
    Gpio_SetAfMode(MD_LCD_I2C0_SCL_PORT, MD_LCD_I2C0_SCL_PIN, MD_LCD_I2C0_SCL_PIN_AFN);
    return Ok;
}

#define MD_LCD_I2C_DEV_ADDR 0x3e

static en_result_t I2C_MasterWriteData(M0P_I2C_TypeDef *I2CX, uint8_t *pu8Data, uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i = 0, u8ErrCnt = 0, u8State;
    if (u32Len == 0)
    {
        return enRet;
    }
    I2C_SetFunc(I2CX, I2cStart_En);
    while (1)
    {
        while (0 == I2C_GetIrq(I2CX))
        {
            ;
        }
        u8State = I2C_GetState(I2CX);
        switch (u8State)
        {
        case 0x08:
            I2C_ClearFunc(I2CX, I2cStart_En);
            I2C_WriteByte(I2CX, (MD_LCD_I2C_DEV_ADDR << 1));
            break;
        case 0x10:
            I2C_ClearFunc(I2CX, I2cStart_En);
            I2C_WriteByte(I2CX, (MD_LCD_I2C_DEV_ADDR << 1));
            break;
        case 0x18:
        case 0x28:
            if (u8i < u32Len)
            {
                I2C_WriteByte(I2CX, pu8Data[u8i]);
            }
            u8i++;
            break;
        case 0x20:
        case 0x38:
            if (u8ErrCnt < 2)
            {
                u8ErrCnt++;
                I2C_SetFunc(I2CX, I2cStart_En);
            }
            else
            {
                return enRet;
            }
            break;
        case 0x30:
            I2C_SetFunc(I2CX, I2cStop_En);
            delay100us(1);
            return enRet;
            break;
        default:
            break;
        }
        if (u8i > u32Len)
        {
            I2C_SetFunc(I2CX, I2cStop_En);
            delay100us(1);
            I2C_ClearIrq(I2CX);
            break;
        }
        I2C_ClearIrq(I2CX);
    }
    enRet = Ok;
    return enRet;
}

static en_result_t I2C_MasterReadData(M0P_I2C_TypeDef *I2CX, uint8_t *pu8Data, uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i = 0, u8ErrCnt = 0, u8State;
    if (u32Len == 0)
    {
        return enRet;
    }
    I2C_SetFunc(I2CX, I2cStart_En);

    while (1)
    {
        while (0 == I2C_GetIrq(I2CX))
        {
            ;
        }
        u8State = I2C_GetState(I2CX);
        switch (u8State)
        {
        case 0x08: // �ѷ�����ʼ������������SLA+R
            I2C_ClearFunc(I2CX, I2cStart_En);
            I2C_WriteByte(I2CX, (MD_LCD_I2C_DEV_ADDR << 1) | 0x01); // ����SLA+R
            break;

        case 0x10: // �ѷ����ظ���ʼ����
            I2C_ClearFunc(I2CX, I2cStart_En);
            I2C_WriteByte(I2CX, (MD_LCD_I2C_DEV_ADDR << 1) | 0x01); // �������
            break;
        case 0x40: // �ѷ���SLA+R�������յ�ACK
            if (u32Len > 1)
            {
                I2C_SetFunc(I2CX, I2cAck_En);
            }
            break;
        case 0x50: // �ѽ��������ֽڣ����ѷ���ACK�ź�
            pu8Data[u8i++] = I2C_ReadByte(I2CX);
            if (u8i == u32Len - 1)
            {
                I2C_ClearFunc(I2CX, I2cAck_En); // ������ʱ�������ڶ����ֽ�ACK�ر�
            }
            break;
        case 0x58: // �ѽ��յ����һ�����ݣ�NACK�ѷ���
            pu8Data[u8i++] = I2C_ReadByte(I2CX);
            I2C_SetFunc(I2CX, I2cStop_En); // ����ֹͣ����
            break;
        case 0x38: // �ڷ��͵�ַ������ʱ���ٲö�ʧ
        case 0x48: // ����SLA+R���յ�һ��NACK
            if (u8ErrCnt < 2)
            {
                u8ErrCnt++;
                // I2C_SetFunc(I2CX,I2cStop_En);
                I2C_SetFunc(I2CX, I2cStart_En);
            }
            else
            {
                return enRet;
            }
        default: // ��������״̬�����·�����ʼ����
            I2C_SetFunc(I2CX, I2cStart_En);
            break;
        }
        I2C_ClearIrq(I2CX); // ����ж�״̬��־λ
        I2C_ClearFunc(I2CX, I2cAck_En);
        if (u8i == u32Len) // ����ȫ����ȡ��ɣ�����whileѭ��
        {
            break;
        }
    }
    enRet = Ok;
    return enRet;
}

void App_Disp_IC_Init(void)
{
    // uint8_t lcd_bias_duty[2] = {0x82, 0};
    // uint8_t lcd_frame_freq[2] = {0x86, 0};
    uint8_t lcd_seg_vlcd_vreg[2] = {0x8a, 0 << 4};
    //uint8_t lcd_blink_freq[2] = {0x88, 0};
    uint8_t lcd_osc_dis_on[2] = {0x84, 3};
    // if (I2C_MasterWriteData(MD_LCD_I2CX_SEL, lcd_bias_duty, 2) == Error)
    // {
    //     return;
    // }
    // if (I2C_MasterWriteData(MD_LCD_I2CX_SEL, lcd_frame_freq, 2) == Error)
    // {
    //     return;
    // }
    if (I2C_MasterWriteData(MD_LCD_I2CX_SEL, lcd_seg_vlcd_vreg, 2) == Error)
    {
        return;
    }
    //if (I2C_MasterWriteData(MD_LCD_I2CX_SEL, lcd_blink_freq, 2) == Error)
    //{
    //    return;
    //}
    if (I2C_MasterWriteData(MD_LCD_I2CX_SEL, lcd_osc_dis_on, 2) == Error)
    {
        return;
    }
}

 static void write_disp_ram(uint8_t ram_index, uint8_t dat)
 {
//     uint8_t dis_dat[3] = {0x80, ram_index, dat};
//     I2C_MasterWriteData(MD_LCD_I2CX_SEL, dis_dat, 3);
 }
//#define write_disp_ram(ram_index, dat)  __NOP()

#define BK_LN
#define MD_05_PIN_LCD_SEG MD_SEG(0)
#define MD_06_PIN_LCD_SEG MD_SEG(1)
#define MD_07_PIN_LCD_SEG MD_SEG(2)
#define MD_08_PIN_LCD_SEG MD_SEG(3)
#define MD_09_PIN_LCD_SEG MD_SEG(4)
#define MD_10_PIN_LCD_SEG MD_SEG(5)
#define MD_11_PIN_LCD_SEG MD_SEG(6)
#define MD_12_PIN_LCD_SEG MD_SEG(7)
#define MD_13_PIN_LCD_SEG MD_SEG(8)
#define MD_14_PIN_LCD_SEG MD_SEG(9)
#define MD_15_PIN_LCD_SEG MD_SEG(10)
#define MD_16_PIN_LCD_SEG MD_SEG(11)
#define MD_17_PIN_LCD_SEG MD_SEG(12)
#define MD_18_PIN_LCD_SEG MD_SEG(13)
#define MD_19_PIN_LCD_SEG MD_SEG(14)
#define MD_20_PIN_LCD_SEG MD_SEG(15)
#define MD_21_PIN_LCD_SEG MD_SEG(16)
#define MD_22_PIN_LCD_SEG MD_SEG(17)
#define MD_23_PIN_LCD_SEG MD_SEG(18)
#define MD_24_PIN_LCD_SEG MD_SEG(19)
#define MD_25_PIN_LCD_SEG MD_SEG(20)
#define MD_26_PIN_LCD_SEG MD_SEG(21)
#define MD_27_PIN_LCD_SEG MD_SEG(22)
#define MD_28_PIN_LCD_SEG MD_SEG(23)
#define MD_29_PIN_LCD_SEG MD_SEG(24)
#define MD_30_PIN_LCD_SEG MD_SEG(25)
#define MD_31_PIN_LCD_SEG MD_SEG(26)
#define MD_32_PIN_LCD_SEG MD_SEG(27)
#define MD_33_PIN_LCD_SEG MD_SEG(28)
#define MD_34_PIN_LCD_SEG MD_SEG(29)
#define MD_35_PIN_LCD_SEG MD_SEG(30)
#define MD_36_PIN_LCD_SEG MD_SEG(31)
#define MD_37_PIN_LCD_SEG MD_SEG(32)
#define MD_38_PIN_LCD_SEG MD_SEG(33)
#define MD_39_PIN_LCD_SEG MD_SEG(34)
#define MD_40_PIN_LCD_SEG MD_SEG(35)
#define MD_41_PIN_LCD_SEG MD_SEG(36)
#define MD_42_PIN_LCD_SEG MD_SEG(37)
#define MD_43_PIN_LCD_SEG MD_SEG(38)
#define MD_44_PIN_LCD_SEG MD_SEG(39)
#define MD_45_PIN_LCD_SEG MD_SEG(40)
#define MD_46_PIN_LCD_SEG MD_SEG(41)
#define MD_47_PIN_LCD_SEG MD_SEG(42)
#define MD_48_PIN_LCD_SEG MD_SEG(43)
#define MD_49_PIN_LCD_SEG MD_SEG(44)
#define MD_50_PIN_LCD_SEG MD_SEG(45)
#define MD_51_PIN_LCD_SEG MD_SEG(46)
#define MD_52_PIN_LCD_SEG MD_SEG(47)
#define MD_53_PIN_LCD_SEG MD_SEG(48)
#define MD_54_PIN_LCD_SEG MD_SEG(49)
#define MD_55_PIN_LCD_SEG MD_SEG(50)
#define MD_56_PIN_LCD_SEG MD_SEG(51)
#define MD_SEG(N) (hum_comps.device_driver_ram[N / 2])

#define MD_DIS_S1_REM_USAGE MD_28_PIN_LCD_SEG |= 0x80, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_DIS_S2_REM_AMOUNT MD_28_PIN_LCD_SEG |= 0x40, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_DIS_S3_TOTAL_USAGE MD_30_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_30_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_30_PIN_LCD_SEG)
#define MD_DIS_S4_CSQ MD_45_PIN_LCD_SEG |= 0x08, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_DIS_S5_CSQ MD_45_PIN_LCD_SEG |= 0x04, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_DIS_S6_CSQ MD_45_PIN_LCD_SEG |= 0x02, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_DIS_S7_CSQ MD_45_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_DIS_S8_YUAN MD_44_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_44_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_44_PIN_LCD_SEG)
#define MD_DIS_S9_N MD_34_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_34_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_34_PIN_LCD_SEG)
#define MD_DIS_S10_M3 MD_32_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_32_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_32_PIN_LCD_SEG)

#define MD_DIS_S11_STD_CONDITION MD_05_PIN_LCD_SEG |= 0x02, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)
#define MD_DIS_S12_PRESS MD_28_PIN_LCD_SEG |= 0x20, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_DIS_S13_N MD_53_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_53_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_53_PIN_LCD_SEG)
#define MD_DIS_S14_M3_H MD_28_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_DIS_S15_MPA MD_23_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_23_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_23_PIN_LCD_SEG)
#define MD_DIS_S16_KPA MD_14_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)

#define MD_DIS_S17_PRICE MD_05_PIN_LCD_SEG |= 0x04, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)
#define MD_DIS_S18_WORK_CONDITION MD_05_PIN_LCD_SEG |= 0x08, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)
#define MD_DIS_S19_YUAN MD_14_PIN_LCD_SEG |= 0x40, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)
#define MD_DIS_S20_TEMP MD_14_PIN_LCD_SEG |= 0x20, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)
#define MD_DIS_S21_M_3 MD_13_PIN_LCD_SEG |= 0x08, write_disp_ram(&MD_13_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_13_PIN_LCD_SEG)
#define MD_DIS_S22_H MD_14_PIN_LCD_SEG |= 0x80, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)
#define MD_DIS_S23_CELSIUS MD_20_PIN_LCD_SEG |= 0x80, write_disp_ram(&MD_20_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_20_PIN_LCD_SEG)
#define MD_DIS_S24_VALVE MD_21_PIN_LCD_SEG |= 0x04, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_DIS_S25_OPEN MD_21_PIN_LCD_SEG |= 0x08, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_DIS_S26_CLOSE MD_22_PIN_LCD_SEG |= 0x80, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)

#define MD_DIS_S27_BAT MD_21_PIN_LCD_SEG |= 0x02, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_DIS_S28_BAT MD_21_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_DIS_S29_BAT MD_22_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)
#define MD_DIS_S30_BAT MD_22_PIN_LCD_SEG |= 0x20, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)
#define MD_DIS_S31_BAT MD_22_PIN_LCD_SEG |= 0x40, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)

#define MD_DIS_P1_DIG0_4_DOT MD_36_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_36_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_36_PIN_LCD_SEG)
#define MD_DIS_P2_DIG0_3_DOT MD_38_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_38_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_38_PIN_LCD_SEG)
#define MD_DIS_P3_DIG0_2_DOT MD_40_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_40_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_40_PIN_LCD_SEG)
#define MD_DIS_P4_DIG0_1_DOT MD_42_PIN_LCD_SEG |= 0x10, write_disp_ram(&MD_42_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_42_PIN_LCD_SEG)
#define MD_DIS_P5_DIG1_3_DOT MD_47_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_47_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_47_PIN_LCD_SEG)
#define MD_DIS_P6_DIG1_2_DOT MD_49_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_49_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_49_PIN_LCD_SEG)
#define MD_DIS_P7_DIG1_1_DOT MD_51_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_51_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_51_PIN_LCD_SEG)
#define MD_DIS_P8_DIG2_3_DOT MD_55_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_55_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_55_PIN_LCD_SEG)
#define MD_DIS_P9_DIG2_2_DOT MD_27_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_27_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_27_PIN_LCD_SEG)
#define MD_DIS_P10_DIG2_1_DOT MD_25_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_25_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_25_PIN_LCD_SEG)
#define MD_DIS_P11_DIG3_3_DOT MD_07_PIN_LCD_SEG |= 0x08, write_disp_ram(&MD_07_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_07_PIN_LCD_SEG)
#define MD_DIS_P12_DIG3_2_DOT MD_09_PIN_LCD_SEG |= 0x08, write_disp_ram(&MD_09_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_09_PIN_LCD_SEG)
#define MD_DIS_P13_DIG3_1_DOT MD_11_PIN_LCD_SEG |= 0x08, write_disp_ram(&MD_11_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_11_PIN_LCD_SEG)
#define MD_DIS_P14_DIG4_2_DOT MD_16_PIN_LCD_SEG |= 0x80, write_disp_ram(&MD_16_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_16_PIN_LCD_SEG)
#define MD_DIS_P15_DIG4_1_DOT MD_18_PIN_LCD_SEG |= 0x80, write_disp_ram(&MD_18_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_18_PIN_LCD_SEG)
#define MD_DIS_P16_DIG3_T_DOT MD_05_PIN_LCD_SEG |= 0x01, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)

#define MD_HIDE_S1_REM_USAGE MD_28_PIN_LCD_SEG &= ~0x80, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_HIDE_S2_REM_AMOUNT MD_28_PIN_LCD_SEG &= ~0x40, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_HIDE_S3_TOTAL_USAGE MD_30_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_30_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_30_PIN_LCD_SEG)
#define MD_HIDE_S4_CSQ MD_45_PIN_LCD_SEG &= ~0x08, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_HIDE_S5_CSQ MD_45_PIN_LCD_SEG &= ~0x04, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_HIDE_S6_CSQ MD_45_PIN_LCD_SEG &= ~0x02, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_HIDE_S7_CSQ MD_45_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_45_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_45_PIN_LCD_SEG)
#define MD_HIDE_S8_YUAN MD_44_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_44_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_44_PIN_LCD_SEG)
#define MD_HIDE_S9_N MD_34_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_34_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_34_PIN_LCD_SEG)
#define MD_HIDE_S10_M3 MD_32_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_32_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_32_PIN_LCD_SEG)

#define MD_HIDE_S11_STD_CONDITION MD_05_PIN_LCD_SEG &= ~0x02, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)
#define MD_HIDE_S12_PRESS MD_28_PIN_LCD_SEG &= ~0x20, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_HIDE_S13_N MD_53_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_53_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_53_PIN_LCD_SEG)
#define MD_HIDE_S14_M3_H MD_28_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_28_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_28_PIN_LCD_SEG)
#define MD_HIDE_S15_MPA MD_23_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_23_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_23_PIN_LCD_SEG)
#define MD_HIDE_S16_KPA MD_14_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)

#define MD_HIDE_S17_PRICE MD_05_PIN_LCD_SEG &= ~0x04, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)
#define MD_HIDE_S18_WORK_CONDITION MD_05_PIN_LCD_SEG &= ~0x08, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)
#define MD_HIDE_S19_YUAN MD_14_PIN_LCD_SEG &= ~0x40, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)
#define MD_HIDE_S20_TEMP MD_14_PIN_LCD_SEG &= ~0x20, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)
#define MD_HIDE_S21_M_3 MD_13_PIN_LCD_SEG &= ~0x08, write_disp_ram(&MD_13_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_13_PIN_LCD_SEG)
#define MD_HIDE_S22_H MD_14_PIN_LCD_SEG &= ~0x80, write_disp_ram(&MD_14_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_14_PIN_LCD_SEG)
#define MD_HIDE_S23_CELSIUS MD_20_PIN_LCD_SEG &= ~0x80, write_disp_ram(&MD_20_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_20_PIN_LCD_SEG)
#define MD_HIDE_S24_VALVE MD_21_PIN_LCD_SEG &= ~0x04, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_HIDE_S25_OPEN MD_21_PIN_LCD_SEG &= ~0x08, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_HIDE_S26_CLOSE MD_22_PIN_LCD_SEG &= ~0x80, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)

#define MD_HIDE_S27_BAT MD_21_PIN_LCD_SEG &= ~0x02, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_HIDE_S28_BAT MD_21_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_21_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_21_PIN_LCD_SEG)
#define MD_HIDE_S29_BAT MD_22_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)
#define MD_HIDE_S30_BAT MD_22_PIN_LCD_SEG &= ~0x20, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)
#define MD_HIDE_S31_BAT MD_22_PIN_LCD_SEG &= ~0x40, write_disp_ram(&MD_22_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_22_PIN_LCD_SEG)

#define MD_HIDE_P1_DIG0_4_DOT MD_36_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_36_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_36_PIN_LCD_SEG)
#define MD_HIDE_P2_DIG0_3_DOT MD_38_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_38_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_38_PIN_LCD_SEG)
#define MD_HIDE_P3_DIG0_2_DOT MD_40_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_40_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_40_PIN_LCD_SEG)
#define MD_HIDE_P4_DIG0_1_DOT MD_42_PIN_LCD_SEG &= ~0x10, write_disp_ram(&MD_42_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_42_PIN_LCD_SEG)
#define MD_HIDE_P5_DIG1_3_DOT MD_47_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_47_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_47_PIN_LCD_SEG)
#define MD_HIDE_P6_DIG1_2_DOT MD_49_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_49_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_49_PIN_LCD_SEG)
#define MD_HIDE_P7_DIG1_1_DOT MD_51_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_51_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_51_PIN_LCD_SEG)
#define MD_HIDE_P8_DIG2_3_DOT MD_55_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_55_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_55_PIN_LCD_SEG)
#define MD_HIDE_P9_DIG2_2_DOT MD_27_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_27_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_27_PIN_LCD_SEG)
#define MD_HIDE_P10_DIG2_1_DOT MD_25_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_25_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_25_PIN_LCD_SEG)
#define MD_HIDE_P11_DIG3_3_DOT MD_07_PIN_LCD_SEG &= ~0x08, write_disp_ram(&MD_07_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_07_PIN_LCD_SEG)
#define MD_HIDE_P12_DIG3_2_DOT MD_09_PIN_LCD_SEG &= ~0x08, write_disp_ram(&MD_09_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_09_PIN_LCD_SEG)
#define MD_HIDE_P13_DIG3_1_DOT MD_11_PIN_LCD_SEG &= ~0x08, write_disp_ram(&MD_11_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_11_PIN_LCD_SEG)
#define MD_HIDE_P14_DIG4_2_DOT MD_16_PIN_LCD_SEG &= ~0x80, write_disp_ram(&MD_16_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_16_PIN_LCD_SEG)
#define MD_HIDE_P15_DIG4_1_DOT MD_18_PIN_LCD_SEG &= ~0x80, write_disp_ram(&MD_18_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_18_PIN_LCD_SEG)
#define MD_HIDE_P16_DIG3_T_DOT MD_05_PIN_LCD_SEG &= ~0x01, write_disp_ram(&MD_05_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_05_PIN_LCD_SEG)

static void MD_DISPLAY_WRITE_D0_0(uint8_t a)
{
    MD_44_PIN_LCD_SEG &= 0x10;
    MD_44_PIN_LCD_SEG |= hum_comps.SEG_TAB[a];
    write_disp_ram(&MD_44_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_44_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D0_1(uint8_t a)
{
    MD_42_PIN_LCD_SEG = hum_comps.SEG_TAB[a];
    if (hum_comps.dot0_pos == 1)
        ((hum_comps.cursor_0 == -1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? (MD_42_PIN_LCD_SEG |= 0x10) : (MD_42_PIN_LCD_SEG &= ~0x10)) : (MD_42_PIN_LCD_SEG |= 0x10);
    write_disp_ram(&MD_42_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_42_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D0_2(uint8_t a)
{
    MD_40_PIN_LCD_SEG = hum_comps.SEG_TAB[a];
    if (hum_comps.dot0_pos == 2)
        ((hum_comps.cursor_0 == -1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? (MD_40_PIN_LCD_SEG |= 0x10) : (MD_40_PIN_LCD_SEG &= ~0x10)) : (MD_40_PIN_LCD_SEG |= 0x10);
    write_disp_ram(&MD_40_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_40_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D0_3(uint8_t a)
{
    MD_38_PIN_LCD_SEG = hum_comps.SEG_TAB[a];
    if (hum_comps.dot0_pos == 3)
        ((hum_comps.cursor_0 == -1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? (MD_38_PIN_LCD_SEG |= 0x10) : (MD_38_PIN_LCD_SEG &= ~0x10)) : (MD_38_PIN_LCD_SEG |= 0x10);
    write_disp_ram(&MD_38_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_38_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D0_4(uint8_t a)
{
    MD_36_PIN_LCD_SEG = hum_comps.SEG_TAB[a];
    if (hum_comps.dot0_pos == 4)
        ((hum_comps.cursor_0 == -1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? (MD_36_PIN_LCD_SEG |= 0x10) : (MD_36_PIN_LCD_SEG &= ~0x10)) : (MD_36_PIN_LCD_SEG |= 0x10);
    write_disp_ram(&MD_36_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_36_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D0_5(uint8_t a)
{
    MD_34_PIN_LCD_SEG &= 0x10;
    MD_34_PIN_LCD_SEG |= hum_comps.SEG_TAB[a];
    write_disp_ram(&MD_34_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_34_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D0_6(uint8_t a)
{
    MD_32_PIN_LCD_SEG &= 0x10;
    MD_32_PIN_LCD_SEG |= hum_comps.SEG_TAB[a];
    write_disp_ram(&MD_32_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_32_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D0_7(uint8_t a)
{
    MD_30_PIN_LCD_SEG &= 0x10;
    MD_30_PIN_LCD_SEG |= hum_comps.SEG_TAB[a];
    write_disp_ram(&MD_30_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_30_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D1_0(uint8_t a)
{
    MD_53_PIN_LCD_SEG &= 0xf1;
    MD_53_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    write_disp_ram(&MD_53_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_53_PIN_LCD_SEG);

    MD_52_PIN_LCD_SEG &= 0x0f;
    MD_52_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_52_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_52_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D1_1(uint8_t a)
{
    MD_51_PIN_LCD_SEG &= 0xf0;
    MD_51_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    if (hum_comps.dot1_pos == 1)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_51_PIN_LCD_SEG |= 0x01) : (MD_51_PIN_LCD_SEG &= ~0x01)) : (MD_51_PIN_LCD_SEG |= 0x01);
    write_disp_ram(&MD_51_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_51_PIN_LCD_SEG);

    MD_50_PIN_LCD_SEG &= 0x0f;
    MD_50_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_50_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_50_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D1_2(uint8_t a)
{

    MD_49_PIN_LCD_SEG &= 0xf0;
    MD_49_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    if (hum_comps.dot1_pos == 2)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_49_PIN_LCD_SEG |= 0x01) : (MD_49_PIN_LCD_SEG &= ~0x01)) : (MD_49_PIN_LCD_SEG |= 0x01);
    write_disp_ram(&MD_49_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_49_PIN_LCD_SEG);

    MD_48_PIN_LCD_SEG &= 0x0f;
    MD_48_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_48_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_48_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D1_3(uint8_t a)
{
    MD_47_PIN_LCD_SEG &= 0xf0;
    MD_47_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    if (hum_comps.dot1_pos == 3)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_47_PIN_LCD_SEG |= 0x01) : (MD_47_PIN_LCD_SEG &= ~0x01)) : (MD_47_PIN_LCD_SEG |= 0x01);
    write_disp_ram(&MD_47_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_47_PIN_LCD_SEG);

    MD_46_PIN_LCD_SEG &= 0x0f;
    MD_46_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_46_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_46_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D1_4(uint8_t a)
{
}

static void MD_DISPLAY_WRITE_D2_0(uint8_t a)
{
    MD_23_PIN_LCD_SEG &= 0xf1;
    MD_23_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    write_disp_ram(&MD_23_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_23_PIN_LCD_SEG);

    MD_24_PIN_LCD_SEG &= 0x0f;
    MD_24_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_24_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_24_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D2_1(uint8_t a)
{
    MD_25_PIN_LCD_SEG &= 0xf0;
    MD_25_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    if (hum_comps.dot2_pos == 1)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_25_PIN_LCD_SEG |= 0x01) : (MD_25_PIN_LCD_SEG &= ~0x01)) : (MD_25_PIN_LCD_SEG |= 0x01);
    write_disp_ram(&MD_25_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_25_PIN_LCD_SEG);

    MD_26_PIN_LCD_SEG &= 0x0f;
    MD_26_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_26_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_26_PIN_LCD_SEG);
}
static void MD_DISPLAY_WRITE_D2_2(uint8_t a)
{
    MD_27_PIN_LCD_SEG &= 0xf0;
    MD_27_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    if (hum_comps.dot2_pos == 2)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_27_PIN_LCD_SEG |= 0x01) : (MD_27_PIN_LCD_SEG &= ~0x01)) : (MD_27_PIN_LCD_SEG |= 0x01);
    write_disp_ram(&MD_27_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_27_PIN_LCD_SEG);

    MD_56_PIN_LCD_SEG &= 0x0f;
    MD_56_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_56_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_56_PIN_LCD_SEG);
}
static void MD_DISPLAY_WRITE_D2_3(uint8_t a)
{
    MD_55_PIN_LCD_SEG &= 0xf0;
    MD_55_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] >> 4;
    if (hum_comps.dot2_pos == 3)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_55_PIN_LCD_SEG |= 0x01) : (MD_55_PIN_LCD_SEG &= ~0x01)) : (MD_55_PIN_LCD_SEG |= 0x01);
    write_disp_ram(&MD_55_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_55_PIN_LCD_SEG);

    MD_54_PIN_LCD_SEG &= 0x0f;
    MD_54_PIN_LCD_SEG |= hum_comps.SEG_TAB[a] << 4;
    write_disp_ram(&MD_54_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_54_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D3_0(uint8_t a)
{
    MD_13_PIN_LCD_SEG &= 0xf8;
    MD_13_PIN_LCD_SEG |= reverse_L(hum_comps.SEG_TAB[a] >> 4);
    write_disp_ram(&MD_13_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_13_PIN_LCD_SEG);

    MD_12_PIN_LCD_SEG &= 0x0f;
    MD_12_PIN_LCD_SEG |= reverse_H(hum_comps.SEG_TAB[a] << 4);
    write_disp_ram(&MD_12_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_12_PIN_LCD_SEG);
}
static void MD_DISPLAY_WRITE_D3_1(uint8_t a)
{
    MD_11_PIN_LCD_SEG &= 0xf0;
    MD_11_PIN_LCD_SEG |= reverse_L(hum_comps.SEG_TAB[a] >> 4);
    if (hum_comps.dot3_pos == 1)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_11_PIN_LCD_SEG |= 0x08) : (MD_11_PIN_LCD_SEG &= ~0x08)) : (MD_11_PIN_LCD_SEG |= 0x08);
    write_disp_ram(&MD_11_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_11_PIN_LCD_SEG);

    MD_10_PIN_LCD_SEG &= 0x0f;
    MD_10_PIN_LCD_SEG |= reverse_H(hum_comps.SEG_TAB[a] << 4);
    write_disp_ram(&MD_10_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_10_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D3_2(uint8_t a)
{
    MD_09_PIN_LCD_SEG &= 0xf0;
    MD_09_PIN_LCD_SEG |= reverse_L(hum_comps.SEG_TAB[a] >> 4);
    if (hum_comps.dot3_pos == 2)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_09_PIN_LCD_SEG |= 0x08) : (MD_09_PIN_LCD_SEG &= ~0x08)) : (MD_09_PIN_LCD_SEG |= 0x08);
    write_disp_ram(&MD_09_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_09_PIN_LCD_SEG);

    MD_08_PIN_LCD_SEG &= 0x0f;
    MD_08_PIN_LCD_SEG |= reverse_H(hum_comps.SEG_TAB[a] << 4);
    write_disp_ram(&MD_08_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_08_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D3_3(uint8_t a)
{
    MD_07_PIN_LCD_SEG &= 0xf0;
    MD_07_PIN_LCD_SEG |= reverse_L(hum_comps.SEG_TAB[a] >> 4);
    if (hum_comps.dot3_pos == 3)
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? (MD_07_PIN_LCD_SEG |= 0x08) : (MD_07_PIN_LCD_SEG &= ~0x08)) : (MD_07_PIN_LCD_SEG |= 0x08);
    write_disp_ram(&MD_07_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_07_PIN_LCD_SEG);

    MD_06_PIN_LCD_SEG &= 0x0f;
    MD_06_PIN_LCD_SEG |= reverse_H(hum_comps.SEG_TAB[a] << 4);
    write_disp_ram(&MD_06_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_06_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D4_0(uint8_t a)
{
    MD_20_PIN_LCD_SEG &= 0x80;
    MD_20_PIN_LCD_SEG |= reverse_H(hum_comps.SEG_TAB[a]) + reverse_L(hum_comps.SEG_TAB[a]);
    write_disp_ram(&MD_20_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_20_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D4_1(uint8_t a)
{
    MD_18_PIN_LCD_SEG = reverse_H(hum_comps.SEG_TAB[a]) + reverse_L(hum_comps.SEG_TAB[a]);
    if (hum_comps.dot4_pos == 1)
        ((hum_comps.cursor_0 == -1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? (MD_18_PIN_LCD_SEG |= 0x80) : (MD_18_PIN_LCD_SEG &= ~0x80)) : (MD_18_PIN_LCD_SEG |= 0x80);
    write_disp_ram(&MD_18_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_18_PIN_LCD_SEG);
}

static void MD_DISPLAY_WRITE_D4_2(uint8_t a)
{
    MD_16_PIN_LCD_SEG = reverse_H(hum_comps.SEG_TAB[a]) + reverse_L(hum_comps.SEG_TAB[a]);
    if (hum_comps.dot4_pos == 2)
        ((hum_comps.cursor_0 == -1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? (MD_16_PIN_LCD_SEG |= 0x80) : (MD_16_PIN_LCD_SEG &= ~0x80)) : (MD_16_PIN_LCD_SEG |= 0x80);
    write_disp_ram(&MD_16_PIN_LCD_SEG - hum_comps.device_driver_ram, MD_16_PIN_LCD_SEG);
}

static void display_line0_data(void) // display 1th line data
{
#define MD_DIS_ALL_0_LINE_DATA (MD_DISPLAY_WRITE_D0_7(hum_comps.dig0_7), MD_DISPLAY_WRITE_D0_6(hum_comps.dig0_6), MD_DISPLAY_WRITE_D0_5(hum_comps.dig0_5), MD_DISPLAY_WRITE_D0_4(hum_comps.dig0_4), MD_DISPLAY_WRITE_D0_3(hum_comps.dig0_3), MD_DISPLAY_WRITE_D0_2(hum_comps.dig0_2), MD_DISPLAY_WRITE_D0_1(hum_comps.dig0_1), MD_DISPLAY_WRITE_D0_0(hum_comps.dig0_0))
#define MD_HID_ALL_0_LINE_DATA (MD_DISPLAY_WRITE_D0_7(MD_HIDE_DISP), MD_DISPLAY_WRITE_D0_6(MD_HIDE_DISP), MD_DISPLAY_WRITE_D0_5(MD_HIDE_DISP), MD_DISPLAY_WRITE_D0_4(MD_HIDE_DISP), MD_DISPLAY_WRITE_D0_3(MD_HIDE_DISP), MD_DISPLAY_WRITE_D0_2(MD_HIDE_DISP), MD_DISPLAY_WRITE_D0_1(MD_HIDE_DISP), MD_DISPLAY_WRITE_D0_0(MD_HIDE_DISP))
    if (hum_comps.cursor_0 > -1)
    {
        ((hum_comps.cursor_0 == 7) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_7(hum_comps.dig0_7) : MD_DISPLAY_WRITE_D0_7(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_7(hum_comps.dig0_7);
        ((hum_comps.cursor_0 == 6) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_6(hum_comps.dig0_6) : MD_DISPLAY_WRITE_D0_6(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_6(hum_comps.dig0_6);
        ((hum_comps.cursor_0 == 5) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_5(hum_comps.dig0_5) : MD_DISPLAY_WRITE_D0_5(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_5(hum_comps.dig0_5);
        ((hum_comps.cursor_0 == 4) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_4(hum_comps.dig0_4) : MD_DISPLAY_WRITE_D0_4(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_4(hum_comps.dig0_4);
        ((hum_comps.cursor_0 == 3) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_3(hum_comps.dig0_3) : MD_DISPLAY_WRITE_D0_3(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_3(hum_comps.dig0_3);
        ((hum_comps.cursor_0 == 2) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_2(hum_comps.dig0_2) : MD_DISPLAY_WRITE_D0_2(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_2(hum_comps.dig0_2);
        ((hum_comps.cursor_0 == 1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_1(hum_comps.dig0_1) : MD_DISPLAY_WRITE_D0_1(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_1(hum_comps.dig0_1);
        ((hum_comps.cursor_0 == 0) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DISPLAY_WRITE_D0_0(hum_comps.dig0_0) : MD_DISPLAY_WRITE_D0_0(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D0_0(hum_comps.dig0_0);
    }
    else
    {
        ((hum_comps.cursor_0 == -1) && (hum_comps.dis_oper_mark._bit.cur0)) ? ((hum_comps.dis_oper_mark._bit.dis0) ? MD_DIS_ALL_0_LINE_DATA : MD_HID_ALL_0_LINE_DATA) : MD_DIS_ALL_0_LINE_DATA;
    }
    //  #undef MD_DIS_ALL_1_LINE_DATA
    //  #undef MD_HID_ALL_1_LINE_DATA
}

static void display_line1_data(void) // display 1th line data
{
#define MD_DIS_ALL_1_LINE_DATA (MD_DISPLAY_WRITE_D1_4(hum_comps.dig1_4), MD_DISPLAY_WRITE_D1_3(hum_comps.dig1_3), MD_DISPLAY_WRITE_D1_2(hum_comps.dig1_2), MD_DISPLAY_WRITE_D1_1(hum_comps.dig1_1), MD_DISPLAY_WRITE_D1_0(hum_comps.dig1_0))
#define MD_HID_ALL_1_LINE_DATA (MD_DISPLAY_WRITE_D1_4(MD_HIDE_DISP), MD_DISPLAY_WRITE_D1_3(MD_HIDE_DISP), MD_DISPLAY_WRITE_D1_2(MD_HIDE_DISP), MD_DISPLAY_WRITE_D1_1(MD_HIDE_DISP), MD_DISPLAY_WRITE_D1_0(MD_HIDE_DISP))
    if (hum_comps.cursor_1 > -1)
    {
        ((hum_comps.cursor_1 == 3) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? MD_DISPLAY_WRITE_D1_3(hum_comps.dig1_3) : MD_DISPLAY_WRITE_D1_3(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D1_3(hum_comps.dig1_3);
        ((hum_comps.cursor_1 == 2) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? MD_DISPLAY_WRITE_D1_2(hum_comps.dig1_2) : MD_DISPLAY_WRITE_D1_2(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D1_2(hum_comps.dig1_2);
        ((hum_comps.cursor_1 == 1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? MD_DISPLAY_WRITE_D1_1(hum_comps.dig1_1) : MD_DISPLAY_WRITE_D1_1(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D1_1(hum_comps.dig1_1);
        ((hum_comps.cursor_1 == 0) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? MD_DISPLAY_WRITE_D1_0(hum_comps.dig1_0) : MD_DISPLAY_WRITE_D1_0(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D1_0(hum_comps.dig1_0);
    }
    else
    {
        ((hum_comps.cursor_1 == -1) && (hum_comps.dis_oper_mark._bit.cur1)) ? ((hum_comps.dis_oper_mark._bit.dis1) ? MD_DIS_ALL_1_LINE_DATA : MD_HID_ALL_1_LINE_DATA) : MD_DIS_ALL_1_LINE_DATA;
    }
    // #undef MD_DIS_ALL_1_LINE_DATA
    //  #undef MD_HID_ALL_1_LINE_DATA
}

static void display_line2_data(void) // display 0th line data
{

#define MD_DIS_ALL_2_LINE_DATA (MD_DISPLAY_WRITE_D2_3(hum_comps.dig2_3), MD_DISPLAY_WRITE_D2_2(hum_comps.dig2_2), MD_DISPLAY_WRITE_D2_1(hum_comps.dig2_1), MD_DISPLAY_WRITE_D2_0(hum_comps.dig2_0))
#define MD_HID_ALL_2_LINE_DATA (MD_DISPLAY_WRITE_D2_3(MD_HIDE_DISP), MD_DISPLAY_WRITE_D2_2(MD_HIDE_DISP), MD_DISPLAY_WRITE_D2_1(MD_HIDE_DISP), MD_DISPLAY_WRITE_D2_0(MD_HIDE_DISP))
    if (hum_comps.cursor_2 > -1)
    {

        ((hum_comps.cursor_2 == 3) && (hum_comps.dis_oper_mark._bit.cur2)) ? ((hum_comps.dis_oper_mark._bit.dis2) ? MD_DISPLAY_WRITE_D2_3(hum_comps.dig2_3) : MD_DISPLAY_WRITE_D2_3(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D2_3(hum_comps.dig2_3);
        ((hum_comps.cursor_2 == 2) && (hum_comps.dis_oper_mark._bit.cur2)) ? ((hum_comps.dis_oper_mark._bit.dis2) ? MD_DISPLAY_WRITE_D2_2(hum_comps.dig2_2) : MD_DISPLAY_WRITE_D2_2(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D2_2(hum_comps.dig2_2);
        ((hum_comps.cursor_2 == 1) && (hum_comps.dis_oper_mark._bit.cur2)) ? ((hum_comps.dis_oper_mark._bit.dis2) ? MD_DISPLAY_WRITE_D2_1(hum_comps.dig2_1) : MD_DISPLAY_WRITE_D2_1(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D2_1(hum_comps.dig2_1);
        ((hum_comps.cursor_2 == 0) && (hum_comps.dis_oper_mark._bit.cur2)) ? ((hum_comps.dis_oper_mark._bit.dis2) ? MD_DISPLAY_WRITE_D2_0(hum_comps.dig2_0) : MD_DISPLAY_WRITE_D2_0(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D2_0(hum_comps.dig2_0);
    }
    else
    {
        ((hum_comps.cursor_2 == -1) && (hum_comps.dis_oper_mark._bit.cur2)) ? ((hum_comps.dis_oper_mark._bit.dis2) ? MD_DIS_ALL_2_LINE_DATA : MD_HID_ALL_2_LINE_DATA) : MD_DIS_ALL_2_LINE_DATA;
    }
    // #undef MD_DIS_ALL_2_LINE_DATA
    // #undef MD_HID_ALL_2_LINE_DATA
}

static void display_line3_data(void) // display 3th line data
{
#define MD_DIS_ALL_3_LINE_DATA (MD_DISPLAY_WRITE_D3_3(hum_comps.dig3_3), MD_DISPLAY_WRITE_D3_2(hum_comps.dig3_2), MD_DISPLAY_WRITE_D3_1(hum_comps.dig3_1), MD_DISPLAY_WRITE_D3_0(hum_comps.dig3_0))
#define MD_HID_ALL_3_LINE_DATA (MD_DISPLAY_WRITE_D3_3(MD_HIDE_DISP), MD_DISPLAY_WRITE_D3_2(MD_HIDE_DISP), MD_DISPLAY_WRITE_D3_1(MD_HIDE_DISP), MD_DISPLAY_WRITE_D3_0(MD_HIDE_DISP))
    if (hum_comps.cursor_3 > -1)
    {
        ((hum_comps.cursor_3 == 3) && (hum_comps.dis_oper_mark._bit.cur3)) ? ((hum_comps.dis_oper_mark._bit.dis3) ? MD_DISPLAY_WRITE_D3_3(hum_comps.dig3_3) : MD_DISPLAY_WRITE_D3_3(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D3_3(hum_comps.dig3_3);
        ((hum_comps.cursor_3 == 2) && (hum_comps.dis_oper_mark._bit.cur3)) ? ((hum_comps.dis_oper_mark._bit.dis3) ? MD_DISPLAY_WRITE_D3_2(hum_comps.dig3_2) : MD_DISPLAY_WRITE_D3_2(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D3_2(hum_comps.dig3_2);
        ((hum_comps.cursor_3 == 1) && (hum_comps.dis_oper_mark._bit.cur3)) ? ((hum_comps.dis_oper_mark._bit.dis3) ? MD_DISPLAY_WRITE_D3_1(hum_comps.dig3_1) : MD_DISPLAY_WRITE_D3_1(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D3_1(hum_comps.dig3_1);
        ((hum_comps.cursor_3 == 0) && (hum_comps.dis_oper_mark._bit.cur3)) ? ((hum_comps.dis_oper_mark._bit.dis3) ? MD_DISPLAY_WRITE_D3_0(hum_comps.dig3_0) : MD_DISPLAY_WRITE_D3_0(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D3_0(hum_comps.dig3_0);
    }
    else
    {

        ((hum_comps.cursor_3 == -1) && (hum_comps.dis_oper_mark._bit.cur3)) ? ((hum_comps.dis_oper_mark._bit.dis3) ? MD_DIS_ALL_3_LINE_DATA : MD_HID_ALL_3_LINE_DATA) : MD_DIS_ALL_3_LINE_DATA;
    }
    //  #undef MD_DIS_ALL_3_LINE_DATA
    //  #undef MD_HID_ALL_3_LINE_DATA
}

static void display_line4_data(void) // display 0th line data
{

#define MD_DIS_ALL_4_LINE_DATA (MD_DISPLAY_WRITE_D4_2(hum_comps.dig4_2), MD_DISPLAY_WRITE_D4_1(hum_comps.dig4_1), MD_DISPLAY_WRITE_D4_0(hum_comps.dig4_0))
#define MD_HID_ALL_4_LINE_DATA (MD_DISPLAY_WRITE_D4_2(MD_HIDE_DISP), MD_DISPLAY_WRITE_D4_1(MD_HIDE_DISP), MD_DISPLAY_WRITE_D4_0(MD_HIDE_DISP))
    if (hum_comps.cursor_4 > -1)
    {
        ((hum_comps.cursor_4 == 2) && (hum_comps.dis_oper_mark._bit.cur4)) ? ((hum_comps.dis_oper_mark._bit.dis4) ? MD_DISPLAY_WRITE_D4_2(hum_comps.dig4_2) : MD_DISPLAY_WRITE_D4_2(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D4_2(hum_comps.dig4_2);
        ((hum_comps.cursor_4 == 1) && (hum_comps.dis_oper_mark._bit.cur4)) ? ((hum_comps.dis_oper_mark._bit.dis4) ? MD_DISPLAY_WRITE_D4_1(hum_comps.dig4_1) : MD_DISPLAY_WRITE_D4_1(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D4_1(hum_comps.dig4_1);
        ((hum_comps.cursor_4 == 0) && (hum_comps.dis_oper_mark._bit.cur4)) ? ((hum_comps.dis_oper_mark._bit.dis4) ? MD_DISPLAY_WRITE_D4_0(hum_comps.dig4_0) : MD_DISPLAY_WRITE_D4_0(MD_HIDE_DISP)) : MD_DISPLAY_WRITE_D4_0(hum_comps.dig4_0);
    }
    else
    {
        ((hum_comps.cursor_4 == -1) && (hum_comps.dis_oper_mark._bit.cur4)) ? ((hum_comps.dis_oper_mark._bit.dis4) ? MD_DIS_ALL_4_LINE_DATA : MD_HID_ALL_4_LINE_DATA) : MD_DIS_ALL_4_LINE_DATA;
    }
    // #undef MD_DIS_ALL_4_LINE_DATA
    // #undef MD_HID_ALL_4_LINE_DATA
}

static void dis_all_symbol(void)
{
    MD_DIS_S1_REM_USAGE;
    MD_DIS_S2_REM_AMOUNT;
    MD_DIS_S3_TOTAL_USAGE;
    MD_DIS_S4_CSQ;
    MD_DIS_S5_CSQ;
    MD_DIS_S6_CSQ;
    MD_DIS_S7_CSQ;
    MD_DIS_S8_YUAN;
    MD_DIS_S9_N;
    MD_DIS_S10_M3;

    MD_DIS_S11_STD_CONDITION;
    MD_DIS_S12_PRESS;
    MD_DIS_S13_N;
    MD_DIS_S14_M3_H;
    MD_DIS_S15_MPA;
    MD_DIS_S16_KPA;

    MD_DIS_S17_PRICE;
    MD_DIS_S18_WORK_CONDITION;
    MD_DIS_S19_YUAN;
    MD_DIS_S20_TEMP;
    MD_DIS_S21_M_3;
    MD_DIS_S22_H;
    MD_DIS_S23_CELSIUS;
    MD_DIS_S24_VALVE;
    MD_DIS_S25_OPEN;
    MD_DIS_S26_CLOSE;

    MD_DIS_S27_BAT;
    MD_DIS_S28_BAT;
    MD_DIS_S29_BAT;
    MD_DIS_S30_BAT;
    MD_DIS_S31_BAT;

    MD_DIS_P1_DIG0_4_DOT;
    MD_DIS_P2_DIG0_3_DOT;
    MD_DIS_P3_DIG0_2_DOT;
    MD_DIS_P4_DIG0_1_DOT;
    MD_DIS_P5_DIG1_3_DOT;
    MD_DIS_P6_DIG1_2_DOT;
    MD_DIS_P7_DIG1_1_DOT;
    MD_DIS_P8_DIG2_3_DOT;
    MD_DIS_P9_DIG2_2_DOT;
    MD_DIS_P10_DIG2_1_DOT;
    MD_DIS_P11_DIG3_3_DOT;
    MD_DIS_P12_DIG3_2_DOT;
    MD_DIS_P13_DIG3_1_DOT;
    MD_DIS_P14_DIG4_2_DOT;
    MD_DIS_P15_DIG4_1_DOT;
    MD_DIS_P16_DIG3_T_DOT;
}

static void hide_all_symbol(void)
{
    MD_HIDE_S1_REM_USAGE;
    MD_HIDE_S2_REM_AMOUNT;
    MD_HIDE_S3_TOTAL_USAGE;
    MD_HIDE_S4_CSQ;
    MD_HIDE_S5_CSQ;
    MD_HIDE_S6_CSQ;
    MD_HIDE_S7_CSQ;
    MD_HIDE_S8_YUAN;
    MD_HIDE_S9_N;
    MD_HIDE_S10_M3;

    MD_HIDE_S11_STD_CONDITION;
    MD_HIDE_S12_PRESS;
    MD_HIDE_S13_N;
    MD_HIDE_S14_M3_H;
    MD_HIDE_S15_MPA;
    MD_HIDE_S16_KPA;

    MD_HIDE_S17_PRICE;
    MD_HIDE_S18_WORK_CONDITION;
    MD_HIDE_S19_YUAN;
    MD_HIDE_S20_TEMP;
    MD_HIDE_S21_M_3;
    MD_HIDE_S22_H;
    MD_HIDE_S23_CELSIUS;
    MD_HIDE_S24_VALVE;
    MD_HIDE_S25_OPEN;
    MD_HIDE_S26_CLOSE;

    MD_HIDE_S27_BAT;
    MD_HIDE_S28_BAT;
    MD_HIDE_S29_BAT;
    MD_HIDE_S30_BAT;
    MD_HIDE_S31_BAT;

    MD_HIDE_P1_DIG0_4_DOT;
    MD_HIDE_P2_DIG0_3_DOT;
    MD_HIDE_P3_DIG0_2_DOT;
    MD_HIDE_P4_DIG0_1_DOT;
    MD_HIDE_P5_DIG1_3_DOT;
    MD_HIDE_P6_DIG1_2_DOT;
    MD_HIDE_P7_DIG1_1_DOT;
    MD_HIDE_P8_DIG2_3_DOT;
    MD_HIDE_P9_DIG2_2_DOT;
    MD_HIDE_P10_DIG2_1_DOT;
    MD_HIDE_P11_DIG3_3_DOT;
    MD_HIDE_P12_DIG3_2_DOT;
    MD_HIDE_P13_DIG3_1_DOT;
    MD_HIDE_P14_DIG4_2_DOT;
    MD_HIDE_P15_DIG4_1_DOT;
    MD_HIDE_P16_DIG3_T_DOT;
}

static void hide_line0_all_unit(void)
{
    MD_HIDE_S1_REM_USAGE;
    MD_HIDE_S2_REM_AMOUNT;
    MD_HIDE_S3_TOTAL_USAGE;
    MD_HIDE_S8_YUAN;
    MD_HIDE_S9_N;
    MD_HIDE_S10_M3;
}

static void hide_line1_all_unit(void)
{
    MD_HIDE_S11_STD_CONDITION;
    MD_HIDE_S13_N;
    MD_HIDE_S14_M3_H;
}

static void hide_line2_all_unit(void)
{
    MD_HIDE_S12_PRESS;
    MD_HIDE_S15_MPA;
    MD_HIDE_S16_KPA;
}

static void hide_line3_all_unit(void)
{
    MD_HIDE_S17_PRICE;
    MD_HIDE_S18_WORK_CONDITION;
    MD_HIDE_S19_YUAN;
    MD_HIDE_S21_M_3;
    MD_HIDE_S22_H;
}

static void hide_line4_all_unit(void)
{
    MD_HIDE_S20_TEMP;
    MD_HIDE_S23_CELSIUS;
}

void clr_lcd(void)
{
    // uint8_t data[29] = {0};
    // data[0] = 0x80;
    MD_05_PIN_LCD_SEG = 0;
    MD_06_PIN_LCD_SEG = 0;
    MD_07_PIN_LCD_SEG = 0;
    MD_08_PIN_LCD_SEG = 0;
    MD_09_PIN_LCD_SEG = 0;
    MD_10_PIN_LCD_SEG = 0;
    MD_11_PIN_LCD_SEG = 0;
    MD_12_PIN_LCD_SEG = 0;
    MD_13_PIN_LCD_SEG = 0;
    MD_14_PIN_LCD_SEG = 0;
    MD_15_PIN_LCD_SEG = 0;
    MD_16_PIN_LCD_SEG = 0;
    MD_17_PIN_LCD_SEG = 0;
    MD_18_PIN_LCD_SEG = 0;
    MD_19_PIN_LCD_SEG = 0;
    MD_20_PIN_LCD_SEG = 0;
    MD_21_PIN_LCD_SEG = 0;
    MD_22_PIN_LCD_SEG = 0;
    MD_23_PIN_LCD_SEG = 0;
    MD_24_PIN_LCD_SEG = 0;
    MD_25_PIN_LCD_SEG = 0;
    MD_26_PIN_LCD_SEG = 0;
    MD_27_PIN_LCD_SEG = 0;
    MD_28_PIN_LCD_SEG = 0;
    MD_29_PIN_LCD_SEG = 0;
    MD_30_PIN_LCD_SEG = 0;
    MD_31_PIN_LCD_SEG = 0;
    MD_32_PIN_LCD_SEG = 0;
    MD_33_PIN_LCD_SEG = 0;
    MD_34_PIN_LCD_SEG = 0;
    MD_35_PIN_LCD_SEG = 0;
    MD_36_PIN_LCD_SEG = 0;
    MD_37_PIN_LCD_SEG = 0;
    MD_38_PIN_LCD_SEG = 0;
    MD_39_PIN_LCD_SEG = 0;
    MD_40_PIN_LCD_SEG = 0;
    MD_41_PIN_LCD_SEG = 0;
    MD_42_PIN_LCD_SEG = 0;
    MD_43_PIN_LCD_SEG = 0;
    MD_44_PIN_LCD_SEG = 0;
    MD_45_PIN_LCD_SEG = 0;
    MD_46_PIN_LCD_SEG = 0;
    MD_47_PIN_LCD_SEG = 0;
    MD_48_PIN_LCD_SEG = 0;
    MD_49_PIN_LCD_SEG = 0;
    MD_50_PIN_LCD_SEG = 0;
    MD_51_PIN_LCD_SEG = 0;
    MD_52_PIN_LCD_SEG = 0;
    MD_53_PIN_LCD_SEG = 0;
    MD_54_PIN_LCD_SEG = 0;
    MD_55_PIN_LCD_SEG = 0;
    MD_56_PIN_LCD_SEG = 0;
    // I2C_MasterWriteData(MD_LCD_I2CX_SEL, data, sizeof(data));
}

static void nop(void)
{
    __NOP();
}

static int32_t isNum(int16_t dig)
{
    if (dig < 10)
    {
        return dig;
    }
    else
    {
        return 0;
    }
}

static int32_t get_line0_num(uint8_t *err)
{
    int32_t num = 0;
    if (hum_comps.dig0_0 == 0x0f)
    {
        *err = 1;
        return num;
    }
    num = isNum(hum_comps.dig0_7) * 10000000 + isNum(hum_comps.dig0_6) * 1000000 + isNum(hum_comps.dig0_5) * 100000 + isNum(hum_comps.dig0_4) * 10000 + isNum(hum_comps.dig0_3) * 1000 + isNum(hum_comps.dig0_2) * 100 + isNum(hum_comps.dig0_1) * 10 + isNum(hum_comps.dig0_0);
    return num;
}

static void hide_zero(uint8_t *const hi_addr, uint8_t total_width, uint8_t reserved_num) // Number of digits reserved
{
    int16_t i = 0;
    uint8_t *buf = hi_addr;
    for (i = 0; i < total_width - reserved_num; i++)
    {
        //		if(*(buf-i)>0 && *(buf-i)<10)
        //		{
        //			break;
        //		}
        //		else
        //		{
        //			*(buf-i)=MD_HIDE_DISP;
        //
        //		}
        if (*(buf - i) == 0)
        {

            *(buf - i) = MD_HIDE_DISP;
        }
        else
        {
            break;
        }
    }
}

static uint8_t Check_Sum_5A(const void *Data, uint8_t Len)
{
    uint8_t Sum = 0x5A;
    uint8_t i = 0;
    uint8_t *data = (uint8_t *)Data;
    for (i = 0; i < Len; i++)
    {
        Sum += data[i];
    }
    return Sum;
}

static uint8_t lg(const int32_t n)
{
    if (n == 1)
    {
        return 0;
    }
    if (n == 10)
    {
        return 1;
    }
    if (n == 100)
    {
        return 2;
    }
    if (n == 1000)
    {
        return 3;
    }
    return 1;
}

static int32_t pwr(int16_t n)
{
    if (n == 0)
    {
        return 1;
    }
    if (n == 1)
    {
        return 10;
    }
    if (n == 2)
    {
        return 100;
    }
    if (n == 3)
    {
        return 1000;
    }
    if (n == 4)
    {
        return 10000;
    }
    if (n == 5)
    {
        return 100000;
    }
    if (n == 6)
    {
        return 1000000;
    }
    if (n == 7)
    {
        return 10000000;
    }
    if (n == 8)
    {
        return 100000000;
    }
    if (n == 9)
    {
        return 1000000000;
    }
    return 1;
}

static float32_t f_mul(float32_t a, float32_t b)
{
    return a * b;
}
static float32_t f_div(float32_t a, float32_t b)
{
    return a / b;
}
static int32_t dis_range[][2] = {{-99999, 999999},
                                 {0, 9},
                                 {-9, 99},
                                 {-99, 999},
                                 {-999, 9999},
                                 {-9999, 99999},
                                 {-99999, 999999},
                                 {-999999, 9999999},
                                 {-9999999, 99999999},
                                 {-99999999, 999999999}};
static int32_t get_float_disp_num(float32_t f, int16_t dis_width, uint8_t *dot)
{
    int32_t num = 0;
    int16_t k;
    if (dis_width < 2)
    {
        num = f;
        *dot = 0;
        return num;
    }
    if (f < 0)
    {
        dis_width -= 1;
    }
    for (k = 0; k < dis_width - 1; k++)
    {
        if (fabs(f) < pwr(k + 1))
        {
            num = f * pwr(dis_width - k - 1);
            *dot = dis_width - k - 1;
            break;
        }
    }
    if (k == dis_width - 1)
    {
        num = f;
        *dot = 0;
    }
    return num;
}

static int32_t get_float_disp_coded_num(float32_t u) // fix width:8
{
    int32_t num = 0;
    float32_t f = fabs(u);
    if (u < 0.0f)
    {
        num = num + 10000000;
    }

    if ((f) < 0.001f)
    {
        num = num + (int32_t)(f * pwr(9)) + 9000000;
    }
    else if ((f) < 0.01f)
    {
        num = num + (int32_t)(f * pwr(8)) + 8000000;
    }
    else if ((f) < 0.1f)
    {
        num = num + (int32_t)(f * pwr(7)) + 7000000;
    }
    else if ((f) < 1.f)
    {
        num = num + (int32_t)(f * pwr(6)) + 6000000;
    }
    else if ((f) < 10.f)
    {
        num = num + (int32_t)(f * pwr(5)) + 5000000;
    }
    else if ((f) < 100.f)
    {
        num = num + (int32_t)(f * pwr(4)) + 4000000;
    }
    else if ((f) < 1000.f)
    {
        num = num + (int32_t)(f * pwr(3)) + 3000000;
    }
    else if ((f) < 10000.f)
    {
        num = num + (int32_t)(f * pwr(2)) + 2000000;
    }
    else if ((f) < 100000.f)
    {
        num = num + (int32_t)(f * pwr(1)) + 1000000;
    }
    else if ((f) < 1000000.f)
    {
        num = num + (int32_t)(f * pwr(0));
    }
    else
    {
        num = 0x7fffffff; // over range disp FFFFFF
    }

    return num;
}

static void calc_seg_value(uint8_t *dig, int16_t dis_width, int32_t num, int16_t isHideZero, int16_t reserved_num)
{
    int16_t i = 0;
    if (dis_width == 0)
    {
        dig[0] = 0x0e;
        return;
    }
    if (num < dis_range[dis_width][0] || num > dis_range[dis_width][1])
    {
        for (i = 0; i < dis_width; i++)
        {
            dig[i] = 0x0f;
        }
        return;
    }

    if (num < 0)
    {
        num = -num;
        dis_width -= 1;
        dig[dis_width] = MD_HIDE_DISP - 1;
    }
    for (i = 0; i < dis_width; i++)
    {
        dig[i] = num / pwr(i) % 10;
    }

    if (isHideZero)
    {
        hide_zero(dig + dis_width - 1, dis_width, reserved_num);
    }
}

static void lcd_test(hum_comps_t *const this)
{
    if (this->count == 0 * 6)
    {
        MD_DISPLAY_WRITE_D0_0(this->dig0_0); //
        MD_DISPLAY_WRITE_D1_0(this->dig1_0); //
        MD_DISPLAY_WRITE_D2_0(this->dig2_0); //
        MD_DISPLAY_WRITE_D3_0(this->dig3_0); //
        MD_DISPLAY_WRITE_D4_0(this->dig4_0); //
    }
    if (this->count == 1 * 6)
    {
        MD_DISPLAY_WRITE_D0_1(this->dig0_1); //
        MD_DISPLAY_WRITE_D1_1(this->dig1_1); //
        MD_DISPLAY_WRITE_D2_1(this->dig2_1); //
        MD_DISPLAY_WRITE_D3_1(this->dig3_1); //
        MD_DISPLAY_WRITE_D4_1(this->dig4_1); //
    }
    if (this->count == 2 * 6)
    {
        MD_DISPLAY_WRITE_D0_2(this->dig0_2); //
        MD_DISPLAY_WRITE_D1_2(this->dig1_2); //
        MD_DISPLAY_WRITE_D2_2(this->dig2_2); //
        MD_DISPLAY_WRITE_D3_2(this->dig3_2); //
        MD_DISPLAY_WRITE_D4_2(this->dig4_2); //
    }
    if (this->count == 3 * 6)
    {
        MD_DISPLAY_WRITE_D0_3(this->dig0_3); //
        MD_DISPLAY_WRITE_D1_3(this->dig1_3); //
        MD_DISPLAY_WRITE_D2_3(this->dig2_3); //
        MD_DISPLAY_WRITE_D3_3(this->dig3_3); //
                                             // MD_DISPLAY_WRITE_D4_3(this->dig4_3);//
    }
    if (this->count == 4 * 6)
    {
        MD_DISPLAY_WRITE_D0_4(this->dig0_4); //
                                             //		MD_DISPLAY_WRITE_D1_4(this->dig1_4);//
                                             //		MD_DISPLAY_WRITE_D2_4(this->dig2_4);//
                                             //		MD_DISPLAY_WRITE_D3_4(this->dig3_4);//
                                             //		MD_DISPLAY_WRITE_D4_4(this->dig4_4);//
                                             //		MD_DISPLAY_WRITE_D5_4(this->dig5_4);//
    }
    if (this->count == 5 * 6)
    {
        MD_DISPLAY_WRITE_D0_5(this->dig0_5); //
                                             // MD_DISPLAY_WRITE_D1_5(this->dig1_5);//
        // MD_DISPLAY_WRITE_D2_5(this->dig2_5);//
        // MD_DISPLAY_WRITE_D3_5(this->dig3_5);//
        // MD_DISPLAY_WRITE_D4_5(this->dig4_5);//
        // MD_DISPLAY_WRITE_D5_5(this->dig5_5);//
    }
    if (this->count == 6 * 6)
    {
        MD_DISPLAY_WRITE_D0_6(this->dig0_6); //
                                             //		MD_DISPLAY_WRITE_D1_6(this->dig1_6);//
                                             //		MD_DISPLAY_WRITE_D2_6(this->dig2_6);//
                                             //		MD_DISPLAY_WRITE_D3_6(this->dig3_6);//
                                             //		MD_DISPLAY_WRITE_D4_6(this->dig4_6);//
                                             //		MD_DISPLAY_WRITE_D5_6(this->dig5_6);//
    }
    if (this->count == 7 * 6)
    {
        MD_DISPLAY_WRITE_D0_7(this->dig0_7); //
                                             //		MD_DISPLAY_WRITE_D1_7(this->dig1_7);//
                                             //		MD_DISPLAY_WRITE_D2_7(this->dig2_7);//
                                             //		MD_DISPLAY_WRITE_D3_7(this->dig3_7);//
                                             //		MD_DISPLAY_WRITE_D4_7(this->dig4_7);//
                                             //		MD_DISPLAY_WRITE_D5_7(this->dig5_7);//
    }
    if (this->count == 8 * 6)
    {
        //		MD_DISPLAY_WRITE_D0_8(this->dig0_8);//
        //		MD_DISPLAY_WRITE_D1_8(this->dig1_8);//
        //		MD_DISPLAY_WRITE_D2_8(this->dig2_8);//
        //		MD_DISPLAY_WRITE_D3_8(this->dig3_8);//
        //		MD_DISPLAY_WRITE_D4_8(this->dig4_8);//
        //		MD_DISPLAY_WRITE_D5_8(this->dig5_8);//
    }
    if (this->count == 9 * 6)
    {
        dis_all_symbol();
    }
    if (this->count == 10 * 6)
    {
        clr_lcd();
        MD_DISPLAY_WRITE_D0_4(MD_DIS_r);             // R
        MD_DISPLAY_WRITE_D0_3(5);                    // S
        MD_DISPLAY_WRITE_D0_2(MD_DIS_t);             // T
        MD_DISPLAY_WRITE_D0_1(MD_HIDE_DISP - 1);     //-
        MD_DISPLAY_WRITE_D0_0(systemComps.rst_code); //-
    }
}

/////////////////////////////////////mode enter and exit//////////////////

#define BK_LN

static void enter_normal_mode(void)
{
    clr_lcd();
    hum_comps.current_mode = EM_NORMAL_MODE;
    mode_comps[hum_comps.current_mode].dis_option = 0;
    hum_comps.dis_oper_mark._bit.cur0 = 0;
    hum_comps.dis_oper_mark._bit.cur1 = 0;
    hum_comps.dis_oper_mark._bit.cur2 = 0;
    hum_comps.dis_oper_mark._bit.cur3 = 0;
    hum_comps.dis_oper_mark._bit.cur4 = 0;
    hum_comps.dis_oper_mark._bit.cur5 = 0;
    hum_comps.dis_oper_mark._bit.refressh_meter_data = 1;
 
}

static void enter_default_mode(int16_t opt)
{
    enter_normal_mode();
}
static void enter_debug_mode(void)
{
    hum_comps.current_mode = EM_DEBUG_MODE;
    hum_comps.dis_oper_mark._bit.cur0 = 0;
    hum_comps.dis_oper_mark._bit.cur1 = 0;
    hum_comps.dis_oper_mark._bit.cur2 = 0;
    hum_comps.dis_oper_mark._bit.cur3 = 0;
    hum_comps.dis_oper_mark._bit.cur4 = 0;
    hum_comps.dis_oper_mark._bit.cur5 = 0;
    mode_comps[hum_comps.current_mode].displayTimer = (uint8_t)-1;
    mode_comps[hum_comps.current_mode].dis_option = 0;
    hum_comps.dis_oper_mark._bit.refresh_debug_param = 1;
    clr_lcd();
}

static void enter_pawd_mode(void)
{
    hum_comps.current_mode = EM_PWD_MODE;
    hum_comps.dig0_0 = 0;
    hum_comps.dig0_1 = 0;
    hum_comps.dig0_2 = 0;
    hum_comps.dig0_3 = 0;
    hum_comps.dig0_4 = 0;
    hum_comps.dig0_5 = 0;
    hum_comps.dig0_6 = MD_HIDE_DISP;
    hum_comps.dig0_7 = MD_HIDE_DISP;
    hum_comps.cursor_0 = 5;
    hum_comps.dot0_pos = 0;
    hum_comps.cursor_0_count = 0;
    hum_comps.dis_oper_mark._bit.cur0 = 1;
    hum_comps.dis_oper_mark._bit.dis0 = 0;
    clr_lcd();
}

static void enter_cal_query_mode(int16_t opt)
{
    hum_comps.current_mode = EM_CAL_QUERY_MODE;
    mode_comps[hum_comps.current_mode].dis_option = opt;
    hum_comps.dis_oper_mark._bit.cur0 = 0;
    hum_comps.dis_oper_mark._bit.refresh_cal_param = 1;
    clr_lcd();
}

static void enter_cal_modify_mode(int16_t opt)
{

    hum_comps.current_mode = EM_CAL_MODIFY_MODE;
    mode_comps[hum_comps.current_mode].dis_option = opt;
    if (device_comps.cal_type == EM_CAL_PRESS || device_comps.cal_type == EM_CAL_RES)
    {
        hum_comps.cursor_0 = 5;
        hum_comps.dis_oper_mark._bit.cur0 = 1;
    }
    else
    {
        hum_comps.cursor_0 = 7;
        hum_comps.dis_oper_mark._bit.cur0 = 1;
    }
    clr_lcd();
}

static void enter_param_query_mode(int16_t opt)
{

    hum_comps.current_mode = EM_PARAM_QUERY_MODE;
    mode_comps[hum_comps.current_mode].dis_option = opt;
    hum_comps.dis_oper_mark._bit.refresh_param = 1;
    hum_comps.dis_oper_mark._bit.cur0 = 0;
    clr_lcd();
}

static void enter_param_modify_mode(int16_t opt)
{

    hum_comps.current_mode = EM_PARAM_MODIFY_MODE;
    mode_comps[hum_comps.current_mode].dis_option = opt;
    if (device_comps.param_type == EM_PARAM_USER || device_comps.param_type == EM_PARAM_FACTORY)
    {
        hum_comps.cursor_0 = 5;
        hum_comps.dis_oper_mark._bit.cur0 = 1;
    }
    else
    {
        hum_comps.cursor_0 = 7;
        hum_comps.dis_oper_mark._bit.cur0 = 1;
    }
    // clr_lcd();
}

static void enter_self_test_mode(void)
{
    hum_comps.current_mode = EM_SELF_TEST_MODE;
    hum_comps.dis_oper_mark._bit.cur0 = 0;
    hum_comps.dis_oper_mark._bit.cur1 = 0;
    hum_comps.dis_oper_mark._bit.cur2 = 0;
    hum_comps.dis_oper_mark._bit.cur3 = 0;
    hum_comps.dis_oper_mark._bit.cur4 = 0;
    hum_comps.dis_oper_mark._bit.cur5 = 0;
    clr_lcd();
}

static void enter_report_mode(void)
{

    hum_comps.current_mode = EM_REPORT_MODE;
    mode_comps[hum_comps.current_mode].dis_option = 0;
    hum_comps.dis_oper_mark._bit.cur0 = 0;
    hum_comps.dis_oper_mark._bit.cur1 = 0;
    hum_comps.dis_oper_mark._bit.cur2 = 0;
    hum_comps.dis_oper_mark._bit.cur3 = 0;
    hum_comps.dis_oper_mark._bit.cur4 = 0;
    hum_comps.dis_oper_mark._bit.cur5 = 0;
    mode_comps[hum_comps.current_mode].displayTimer = 0;
    clr_lcd();
}

/*********************************start  key function************************************************************/

#define BK_LN

static void normal_mode_on_s_key(void)
{
    boolean_t st = GetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BACK_LED_PORT), MD_BACK_LED_PIN);
    SetBit(((uint32_t)&M0P_GPIO->PAOUT + MD_BACK_LED_PORT), MD_BACK_LED_PIN, (boolean_t)!st);
    if (st == FALSE)
    {
        hum_comps.back_led_timer = device_comps.coe.back_led_on_time;
    }
    else
    {
        hum_comps.back_led_timer = 0;
    }
}

static void normal_mode_on_m_key(void)
{
    mode_comps[hum_comps.current_mode].dis_option++;
    mode_comps[hum_comps.current_mode].displayTimer = (uint8_t)-1;
    hum_comps.dis_oper_mark._bit.refressh_meter_data = 1;
    hum_comps.dis_oper_mark._bit.refresh_normal_mode_info = 1;
    hum_comps.dis_oper_mark._bit.refressh_broken_date = 1;
}

static void normal_mode_on_j_key(void)
{
}

static void normal_mode_on_long_s_key(void)
{
    protocolComps.triggerIrq._bit.key = 1;
    // device_comps.buzzer.start(61);
}

static void normal_mode_on_long_m_key(void)
{
    enter_pawd_mode();
}

static void normal_mode_on_long_j_key(void)
{
    // device_comps.sw._bit.com_key_en=!device_comps.sw._bit.com_key_en;
}

static void normal_mode_on_long_s_and_j_key(void)
{
    enter_debug_mode();
}

static void debug_mode_on_s_key(void)
{
}
static void debug_mode_on_m_key(void)
{
    mode_comps[hum_comps.current_mode].dis_option++;
    hum_comps.dis_oper_mark._bit.refresh_debug_param = 1;
}
static void debug_mode_on_j_key(void)
{
}
static void debug_mode_on_long_m_key(void)
{
}
static void debug_mode_on_long_s_and_j_key(void)
{
    enter_default_mode(0);
}

static void pwd_mode_on_s_key(void)
{
    hum_comps.cursor_0 += 5;
    hum_comps.cursor_0 %= 6; // 4->3->2...>0
}
static void pwd_mode_on_m_key(void)
{
    int32_t pwd = isNum(hum_comps.dig0_2) * 100 + isNum(hum_comps.dig0_1) * 10 + isNum(hum_comps.dig0_0);
    if (pwd == 235)
    {
        device_comps.cal_type = (cal_type_t)hum_comps.dig0_5;
        if (device_comps.cal_type == EM_CAL_FLOW) // dig3_5 cal_type 0:press 1:pt 4, flow_meter
        {
        }
        if (device_comps.cal_type == EM_CAL_PRESS)
        {
            device_comps.press_cal_param_bak.dot = hum_comps.dig0_4;
            device_comps.press_cal_param_bak.unit = hum_comps.dig0_3;
            if (device_comps.press_cal_param_bak.dot < 3 || device_comps.press_cal_param_bak.dot > 5 || device_comps.press_cal_param_bak.unit > 1)
            {
                return;
            }
        }
        else if (device_comps.cal_type == EM_CAL_RES)
        {
            device_comps.res_cal_param_bak.dot = 3;
            device_comps.res_cal_param_bak.unit = hum_comps.dig0_3;
        }
        if (device_comps.cal_type == EM_CAL_FLOW || device_comps.cal_type == EM_CAL_PRESS || device_comps.cal_type == EM_CAL_RES)
        {
            enter_cal_query_mode(0);
        }
    }

    else if (pwd == 237)
    {
        if (hum_comps.dig0_5 == 1 && hum_comps.dig0_4 == 1 && hum_comps.dig0_3 == 1)
        {
            device_comps.param_type = EM_PARAM_USER;
            enter_param_query_mode(0);
        }
        else if (hum_comps.dig0_5 == 3 && hum_comps.dig0_4 == 2 && hum_comps.dig0_3 == 7)
        {
            device_comps.param_type = EM_PARAM_FACTORY;
            enter_param_query_mode(0);
        }
    }
    else if (pwd == 357) //
    {
        if (hum_comps.dig0_5 == 0 && hum_comps.dig0_4 == 0 && hum_comps.dig0_3 == 0)
        {
            device_comps.param_type = EM_PARAM_USER_A;
            enter_param_query_mode(0);
        }
    }
}
static void pwd_mode_on_j_key(void)
{
    *(&hum_comps.dig0_0 + hum_comps.cursor_0) = (*(&hum_comps.dig0_0 + hum_comps.cursor_0) + 1) % 10;
}
static void pwd_mode_on_long_m_key(void)
{
    enter_default_mode(0);
}
static void pwd_mode_on_long_s_and_j_key(void)
{
}

static void cal_query_mode_on_s_key(void)
{
    enter_cal_modify_mode(mode_comps[hum_comps.current_mode].dis_option);
}

static void cal_query_mode_on_m_key(void)
{
    mode_comps[hum_comps.current_mode].dis_option++;
    hum_comps.dis_oper_mark._bit.refresh_cal_param = 1;
}
static void cal_query_mode_on_j_key(void)
{
}

static void cal_query_mode_on_long_m_key(void)
{
    enter_default_mode(0);
}

static void cal_modify_mode_on_s_key(void)
{
    int16_t opt = mode_comps[hum_comps.current_mode].dis_option;
    if (device_comps.cal_type == EM_CAL_PRESS || device_comps.cal_type == EM_CAL_RES)
    {
        if (1)
        {
            hum_comps.cursor_0 += 5;
            hum_comps.cursor_0 %= 6; // 5->4->3->2...>0
        }
    }
    else if (device_comps.cal_type == EM_CAL_FLOW)
    {
        hum_comps.cursor_0 += 7;
        hum_comps.cursor_0 %= 8; // 7->6...->3->2...>0
    }
}

static void cal_modify_mode_on_m_key(void)
{
    int32_t temp;
    char *ptsz;
    int16_t opt = mode_comps[hum_comps.current_mode].dis_option;
    enum
    {
        EM_NULL = 0,
        EM_FLOW_METER,
        EM_MODBUS,
        EM_LORA,
        EM_ACCESS,
        EM_REPORT,
        EM_METER,
        EM_COE,
        EM_GPS,
        EM_MISC,
        EM_SYSTEM_TIME,
        EM_FLOW_CAL_PARAM
    } save_data_type = EM_NULL;
    if (device_comps.cal_type == EM_CAL_FLOW)
    {
        uint8_t err = 0;
        int16_t dot = hum_comps.dig0_6;
        float32_t sgn = (hum_comps.dig0_7 > 0) ? (-1.0f) : (1.0f);
        int32_t num = get_line0_num(&err) % 1000000;
        if (err > 0)
        {
            return;
        }
        float32_t fnum = f_div(sgn * num, pwr(dot));
        switch (opt)
        {
        case 0:
            device_comps.flow_cal_param.freq_divd_pos = num % (MD_FLOW_MAX_CAL_POS + 1); //;//
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 1:
            device_comps.flow_cal_param.freq_poly_coe[0] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 2:
            device_comps.flow_cal_param.freq_poly_coe[1] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 3:
            device_comps.flow_cal_param.freq_poly_coe[2] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 4:
            device_comps.flow_cal_param.freq_poly_coe[3] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 5:
            device_comps.flow_cal_param.freq_poly_coe[4] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;

        case 6:
            device_comps.flow_cal_param.meter_coe = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            ;
            break;
        case 7:
            device_comps.flow_cal_param.RefDN = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            ;
            break;
        case 8:
            device_comps.flow_cal_param.RefMaxQ = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            ;
            break;
        case 9:
            device_comps.flow_cal_param.calc_mode = num % 4;
            save_data_type = EM_FLOW_CAL_PARAM;
            ;
            break;
        case 10:
            device_comps.flow_cal_param.freq_out_mode = num % 2;
            save_data_type = EM_FLOW_CAL_PARAM;
            ;
            break;

        case 11:
            device_comps.flow_cal_param.freq_divd_value[0] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 12:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[0] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 13:
            device_comps.flow_cal_param.freq_divd_value[1] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 14:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[1] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 15:
            device_comps.flow_cal_param.freq_divd_value[2] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 16:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[2] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 17:
            device_comps.flow_cal_param.freq_divd_value[3] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 18:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[3] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 19:
            device_comps.flow_cal_param.freq_divd_value[4] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 20:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[4] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 21:
            device_comps.flow_cal_param.freq_divd_value[5] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 22:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[5] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 23:
            device_comps.flow_cal_param.freq_divd_value[6] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 24:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[6] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 25:
            device_comps.flow_cal_param.freq_divd_value[7] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 26:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[7] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 27:
            device_comps.flow_cal_param.freq_divd_value[8] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 28:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[8] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 29:
            device_comps.flow_cal_param.freq_divd_value[9] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 30:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[9] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;

        case 31:
            device_comps.flow_cal_param.freq_divd_value[10] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 32:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[10] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 33:
            device_comps.flow_cal_param.freq_divd_value[11] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 34:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[11] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 35:
            device_comps.flow_cal_param.freq_divd_value[12] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 36:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[12] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 37:
            device_comps.flow_cal_param.freq_divd_value[13] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 38:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[13] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 39:
            device_comps.flow_cal_param.freq_divd_value[14] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 40:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[14] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 41:
            device_comps.flow_cal_param.freq_divd_value[15] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 42:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[15] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 43:
            device_comps.flow_cal_param.freq_divd_value[16] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 44:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[16] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 45:
            device_comps.flow_cal_param.freq_divd_value[17] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 46:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[17] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 47:
            device_comps.flow_cal_param.freq_divd_value[18] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 48:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[18] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 49:
            device_comps.flow_cal_param.freq_divd_value[19] = fnum * 10;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        case 50:
            device_comps.flow_cal_param.freq_divd_value_meter_coe[19] = fnum;
            save_data_type = EM_FLOW_CAL_PARAM;
            break;
        default:
            break;
        }
    }
    if (save_data_type == EM_NULL)
    {
        return;
    }
    switch (save_data_type)
    {
    case EM_FLOW_METER:
        device_comps.flow_meter.cs = Check_Sum_5A(&device_comps.flow_meter, &device_comps.flow_meter.cs - (uint8_t *)&device_comps.flow_meter);
        device_comps.save_flow_meter(&device_comps.flow_meter, sizeof(device_comps.flow_meter));
        break;
    case EM_MODBUS:
        modbusComps.save_param();
        break;
    case EM_LORA:

        loraComps.save_cfg_info();
        break;
    case EM_ACCESS:
        device_comps.access_param.cs = Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs - (uint8_t *)&device_comps.access_param);
        device_comps.save_access_param(&device_comps.access_param, sizeof(device_comps.access_param));
        break;
    case EM_REPORT:
        device_comps.report_interval_timer = 0;
        device_comps.report_param.cs = Check_Sum_5A(&device_comps.report_param, &device_comps.report_param.cs - (uint8_t *)&device_comps.report_param);
        device_comps.save_report_param(&device_comps.report_param, sizeof(device_comps.report_param));
        break;
    case EM_METER:
        device_comps.meter.cs = Check_Sum_5A(&device_comps.meter, &device_comps.meter.cs - (uint8_t *)&device_comps.meter);
        device_comps.save_meter(&device_comps.meter, sizeof(device_comps.meter));
        break;
    case EM_COE:
        device_comps.coe.cs = Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs - (uint8_t *)&device_comps.coe);
        device_comps.save_coe(&device_comps.coe, sizeof(device_comps.coe));
        break;
    case EM_MISC:
        device_comps.misc_param.cs = Check_Sum_5A(&device_comps.misc_param, &device_comps.misc_param.cs - (uint8_t *)&device_comps.misc_param);
        device_comps.save_misc_param(&device_comps.misc_param, sizeof(device_comps.misc_param));
        break;
    case EM_SYSTEM_TIME:
        device_comps.system_time.cs = Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs - (uint8_t *)&device_comps.system_time);
        device_comps.save_system_time(&device_comps.system_time, sizeof(device_comps.system_time));
        device_comps.set_system_time(&device_comps.system_time.time);
        ertc_comps.write_broken_time(&device_comps.system_time.time);
    case EM_FLOW_CAL_PARAM:
        device_comps.flow_cal_param.cs = Check_Sum_5A(&device_comps.flow_cal_param, &device_comps.flow_cal_param.cs - (uint8_t *)&device_comps.flow_cal_param);
        device_comps.save_flow_cal_param(&device_comps.flow_cal_param, sizeof(device_comps.flow_cal_param));
    default:
        break;
    }
    enter_cal_query_mode(opt);
}
static void cal_modify_mode_on_j_key(void)
{
    int16_t opt = mode_comps[hum_comps.current_mode].dis_option;
    if (device_comps.cal_type == EM_CAL_PRESS || device_comps.cal_type == EM_CAL_RES)
    {
        *(&hum_comps.dig0_0 + hum_comps.cursor_0) = (*(&hum_comps.dig0_0 + hum_comps.cursor_0) + 1) % 10;
    }
    else if (device_comps.cal_type == EM_CAL_FLOW)
    {
        if (*(&hum_comps.dig0_0 + hum_comps.cursor_0) < 10)
        {
            *(&hum_comps.dig0_0 + hum_comps.cursor_0) = (*(&hum_comps.dig0_0 + hum_comps.cursor_0) + 1) % 10;
        }
    }
}
static void cal_modify_mode_on_long_s_key(void)
{
    enter_default_mode(0);
}

static void param_query_mode_on_s_key(void)
{
    enter_param_modify_mode(mode_comps[hum_comps.current_mode].dis_option);
}

static void param_query_mode_on_m_key(void)
{
    mode_comps[hum_comps.current_mode].dis_option++;
    hum_comps.dis_oper_mark._bit.refresh_param = 1;
}

static void param_query_mode_on_long_m_key(void)
{
    enter_default_mode(0);
}

static void param_modify_mode_on_s_key(void)
{
    int16_t opt = mode_comps[hum_comps.current_mode].dis_option;
    if (device_comps.param_type == EM_PARAM_USER || device_comps.param_type == EM_PARAM_FACTORY)
    {
        if (1)
        {
            hum_comps.cursor_0 += 5;
            hum_comps.cursor_0 %= 6; // 4->3->2...>0
        }
    }
    else if (device_comps.param_type == EM_PARAM_USER_A)
    {
        hum_comps.cursor_0 += 7;
        hum_comps.cursor_0 %= 8; // 4->3->2...>0
    }
}

static void param_modify_mode_on_m_key(void)
{
    int32_t temp;
    char *ptsz;
    int16_t opt = mode_comps[hum_comps.current_mode].dis_option;

    enum
    {
        EM_NULL = 0,
        EM_FLOW_METER,
        EM_MODBUS,
        EM_LORA,
        EM_ACCESS,
        EM_REPORT,
        EM_METER,
        EM_COE,
        EM_GPS,
        EM_MISC,
        EM_SYSTEM_TIME
    } save_data_type = EM_NULL;
    if (device_comps.param_type == EM_PARAM_USER)
    {
        uint8_t err;
        int32_t num = get_line0_num(&err) % 1000000;
        switch (opt)
        {
        case 0:
            device_comps.flow_meter.unit = num;
            save_data_type = EM_FLOW_METER;
            break;
        case 1:
            device_comps.misc_param.I_o_low = num * 100;
            save_data_type = EM_MISC;
            break;
        case 2:
            device_comps.misc_param.I_o_high = num * 100;
            save_data_type = EM_MISC;
            break;
        case 3:
            device_comps.misc_param.I_o_dir = num;
            save_data_type = EM_MISC;
            break;
        case 4:
            device_comps.flow_meter.pluse_equ = num; // 0.001m^3 std unit
            save_data_type = EM_FLOW_METER;
            break;
        case 5:                                        //
            device_comps.flow_meter.pluse_width = num; // 0.1ms
            save_data_type = EM_FLOW_METER;
            break;
        case 6:
            device_comps.misc_param.density = num;
            save_data_type = EM_MISC;
            break;
        case 7: //
            modbusComps.param_pt->addr = num;
            save_data_type = EM_MODBUS;
            break;
        case 8: //
            modbusComps.param_pt->baud = num % 4;
            modbusComps.modify_baud(modbusComps.param_pt->baud, 0);
            save_data_type = EM_MODBUS;
            break;
        case 9: //
            loraComps.cfg_info_p->freq = num;
            save_data_type = EM_LORA;
            break;
        case 10: //
            loraComps.cfg_info_p->netId = num;
            save_data_type = EM_LORA;
            break;
        case 11: //
            loraComps.cfg_info_p->nodeId &= 0x0000ffff;
            loraComps.cfg_info_p->nodeId |= (num % 65536) << 16;
            save_data_type = EM_LORA;
            break;
        case 12: //
            loraComps.cfg_info_p->nodeId &= 0xffff0000;
            loraComps.cfg_info_p->nodeId |= (num % 65536);
            save_data_type = EM_LORA;
            break;
        case 13: //
            ptsz = strchr((char *)device_comps.access_param.ip, '.');
            ptsz = strchr(ptsz + 1, '.');
            temp = strtol(ptsz + 1, &ptsz, 10);
            temp *= 1000;
            temp = temp + strtol(ptsz + 1, &ptsz, 10);
            memset(device_comps.access_param.ip, 0, sizeof(device_comps.access_param.ip));
            sprintf(device_comps.access_param.ip, "%d.", (int)num / 1000);
            sprintf(device_comps.access_param.ip + strlen(device_comps.access_param.ip), "%d.", (int)num % 1000);
            sprintf(device_comps.access_param.ip + strlen(device_comps.access_param.ip), "%d.", (int)temp / 1000);
            sprintf(device_comps.access_param.ip + strlen(device_comps.access_param.ip), "%d", (int)temp % 1000);
            save_data_type = EM_ACCESS;
            break;
        case 14:
            temp = strtol((char *)device_comps.access_param.ip, &ptsz, 10);
            temp *= 1000;
            temp = temp + strtol(ptsz + 1, &ptsz, 10);
            memset(device_comps.access_param.ip, 0, sizeof(device_comps.access_param.ip));
            sprintf(device_comps.access_param.ip, "%d.", (int)temp / 1000);
            sprintf(device_comps.access_param.ip + strlen(device_comps.access_param.ip), "%d.", (int)temp % 1000);
            sprintf(device_comps.access_param.ip + strlen(device_comps.access_param.ip), "%d.", (int)num / 1000);
            sprintf(device_comps.access_param.ip + strlen(device_comps.access_param.ip), "%d", (int)num % 1000);
            save_data_type = EM_ACCESS;
            break;
        case 15:
            device_comps.access_param.port = num;
            save_data_type = EM_ACCESS;
            break;
        case 16:
            device_comps.report_param.u16Minute_Interval = num;
            save_data_type = EM_REPORT;
            break;
        case 17:
            device_comps.system_time.time.u8Year = DEC2BCD(num / 10000);
            device_comps.system_time.time.u8Month = DEC2BCD(num / 100 % 100);
            device_comps.system_time.time.u8Day = DEC2BCD(num % 100);
            save_data_type = EM_SYSTEM_TIME;
            break;
        case 18:
            device_comps.system_time.time.u8Hour = DEC2BCD(num / 10000);
            device_comps.system_time.time.u8Minute = DEC2BCD(num / 100 % 100);
            device_comps.system_time.time.u8Second = DEC2BCD(num % 100);
            save_data_type = EM_SYSTEM_TIME;
            //
        default:
            break;
        }
    }
    else if (device_comps.param_type == EM_PARAM_USER_A)
    {
        uint8_t err = 0;
        int32_t num = get_line0_num(&err);
        switch (opt)
        {
        case 0:
            device_comps.meter_backup.total_int = device_comps.meter.total_int = num;
            device_comps.meter_backup.total_intN = device_comps.meter.total_intN = num;
            save_data_type = EM_METER;
            break;
        case 1:
            device_comps.meter_backup.total_int = device_comps.meter.total_int = device_comps.meter.total_int + num / 1000000;
            device_comps.meter_backup.total_dec = device_comps.meter.total_dec = f_div(num % 1000000, 1000.f); // unit:L
            device_comps.meter_backup.total_intN = device_comps.meter.total_intN = device_comps.meter.total_int + num / 1000000;
            device_comps.meter_backup.total_decN = device_comps.meter.total_decN = f_div(num % 1000000, 1000.f); // unit:L
            save_data_type = EM_METER;
            break;
        default:

            break;
        }
    }
    else if (device_comps.param_type == EM_PARAM_FACTORY)
    {
        uint8_t err;
        int32_t num = get_line0_num(&err) % 1000000;
        switch (opt)
        {
        case 0:
            device_comps.coe.flow = num;
            save_data_type = EM_COE;
            break;

        case 1:
            device_comps.coe.out_4_20ma = num;
            save_data_type = EM_COE;
            break;
        case 2:
            device_comps.coe.pt_temp = num;
            save_data_type = EM_COE;
            break;
        case 3:
            device_comps.coe.press = num;
            save_data_type = EM_COE;
            break;
        case 4:
            device_comps.gps.sel = num;
            save_data_type = EM_GPS;
            break;

        case 5:
            device_comps.flow_meter.sensor_low_freq_cutoff = num;
            save_data_type = EM_FLOW_METER;
            break;
        case 6:
            num = device_comps.flow_meter.avg_freq_filter_timer = num;
            save_data_type = EM_FLOW_METER;
            break;
        case 7:
            device_comps.coe._4ma_raw_value = num;
            save_data_type = EM_COE;
            break;
        case 8:
            device_comps.coe._20ma_raw_value = num;
            save_data_type = EM_COE;
            break;
        default:
            break;
        }
    }
    if (save_data_type == EM_NULL)
    {
        return;
    }
    switch (save_data_type)
    {
    case EM_FLOW_METER:
        device_comps.flow_meter.cs = Check_Sum_5A(&device_comps.flow_meter, &device_comps.flow_meter.cs - (uint8_t *)&device_comps.flow_meter);
        device_comps.save_flow_meter(&device_comps.flow_meter, sizeof(device_comps.flow_meter));
        break;
    case EM_MODBUS:
        modbusComps.save_param();
        break;
    case EM_LORA:

        loraComps.save_cfg_info();
        loraComps.sw._bit.param_modified = 1;
        break;
    case EM_ACCESS:
        device_comps.access_param.cs = Check_Sum_5A(&device_comps.access_param, &device_comps.access_param.cs - (uint8_t *)&device_comps.access_param);
        device_comps.save_access_param(&device_comps.access_param, sizeof(device_comps.access_param));
        break;
    case EM_REPORT:
        device_comps.report_interval_timer = 0;
        device_comps.report_param.cs = Check_Sum_5A(&device_comps.report_param, &device_comps.report_param.cs - (uint8_t *)&device_comps.report_param);
        device_comps.save_report_param(&device_comps.report_param, sizeof(device_comps.report_param));
        break;
    case EM_METER:
        device_comps.meter.cs = Check_Sum_5A(&device_comps.meter, &device_comps.meter.cs - (uint8_t *)&device_comps.meter);
        device_comps.save_meter(&device_comps.meter, sizeof(device_comps.meter));
        device_comps.meter_backup.cs = Check_Sum_5A(&device_comps.meter_backup, &device_comps.meter_backup.cs - (uint8_t *)&device_comps.meter_backup);
        device_comps.save_meter_backup(&device_comps.meter_backup, sizeof(device_comps.meter_backup));
        break;
    case EM_COE:
        device_comps.coe.cs = Check_Sum_5A(&device_comps.coe, &device_comps.coe.cs - (uint8_t *)&device_comps.coe);
        device_comps.save_coe(&device_comps.coe, sizeof(device_comps.coe));
        break;
    case EM_GPS:
        device_comps.gps.cs = Check_Sum_5A(&device_comps.gps, &device_comps.gps.cs - (uint8_t *)&device_comps.gps);
        device_comps.save_gps_param(&device_comps.gps, sizeof(device_comps.gps));
        break;
    case EM_MISC:
        device_comps.misc_param.cs = Check_Sum_5A(&device_comps.misc_param, &device_comps.misc_param.cs - (uint8_t *)&device_comps.misc_param);
        device_comps.save_misc_param(&device_comps.misc_param, sizeof(device_comps.misc_param));
        break;
    case EM_SYSTEM_TIME:
        device_comps.system_time.cs = Check_Sum_5A(&device_comps.system_time, &device_comps.system_time.cs - (uint8_t *)&device_comps.system_time);
        device_comps.save_system_time(&device_comps.system_time, sizeof(device_comps.system_time));
        device_comps.set_system_time(&device_comps.system_time.time);
        ertc_comps.write_broken_time(&device_comps.system_time.time);
    default:
        break;
    }
    enter_param_query_mode(opt);
}

static void param_modify_mode_on_j_key(void)
{
    int16_t opt = mode_comps[hum_comps.current_mode].dis_option;

    if (device_comps.param_type == EM_PARAM_USER || device_comps.param_type == EM_PARAM_FACTORY)
    {
        *(&hum_comps.dig0_0 + hum_comps.cursor_0) = (*(&hum_comps.dig0_0 + hum_comps.cursor_0) + 1) % 10;
    }
    else if (device_comps.param_type == EM_PARAM_USER_A)
    {
        *(&hum_comps.dig0_0 + hum_comps.cursor_0) = (*(&hum_comps.dig0_0 + hum_comps.cursor_0) + 1) % 10;
    }
}

static void param_modify_mode_on_long_s_key(void)
{
    enter_default_mode(0);
}

static void on_s_key(void)
{
    mode_comps[hum_comps.current_mode].on_s_key();
}
static void on_m_key(void)
{
    mode_comps[hum_comps.current_mode].on_m_key();
}
static void on_j_key(void)
{
    mode_comps[hum_comps.current_mode].on_j_key();
}
static void on_long_s_key(void)
{
    mode_comps[hum_comps.current_mode].on_long_s_key();
}
static void on_long_m_key(void)
{
    mode_comps[hum_comps.current_mode].on_long_m_key();
}
static void on_long_j_key(void)
{
    mode_comps[hum_comps.current_mode].on_long_j_key();
}
static void on_long_s_and_j_key(void)
{
    mode_comps[hum_comps.current_mode].on_long_s_and_j_key();
}

///////////////////////END KEY FUNCTION////////////////////

//////////////START DISPLAY///////////////////////////////

#define BK_LN

static void do_cursor0_count(int16_t times)
{
    hum_comps.cursor_0_count++;
    if (hum_comps.cursor_0_count > times)
    {
        hum_comps.cursor_0_count = 0;
        hum_comps.dis_oper_mark._bit.dis0 = !hum_comps.dis_oper_mark._bit.dis0;
    }
}

static void do_cursor3_count(int16_t times)
{
    hum_comps.cursor_3_count++;
    if (hum_comps.cursor_3_count > times)
    {
        hum_comps.cursor_3_count = 0;
        hum_comps.dis_oper_mark._bit.dis3 = !hum_comps.dis_oper_mark._bit.dis3;
    }
}
static void do_cursor5_count(int16_t times)
{
    hum_comps.cursor_5_count++;
    if (hum_comps.cursor_5_count > times)
    {
        hum_comps.cursor_5_count = 0;
        hum_comps.dis_oper_mark._bit.dis5 = !hum_comps.dis_oper_mark._bit.dis5;
    }
}

static void display_opt(int16_t opt)
{
    hum_comps.dig4_0 = opt % 10;
    hum_comps.dig4_1 = opt / 10 % 10;
    hum_comps.dig4_2 = MD_HIDE_DISP;
    hum_comps.dot4_pos = 0;
    hide_line4_all_unit();
    display_line4_data();
}

static void display_temp_adc(int16_t line)
{
    int32_t num;
    if (line == 3)
    {
        num = device_comps.temp_p_temp_n_average_result;
        hum_comps.dot3_pos = 0;
        calc_seg_value(&hum_comps.dig3_0, 6, num, 0, hum_comps.dot3_pos + 1);

        if (device_comps.sw._bit.temp_adc_stb)
        {
            hum_comps.dis_oper_mark._bit.cur3 = 0;
        }
        else
        {
            hum_comps.dis_oper_mark._bit.cur3 = 1;
        }
        if (hum_comps.dis_oper_mark._bit.cur3)
        {
            do_cursor3_count(2);
        }
        display_line3_data();
    }
    else
    {

        num = device_comps.temp_p_temp_n_average_result;
        hum_comps.dot3_pos = 0;
        calc_seg_value(&hum_comps.dig3_0, 6, num, 0, hum_comps.dot3_pos + 1);
        if (device_comps.sw._bit.temp_adc_stb)
        {
            hum_comps.dis_oper_mark._bit.cur3 = 0;
        }
        else
        {
            hum_comps.dis_oper_mark._bit.cur3 = 1;
        }
        if (hum_comps.dis_oper_mark._bit.cur3)
        {
            do_cursor3_count(2);
        }
        display_line3_data();
    }
}

static void display_press_adc(void)
{
    int32_t num;
    hum_comps.dot3_pos = 0;
    num = device_comps.ad1_ad2_average_result;
    calc_seg_value(&hum_comps.dig3_0, 6, num, 0, hum_comps.dot3_pos + 1);
    if (device_comps.sw._bit.adc_stb)
    {
        hum_comps.dis_oper_mark._bit.cur3 = 0;
    }
    else
    {
        hum_comps.dis_oper_mark._bit.cur3 = 1;
    }
    if (hum_comps.dis_oper_mark._bit.cur3)
    {
        do_cursor3_count(2);
    }
    display_line3_data();
}

/*********************************END DISPLAY FNCTION************************************************************/

static void display_current_press(uint8_t opt)
{
    int32_t num;
    hide_line2_all_unit();
    if (device_comps.press_cal_param.is_calibrated)
    {
        if (device_comps.press_cal_param.unit == 0)
        {
            if (device_comps.press_cal_param.dot == 5 && device_comps.current_press / 100 < 1000)
            {
                num = device_comps.current_press / 10;
                hum_comps.dot2_pos = 1;
                MD_DIS_S16_KPA;
            }
            else
            {
                num = device_comps.current_press / 100;
                hum_comps.dot2_pos = device_comps.press_cal_param.dot - 2;
                MD_DIS_S15_MPA;
            }
        }
        else
        {
            num = device_comps.current_press / 100;
            hum_comps.dot2_pos = device_comps.press_cal_param.dot - 2;
            MD_DIS_S16_KPA;
        }
        calc_seg_value(&hum_comps.dig2_0, 4, num, 1, hum_comps.dot2_pos + 1);
    }
    else
    {
        hum_comps.dig2_0 = hum_comps.dig2_1 = hum_comps.dig2_2 = hum_comps.dig2_3 = MD_HIDE_DISP - 1;
        hum_comps.dot2_pos = 0;
    }
    MD_DIS_S12_PRESS;
    display_line2_data();
}

// static void display_current_v(uint8_t opt)
//{
//     int32_t num=device_comps.current_v;
//     hide_line0_all_unit();
//     MD_DIS_T2_V;
//     MD_DIS_T3_M_S;
//     hum_comps.dot0_pos=2;
//     calc_seg_value(&hum_comps.dig0_0,3,num,1,hum_comps.dot0_pos+1);
//     display_line0_data();
// }

// static void display_flow_freq(uint8_t opt)
//{
//     int32_t num=device_comps.flow_freq_comped;
//     hide_line1_all_unit();
//     MD_DIS_T4_F;
//     MD_DIS_T5_HZ;
//     hum_comps.dot1_pos=1;
//     calc_seg_value(&hum_comps.dig1_0,5,num,1,hum_comps.dot1_pos+1);
//     display_line1_data();
// }

// static void display_current_i(uint8_t opt)
//{
//     int32_t num=device_comps.current_i/10;
//     hide_line2_all_unit();
//     MD_DIS_T6_I;
//     MD_DIS_T7_MA;
//     hum_comps.dot2_pos=2;
//     calc_seg_value(&hum_comps.dig2_0,4,num,1,hum_comps.dot2_pos+1);
//     display_line2_data();
// }

static void display_current_flow_N(uint8_t opt)
{
    int32_t num;
    hide_line1_all_unit();
    num = device_comps.current_flowN / pwr(2);
    hum_comps.dot1_pos = device_comps.flow_cal_param.dot - 2;
    if (num > 9999 && hum_comps.dot1_pos > 0)
    {
        num /= 10;
        hum_comps.dot1_pos -= 1;
        if (num > 9999 && hum_comps.dot1_pos > 0)
        {
            num /= 10;
            hum_comps.dot1_pos -= 1;
        }
    }
    MD_DIS_S11_STD_CONDITION;
    MD_DIS_S13_N;
    MD_DIS_S14_M3_H;
    calc_seg_value(&hum_comps.dig1_0, 4, num, 1, hum_comps.dot1_pos + 1);
    display_line1_data();
}

static void display_current_flow(uint8_t opt)
{
    int32_t num;
    hide_line3_all_unit();

    num = device_comps.current_flow / pwr(2);
    hum_comps.dot3_pos = device_comps.flow_cal_param.dot - 2;
    MD_DIS_S18_WORK_CONDITION;
    MD_DIS_S21_M_3;
    MD_DIS_S22_H;
    calc_seg_value(&hum_comps.dig3_0, 4, num, 1, hum_comps.dot3_pos + 1);
    display_line3_data();
}

static void display_current_temp(uint8_t opt)
{
    int32_t num = device_comps.current_temp;
    hide_line4_all_unit();
    if (device_comps.res_cal_param.is_calibrated)
    {
        MD_DIS_S23_CELSIUS;
        if (num < 0)
        {
            num /= 10;
            hum_comps.dot4_pos = 0;
        }
        else if (num < 1000)
        {
            hum_comps.dot4_pos = 1;
        }
        else
        {
            num /= 10;
            hum_comps.dot4_pos = 0;
        }
        calc_seg_value(&hum_comps.dig4_0, 3, num, 1, hum_comps.dot4_pos + 1);
    }
    else
    {
        hum_comps.dig4_0 = hum_comps.dig4_1 = hum_comps.dig4_2 = hum_comps.dig4_3 = MD_HIDE_DISP - 1;
        hum_comps.dot4_pos = 0;
    }
    MD_DIS_S20_TEMP;
    display_line4_data();
}

static void display_current_totalN(uint8_t opt)
{
    int32_t total_int = device_comps.meter.total_intN;
    float32_t total_dec = device_comps.meter.total_decN;
    int32_t num = total_int;
    hide_line0_all_unit();
    // if     (num < 1000)          {num=num*1000000 +(uint32_t)(total_dec*1000) ; hum_comps.dot0_pos=6;}
    // else if(num < 10000)         {num=num*100000  +(uint32_t)(total_dec*100 ) ; hum_comps.dot0_pos=5;}
    if (num < 10000)
    {
        num = num * 10000 + (uint32_t)(total_dec * 10);
        hum_comps.dot0_pos = 4;
    }
    else if (num < 100000)
    {
        num = num * 1000 + (uint32_t)total_dec;
        hum_comps.dot0_pos = 3;
    }
    else if (num < 1000000)
    {
        num = num * 100 + (uint32_t)total_dec / 10;
        hum_comps.dot0_pos = 2;
    }
    else if (num < 10000000)
    {
        num = num * 10 + (uint32_t)total_dec / 100;
        hum_comps.dot0_pos = 1;
    }
    else if (num < 100000000)
    {
        num = num * 1 + (uint32_t)total_dec / 1000;
        hum_comps.dot0_pos = 0;
    }
    calc_seg_value(&hum_comps.dig0_0, 8, num, 1, hum_comps.dot0_pos + 1);
    display_line0_data();
    MD_DIS_S3_TOTAL_USAGE;
    MD_DIS_S9_N;
    MD_DIS_S10_M3;
}

static void normal_mode_display(uint8_t opt)
{
    int32_t num;
    char *ptsz;
    if (opt > 0)
    {
        display_opt(opt);
    }
    switch (opt)
    {
    case 0:
        if (hum_comps.dis_oper_mark._bit.refressh_meter_data)
        {
            // display_current_v(opt);
            // display_flow_freq(opt);
            // display_current_i(opt);
            display_current_press(opt);
            display_current_flow(opt);
            display_current_flow_N(opt);
            display_current_temp(opt);
            display_current_totalN(opt);
            hum_comps.dis_oper_mark._bit.refressh_meter_data = 0;
        }
        break;
    case 1:
        if (hum_comps.dis_oper_mark._bit.refressh_broken_date)
        {

            hum_comps.dot1_pos = 0;
            hide_line1_all_unit();
            MD_HID_ALL_1_LINE_DATA;

            hum_comps.dot2_pos = 0;
            hide_line2_all_unit();
            MD_HID_ALL_2_LINE_DATA;

            hum_comps.dot3_pos = 0;
            hide_line3_all_unit();
            MD_HID_ALL_3_LINE_DATA;

            hide_line0_all_unit();
            hum_comps.dot0_pos = 0;
            hum_comps.dig0_0 = device_comps.system_time.time.u8Day & 0x0f;
            hum_comps.dig0_1 = device_comps.system_time.time.u8Day >> 4;
            hum_comps.dig0_2 = device_comps.system_time.time.u8Month & 0x0f;
            hum_comps.dig0_3 = device_comps.system_time.time.u8Month >> 4;
            hum_comps.dig0_4 = device_comps.system_time.time.u8Year & 0x0f;
            hum_comps.dig0_5 = device_comps.system_time.time.u8Year >> 4;
            hum_comps.dig0_6 = MD_HIDE_DISP;
            hum_comps.dig0_7 = MD_HIDE_DISP;
            display_line0_data(); //
            MD_DIS_P1_DIG0_4_DOT;
            MD_DIS_P3_DIG0_2_DOT;
            hum_comps.dis_oper_mark._bit.refressh_broken_date = 0;
        }
        break;
    case 2:
        if (hum_comps.dis_oper_mark._bit.refressh_broken_date)
        {
            hide_line0_all_unit();
            hum_comps.dot0_pos = 0;
            hum_comps.dig0_0 = device_comps.system_time.time.u8Second & 0x0f;
            hum_comps.dig0_1 = device_comps.system_time.time.u8Second >> 4;
            hum_comps.dig0_2 = device_comps.system_time.time.u8Minute & 0x0f;
            hum_comps.dig0_3 = device_comps.system_time.time.u8Minute >> 4;
            hum_comps.dig0_4 = device_comps.system_time.time.u8Hour & 0x0f;
            hum_comps.dig0_5 = device_comps.system_time.time.u8Hour >> 4;
            display_line0_data();
            MD_DIS_P1_DIG0_4_DOT;
            MD_DIS_P3_DIG0_2_DOT;
            hum_comps.dis_oper_mark._bit.refressh_broken_date = 0;
        }
        break;

    case 3: //
        if (hum_comps.dis_oper_mark._bit.refresh_normal_mode_info)
        {
            hide_line0_all_unit();
            num = strtol((char *)&device_comps.access_param.ip, &ptsz, 10);
            num *= 1000;
            num = num + strtol(ptsz + 1, &ptsz, 10);
            hum_comps.dot0_pos = 3;
            calc_seg_value(&hum_comps.dig0_0, 6, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
            hum_comps.dis_oper_mark._bit.refresh_normal_mode_info = 0;
        }
        break;
    case 4:
        if (hum_comps.dis_oper_mark._bit.refresh_normal_mode_info)
        {
            hide_line0_all_unit();
            ptsz = strchr((char *)&device_comps.access_param.ip, '.');
            ptsz = strchr(ptsz + 1, '.');
            num = strtol(ptsz + 1, &ptsz, 10);
            num *= 1000;
            num = num + strtol(ptsz + 1, &ptsz, 10);
            hum_comps.dot0_pos = 3;
            calc_seg_value(&hum_comps.dig0_0, 6, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
            hum_comps.dis_oper_mark._bit.refresh_normal_mode_info = 0;
        }
        break;
    case 5:
        if (hum_comps.dis_oper_mark._bit.refresh_normal_mode_info)
        {
            hide_line0_all_unit();
            num = (int32_t)device_comps.access_param.port;
            hum_comps.dot0_pos = 0;
            calc_seg_value(&hum_comps.dig0_0, 6, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
            hum_comps.dis_oper_mark._bit.refresh_normal_mode_info = 0;
        }
        break;
    case 6:
        if (hum_comps.dis_oper_mark._bit.refresh_normal_mode_info)
        {
            hide_line0_all_unit();
            num = (int32_t)modbusComps.param_pt->addr;
            hum_comps.dot0_pos = 0;
            calc_seg_value(&hum_comps.dig0_0, 6, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
            hum_comps.dis_oper_mark._bit.refresh_normal_mode_info = 0;
        }

        break;
    case 7: // refresh ver
        if (hum_comps.dis_oper_mark._bit.refresh_normal_mode_info)
        {
            hide_line0_all_unit();
            hum_comps.dot0_pos = 1;
            hum_comps.dig0_0 = MD_FL_VER % 10;
            hum_comps.dig0_1 = MD_FL_VER / 10 % 10;
            hum_comps.dig0_2 = MD_HIDE_DISP - 1;
            hum_comps.dig0_3 = 0x0f;
            hum_comps.dig0_4 = MD_HIDE_DISP;
            hum_comps.dig0_5 = MD_HIDE_DISP;
            display_line0_data(); //
            hum_comps.dis_oper_mark._bit.refresh_normal_mode_info = 0;
        }

        break;
    default:
        mode_comps[hum_comps.current_mode].dis_option = 0;
        break;
    }
}

static void debug_mode_display(uint8_t opt)
{

    int32_t num = 0;
    display_opt(opt);
    if (hum_comps.dis_oper_mark._bit.refresh_debug_param)
    {
        hum_comps.dis_oper_mark._bit.refresh_debug_param = 0;
        switch (opt)
        {
        case 0:
            hide_line1_all_unit();

            // num=device_comps.flow_roll_freq_cur/10;
            // hum_comps.dot1_pos=0;
            num = get_float_disp_num(device_comps.flow_roll_freq_cur_ft, 4, &hum_comps.dot1_pos);
            calc_seg_value(&hum_comps.dig1_0, 4, num, 4, hum_comps.dot1_pos + 1);
            display_line1_data();

            // num=device_comps.flow_roll_freq_comped_cur/10;
            // hum_comps.dot2_pos=0;
            num = get_float_disp_num(device_comps.flow_roll_freq_comped_cur_ft, 4, &hum_comps.dot2_pos);
            calc_seg_value(&hum_comps.dig2_0, 4, num, 4, hum_comps.dot2_pos + 1);
            display_line2_data();

            num = get_float_disp_num(device_comps.flow_run_meter_coe, 4, &hum_comps.dot3_pos);
            calc_seg_value(&hum_comps.dig3_0, 6, num, 0, hum_comps.dot3_pos + 1);
            display_line3_data();

            num = device_comps.flow_total_cnts;
            hum_comps.dot0_pos = 0; //
            calc_seg_value(&hum_comps.dig0_0, 8, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
            break;

        case 1:
            num = device_comps.flow_roll_freq_cur / 10;
            hum_comps.dot1_pos = 0;
            calc_seg_value(&hum_comps.dig1_0, 4, num, 4, hum_comps.dot1_pos + 1);
            display_line1_data();

            num = device_comps.flow_roll_freq_comped_cur / 10;
            hum_comps.dot2_pos = 0;
            calc_seg_value(&hum_comps.dig2_0, 4, num, 4, hum_comps.dot2_pos + 1);
            display_line2_data();

            num = get_float_disp_num(device_comps.flow_run_meter_coe, 4, &hum_comps.dot3_pos);
            calc_seg_value(&hum_comps.dig3_0, 6, num, 0, hum_comps.dot3_pos + 1);
            display_line3_data();

            num = device_comps.report_param.triggerTimes;
            hum_comps.dot0_pos = 0; //
            calc_seg_value(&hum_comps.dig0_0, 6, num, 0, hum_comps.dot0_pos + 1);
            hum_comps.dis_oper_mark._bit.cur0 = 0;
            hum_comps.dig0_6 = MD_HIDE_DISP - 1;
            hum_comps.dig0_7 = MD_DIS_r;
            display_line0_data();
            break;
        default:
            mode_comps[hum_comps.current_mode].dis_option = 0;
            hum_comps.dis_oper_mark._bit.refresh_debug_param = 1;
            return;
        }
    }
}

static void pwd_mode_display(uint8_t opt)
{
    do_cursor0_count(2);
    display_line0_data();
}

static void cal_query_mode_display(uint8_t opt)
{
    int32_t num;
    char *ptsz;
    uint8_t max_opt_nums = 10 + device_comps.flow_cal_param.freq_divd_pos * 2;
    if (opt > max_opt_nums)
    {
        opt = mode_comps[hum_comps.current_mode].dis_option = 0;
        hum_comps.dis_oper_mark._bit.refresh_cal_param = 1;
    }
    display_opt(opt);
    if (hum_comps.dis_oper_mark._bit.refresh_cal_param)
    {
        hum_comps.dis_oper_mark._bit.refresh_cal_param = 0;
        if (device_comps.cal_type == EM_CAL_FLOW)
        {
            hide_line0_all_unit();
            hum_comps.dot0_pos = 0;
            switch (opt)
            {
            case 0:
                num = device_comps.flow_cal_param.freq_divd_pos;
                break;
            case 1:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_poly_coe[0]);
                break;
            case 2:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_poly_coe[1]);
                break;
            case 3:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_poly_coe[2]);
                break;
            case 4:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_poly_coe[3]);
                break;
            case 5:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_poly_coe[4]);
                break;
            case 6:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.meter_coe);
                break;
            case 7:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.RefDN / 10.f);
                break;
            case 8:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.RefMaxQ / 10.f);
                break;
            case 9:
                num = device_comps.flow_cal_param.calc_mode;
                break;
            case 10:
                num = device_comps.flow_cal_param.freq_out_mode;
                break;

            case 11:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[0] / 10.f);
                break;
            case 12:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[0]);
                break;
            case 13:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[1] / 10.f);
                break;
                ;
            case 14:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[1]);
                break;
            case 15:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[2] / 10.f);
                break;
            case 16:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[2]);
                break;
            case 17:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[3] / 10.f);
                break;
            case 18:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[3]);
                break;
            case 19:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[4] / 10.f);
                break;
            case 20:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[4]);
                break;
            case 21:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[5] / 10.f);
                break;
            case 22:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[5]);
                break;
            case 23:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[6] / 10.f);
                break;
            case 24:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[6]);
                break;
            case 25:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[7] / 10.f);
                break;
            case 26:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[7]);
                break;
            case 27:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[8] / 10.f);
                break;
            case 28:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[8]);
                break;
            case 29:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[9] / 10.f);
                break;
            case 30:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[9]);
                break;

            case 31:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[10] / 10.f);
                break;
            case 32:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[10]);
                break;
            case 33:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[11] / 10.f);
                break;
                ;
            case 34:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[11]);
                break;
            case 35:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[12] / 10.f);
                break;
            case 36:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[12]);
                break;
            case 37:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[13] / 10.f);
                break;
            case 38:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[13]);
                break;
            case 39:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[14] / 10.f);
                break;
            case 40:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[14]);
                break;
            case 41:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[15] / 10.f);
                break;
            case 42:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[15]);
                break;
            case 43:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[16] / 10.f);
                break;
            case 44:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[16]);
                break;
            case 45:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[17] / 10.f);
                break;
            case 46:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[17]);
                break;
            case 47:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[18] / 10.f);
                break;
            case 48:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[18]);
                break;
            case 49:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value[19] / 10.f);
                break;
            case 50:
                num = get_float_disp_coded_num(device_comps.flow_cal_param.freq_divd_value_meter_coe[19]);
                break;
            default:
                mode_comps[hum_comps.current_mode].dis_option = 0;
                hum_comps.dis_oper_mark._bit.refresh_cal_param = 1;
                return;
            }
        }

        if (device_comps.cal_type == EM_CAL_PRESS || device_comps.cal_type == EM_CAL_RES)
        {
            calc_seg_value(&hum_comps.dig0_0, 6, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
        }
        else if (device_comps.cal_type == EM_CAL_FLOW)
        {
            calc_seg_value(&hum_comps.dig0_0, 8, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
        }
    }
}

static void cal_modify_mode_display(uint8_t opt)
{
    display_opt(opt);
    if (device_comps.cal_type == EM_CAL_PRESS || device_comps.cal_type == EM_CAL_RES)
    {
        do_cursor0_count(2);
        display_line0_data();
    }
    else if (device_comps.cal_type == EM_CAL_FLOW)
    {
        do_cursor0_count(2);
        display_line0_data();
    }
}

static void param_query_mode_display(uint8_t opt)
{
    int32_t num;
    char *ptsz;
    display_opt(opt);
    if (hum_comps.dis_oper_mark._bit.refresh_param)
    {
        hum_comps.dis_oper_mark._bit.refresh_param = 0;
        hide_line0_all_unit();
        if (device_comps.param_type == EM_PARAM_USER)
        {

            switch (opt)
            {
            case 0:
                hum_comps.dot0_pos = 0;
                num = device_comps.flow_meter.unit;
                break;
            case 1:
                hum_comps.dot0_pos = device_comps.flow_cal_param.dot - 2;
                num = device_comps.misc_param.I_o_low / 100;
                MD_DIS_S9_N;
                MD_DIS_S10_M3;
                break;
            case 2:
                hum_comps.dot0_pos = device_comps.flow_cal_param.dot - 2;
                num = device_comps.misc_param.I_o_high / 100;
                MD_DIS_S9_N;
                MD_DIS_S10_M3;
                break;
            case 3:
                hum_comps.dot0_pos = 0;
                num = device_comps.misc_param.I_o_dir;
                break;
            case 4:
                hum_comps.dot0_pos = 3;
                num = device_comps.flow_meter.pluse_equ; // 0.001m^3 std unit
                break;
            case 5: //
                hum_comps.dot0_pos = 1;
                num = device_comps.flow_meter.pluse_width; // 0.1ms
                break;
            case 6:
                hum_comps.dot0_pos = 2;
                num = device_comps.misc_param.density;
                break;
            case 7: //
                hum_comps.dot0_pos = 0;
                num = modbusComps.param_pt->addr;
                break;
            case 8: //
                hum_comps.dot0_pos = 0;
                num = modbusComps.param_pt->baud;
                break;
            case 9: //
                hum_comps.dot0_pos = 0;
                num = loraComps.cfg_info_p->freq;
                break;
            case 10: //
                hum_comps.dot0_pos = 0;
                num = loraComps.cfg_info_p->netId;
                break;
            case 11: //
                hum_comps.dot0_pos = 0;
                num = loraComps.cfg_info_p->nodeId >> 16;
                break;
            case 12: //
                hum_comps.dot0_pos = 0;
                num = loraComps.cfg_info_p->nodeId & 0xffff;
                break;
            case 13: //
                num = strtol((char *)&device_comps.access_param.ip, &ptsz, 10);
                num *= 1000;
                num = num + strtol(ptsz + 1, &ptsz, 10);
                hum_comps.dot0_pos = 3;
                break;
            case 14:
                ptsz = strchr((char *)&device_comps.access_param.ip, '.');
                ptsz = strchr(ptsz + 1, '.');
                num = strtol(ptsz + 1, &ptsz, 10);
                num *= 1000;
                num = num + strtol(ptsz + 1, &ptsz, 10);
                hum_comps.dot0_pos = 3;
                break;
            case 15:
                num = (int32_t)device_comps.access_param.port;
                hum_comps.dot0_pos = 0;
                break;
            case 16:
                num = device_comps.report_param.u16Minute_Interval;
                hum_comps.dot0_pos = 0;
                break;
            case 17:
                hum_comps.dot0_pos = 0;
                num = BCD2DEC(device_comps.system_time.time.u8Year) * 10000 + BCD2DEC(device_comps.system_time.time.u8Month) * 100 + BCD2DEC(device_comps.system_time.time.u8Day);
                break;
            case 18:
                hum_comps.dot0_pos = 0;
                num = BCD2DEC(device_comps.system_time.time.u8Hour) * 10000 + BCD2DEC(device_comps.system_time.time.u8Minute) * 100 + BCD2DEC(device_comps.system_time.time.u8Second);
                break;
            default:
                mode_comps[hum_comps.current_mode].dis_option = 0;
                hum_comps.dis_oper_mark._bit.refresh_param = 1;
                break;
            }
        }
        else if (device_comps.param_type == EM_PARAM_USER_A)
        {
            hide_line0_all_unit();
            switch (opt)
            {
            case 0:
                num = device_comps.meter.total_intN;
                hum_comps.dot0_pos = 0;
                MD_DIS_S9_N; // M_3;
                MD_DIS_S10_M3;
                break;
            case 1:
                num = f_mul(device_comps.meter.total_decN, 10);
                hum_comps.dot0_pos = 4;
                MD_DIS_S9_N; // M_3;
                MD_DIS_S10_M3;
                break;
            default:
                mode_comps[hum_comps.current_mode].dis_option = 0;
                hum_comps.dis_oper_mark._bit.refresh_param = 1;
                break;
            }
        }
        else if (device_comps.param_type == EM_PARAM_FACTORY)
        {
            hide_line0_all_unit();
            switch (opt)
            {
            case 0:
                hum_comps.dot0_pos = 4;
                num = device_comps.coe.flow;
                break;
            case 1:
                hum_comps.dot0_pos = 4;
                num = device_comps.coe.out_4_20ma;
                break;
            case 2:
                hum_comps.dot0_pos = 4;
                num = device_comps.coe.pt_temp; // 0.0001
                break;
            case 3:
                hum_comps.dot0_pos = 4;
                num = device_comps.coe.press;
                break;
            case 4:
                hum_comps.dot0_pos = 0;
                num = device_comps.gps.sel;
                break;
            case 5:
                hum_comps.dot0_pos = 1;
                num = device_comps.flow_meter.sensor_low_freq_cutoff;
                break;
            case 6:
                hum_comps.dot0_pos = 0;
                num = device_comps.flow_meter.avg_freq_filter_timer;
                break;
            case 7:
                hum_comps.dot0_pos = 3;
                num = device_comps.coe._4ma_raw_value;
                break;
            case 8:
                hum_comps.dot0_pos = 3;
                num = device_comps.coe._20ma_raw_value;
                break;

            default:
                mode_comps[hum_comps.current_mode].dis_option = 0;
                hum_comps.dis_oper_mark._bit.refresh_param = 1;
                break;
            }
        }
        if (device_comps.param_type == EM_PARAM_USER || device_comps.param_type == EM_PARAM_FACTORY)
        {
            calc_seg_value(&hum_comps.dig0_0, 6, num, 0, hum_comps.dot0_pos + 1);
            hum_comps.dig0_6 = MD_HIDE_DISP;
            hum_comps.dig0_7 = MD_HIDE_DISP;
            display_line0_data();
        }
        else if (device_comps.param_type == EM_PARAM_USER_A)
        {
            calc_seg_value(&hum_comps.dig0_0, 8, num, 0, hum_comps.dot0_pos + 1);
            display_line0_data();
        }
    }
}

static void param_modify_mode_display(uint8_t opt)
{
    display_opt(opt);
    if (device_comps.param_type == EM_PARAM_USER || device_comps.param_type == EM_PARAM_FACTORY)
    {
        do_cursor0_count(2);
        display_line0_data();
    }
    else if (device_comps.param_type == EM_PARAM_USER_A)
    {
        do_cursor0_count(2);
        display_line0_data();
    }
}

static void self_test_mode_display(uint8_t opt)
{

    hum_comps.dig0_0 = !!device_comps.sw._bit.e2prom_driver_err;
    hum_comps.dig0_1 = !!device_comps.sw._bit.adx_driver_err;
    hum_comps.dig0_2 = !!device_comps.sw._bit.lora_module_err;
    hum_comps.dig0_3 = 0;
    hum_comps.dig0_4 = !!device_comps.sw._bit.rtc_module_err;
    hum_comps.dig0_5 = 0x0e;
    hum_comps.dig0_6 = MD_HIDE_DISP;
    hum_comps.dig0_7 = MD_HIDE_DISP;
    hum_comps.dot0_pos = 0; //
    display_line0_data();
}

static void display_ip(uint8_t opt)
{
    int16_t flag;

    if (netComps.net_info.currentIP_No < 2)
    {
        if (netComps.St._bit.ResolvedIP)
        {
            flag = 1; // fill ip
        }
        else
        {
            flag = 2; // fill __
        }
    }
    else // no set ip
    {
        flag = 0; // fill 0
    }

    switch (opt)
    {
    case 0:
        if (flag == 1)
        {
            hum_comps.dig3_0 = netComps.net_info.ResolvedIP[2] & 0x0f;
            hum_comps.dig3_1 = (netComps.net_info.ResolvedIP[2] >> 4) & 0x0f;
            hum_comps.dig3_2 = netComps.net_info.ResolvedIP[1] & 0x0f;
            hum_comps.dig3_3 = (netComps.net_info.ResolvedIP[1] >> 4) & 0x0f;
            hum_comps.dig3_4 = netComps.net_info.ResolvedIP[0] & 0x0f;
            hum_comps.dig3_5 = (netComps.net_info.ResolvedIP[0] >> 4) & 0x0f;
            hum_comps.dot3_pos = 2; //
        }
        else if (flag == 2)
        {
            hum_comps.dig3_0 = hum_comps.dig3_1 = hum_comps.dig3_2 = MD_DIS__;
            hum_comps.dig3_3 = hum_comps.dig3_4 = hum_comps.dig3_5 = MD_DIS__;
            hum_comps.dot3_pos = 2; //
        }
        else // no set ip
        {
            hum_comps.dig3_0 = MD_DIS_p;
            hum_comps.dig3_1 = 0x01;
            hum_comps.dig3_2 = MD_DIS_o;
            hum_comps.dig3_3 = MD_DIS_n;
            hum_comps.dig3_4 = hum_comps.dig3_5 = MD_HIDE_DISP;
            hum_comps.dot3_pos = 2; //
        }
        break;

    case 1:
        if (flag == 1)
        {
            uint16_t port = 0;
            if (netComps.net_info.currentIP_No == EM_IP0)
            {
                port = device_comps.access_param.port;
            }
            else if (netComps.net_info.currentIP_No == EM_IP1)
            {
                port = device_comps.access_param.port1;
            }

            hum_comps.dig3_0 = port & 0x0f;
            hum_comps.dig3_1 = (port >> 4) & 0x0f;
            ;
            hum_comps.dig3_2 = (port >> 8) & 0x0f;
            hum_comps.dig3_3 = (port >> 12) & 0x0f;
            hum_comps.dig3_4 = netComps.net_info.ResolvedIP[3] & 0x0f;
            hum_comps.dig3_5 = (netComps.net_info.ResolvedIP[3] >> 4) & 0x0f;
            hum_comps.dot3_pos = 0;
        }
        else if (flag == 2)
        {
            hum_comps.dig3_0 = hum_comps.dig3_1 = hum_comps.dig3_2 = MD_DIS__;
            hum_comps.dig3_3 = hum_comps.dig3_4 = hum_comps.dig3_5 = MD_DIS__;
            hum_comps.dot3_pos = 0; //
        }
        else // no set ip
        {
            hum_comps.dig3_0 = hum_comps.dig3_1 = hum_comps.dig3_2 = MD_HIDE_DISP - 1;
            hum_comps.dig3_3 = hum_comps.dig3_4 = hum_comps.dig3_5 = MD_HIDE_DISP - 1;
            hum_comps.dot3_pos = 0; //
        }
        break;

    default:
        break;
    }
    hum_comps.dis_oper_mark._bit.cur3 = 0;
    display_line3_data();
}

static void display_net_op_window_tmr(uint8_t opt)
{
    int32_t num = netComps.op_window_tmr;
    hum_comps.dot4_pos = 0;
    calc_seg_value(&hum_comps.dig4_0, 3, num, 1, hum_comps.dot4_pos + 1);
    display_line4_data();
}

static void report_mode_display(uint8_t opt)
{
    mode_comps[hum_comps.current_mode].displayTimer++;
    if (mode_comps[hum_comps.current_mode].displayTimer >= 60)
    {
        mode_comps[hum_comps.current_mode].dis_option++;
        mode_comps[hum_comps.current_mode].dis_option %= 2;
        mode_comps[hum_comps.current_mode].displayTimer = 0;
    }
    display_ip(opt);
    display_net_op_window_tmr(opt);

    switch (netComps.disCode)
    {
    case EM_DIS_ACT: //---act
        hum_comps.dis_oper_mark._bit.cur0 = 0;
        hum_comps.dig0_0 = 0x0A;
        ;
        hum_comps.dig0_1 = *protocolComps.event_index_pt % 10;
        hum_comps.dig0_2 = *protocolComps.event_index_pt / 10 % 10;
        hum_comps.dig0_3 = MD_HIDE_DISP - 1;
        hum_comps.dig0_4 = *netComps.run_st % 10;
        hum_comps.dig0_5 = *netComps.run_st / 10 % 10;
        hum_comps.dig0_6 = MD_HIDE_DISP;
        hum_comps.dig0_7 = MD_HIDE_DISP;
        // hum_comps.dig0_8=MD_HIDE_DISP;
        hum_comps.dot0_pos = 0;
        display_line0_data();
        break;

    case EM_DIS_OFF: // suc   or fal
        hum_comps.dis_oper_mark._bit.cur0 = 0;
        if (protocolComps.sw._bit.dataPushYet && protocolComps.sw._bit.dataPushYet1)
        {
            hum_comps.dig0_0 = 0x0c;
            hum_comps.dig0_1 = MD_DIS_U;
            hum_comps.dig0_2 = 5; // s
            hum_comps.dig0_3 = 0x0c;
            hum_comps.dig0_4 = MD_DIS_U;
            hum_comps.dig0_5 = 5;
            hum_comps.dot0_pos = 0;
        }
        else if (!protocolComps.sw._bit.dataPushYet && !protocolComps.sw._bit.dataPushYet1)
        {
            hum_comps.dig0_0 = 0x01;
            hum_comps.dig0_1 = 0x0a;
            hum_comps.dig0_2 = 0x0f; //
            hum_comps.dig0_3 = 0x01;
            hum_comps.dig0_4 = 0x0a;
            hum_comps.dig0_5 = 0x0f;
            hum_comps.dot0_pos = 0;
        }
        else if (protocolComps.sw._bit.dataPushYet && !protocolComps.sw._bit.dataPushYet1)
        {
            hum_comps.dig0_0 = 0x0c;
            hum_comps.dig0_1 = MD_DIS_U;
            hum_comps.dig0_2 = 5; // s
            hum_comps.dig0_3 = 0x01;
            hum_comps.dig0_4 = 0x0a;
            hum_comps.dig0_5 = 0x0f;
            hum_comps.dot0_pos = 0;
        }
        else if (!protocolComps.sw._bit.dataPushYet && protocolComps.sw._bit.dataPushYet1)
        {
            hum_comps.dig0_0 = 0x01;
            hum_comps.dig0_1 = 0x0a;
            hum_comps.dig0_2 = 0x0f; // s
            hum_comps.dig0_3 = 0x0c;
            hum_comps.dig0_4 = MD_DIS_U;
            hum_comps.dig0_5 = 5;
            hum_comps.dot0_pos = 0;
        }
        display_line0_data();
        break;

    case EM_DIS_GPS_STATUS:
        hum_comps.dig0_0 = 5;
        ;
        hum_comps.dig0_1 = MD_DIS_p;
        hum_comps.dig0_2 = MD_DIS_G;
        hum_comps.dig0_3 = MD_HIDE_DISP - 1;
        hum_comps.dig0_4 = *netComps.run_st % 10;
        hum_comps.dig0_5 = *netComps.run_st / 10 % 10;
        hum_comps.dot0_pos = 0;
        if (!device_comps.gps.sw._bit.isLocSuc)
        {
            hum_comps.dis_oper_mark._bit.cur0 = 1;
        }
        else
        {
            hum_comps.dis_oper_mark._bit.cur0 = 0;
        }
        hum_comps.cursor_0 = 0;
        hum_comps.cursor_0_count++;
        if (hum_comps.cursor_0_count > 5)
        {
            hum_comps.cursor_0_count = 0;
            if (hum_comps.dis_oper_mark._bit.dis0)
            {
                hum_comps.dis_oper_mark._bit.dis0 = 0;
            }
            else
            {
                hum_comps.dis_oper_mark._bit.dis0 = 1;
            }
        }

        display_line0_data();
        break;
    case EM_DIS_SEARCH_NET:
        hum_comps.dis_oper_mark._bit.cur0 = 0;
        hum_comps.dig0_0 = netComps.net_info.Csq % 10;
        hum_comps.dig0_1 = netComps.net_info.Csq / 10 % 10;
        hum_comps.dig0_2 = MD_DIS_r;
        hum_comps.dig0_3 = MD_HIDE_DISP - 1;
        hum_comps.dig0_4 = *netComps.run_st % 10;
        hum_comps.dig0_5 = *netComps.run_st / 10 % 10;
        hum_comps.dot0_pos = 0;

        display_line0_data();
        break;

    case EM_DIS_REGISTER_NET:
        hum_comps.dig0_0 = MD_DIS__;
        ;
        hum_comps.dig0_1 = MD_DIS_G;
        if (strstr(netComps.net_info.band, "LTE"))
        {
            hum_comps.dig0_2 = 4;
        }
        else
        {
            hum_comps.dig0_2 = 2;
        }
        hum_comps.dig0_3 = MD_HIDE_DISP - 1;
        hum_comps.dig0_4 = *netComps.run_st % 10;
        hum_comps.dig0_5 = *netComps.run_st / 10 % 10;
        hum_comps.dot0_pos = 0;

        hum_comps.dis_oper_mark._bit.cur0 = 1;
        hum_comps.cursor_0 = 0;
        hum_comps.cursor_0_count++;
        if (hum_comps.cursor_0_count > 9)
        {
            hum_comps.cursor_0_count = 0;
            if (hum_comps.dis_oper_mark._bit.dis0)
            {
                hum_comps.dis_oper_mark._bit.dis0 = 0;
            }
            else
            {
                hum_comps.dis_oper_mark._bit.dis0 = 1;
            }
        }

        display_line0_data();

        break;
    case EM_DIS_SEND: // send
        hum_comps.dis_oper_mark._bit.cur0 = 0;
        hum_comps.dig0_0 = 5; // s
        hum_comps.dig0_1 = MD_DIS_G;
        if (strstr(netComps.net_info.band, "LTE"))
        {
            hum_comps.dig0_2 = 4;
        }
        else
        {
            hum_comps.dig0_2 = 2;
        }
        hum_comps.dig0_3 = MD_HIDE_DISP - 1;
        hum_comps.dig0_4 = *netComps.run_st % 10;
        hum_comps.dig0_5 = *netComps.run_st / 10 % 10;
        hum_comps.dot0_pos = 0;
        display_line0_data();
        break;
    case EM_DIS_RECV: // rece
        hum_comps.dis_oper_mark._bit.cur0 = 0;
        hum_comps.dig0_0 = MD_DIS_r; // R
        hum_comps.dig0_1 = MD_DIS_G;
        if (strstr(netComps.net_info.band, "LTE"))
        {
            hum_comps.dig0_2 = 4;
        }
        else
        {
            hum_comps.dig0_2 = 2;
        }
        hum_comps.dig0_3 = MD_HIDE_DISP - 1;
        hum_comps.dig0_4 = *netComps.run_st % 10;
        hum_comps.dig0_5 = *netComps.run_st / 10 % 10;
        hum_comps.dot0_pos = 0;
        display_line0_data();
        break;
    }
}

#define BK_LN

static void hum_comps_init(hum_comps_t *const this)
{
     uint8_t data[28] = {0};
     data[0] = 0x80;
     memset(hum_comps.device_driver_ram,0,26);
     memcpy(hum_comps.last_display_buff, hum_comps.device_driver_ram, 26);
     I2C_MasterWriteData(MD_LCD_I2CX_SEL, data, sizeof(data));
}

static void display_special_batt(void)
{
    static int16_t init_once = 1;
    if (init_once)
    {
        init_once = 0;
        if (device_comps.batt > MD_BAT_88_PER_100)
        {
            MD_DIS_S27_BAT;
            MD_DIS_S28_BAT;
            MD_DIS_S29_BAT;
            MD_DIS_S30_BAT;
            MD_DIS_S31_BAT;
        }
        else if (device_comps.batt > MD_BAT_85_PER_100)
        {
            MD_DIS_S27_BAT;
            MD_HIDE_S28_BAT;
            MD_DIS_S29_BAT;
            MD_DIS_S30_BAT;
            MD_DIS_S31_BAT;
        }
        else if (device_comps.batt > MD_BAT_80_PER_100)
        {
            MD_DIS_S27_BAT;
            MD_HIDE_S28_BAT;
            MD_HIDE_S29_BAT;
            MD_DIS_S30_BAT;
            MD_DIS_S31_BAT;
        }
        else
        {
            MD_DIS_S27_BAT;
            MD_HIDE_S28_BAT;
            MD_HIDE_S29_BAT;
            MD_HIDE_S30_BAT;
            MD_DIS_S31_BAT;
        }
    }
    else
    {
        if (device_comps.batt > MD_BAT_88_PER_100)
        {
            MD_DIS_S27_BAT;
            MD_DIS_S28_BAT;
            MD_DIS_S29_BAT;
            MD_DIS_S30_BAT;
            MD_DIS_S31_BAT;
        }
        else if (device_comps.batt > MD_BAT_85_PER_100)
        {
            if (device_comps.batt < MD_BAT_88_PER_100)
            {
                MD_DIS_S27_BAT;
                MD_HIDE_S28_BAT;
                MD_DIS_S29_BAT;
                MD_DIS_S30_BAT;
                MD_DIS_S31_BAT;
            }
        }
        else if (device_comps.batt > MD_BAT_80_PER_100)
        {
            if (device_comps.batt < MD_BAT_85_PER_100)
            {
                MD_DIS_S27_BAT;
                MD_HIDE_S28_BAT;
                MD_HIDE_S29_BAT;
                MD_DIS_S30_BAT;
                MD_DIS_S31_BAT;
            }
        }
        else if (device_comps.batt < MD_BAT_80_PER_100)
        {
            MD_DIS_S27_BAT;
            MD_HIDE_S28_BAT;
            MD_HIDE_S29_BAT;
            MD_HIDE_S30_BAT;
            MD_DIS_S31_BAT;
        }
    }
}

static void display_special_symbol(void)
{

    if (!ircComps.sw._bit.runing && !modbusComps.sw._bit.runing && !loraComps.sw._bit.runing)
    {
        // MD_HIDE_T1_COMMUNICATION;
    }
    else
    {
        // MD_DIS_T1_COMMUNICATION;
    }

    if (device_comps.sw._bit.is_4_20ma_Connected || device_comps.sw._bit.isExtPowerConnected)
    {
        MD_DIS_S25_OPEN;
    }
    else
    {
        MD_HIDE_S25_OPEN;
    }

    if ((netComps.St._bit.running) && (netComps.disCode >= EM_DIS_SEARCH_NET))
    {
        if (netComps.net_info.Csq < 7)
        {
            MD_DIS_S4_CSQ;
            MD_HIDE_S5_CSQ;
            MD_HIDE_S6_CSQ;
            MD_HIDE_S7_CSQ;
        }
        else if (netComps.net_info.Csq < 15)
        {
            MD_DIS_S4_CSQ;
            MD_DIS_S5_CSQ;
            MD_HIDE_S6_CSQ;
            MD_HIDE_S7_CSQ;
        }
        else if (netComps.net_info.Csq < 22)
        {
            MD_DIS_S4_CSQ;
            MD_DIS_S5_CSQ;
            MD_DIS_S6_CSQ;
            MD_HIDE_S7_CSQ;
        }
        else if (netComps.net_info.Csq < 32)
        {
            MD_DIS_S4_CSQ;
            MD_DIS_S5_CSQ;
            MD_DIS_S6_CSQ;
            MD_DIS_S7_CSQ;
        }
        else
        {
            MD_HIDE_S4_CSQ;
            MD_HIDE_S5_CSQ;
            MD_HIDE_S6_CSQ;
            MD_HIDE_S7_CSQ;
        }
    }
    display_special_batt();
}

// return low level key_value
static key_type_t get_key(hum_comps_t *const this)
{
    static int16_t key_delay_count;
    static int16_t current_key;
    static int16_t last_key_type; // 0 short 1 int32_t
    static int16_t last_key;
    static int16_t init_once = 1;
    int16_t key_value;
    key_type_t key_type;
    key_type = EM_NO_KEY;
    // MD_KEY_VDD_ENABLE;
    // delay10us(1);
    key_value = (MD_KEY_DATA & MD_KEY_MASK);
    // MD_KEY_VDD_DISENABLE;

    if (init_once)
    {
        key_delay_count = 0;
        current_key = key_value;
        last_key_type = EM_SHORT_KEY;
        init_once = 0;
        return key_type;
    }
    if (key_value != current_key)
    {
        //        if(key_delay_count==83)
        //        {
        //            key_delay_count=0;
        //        }
        //        else
        {
            last_key = current_key;
            key_delay_count = 83;
        }
        current_key = key_value;
    }
    else
    {
        if (key_delay_count > 0)
        {
            key_delay_count--;
            if (key_delay_count == 82)
            {
                if (last_key_type == EM_SHORT_KEY)
                {
                    this->up_key = (current_key ^ last_key) & current_key & MD_KEY_MASK; // bit=1  up
                                                                                         // this->up_key=current_key & MD_KEY_MASK;
                }
                else
                {
                    this->up_key = 0; // ignore up key
                }
                this->down_key = (~(current_key ^ last_key) | current_key) & MD_KEY_MASK; // bit==0 down
                // this->down_key=current_key & MD_KEY_MASK;
                key_type = EM_SHORT_KEY;
                last_key_type = EM_SHORT_KEY;
            }
            else if (key_delay_count == 23)
            {
                this->up_key = current_key & MD_KEY_MASK;   // bit=1  up
                this->down_key = current_key & MD_KEY_MASK; // bit==0 down
                key_type = EM_LONG_KEY;
                last_key_type = EM_LONG_KEY;
            }
        }
    }
    return key_type;
}

static void handle_key(hum_comps_t *this)
{
    key_type_t key_type;
    key_type = get_key(this);
    if (key_type == EM_NO_KEY)
    {
        return;
    }
    else if (key_type == EM_SHORT_KEY)
    {
        switch (this->down_key)
        {
            case MD_S_KEY:
                break;
            default:
                break;
        }

        switch (this->up_key)
        {
            case ~MD_S_KEY &MD_KEY_MASK:
                on_s_key();
                break;
            case ~MD_M_KEY &MD_KEY_MASK:
                on_m_key();
                break;
            case ~MD_J_KEY &MD_KEY_MASK:
                on_j_key();
                break;
                //            case ~MD_POWER_KEY&MD_KEY_MASK:
                //                  #if (MD_MEASURE_TYPE==MD_ELE_HIGH_PRESS_DIG)
                //                    device_comps.DM.sw._bit.isPowerOnYet=MD_DM_POWER_ON_PIN=!MD_DM_POWER_ON_PIN;
                //                    if(!device_comps.DM.sw._bit.isPowerOnYet)
                //                    {
                //                        device_comps.DM.sw.All=0;
                //                    }
                //                  #endif
                //                    break;
            default:
                break;
        }
    }
    else if (key_type == EM_LONG_KEY)
    {
        switch (this->down_key)
        {
        case MD_S_KEY:
            on_long_s_key();
            break;
        case MD_M_KEY:
            on_long_m_key();
            break;
        case MD_J_KEY:
            on_long_j_key();
            break;
        case MD_S_KEY &MD_J_KEY:
            on_long_s_and_j_key();
            break;
            //			case MD_POWER_KEY:
            //                    systemComps.power_off();
            //                    break;
        default:
            break;
        }
        switch (this->up_key)
        {
        case (~MD_S_KEY | ~MD_J_KEY) & MD_KEY_MASK:
            break;
        default:
            break;
        }
    }
}

static void handle_lcd_refresh(void)
{
    mode_comps_t *mode_comps_p = &mode_comps[hum_comps.current_mode];
    mode_comps_p->display(mode_comps_p->dis_option);
    if (hum_comps.current_mode != EM_SELF_TEST_MODE)
    {
        if (hum_comps.dis_oper_mark._bit.refresh_special_symbol)
        {
            display_special_symbol();
            hum_comps.dis_oper_mark._bit.refresh_special_symbol = 0;
        }
    }
    
} 

static void flush_display_buffer(void)
{
    if(memcmp(hum_comps.device_driver_ram, hum_comps.last_display_buff, 26) != 0)
    {
        uint8_t data[28] = {0};
        data[0] = 0x80;
        memcpy(data + 2, hum_comps.device_driver_ram, 26);
        memcpy(hum_comps.last_display_buff, hum_comps.device_driver_ram, 26);
        I2C_MasterWriteData(MD_LCD_I2CX_SEL, data, sizeof(data));
    }
}
static void hum_comps_task_handle(void) ////Execution interval is 50 ms
{
    hum_comps_t *const this = hum_comps.this;
    if (this->do_init == 1)
    {
        MD_LCD_POWER_PIN_ENABLE;
        delay1ms(5);
        App_Lcd_I2cCfg(MD_LCD_I2CX_SEL);
        App_Disp_IC_Init();
        hum_comps_init(this);
        this->do_init = 0;
    }
    if (this->count >= 93)
    {
        handle_key(this);
        if (this->count == 93 + 100)
        {
            App_Disp_IC_Init();
            this->count = 93;
        }
        handle_lcd_refresh();
    }
    else if (!(this->count % 6) && (this->count < 92))
    {
        lcd_test(this); // interval time 300ms
    }
    else if (this->count == 92)
    {
        hum_comps.dis_oper_mark._bit.test_ok = 1;
        enter_self_test_mode();
    }
    this->count++;
    flush_display_buffer();
  
}

#define BK_LN
mode_comps_t mode_comps[] = // Handling of keys in different modes
    {                       // mode_comps_t  mode_comps[5];
        {"normal mode", EM_NORMAL_MODE, normal_mode_on_s_key, normal_mode_on_m_key, normal_mode_on_j_key, normal_mode_on_long_s_key, normal_mode_on_long_m_key, normal_mode_on_long_j_key, normal_mode_on_long_s_and_j_key, normal_mode_display, 0, 0},
        {"debug mode", EM_DEBUG_MODE, debug_mode_on_s_key, debug_mode_on_m_key, debug_mode_on_j_key, nop, debug_mode_on_long_m_key, nop, debug_mode_on_long_s_and_j_key, debug_mode_display, 0, 0},
        {"password mode", EM_PWD_MODE, pwd_mode_on_s_key, pwd_mode_on_m_key, pwd_mode_on_j_key, nop, pwd_mode_on_long_m_key, nop, nop, pwd_mode_display, 0, 0},
        {"cal_query mode", EM_CAL_QUERY_MODE, cal_query_mode_on_s_key, cal_query_mode_on_m_key, cal_query_mode_on_j_key, nop, cal_query_mode_on_long_m_key, nop, nop, cal_query_mode_display, 0, 0},
        {"cal_modify mode", EM_CAL_MODIFY_MODE, cal_modify_mode_on_s_key, cal_modify_mode_on_m_key, cal_modify_mode_on_j_key, cal_modify_mode_on_long_s_key, nop, nop, nop, cal_modify_mode_display, 0, 0},

        {"param_query mode", EM_PARAM_QUERY_MODE, param_query_mode_on_s_key, param_query_mode_on_m_key, nop, nop, param_query_mode_on_long_m_key, nop, nop, param_query_mode_display, 0, 0},
        {"param_modify mode", EM_PARAM_MODIFY_MODE, param_modify_mode_on_s_key, param_modify_mode_on_m_key, param_modify_mode_on_j_key, param_modify_mode_on_long_s_key, nop, nop, nop, param_modify_mode_display, 0, 0},
        {"self_test_mode", EM_SELF_TEST_MODE, nop, nop, nop, nop, nop, nop, nop, self_test_mode_display, 0, 0},
        {"report_mode", EM_REPORT_MODE, nop, nop, nop, nop, nop, nop, nop, report_mode_display, 0, 0}

};

hum_comps_t hum_comps =
    {

        "",         // uint8_t *desc;
        &hum_comps, // struct _HUM_COMPONENTS *this;
        1,          // int16_t   do_init;//Whether to initialize,1:init 0,no init

        0, // uint32_t count;            //task_handle Called counter

        ///******************************lcd seg define******************************/

        // 0    1     2     3    4     5    6    7    8      9     A   b     C     d    E     F     H     G   h    c      J     L    n    N    o    P    q    r    t    U   _    -   NULL
        // {0x5F,0x50, 0x3D, 0x79, 0x72,0x6B,0x6F,0x51, 0x7F, 0x7B,0x77,0x6E, 0x0F, 0x7C,0x2F, 0x27, 0x76,0x4F,0x66, 0x2C,0x58, 0x0E, 0x64,0x57,0x6C,0x37,0x73,0x24,0x2E,0x5E,0x08,0x20,0x00},
        {0XAF, 0XA0, 0XCB, 0XE9, 0XE4, 0X6D, 0X6F, 0XA8, 0XEF, 0XED, 0XEE, 0X67, 0X0F, 0XE3, 0X4F, 0X4E, 0XE6, 0X2F, 0X66, 0X43, 0XA1, 0X07, 0X62, 0XAE, 0X63, 0XCE, 0XEC, 0X42, 0X47, 0XA7, 0X01, 0X40, 0X00},
        9, //    uint8_t dig0_0;
        9, //    uint8_t dig0_1;
        9, //    uint8_t dig0_2;
        9, //    uint8_t dig0_3;
        9, //    uint8_t dig0_4;
        9, //    uint8_t dig0_5;
        9, //    uint8_t dig0_6;
        9, //    uint8_t dig0_7;
        9, //    uint8_t dig0_8;
        0, // uint8_t dot0_pos;

        9, // uint8_t dig1_0;
        9, // uint8_t dig1_1;
        9, // uint8_t dig1_2;
        9, // uint8_t dig1_3;
        9, // uint8_t dig1_4;
        9, // uint8_t dig1_5;
        0, // uint8_t dot1_pos;

        9, // uint8_t dig2_0;
        9, // uint8_t dig2_1;
        9, // uint8_t dig2_2;
        9, // uint8_t dig2_3;
        9, // uint8_t dig2_4;
        9, // uint8_t dig2_5;
        0, // uint8_t dot2_pos;

        9, // uint8_t dig3_0;
        9, // uint8_t dig3_1;
        9, // uint8_t dig3_2;
        9, // uint8_t dig3_3;
        9, // uint8_t dig3_4;
        9, // uint8_t dig3_5;
        0, // uint8_t          dot3_pos;

        9, //    uint8_t dig4_0;
        9, //    uint8_t dig4_1;
        9, //    uint8_t dig4_2;
        9, //    uint8_t dig4_3;
        9, //    uint8_t dig4_4;
        9, //    uint8_t dig4_5;
        0, //    uint8_t          dot4_pos;
           //
        9, //    uint8_t dig5_0;
        9, //    uint8_t dig5_1;
        9, //    uint8_t dig5_2;
        9, //    uint8_t dig5_3;
        9, //    uint8_t dig5_4;
        9, //    uint8_t dig5_5;
        9, //    uint8_t dig5_6;
        9, //    uint8_t dig5_7;
        9, //    uint8_t dig5_8;
        0, //    uint8_t          dot5_pos;

        {0}, // union dis_oper_mark;
        0,   // int16_t cursor_0;//0 line cursor position
        0,   // int16_t cursor_1;
        0,   // int16_t cursor_2;
        0,   // int16_t cursor_3;
        0,   // int16_t cursor_4;
        0,   // int16_t cursor_5;

        0, // int16_t cursor_0_count;
        0, // int16_t cursor_1_count;
        0, // int16_t cursor_2_count;
        0, // int16_t cursor_3_count;
        0, // int16_t cursor_4_count;
        0, // int16_t cursor_5_count;
        /*******************end lcd seg define **************************/
        {0}, // uint8_t device_driver_ram[52];
		{0},
        0,   // int16_t back_led_timer;

        EM_SELF_TEST_MODE, // mode_type_t   current_mode;

        0,           // int16_t up_key;
        MD_KEY_MASK, // int16_t down_key;

        enter_report_mode,
        enter_default_mode,    //    void (*enter_default_mode)(int16_t opt);
        enter_cal_modify_mode, //    void (*enter_cal_modify_momde)(int16_t opt);
        hum_comps_task_handle  // void (*const task_handle)(void);

        // TODO

};
