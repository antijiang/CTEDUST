#ifndef __DUSTSENSOR_H__
#define __DUSTSENSOR_H__

#define BOARDTYPE_MINI                              // 取消注释用于MINI板
#define	REF25_VDD			//p1.2V ain0 基准电压
#include "M051Series.h"

#define UART_BAUD_RATE                  (9600)     // 波特率

#define PWM_FREQ                        (100)       // PWM频率
#define DATA_SIZE                       (100)       // 每次采样的点数
#define PWM_DEFAULT_DUTY_RATIO          (0.032)     // PMW默认占空比
#define TMR_DEFAULT_DELAY_TIME          (0.28)      // 采样时间

#ifndef BOARDTYPE_MINI
#define SENSOR_ADC_CHANEL               (7)         // 传感器ADC输入通道 
#else
#define SENSOR_ADC_CHANEL               (3)         // 传感器ADC输入通道 
#define REF25_ADC_CHANEL               (7)         // 2.5V基准 
#endif

#define TYPE_DISTINGUISH_RANGE          (0.15)      // 在平均值15%上下浮动
#define TYPE_DISTINGUISH_COUNT          (DATA_SIZE * 0.04)    // 在平均值15%上下浮动的数据量超过的数量

#define CLEAN_DOWN_CONFIRM_TIMES        (30)		// 降低B值的确认时间(次  分钟)
#define CLEAN_UP_CONFIRM_TIMES          (100)	    // 提高B值的确认时间(次 分钟)
#define CLEAN_UP_UPDATE_THRESHOLD       (15)        // B值更新门限（达到该值才更新）
#define CLEAN_UP_DISTINGUISH_RANGE      (0.05)	    // 每次计算提高B值的数据浮动范围(与最后一次获取到值的比较值, 若大于该范围则会重新计算)
#define CLEAN_UP_FINAL_RANGE            (0.40)      // 最终更新前判断是否变化值大于该值
#define CLEAN_UP_CALI_VALUE             (0.20)      // 提高B值百分比

#define SENSOR_OUTPUT_RANGE             (500)       // 输出范围(ug/m3)
#define PWM_OUTPUT_PRECISION            (500)       // PWM输出的精度

#define CYCLIC_COUNT                    (60)        // 循环滤波次数(烟)60是标准，30是做实验20141009
#define CYCLIC_TIME                     (5)         // 循环滤波次数(计算尘)

#define UPDATE_CYCLIC_COUNT             (60)        // 循环统计次数(循环秒数现用于判断向上更新）

#define DUST_VALUE_LIMIT                (10)        // 尘数值不能超过平均值的10倍,否则丢弃

#define LCD_WIDTH                       16          // LCD宽度(字)
#define ADC_MAX_VALUE	                0xfff       // ADC的最大值
#define REFVOL			                5000L       // 参考电压

// 默认值
#define CLEANVOLTAGE                    360
#define SENSITIVITY                     5
#define CALIBRATE_A_AD                  0xffff
#define CALIBRATE_A_CONCENTRATION       0xffff
#define CALIBRATE_B_AD                  0xffff
#define CALIBRATE_B_CONCENTRATION       0xffff
#define DUST_CALC_N                     2.8
#define KN1                             1
#define KN2                             1

enum
{
    /* 串口命令 */
    CMD_SET_UART_OUTPUT_MOD          = 0x16,    // 设置输出模式
    
    CMD_CALIBRATE_A                  = 0x20,    // 浓度校准A
    CMD_CALIBRATE_B                  = 0x21,    // 浓度校准B
    CMD_CAL_N                        = 0x22,    // N值校准
    
    CMD_GET_CALIBRATE_VOL            = 0x30,    // 读取校准AD值
    CMD_GET_CURRENT_SMOKE            = 0x31,    // 读取烟
    CMD_GET_CALIBRATE_INFO           = 0x32,    // 读取校准信息
    CMD_GET_CURRENT_DUST             = 0x33,    // 读取尘
    CMD_GET_SENSOR_VALUES            = 0x34,    // 读取所有传感器值
    
    CMD_GET_SEC_VALUE                = 0x40,    // 获取最近一次ADC值
    CMD_GET_AVER_VALUE               = 0x41,    // 获取最近一秒ADC平均值
    CMD_GET_PWM_PERIOD               = 0x42,    // 获取PWM占空比
    CMD_GET_TMR_DELAY                = 0X43,    // 获取定时器延时
    
    CMD_SHIFT                        = 0x50,    // 换挡
    
    CMD_SET_PWM_PERIOD               = 0x60,    // 设置PWM占空比
    CMD_SET_TMR_DELAY                = 0X61,    // 设置定时器延时
    CMD_SET_K                        = 0x62,    // 设置K值
    CMD_SET_B                        = 0x63,    // 设置B值
    CMD_SET_Kn1                      = 0x64,    // 设置K1系数
    CMD_SET_Kn2                      = 0x65,    // 设置K2系数
    CMD_SET_CAL_N                    = 0x66,    // 设置N值
};

extern volatile enum UART_OUTPUT_MOD
{
    UARTMOD_DATA_LINK = 0,          // 该格式上电默认，直接输出浓度数据，每秒输出一次
    UARTMOD_DATA_AD,                // 连续输出AD数据
    UARTMOD_DEBUG,                  // printf 打印的调试数据可以允许输出，
    UARTMOD_NO_AUTO_OUTPUT,         // 关闭自动输出, 用于串口数据交换
    UARTMOD_CAL
}CurrentUARTMod;

/* 每个变量保存的位置 */
enum STORE_OFFSET
{
    STORE_OFFSET_ADDR = 0,
    STORE_OFFSET_SENSITIVITY,
    STORE_OFFSET_CLEANVOLTAGE,
    STORE_OFFSET_SHIFT1_DELAY,
    STORE_OFFSET_AD_A,
    STORE_OFFSET_C_A,
    STORE_OFFSET_AD_B,
    STORE_OFFSET_C_B,
    STORE_OFFSET_DUST_N,
    STORE_OFFSET_KN1,
    STORE_OFFSET_KN2,
};

// 外部接口
void DustSensor_Init(void);
void DustSensor_Process(void);

// 内部函数
void StoreCleanVoltage(void);
void ReadCleanVoltage(void);
void StoreSensitivity(void);
void ReadSensitivity(void);
void StoreCAndAD_A(void);
void ReadCAndAD_A(void);
void StoreCAndAD_B(void);
void ReadCAndAD_B(void);
void StoreDustN(void);
void ReadDustN(void);
void StoreKn1(void);
void ReadKn1(void);
void StoreKn2(void);
void ReadKn2(void);
void LoadDatas(void);

void SYS_Init(void);

void UART0_IRQHandler(void);
void UART0_Init(void);
void ProcessSerialCommand(void);
void Display_Min_Aver(uint16_t Min_Aver);
void PrintToLCD(uint8_t line, char* baseChars, uint32_t value, uint8_t offset);
void SendCalInfo(void);
void SendSensorValues(void);
_Bool GetMachineWorking(void);

void PWMA_IRQHandler(void);
void PWMA_Init(void);
void PWMA_SHIFT(uint8_t gear);
void Start_PWMA_INT(void);
void Stop_PWMA_INT(void);

void PWMB_Init(void);
void Set_PWMB_Output(uint16_t value);

void ADC_Auto_Cal(void);
void ADC_Complete(void);
void ADC_Init(void);
void StartADC(uint8_t adc_channel,uint16_t hbit);
void GetSensorAddr(void);

void TMR0_IRQHandler(void);
void TMR_Init(void);
void AdaptTMRDelay(void);
void StartTMR(void);

void MyDelayms(uint32_t ms);

#endif
