/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】龙邱S9KEAZ128智能车MINI核心板母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2017年11月11日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4
【Target  】S9KEAZ128
【Crystal 】16.000Mhz
【busclock】40.000MHz
【pllclock】40.000MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
/*
ADC_CHANNEL_AD0 -------------- A0   
ADC_CHANNEL_AD1 -------------- A1
ADC_CHANNEL_AD2 -------------- A6 
ADC_CHANNEL_AD3 -------------- A7
ADC_CHANNEL_AD4 -------------- B0
ADC_CHANNEL_AD5 -------------- B1  
ADC_CHANNEL_AD6 -------------- B2
ADC_CHANNEL_AD7 -------------- B3
ADC_CHANNEL_AD8 -------------- C0
ADC_CHANNEL_AD9 -------------- C1 
ADC_CHANNEL_AD10 ------------- C2
ADC_CHANNEL_AD11 ------------- C3
ADC_CHANNEL_AD12 ------------- F4
ADC_CHANNEL_AD13 ------------- F5 
ADC_CHANNEL_AD14 ------------- F6
ADC_CHANNEL_AD15 ------------- F7

UARTR0:PTA3 TX ,PTA2 RX
UARTR1:PTF3 TX ,PTF2 RX
UARTR2:PTI1 TX ,PTI0 RX
*/

/*************************************************************************
*  函数名称：ADC_Init
*  功能说明：AD初始化
*  函数返回：无
*  参数说明：ch  :采集通道
*           bit ：采集位数
*  for example  : ADC_Init(ADC_CHANNEL_AD4,ADC_12BIT);
*************************************************************************/
void ADC_Init( ADCHn  ch,ADC_nbit bit)
{
        SIM->SCGC |= SIM_SCGC_ADC_MASK;   //开启时钟原
        
        ADC->SC3  = (0
                      |ADC_SC3_ADICLK(BUS_CLOCK)   //选择系统时钟
                      |ADC_SC3_MODE(bit)      //8位AD采集
                      //|ADC_SC3_ADLSMP_MASK         //低功耗采集
                      |ADC_SC3_ADIV(ADC_ADIV_1)      //分频为1
                      //|ADC_SC3_ADLPC_MASK            //长步长时间
                    ) ;
        ADC->SC2  = (0
                     // |ADC_SC2_ADTRG_MASK             //1硬件触发,0软件触发
                    //  |ADC_SC2_ACFE_MASK            //采集比较
                    //  |ADC_SC2_ACFGT_MASK           //大于比较值，比较触发
                    ) ;
       ADC->APCTL1 = ADC_APCTL1_ADPC(1<< ch) ; 
       ADC->SC1  = (0
                    |ADC_SC1_ADCH(ch)             //选择采集通道
                   // |ADC_SC1_ADCO_MASK            //连续采集
                   // |ADC_SC1_AIEN_MASK           //中断
                    ) ;
}

/*************************************************************************
*  函数名称：adc_init
*  功能说明：获取ADC采样值(不支持B通道)
*  参数说明：adcn_ch      ADC通道
*           bit          ADC精度（ ADC_8bit,ADC_12bit, ADC_10bit）
*  for example           adc_once(ADC_CHANNEL_AD4,ADC_12BIT);
*************************************************************************/
uint16_t adc_once(ADCHn adcn_ch, ADC_nbit bit) //采集某路模拟量的AD值
{
  uint16_t result = 0;
  ADC_Init( adcn_ch , bit) ;    //启动ADC转换
  while ((ADC->SC1 & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK);   //只支持 A通道
  result = ADC->R;
  ADC->SC1 &= ~ADC_SC1_COCO_MASK;
  return result;
}
/*************************************************************************
*  函数名称：ad_ave
*  功能说明：多次采样，取平均值
*  参数说明： adcn_ch 通道号
*            bit     精度（ ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ）
*            N       均值滤波次数(范围:0~255)
*  for example       adc_once(ADC_CHANNEL_AD4,ADC_12BIT,10);
*************************************************************************/
uint16_t adc_ave(ADCHn adcn_ch, ADC_nbit bit, uint8_t N) //均值滤波
{
    uint32_t tmp = 0;
    uint8_t  i;
    for(i = 0; i < N; i++)
        tmp += adc_once(adcn_ch, bit);
    tmp = tmp / N;
    return (uint16_t)tmp;
}

/*************************************************************************
*  函数名称：adc_sum
*  功能说明：多次采样，求和
*  参数说明：adcn_ch      通道号
*            bit         精度（ ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ）
*            N           求和次数(范围:0~255)
*  for example           adc_sum(ADC_CHANNEL_AD4,ADC_12BIT,10);
*************************************************************************/
uint16_t adc_sum(ADCHn adcn_ch, ADC_nbit bit, uint8_t N)
{
    uint32_t tmp = 0;
    uint8_t  i;
    for(i = 0; i < N; i++)
        tmp += adc_once(adcn_ch, bit);
    return (uint16_t)tmp;
}


void TestADC(void)
{  
  uint8_t  txt[30]="X:";
  uint16 ad[16] = {0};  
 
  ADC_Init(ADC_CHANNEL_AD11,ADC_12BIT);
  uart_init(UARTR2,Remap,9600);
  LCD_Init();  
  while(1)
  { 
    for(int i=4;i<8;i++)
    {
      ad[i]=adc_once((ADCHn)i,ADC_12BIT);
      sprintf((char*)txt,"AD%02d:%04d",i,ad[i]);
      Uart_SendString(UARTR2,txt);
      if(!(i%2))
        LCD_P6x8Str((1*(i%2)),(i/2),txt);
      else
       LCD_P6x8Str((64*(i%2)),(i/2),txt);
    }
    for(int i=12;i<16;i++)
    {
      ad[i]=adc_once((ADCHn)i,ADC_12BIT);
      sprintf((char*)txt,"AD%02d:%04d",i,ad[i]);
      Uart_SendString(UARTR2,txt);
      if(!(i%2))
        LCD_P6x8Str((1*(i%2)),(i/2),txt);
      else
       LCD_P6x8Str((64*(i%2)),(i/2),txt);
    }
    time_delay_ms(5);
  }
}


/*以下是用户自定义函数和变量*/

uint16_t adc_raw[6] = {0,0,0,0,0,0};                    //ADC函数读取的原始值
uint16_t adc_max[6] = {0,0,0,0,0,0};                    //归一化读到的最小值
uint16_t adc_min[6] = {9999,9999,9999,9999,9999,9999};  //归一化读到的最大值

//所有电感读取一次,输出到adc_raw数组中
void read_raw_AD(void)
{
  adc_raw[0] = adc_ave(ADC0, ADC_12BIT, AD_AVETIME);
  adc_raw[1] = adc_ave(ADC1, ADC_12BIT, AD_AVETIME);
  adc_raw[2] = adc_ave(ADC2, ADC_12BIT, AD_AVETIME);
  adc_raw[3] = adc_ave(ADC3, ADC_12BIT, AD_AVETIME);
  adc_raw[4] = adc_ave(ADC4, ADC_12BIT, AD_AVETIME);
  adc_raw[5] = adc_ave(ADC5, ADC_12BIT, AD_AVETIME);  
  return;
}

//归一化，最小值输出到adc_min,最大值输出到adc_max
void AD_value_init(void)
{
  uint8_t i = 0;
  while(gpio_get(PTE6) == 0)
  {
    read_raw_AD();
    for(i = 0; i < 6; i++)
    {
      if(adc_raw[i] > adc_max[i])
        adc_max[i] = adc_raw[i];
      if(adc_raw[i] < adc_min[i])
        adc_min[i] = adc_raw[i];
    }
  }
  //current_State = FSM_STRAIGHT;
  return;
}

//读取归一化后值，输出到adc_res数组中
//惯性滤波器 Y(n)=αX(n) (1-α)Y(n-1)
//输出到adc_fine数组中
void read_AD(void)
{
  uint8_t i;
  uint16_t temp_val;
  read_raw_AD();
  
  adc_res_pos++;
  if(adc_res_pos >= AD_BUFFER)
    adc_res_pos = 0;
  
  for(i = 0; i < 6; i++)
  {
    temp_val = adc_raw[i];
    if(temp_val < adc_min[i])
      temp_val = adc_min[i];
    adc_res[i][adc_res_pos] = AD_VALUE_1 * (temp_val - adc_min[i]) / (adc_max[i] - adc_min[i]);
    adc_fine[i] = AD_ALPHA * adc_res[i][adc_res_pos] + (1.0 - AD_ALPHA) * adc_fine[i];
  }
  return;
}
