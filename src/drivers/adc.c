/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨������S9KEAZ128���ܳ�MINI���İ�ĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
�������汾��V1.0
�������¡�2017��11��11��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.80.4
��Target  ��S9KEAZ128
��Crystal ��16.000Mhz
��busclock��40.000MHz
��pllclock��40.000MHz
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
*  �������ƣ�ADC_Init
*  ����˵����AD��ʼ��
*  �������أ���
*  ����˵����ch  :�ɼ�ͨ��
*           bit ���ɼ�λ��
*  for example  : ADC_Init(ADC_CHANNEL_AD4,ADC_12BIT);
*************************************************************************/
void ADC_Init( ADCHn  ch,ADC_nbit bit)
{
        SIM->SCGC |= SIM_SCGC_ADC_MASK;   //����ʱ��ԭ
        
        ADC->SC3  = (0
                      |ADC_SC3_ADICLK(BUS_CLOCK)   //ѡ��ϵͳʱ��
                      |ADC_SC3_MODE(bit)      //8λAD�ɼ�
                      //|ADC_SC3_ADLSMP_MASK         //�͹��Ĳɼ�
                      |ADC_SC3_ADIV(ADC_ADIV_1)      //��ƵΪ1
                      //|ADC_SC3_ADLPC_MASK            //������ʱ��
                    ) ;
        ADC->SC2  = (0
                     // |ADC_SC2_ADTRG_MASK             //1Ӳ������,0��������
                    //  |ADC_SC2_ACFE_MASK            //�ɼ��Ƚ�
                    //  |ADC_SC2_ACFGT_MASK           //���ڱȽ�ֵ���Ƚϴ���
                    ) ;
       ADC->APCTL1 = ADC_APCTL1_ADPC(1<< ch) ; 
       ADC->SC1  = (0
                    |ADC_SC1_ADCH(ch)             //ѡ��ɼ�ͨ��
                   // |ADC_SC1_ADCO_MASK            //�����ɼ�
                   // |ADC_SC1_AIEN_MASK           //�ж�
                    ) ;
}

/*************************************************************************
*  �������ƣ�adc_init
*  ����˵������ȡADC����ֵ(��֧��Bͨ��)
*  ����˵����adcn_ch      ADCͨ��
*           bit          ADC���ȣ� ADC_8bit,ADC_12bit, ADC_10bit��
*  for example           adc_once(ADC_CHANNEL_AD4,ADC_12BIT);
*************************************************************************/
uint16_t adc_once(ADCHn adcn_ch, ADC_nbit bit) //�ɼ�ĳ·ģ������ADֵ
{
  uint16_t result = 0;
  ADC_Init( adcn_ch , bit) ;    //����ADCת��
  while ((ADC->SC1 & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK);   //ֻ֧�� Aͨ��
  result = ADC->R;
  ADC->SC1 &= ~ADC_SC1_COCO_MASK;
  return result;
}
/*************************************************************************
*  �������ƣ�ad_ave
*  ����˵������β�����ȡƽ��ֵ
*  ����˵���� adcn_ch ͨ����
*            bit     ���ȣ� ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ��
*            N       ��ֵ�˲�����(��Χ:0~255)
*  for example       adc_once(ADC_CHANNEL_AD4,ADC_12BIT,10);
*************************************************************************/
uint16_t adc_ave(ADCHn adcn_ch, ADC_nbit bit, uint8_t N) //��ֵ�˲�
{
    uint32_t tmp = 0;
    uint8_t  i;
    for(i = 0; i < N; i++)
        tmp += adc_once(adcn_ch, bit);
    tmp = tmp / N;
    return (uint16_t)tmp;
}

/*************************************************************************
*  �������ƣ�adc_sum
*  ����˵������β��������
*  ����˵����adcn_ch      ͨ����
*            bit         ���ȣ� ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ��
*            N           ��ʹ���(��Χ:0~255)
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


/*�������û��Զ��庯���ͱ���*/

uint16_t adc_raw[6] = {0,0,0,0,0,0};                    //ADC������ȡ��ԭʼֵ
uint16_t adc_max[6] = {0,0,0,0,0,0};                    //��һ����������Сֵ
uint16_t adc_min[6] = {9999,9999,9999,9999,9999,9999};  //��һ�����������ֵ

//���е�ж�ȡһ��,�����adc_raw������
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

//��һ������Сֵ�����adc_min,���ֵ�����adc_max
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

//��ȡ��һ����ֵ�������adc_res������
//�����˲��� Y(n)=��X(n) (1-��)Y(n-1)
//�����adc_fine������
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