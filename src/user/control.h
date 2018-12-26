#include "include.h"

//adc��Ӧ���λ��
#define AD_LEFT 0
#define AD_MID  1
#define AD_RIGHT 3
#define AD_LEFT_VERTICAL 2
#define AD_RIGHT_VERTICAL 4

/*��в��÷���
��LEFT_VERTICAL   ��RIGHT_VERTICAL




�z       �z       �z
LEFT     MID     RIGHT
*/

//״̬�������ת����
typedef struct Tres                     //״̬�л���ֵ�ṹ�壬�����Ǿ��ô�д��������
{
  uint8_t enter_crossroad;              //����ʮ��·��
  uint8_t enter_corner;                 //�������
  uint8_t enter_straight;               //����ֱ��
  uint8_t enter_sturn;                  //����������
  uint8_t no_val;                       //�޶�����Ĭ��Ϊ0
}Threshold;

//��������ֵ
#define TURN_THRES 750
#define ERROR_THRES 300
#define ENABLE_THRES 1200
#define INTEGRAL_MAX 30
//�ٶ����
#define BB_DUTY_MAX 9000                //bangbang1
#define BB_DUTY_MAX 0                   //bangbang0
#define BB_DEAD_ZONE 20                 //bangbang����
#define SPD_GOAL 0                      //�յ㳵��
#define BANGBANG_ENABLE 0               //�Ƿ�ʹ��bangbang����
//ģ����ز���
#define NB -1000
#define NM -400
#define NS -200
#define ZO 0
#define PS 200
#define PM 400
#define PB 1000

//�û��Զ��庯��
void FSM_Ctrl(void);
void motor_Ctrl(uint16_t left_Spd, uint16_t right_Spd);
void servo_Ctrl(void);
void FSM_select(void);
void fuzzy_Ctrl(void);
void simple_Ctrl(void);
int16_t get_integral(void);
void input_integral(void);
int16_t sqrt_get_error(void);
void get_spd(void);
uint8_t get_left_spd(void);
uint8_t get_right_spd(void);

