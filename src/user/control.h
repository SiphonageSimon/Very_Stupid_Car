#include "include.h"

//adc对应电感位置
#define AD_LEFT 0
#define AD_MID  1
#define AD_RIGHT 3
#define AD_LEFT_VERTICAL 2
#define AD_RIGHT_VERTICAL 4

/*电感布置方案
▋LEFT_VERTICAL   ▋RIGHT_VERTICAL




▃       ▃       ▃
LEFT     MID     RIGHT
*/

//状态机相关跳转参数
typedef struct Tres                     //状态切换阈值结构体，仅仅是觉得大写看着难受
{
  uint8_t enter_crossroad;              //进入十字路口
  uint8_t enter_corner;                 //进入弯道
  uint8_t enter_straight;               //进入直道
  uint8_t enter_sturn;                  //进入连续弯
  uint8_t no_val;                       //无读数，默认为0
}Threshold;

//舵机相关阈值
#define TURN_THRES 750
#define ERROR_THRES 300
#define ENABLE_THRES 1200
#define INTEGRAL_MAX 30
//速度相关
#define BB_DUTY_MAX 9000                //bangbang1
#define BB_DUTY_MAX 0                   //bangbang0
#define BB_DEAD_ZONE 20                 //bangbang死区
#define SPD_GOAL 0                      //终点车速
#define BANGBANG_ENABLE 0               //是否使用bangbang控制
//模糊相关参数
#define NB -1000
#define NM -400
#define NS -200
#define ZO 0
#define PS 200
#define PM 400
#define PB 1000

//用户自定义函数
void FSM_Ctrl(void);
void motor_Ctrl(uint16_t left_Spd, uint16_t right_Spd);
void servo_Ctrl(void);
void FSM_select(void);
void fuzzy_Ctrl(void);
void simple_Ctrl(void);
int16_t get_integral(void);
void input_integral(void);

