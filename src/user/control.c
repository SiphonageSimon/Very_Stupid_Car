#include "control.h"
float Kp, Ki;
float kp[10] = {0.2, 0.24, 0.3, 0.33, 0.5, 0.6, 0.7, 0.8, 0.9, 5.0};
float kd[3] = {0.02, 0.05, 0.1};
float ki[3] = {0.001, 0.005, 0.1};
int16_t integral_error[INTEGRAL_MAX] = {0};//偏差积分数组
int16_t i_error = 0; //偏差积分
int16_t last_error = 0; //上一次偏差值
int16_t error = 0;      //提线偏差值
int16_t error_differ;  //微分值
int16_t newDuty;       //最终输出占空比
uint16_t left_Spd = 0;
uint16_t right_Spd = 0; //左右速度
int Spd_Offset = 0;     //差速
int16_t leftVal, midVal, rightVal,leftVerVal,rightVerVal;
uint16_t left_times = 0; //左编码器读数
uint16_t right_times = 0; //右编码器读数
Threshold threshold;
///test

void simple_Ctrl(void)//长者的智慧，带阈值的控制
{
  int16_t ifout;
  leftVal = adc_fine[AD_LEFT];
  midVal = adc_fine[AD_MID];
  rightVal = adc_fine[AD_RIGHT];
  leftVerVal = adc_fine[AD_LEFT_VERTICAL];
  rightVerVal = adc_fine[AD_RIGHT_VERTICAL];
  last_error = error;
  //i_error = get_integral();
  newDuty = SteerMid; //回中
  FSM_select();
  float kp_error;
  if(midVal > 1000)
    kp_error =  0.2;
  else
  {
    kp_error = (1000 - midVal) * 0.00076;
  }
  error = leftVal - rightVal;
  error_differ = error - last_error;
  last_error = error;
  error = error * kp_error + (leftVerVal - rightVerVal) * 0.005 - error_differ * 0.014;
  //error = error * kp_error - error_differ * 0.012;
  //ifout = sqrt_get_error();
  //testVal = ifout;
  if( midVal < 160 && leftVal > rightVal)
  {
    error = TURN_MAX;
  }
  else if( midVal < 160 && leftVal < rightVal)
  {
    error = -TURN_MAX;
  }
  if(error > TURN_MAX)
  {
    error = TURN_MAX;
  }
  else if(error < - TURN_MAX)
  {
    error = -TURN_MAX;
  }
  
  newDuty = newDuty + error ;
  FTM_PWM_Duty(CFTM1, FTM_CH1, newDuty);
  if(gpio_get(PTE6) == 0)
    motor_Ctrl(17,17); //擦赛道
  else
  {
    if(error == TURN_MAX) //跑赛道
    {
      motor_Ctrl(21,16);
    }
    else if(error == - TURN_MAX)
    {
      motor_Ctrl(16,21);
    }
    else if(leftVerVal <= 5 && rightVerVal <= 5)
    {
      motor_Ctrl(28,28);
    }
    else
    {
      motor_Ctrl(21,21);
    }
  }

#if LINEAR_TEST

  //if(error < 0)
    //GetData(-error,rightVal-leftVal,0,0,OutData);
 // else
    GetData(error,leftVal-rightVal,error_differ,0,OutData);
 
#endif
  return;
  
}

int16_t sqrt_get_error(void)
{
  int16_t outputVal;
  double tempVal, tempLeftVal, tempRightVal;
  if(leftVal == rightVal)
  {
    return 0;
  }
  else
  {
    tempLeftVal = leftVal;
    tempRightVal = rightVal;
    tempVal = (sqrt(tempLeftVal) -sqrt(tempRightVal)) / (tempLeftVal - tempRightVal) * 1000;
    outputVal = tempVal;
    if(leftVal < rightVal)
      outputVal = -outputVal;
  }
  return outputVal;
}
void FSM_select(void)//有限状态机跳转
{
  switch (current_State)
  {
  case FSM_INIT:
    threshold.enter_crossroad = 700; //标定各个状态切换阈值
    threshold.enter_corner = 170;
    threshold.enter_straight = 30;
    threshold.enter_sturn = 400;
    threshold.no_val = 10;
    current_State = FSM_STRAIGHT;
    break;
  case FSM_STRAIGHT:
    if(leftVerVal > threshold.enter_crossroad && rightVerVal > threshold.enter_crossroad)
     current_State = FSM_STRAIGHT; //FSM_CROSSROAD;
    if(leftVerVal > threshold.enter_corner && rightVerVal < threshold.enter_straight && leftVal > threshold.enter_corner)
      current_State = FSM_LEFT_CORNER;
    else if(leftVerVal < threshold.enter_straight && rightVerVal > threshold.enter_corner && rightVal > threshold.enter_corner)
      current_State = FSM_RIGHT_CORNER;
    break;
  case FSM_LEFT_CORNER:
    if(leftVerVal > threshold.enter_crossroad && rightVerVal > threshold.enter_crossroad)
     current_State = FSM_STRAIGHT;
    else if((leftVerVal < threshold.enter_straight && rightVerVal < threshold.enter_straight) && (midVal > rightVal && midVal > leftVal))
      current_State = FSM_STRAIGHT;
    else if(rightVerVal > threshold.enter_sturn && leftVerVal < threshold.no_val  && rightVal > threshold.enter_corner)
      current_State = FSM_RIGHT_CORNER;
  //  else if(leftVerVal < threshold.enter_corner && rightVerVal > threshold.enter_corner)
    //  current_State = FSM_S_TURN;
   // else if(leftVerVal > threshold.enter_crossroad && rightVerVal > threshold.enter_crossroad)
   //   current_State = FSM_STRAIGHT; //FSM_CROSSROAD;
    break;
  case FSM_RIGHT_CORNER:
    if(leftVerVal > threshold.enter_crossroad && rightVerVal > threshold.enter_crossroad)
     current_State = FSM_STRAIGHT;
    else if((leftVerVal < threshold.enter_straight && rightVerVal < threshold.enter_straight) && (midVal > leftVal && midVal > rightVal))
      current_State = FSM_STRAIGHT;
    else if(leftVerVal > threshold.enter_sturn && rightVerVal < threshold.no_val && leftVal > threshold.enter_corner)
      current_State = FSM_LEFT_CORNER;
   // else if(leftVerVal > threshold.enter_corner && rightVerVal < threshold.enter_corner)
     // current_State = FSM_S_TURN;
   // else if(leftVerVal > threshold.enter_crossroad && rightVerVal > threshold.enter_crossroad)
   //   current_State = FSM_STRAIGHT; //FSM_CROSSROAD;
    break;
  case FSM_S_TURN:
    if((leftVerVal < threshold.enter_straight && rightVerVal < threshold.enter_straight))
      current_State = FSM_STRAIGHT;
    break;
  case FSM_CROSSROAD:
    if(leftVerVal > threshold.enter_corner && rightVerVal < threshold.enter_corner)
      current_State = FSM_LEFT_CORNER;
    else if(leftVerVal < threshold.enter_corner && rightVerVal > threshold.enter_corner)
      current_State = FSM_RIGHT_CORNER;
    break;
  case FSM_OUT_OF_COURSE:
    break;
  default:
    break;    
  }
}

void FSM_Ctrl(void)
{
  int16_t LRjudge, LMjudge, MRjudge; //判断线在做还是在右，左正右负
  const float linear_ratio1 = 3.5; //用于线性化的参数
  const float compensate_ratio0 = 700.0;
  const float compensate_ratio1 = 0.667;
  const float compensate_ratio2 = 700.0; //左电感修正系数
  const float compensate_ratio3 = 700.0; //左电感修正系数
  newDuty = SteerMid; //回中
  
  leftVal = adc_fine[AD_LEFT];
  midVal = adc_fine[AD_MID];
  rightVal = adc_fine[AD_RIGHT];
  leftVerVal = adc_fine[AD_LEFT_VERTICAL];
  rightVerVal = adc_fine[AD_RIGHT_VERTICAL];
  
  last_error = error; //储存上次error值
  LRjudge = leftVal - rightVal;
  LMjudge = leftVal - midVal;
  MRjudge = midVal - rightVal;
  FSM_select();//状态跳转
  //提线线性化部分
  if(leftVal > midVal && leftVal > rightVal)
  {
    error = 650;
  }
  else if(rightVal > midVal && rightVal > leftVal)
  {
    error = -650;
  }
  else
  {
    switch(current_State)
    {
    case FSM_STRAIGHT:
      error = LRjudge;
      break;
    case FSM_S_TURN:
      error = LRjudge;
      break;
    case FSM_LEFT_CORNER:
      error = LRjudge;
      break;
    case FSM_RIGHT_CORNER:
      error = LRjudge;
      break;
    }
  }
  /*
  if(midVal > 1000)
  {
    error = LRjudge;
  }
  else if(leftVal > midVal && leftVal > rightVal)
  {
    //error = linear_ratio1 * (1000 - midVal) - compensate_ratio1 * (1000 - midVal - compensate_ratio0) - compensate_ratio2;
    error = 800;
  }
  else if(rightVal > midVal && rightVal > leftVal)
  {
    //error = linear_ratio1 * (1000 - midVal) - compensate_ratio1 * (1000 - midVal - compensate_ratio0) - compensate_ratio3;
    //error = -error;
    error = -800;
  }
  else
  {
    error = LRjudge;
  }
  */
#if LINEAR_TEST
  if(error < 0)
    error = -error;
  if(LRjudge <0)
    LRjudge = -LRjudge;
  GetData(error,LRjudge,0,0,OutData);
#endif

  error_differ = error - last_error;

  switch(current_State)
  {
  case FSM_STRAIGHT:
    newDuty += error * kp[1] + error_differ * kd[0];
    break;
  case FSM_LEFT_CORNER:
    if(error > -100)
      newDuty += error * kp[2] + error_differ * kd[1];
    break;
  case FSM_RIGHT_CORNER:
    if(error < 100)
      newDuty += error * kp[2] + error_differ * kd[1];
    break;
  case FSM_S_TURN:
      newDuty += error * kp[0];
    break;
  default:
    break;
  }
  
  if(newDuty > SteerMid + TURN_MAX)
    newDuty = SteerMid + TURN_MAX;
  else if(newDuty < SteerMid - TURN_MAX)
    newDuty = SteerMid - TURN_MAX;
  FTM_PWM_Duty(CFTM1, FTM_CH1, newDuty);
  
  motor_Ctrl(20,20);
  return;
}

//速度闭环函数，目前用作单纯速度给定
void motor_Ctrl(uint16_t left_Spd_Set, uint16_t right_Spd_Set)
{
  if(leftVal==0 && midVal==0 && rightVal==0)
  {
    FTM_PWM_Duty(CFTM2, FTM_CH3, 0);   //PWM2 PTB3
    FTM_PWM_Duty(CFTM2, FTM_CH4, 0);  //PWM2 PTB2   
    return;
  }
#if BANGBANG_ENABLE
  if(left_Spd_Set <= BB_DEAD_ZONE)
    FTM_PWM_Duty(CFTM2, FTM_CH3, BB_DUTY_MIN);
  else if(left_times < left_Spd_Set - BB_DEAD_ZONE)
     FTM_PWM_Duty(CFTM2, FTM_CH3, BB_DUTY_MAX);
  else if(left_times > left_Spd_Set + BB_DEAD_ZONE)
     FTM_PWM_Duty(CFTM2, FTM_CH3, BB_DUTY_MIN);
  
  if(right_Spd_Set <= BB_DEAD_ZONE)
     FTM_PWM_Duty(CFTM2, FTM_CH4, BB_DUTY_MIN);
  else if(right_times < right_Spd_Set - BB_DEAD_ZONE)
     FTM_PWM_Duty(CFTM2, FTM_CH4, BB_DUTY_MAX);
  else if(right_times > right_Spd_Set + BB_DEAD_ZONE)
     FTM_PWM_Duty(CFTM2, FTM_CH4, BB_DUTY_MIN);
  
#else
  FTM_PWM_Duty(CFTM2, FTM_CH3, left_Spd_Set);   //PWM2 PTB3
  FTM_PWM_Duty(CFTM2, FTM_CH4, right_Spd_Set);  //PWM2 PTB2   
#endif
  return;
}

void servo_Ctrl(void)//烂得一匹
{
  int16_t LRjudge; //判断线在做还是在右，左正右负
  newDuty = SteerMid; //回中
  leftVal = adc_fine[AD_LEFT];
  midVal = adc_fine[AD_MID];
  rightVal = adc_fine[AD_RIGHT];
  leftVerVal = adc_fine[AD_LEFT_VERTICAL];
  rightVerVal = adc_fine[AD_RIGHT_VERTICAL];
  last_error = error; //储存上次error值
  LRjudge = leftVal - rightVal;
  FSM_select();//状态跳转
  //fuzzy_Ctrl(); //此函数仅调节Kp
  if(leftVal == 0 && rightVal == 0)
  {
     FTM_PWM_Duty(CFTM2, FTM_CH3, 0);//PWM0 PTB3              //�����
     FTM_PWM_Duty(CFTM2, FTM_CH4, 0);//PWM0 PTB2              //�����
     return;
  }
  if(midVal <= 1000)
  {
    error = 1000 - midVal;
    if(error < 150)
    {
      //error *= kp0;
      Spd_Offset = 0;
    }
    else if(error < 200)
    {
      //error *= kp1;
      Spd_Offset = 0;
     }
    else if(error < 500)
    {
      //error *= kp2;
      Spd_Offset = 100;
    }
    else
    {
      //error *= kp3;
      Spd_Offset = 200;
    }
    switch(current_State)
    {
    case FSM_STRAIGHT:
      error *= kp[0];
      break;
    default:
      error *= kp[3];
      break;
    }
    if(LRjudge > 0) //左减右为正，线位于车右侧，右转
    {
      error = error;
      FTM_PWM_Duty(CFTM2, FTM_CH3, 6800);//PWM0 PTB3              //�����
      FTM_PWM_Duty(CFTM2, FTM_CH4, 6800 - Spd_Offset);//PWM0 PTB2              //�����
    }
    else //左减右为负，线位于车左侧，左转
    {
      error = -error;
      FTM_PWM_Duty(CFTM2, FTM_CH3, 6800 - Spd_Offset);//PWM0 PTB3              //�����
      FTM_PWM_Duty(CFTM2, FTM_CH4, 6800);//PWM0 PTB2              //�����
    }
  }
  if(error > TURN_MAX)
    error = TURN_MAX;
  else if(error < - TURN_MAX)
    error = -TURN_MAX;
  newDuty = newDuty + error ;
  FTM_PWM_Duty(CFTM1, FTM_CH1, newDuty);
}

//调整舵机的k而已，不是什么正经模糊
void fuzzy_Control(void)
{

  error_differ = error - last_error;
  
}

void get_spd(void)
{
  left_times =left_Counter;
  left_Counter = 0;
  right_times =FTM_count_get(CFTM0);
  FTM_count_clean(CFTM0);
  return;
}
void input_integral(void)
{
  static uint8_t position = 0;
  integral_error[position] = error;
  position++;
  if(position >= INTEGRAL_MAX)
     position = 0;
  return;
}


int16_t get_integral(void)
{
  int16_t temp = 0;
  uint8_t position;
  for(position = 0; position < INTEGRAL_MAX; position++)
  {
    temp += integral_error[position];
  }
  return temp;
}

int getErrorVal(void)
{
  return error;
}

uint16_t getNewDutyVal(void)
{
  return newDuty; 
}

uint8_t getCurrentState(void)
{
  return current_State;
}

uint8_t get_left_spd(void)
{
  return left_times;
}

uint8_t get_right_spd(void)
{
  return right_times;
}
