#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <math.h>

typedef struct PID_TypeDef
{
  double Kp;                       //比例系数Proportional
  double Ki;                       //积分系数Integral
  double Kd;                       //微分系数Derivative
}PID_TypeDef;

namespace waypoint_follower
{
class LongitudinalControl
{
public:
  LongitudinalControl();
  ~LongitudinalControl();

  // for setting data
  void setCurrentSpeed(const double& realspeed)
  {
    currentspeed = realspeed;
  }

  void setCurrentPitch(const double& realpitch)
  {
    currentpitch = realpitch;
  }

  void setTargetSpeed(const double& exptspeed)
  {
    targetspeed = exptspeed;
  }

  // processing
  void SetPID();
  double Speed_PID_Acc();
  double Speed_PID_AccAndTorque(const double goalaccelation);
  double Control(const double error, const double dt);

private:
  ros::NodeHandle private_nh;
  // constant
  double delta;//旋转质量转换系数
  double gamma;//摩擦系数
  double m_b;//汽车质量
  double r_b;//轮胎半径
  double i_b;//减速比
  double A_b;//横截面积
  double C_w;//风阻系数

  // variables
  double targetspeed;
  double currentspeed;
  double currentpitch;
  double kp_;
  double ki_;
  double kd_;
  double dt;
  double integrator_level;
  PID_TypeDef PID_;
  bool first_hit_;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  int integrator_saturation_status_ = 0;
  double integrator_saturation_high_ = 0.0;
  double integrator_saturation_low_ = 0.0;

};
}  // namespace waypoint_follower

#endif  // PID_CONTROL_H
