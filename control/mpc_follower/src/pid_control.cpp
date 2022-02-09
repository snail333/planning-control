#include <mpc_follower/pid_control.h>
using namespace std;

namespace waypoint_follower
{

LongitudinalControl::LongitudinalControl()
	:private_nh("~")
  ,delta(11.4)
  ,gamma(0.017)
	,m_b(2720)
	,r_b(0.36425)
	,i_b(9.07)
	,A_b(3.26502)
	,C_w(0.27)
  ,kp_(0.2)
  ,ki_(0.1)
  ,kd_(0)
  ,dt(0.02)
  ,integrator_level(0.5)
{ 
  private_nh.param<double>("dtime", dt, 0.02);
  private_nh.param<double>("integrator_level", integrator_level, 0.5);
  private_nh.param<double>("kp", kp_, 0.2);
  private_nh.param<double>("ki", ki_, 0.1);
  private_nh.param<double>("kd", kd_, 0);
  first_hit_ = true;
  SetPID();
}

LongitudinalControl::~LongitudinalControl()
{
}

void LongitudinalControl::SetPID()
{
  PID_.Kp = kp_;
  PID_.Ki = ki_;
  PID_.Kd = kd_;
  integrator_saturation_high_ = std::fabs(integrator_level);
  integrator_saturation_low_ = -std::fabs(integrator_level);
}

double LongitudinalControl::Speed_PID_AccAndTorque(const double goalaccelation)
{
  double goaltorque = 0;
  double f_1 = 0;       //滚动阻力
  double f_2 = 0;       //空气阻力
  double f_3 = 0;       //坡度阻力

  f_1 = m_b * gamma * cos(currentpitch) * 9.8;
  f_2 = A_b * C_w * currentspeed * currentspeed / 16.0;
  f_3 = m_b * sin(currentpitch) * 9.8;
  goaltorque = (delta * m_b * goalaccelation + f_1 + f_2 + f_3) * r_b / i_b;

  return goaltorque;
}

double LongitudinalControl::Speed_PID_Acc()
{
  double accelation = 0;
  double error = 0;

  error = targetspeed - currentspeed;
  accelation = Control(error, dt);

  accelation = fabs(accelation) < 0.01? 0 : accelation;  //accelation_deadzone
  return accelation;
}

double LongitudinalControl::Control(const double error, const double dt)
{
  if (dt <= 0) 
  {
    ROS_INFO_STREAM("dt <= 0, will use the last output, dt: " << dt);
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  if (first_hit_) 
    first_hit_ = false;
  else
    diff = (error - previous_error_) / dt;

  if(fabs(error) < 1e-3)  //steady state error
    integral_ = 0;
  else
    integral_ += error * dt * PID_.Ki;

  // apply Ki before integrating to avoid steps when change Ki at steady state
  if (integral_ > integrator_saturation_high_)
  {
    integral_ = integrator_saturation_high_;
    integrator_saturation_status_ = 1;
  } 
  else if (integral_ < integrator_saturation_low_) 
  {
    integral_ = integrator_saturation_low_;
    integrator_saturation_status_ = -1;
  } 
  else 
  {
    integrator_saturation_status_ = 0;
  }

  previous_error_ = error;
  output = error * PID_.Kp + integral_ + diff * PID_.Kd;  // Ki already applied
  if(fabs(currentspeed) < 1 && output > 0.8)
    output = 0.8;
  previous_output_ = output;
  return output;
}

}  // namespace waypoint_follower 
