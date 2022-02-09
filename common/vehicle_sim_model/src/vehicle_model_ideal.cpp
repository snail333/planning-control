#include "vehicle_sim_model/vehicle_model_ideal.h"

VehicleModelIdealTwist::VehicleModelIdealTwist() : VehicleModelInterface(3 /* dim x */, 2 /* dim u */) {}

const double VehicleModelIdealTwist::getX() const
{
  return state_(static_cast<Eigen::Index>(IDX::X));
}
const double VehicleModelIdealTwist::getY() const
{
  return state_(static_cast<Eigen::Index>(IDX::Y));
}
const double VehicleModelIdealTwist::getYaw() const
{
  return state_(static_cast<Eigen::Index>(IDX::YAW));
}
const double VehicleModelIdealTwist::getVx() const
{
  return input_(static_cast<Eigen::Index>(IDX_U::VX_DES));
}
const double VehicleModelIdealTwist::getWz() const
{
  return input_(static_cast<Eigen::Index>(IDX_U::WZ_DES));
}
const double VehicleModelIdealTwist::getSteer() const
{
  return 0.0;
}
void VehicleModelIdealTwist::update(const double& dt)
{
  updateRungeKutta(dt, input_);
}
Eigen::VectorXd VehicleModelIdealTwist::calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
  const double yaw = state(static_cast<Eigen::Index>(IDX::YAW));
  const double vx = input(static_cast<Eigen::Index>(IDX_U::VX_DES));
  const double wz = input(static_cast<Eigen::Index>(IDX_U::WZ_DES));

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(static_cast<Eigen::Index>(IDX::X)) = vx * cos(yaw);
  d_state(static_cast<Eigen::Index>(IDX::Y)) = vx * sin(yaw);
  d_state(static_cast<Eigen::Index>(IDX::YAW)) = wz;

  return d_state;
}

VehicleModelIdealSteer::VehicleModelIdealSteer(double wheelbase)
  : VehicleModelInterface(3 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase) {}

const double VehicleModelIdealSteer::getX() const
{
  return state_(static_cast<Eigen::Index>(IDX::X));
}
const double VehicleModelIdealSteer::getY() const
{
  return state_(static_cast<Eigen::Index>(IDX::Y));
}
const double VehicleModelIdealSteer::getYaw() const
{
  return state_(static_cast<Eigen::Index>(IDX::YAW));
}
const double VehicleModelIdealSteer::getVx() const
{
  return input_(static_cast<Eigen::Index>(IDX_U::VX_DES));
}
const double VehicleModelIdealSteer::getWz() const
{
  return input_(static_cast<Eigen::Index>(IDX_U::VX_DES))
    * std::tan(input_(static_cast<Eigen::Index>(IDX_U::STEER_DES))) / wheelbase_;
}
const double VehicleModelIdealSteer::getSteer() const
{
  return input_(static_cast<Eigen::Index>(IDX_U::STEER_DES));
}
void VehicleModelIdealSteer::update(const double& dt)
{
  updateRungeKutta(dt, input_);
}
Eigen::VectorXd VehicleModelIdealSteer::calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
  const double yaw = state(static_cast<Eigen::Index>(IDX::YAW));
  const double vx = input(static_cast<Eigen::Index>(IDX_U::VX_DES));
  const double steer = input(static_cast<Eigen::Index>(IDX_U::STEER_DES));

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(static_cast<Eigen::Index>(IDX::X)) = vx * cos(yaw);
  d_state(static_cast<Eigen::Index>(IDX::Y)) = vx * sin(yaw);
  d_state(static_cast<Eigen::Index>(IDX::YAW)) = vx * std::tan(steer) / wheelbase_;

  return d_state;
}
