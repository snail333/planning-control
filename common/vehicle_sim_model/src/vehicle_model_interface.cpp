#include "vehicle_sim_model/vehicle_model_interface.h"

VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u) : dim_x_(dim_x), dim_u_(dim_u)
{
  state_ = Eigen::VectorXd::Zero(dim_x_);
  input_ = Eigen::VectorXd::Zero(dim_u_);
}

void VehicleModelInterface::updateRungeKutta(const double& dt, const Eigen::VectorXd& input)
{
  Eigen::VectorXd k1 = calcModel(state_, input);
  Eigen::VectorXd k2 = calcModel(state_ + k1 * 0.5 * dt, input);
  Eigen::VectorXd k3 = calcModel(state_ + k2 * 0.5 * dt, input);
  Eigen::VectorXd k4 = calcModel(state_ + k3 * dt, input);

  state_ += 1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt;
}
void VehicleModelInterface::updateEuler(const double& dt, const Eigen::VectorXd& input)
{
  state_ += calcModel(state_, input) * dt;
}
const Eigen::VectorXd& VehicleModelInterface::getState() const
{
  return state_;
}
const Eigen::VectorXd& VehicleModelInterface::getInput() const
{
  return input_;
}
void VehicleModelInterface::setState(const Eigen::VectorXd& state)
{
  state_ = state;
}
void VehicleModelInterface::setInput(const Eigen::VectorXd& input)
{
  input_ = input;
}
