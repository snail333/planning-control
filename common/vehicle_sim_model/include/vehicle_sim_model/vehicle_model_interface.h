/**
 * @file vehicle_model_interface.h
 * @brief vehicle model interface class
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef VEHICLE_SIM_MODEL_VEHICLE_MODEL_INTERFACE_H
#define VEHICLE_SIM_MODEL_VEHICLE_MODEL_INTERFACE_H

#include <eigen3/Eigen/Core>

/**
 * @class vehicle model class
 * @brief calculate vehicle dynamics
 */
class VehicleModelInterface
{
protected:
  const int dim_x_;        //!< @brief dimension of state x
  const int dim_u_;        //!< @brief dimension of input u
  Eigen::VectorXd state_;  //!< @brief vehicle state vector
  Eigen::VectorXd input_;  //!< @brief vehicle input vector

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   */
  VehicleModelInterface(int dim_x, int dim_u);

  /**
   * @brief get state vector of model
   */
  const Eigen::VectorXd& getState() const;

  /**
   * @brief get input vector of model
   */
  const Eigen::VectorXd& getInput() const;

  /**
   * @brief set state vector of model
   * @param [in] state state vector
   */
  void setState(const Eigen::VectorXd& state);

  /**
   * @brief set input vector of model
   * @param [in] input input vector
   */
  void setInput(const Eigen::VectorXd& input);

  /**
   * @brief update vehicle states with Runge-Kutta methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateRungeKutta(const double& dt, const Eigen::VectorXd& input);

  /**
   * @brief update vehicle states with Euler methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateEuler(const double& dt, const Eigen::VectorXd& input);

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  virtual void update(const double& dt) = 0;

  /**
   * @brief get vehicle position x
   */
  virtual const double getX() const = 0;

  /**
   * @brief get vehicle position y
   */
  virtual const double getY() const = 0;

  /**
   * @brief get vehicle angle yaw
   */
  virtual const double getYaw() const = 0;

  /**
   * @brief get vehicle velocity vx
   */
  virtual const double getVx() const = 0;

  /**
   * @brief get vehicle angular-velocity wz
   */
  virtual const double getWz() const = 0;

  /**
   * @brief get vehicle steering angle
   */
  virtual const double getSteer() const = 0;

  /**
   * @brief calculate derivative of states with vehicle model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  virtual Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) = 0;
};

#endif  // VEHICLE_SIM_MODEL_VEHICLE_MODEL_INTERFACE_H
