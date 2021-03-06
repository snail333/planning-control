/**
 * @file vehicle_model_time_delay.h
 * @brief vehicle model with time delay and 1-dimensional dynamics for velocity & steeiring
 * @author Takamasa Horibe, Kim-Ngoc-Khanh Nguyen
 * @date 2019.08.17
 */

#ifndef VEHICLE_SIM_MODEL_VEHICLE_MODEL_TIME_DELAY_H
#define VEHICLE_SIM_MODEL_VEHICLE_MODEL_TIME_DELAY_H

#include "vehicle_sim_model/vehicle_model_interface.h"

#include <iostream>
#include <queue>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

/**
 * @class vehicle time delay twist model
 * @brief calculate time delay twist dynamics
 */
class VehicleModelTimeDelayTwist : public VehicleModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] angvel_lim angular velocity limit [m/s]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] wz_rate_lim angular acceleration llimit [rad/ss]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] vx_delay time delay for velocity command [s]
   * @param [in] vx_time_constant time constant for 1D model of velocity dynamics
   * @param [in] wx_delay time delay for angular-velocity command [s]
   * @param [in] wz_time_constant time constant for 1D model of angular-velocity dynamics
   */
  VehicleModelTimeDelayTwist(double vx_lim, double angvel_lim, double vx_rate_lim, double wz_rate_lim, double dt,
                           double vx_delay, double vx_time_constant, double wz_delay, double wz_time_constant);

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum class IDX : int
  {
    X = 0,
    Y,
    YAW,
    VX,
    WZ,
  };
  enum class IDX_U : int
  {
    VX_DES = 0,
    WZ_DES,
  };

  const double vx_lim_;       //!< @brief velocity limit
  const double vx_rate_lim_;  //!< @brief acceleration limit
  const double wz_lim_;       //!< @brief angular velocity limit
  const double wz_rate_lim_;  //!< @brief angular acceleration limit

  std::deque<double> vx_input_queue_;  //!< @brief buffer for velocity command
  std::deque<double> wz_input_queue_;  //!< @brief buffer for angular velocity command
  const double vx_delay_;              //!< @brief time delay for velocity command [s]
  const double vx_time_constant_;      //!< @brief time constant for 1D model of velocity dynamics
  const double wz_delay_;              //!< @brief time delay for angular-velocity command [s]
  const double wz_time_constant_;      //!< @brief time constant for 1D model of angular-velocity dynamics

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double& dt);

  /**
   * @brief get vehicle position x
   */
  const double getX() const override;

  /**
   * @brief get vehicle position y
   */
  const double getY() const override;

  /**
   * @brief get vehicle angle yaw
   */
  const double getYaw() const override;

  /**
   * @brief get vehicle velocity vx
   */
  const double getVx() const override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  const double getWz() const override;

  /**
   * @brief get vehicle steering angle
   */
  const double getSteer() const override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double& dt) override;

  /**
   * @brief calculate derivative of states with time delay twist model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
};

class VehicleModelTimeDelaySteer : public VehicleModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] steer_lim steering limit [rad]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] vx_delay time delay for velocity command [s]
   * @param [in] vx_time_constant time constant for 1D model of velocity dynamics
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   */
  VehicleModelTimeDelaySteer(
    double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
    double dt, double vx_delay, double vx_time_constant, double steer_delay,
    double steer_time_constant);

  /**
   * @brief default destructor
   */
  ~VehicleModelTimeDelaySteer() = default;

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum class IDX : int
  {
    X = 0,
    Y,
    YAW,
    VX,
    STEER,
  };
  enum class IDX_U : int
  {
    VX_DES = 0,
    STEER_DES,
  };

  const double vx_lim_;          //!< @brief velocity limit [m/s]
  const double vx_rate_lim_;     //!< @brief acceleration limit [m/ss]
  const double steer_lim_;       //!< @brief steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_;       //!< @brief vehicle wheelbase length [m]

  std::deque<double> vx_input_queue_;     //!< @brief buffer for velocity command
  std::deque<double> steer_input_queue_;  //!< @brief buffer for steering command
  const double vx_delay_;                 //!< @brief time delay for velocity command [s]
  const double vx_time_constant_;         //!< @brief time constant for 1D model of velocity dynamics
  const double steer_delay_;              //!< @brief time delay for steering command [s]
  const double steer_time_constant_;      //!< @brief time constant for 1D model of steering dynamics

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double& dt);

  /**
   * @brief get vehicle position x
   */
  const double getX() const override;

  /**
   * @brief get vehicle position y
   */
  const double getY() const override;

  /**
   * @brief get vehicle angle yaw
   */
  const double getYaw() const override;

  /**
   * @brief get vehicle velocity vx
   */
  const double getVx() const override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  const double getWz() const override;

  /**
   * @brief get vehicle steering angle
   */
  const double getSteer() const override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double& dt) override;

  /**
   * @brief calculate derivative of states with time delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
};

#endif  // VEHICLE_SIM_MODEL_VEHICLE_MODEL_TIME_DELAY_H
