#pragma once
#include <vector>
#include <ros/ros.h>

class Params
{
public:
  Params()
  {
    ros::NodeHandle public_nh;
    GetParam<double>(public_nh, "loader/L", L_);
    GetParam<double>(public_nh, "loader/L1", L1_);
    GetParam<double>(public_nh, "loader/L2", L2_);
    GetParam<double>(public_nh, "loader/d", d_);
    GetParam<double>(public_nh, "loader/d1", d1_);
    GetParam<double>(public_nh, "loader/d2", d2_);
    GetParam<double>(public_nh, "loader/width", width_);

    GetParam<double>(public_nh, "loader/min_velocity", min_velocity_);
    GetParam<double>(public_nh, "loader/max_velocity", max_velocity_);
    GetParam<double>(public_nh, "loader/min_acceleration", min_acceleration_);
    GetParam<double>(public_nh, "loader/max_acceleration", max_acceleration_);
    GetParam<double>(public_nh, "loader/min_gamma", min_gamma_);
    GetParam<double>(public_nh, "loader/max_gamma", max_gamma_);
    GetParam<double>(public_nh, "loader/min_omega", min_omega_);
    GetParam<double>(public_nh, "loader/max_omega", max_omega_);
    GetParam<double>(public_nh, "loader/min_jerk", min_jerk_);
    GetParam<double>(public_nh, "loader/max_jerk", max_jerk_);

    GetParam<double>(public_nh, "nmpc_loader_planner/optimizer/t_resolution", t_resolution_);
    GetParam<double>(public_nh, "nmpc_loader_planner/optimizer/opti_w_kin", opti_w_kin_);
    GetParam<double>(public_nh, "nmpc_loader_planner/optimizer/opti_w_ujerk", opti_w_uj_);
    GetParam<double>(public_nh, "nmpc_loader_planner/optimizer/opti_w_uomega", opti_w_uw_);
    GetParam<double>(public_nh, "nmpc_loader_planner/optimizer/opti_w_time", opti_w_time_);

    GetParam<double>(public_nh, "nmpc_loader_planner/corridor/max_a_axis", max_a_axis_);
    GetParam<double>(public_nh, "nmpc_loader_planner/corridor/max_b_axis", max_b_axis_);
    GetParam<bool>(public_nh, "nmpc_loader_planner/corridor/use_decomp_corridor", use_decomp_corridor_);
    GetParam<bool>(public_nh, "nmpc_loader_planner/corridor/is_sparsity", is_sparsity_);
    GetParam<double>(public_nh, "nmpc_loader_planner/corridor/sparsity_rate", sparsity_rate_);

    ROS_INFO("[nmpc loader planner params]:\nopti_w_kin:\t%f\nopti_w_ujerk:\t%f",opti_w_kin_,opti_w_uj_);
  }

  template <typename T>
  bool GetParam(ros::NodeHandle &nh, const std::string &param_name, T &param_val)
  {
    if (nh.hasParam(param_name))
    {
      if (nh.getParam(param_name, param_val))
      {
        return true;
      }
      else
        return false;
    }
    else
      return false;
  }

  // loader params
  double L_ = 1.5;
  double L1_ = 4.346;
  double L2_ = 3.472;
  double d_ = 0.92;
  double d1_ = 1.323;
  double d2_ = 1.332;
  double width_ = 2.44;

  // kin-dynamic params
  double min_velocity_ = -3;
  double max_velocity_ = 3;
  double min_acceleration_ = -2;
  double max_acceleration_ = 2;
  double min_gamma_ = -30.0 / 180.0 * 3.1415926;
  double max_gamma_ = 30.0 / 180.0 * 3.1415926;
  double min_omega_ = -0.1;
  double max_omega_ = 0.1;
  double min_jerk_ = -3;
  double max_jerk_ = 3;

  // optimizer, to config.yaml modify
  double t_resolution_ = 0.01; //采样周期
  double opti_w_kin_ = 1;
  double opti_w_uj_ = 0.001;
  double opti_w_uw_ = 0.001;
  double opti_w_time_ = 0.001;
  int N_ = 100; // 需要传入,待规划点数量，，，问题的维度   TODO

  // corridor, to config.yaml modify
  double max_a_axis_ = 20;
  double max_b_axis_ = 20;
  bool use_decomp_corridor_ = true;
  bool is_sparsity_ = false;
  double sparsity_rate_ = 0.5;
  std::vector<int> sparse_path_ids_; // unuse
  double expansion_distance_;        // unuse
};