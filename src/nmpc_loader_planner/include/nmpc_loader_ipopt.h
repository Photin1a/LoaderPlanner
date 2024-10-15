#ifndef NMPC_LOADER_LBFGS_H
#ifndef NMPC_LOADER_IPOPT_H
#define NMPC_LOADER_IPOPT_H

#include<vector>
#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include "vehicle_param.h"
#include <Eigen/Core>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>
#include <sensor_msgs/JointState.h>

struct States {
  std::vector<double> x, y, theta, v, gamma, a, omega, jerk, dt;
  void Clear() {
    x.clear();
    y.clear();
    v.clear();
    a.clear();
    dt.clear();
    theta.clear();
    gamma.clear();
    omega.clear();
    jerk.clear();
  }
};
struct States2 {
  std::vector<double> x, y, theta, v, gamma, a, omega, jerk, tau;
  void Clear() {
    x.clear();
    y.clear();
    v.clear();
    a.clear();
    tau.clear();
    theta.clear();
    gamma.clear();
    omega.clear();
    jerk.clear();
  }
};


struct States3 {
  Eigen::VectorXd xut;
  int N_;

  Eigen::Map<Eigen::VectorXd> x(){
    return Eigen::Map<Eigen::VectorXd>(xut.data(),N_-2);
  }
    Eigen::Map<Eigen::VectorXd> y(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+N_-2,N_-2);
  }
    Eigen::Map<Eigen::VectorXd> theta(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+2*(N_-2),N_-2);
  }
    Eigen::Map<Eigen::VectorXd> gamma(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+3*(N_-2),N_-2);
  }
    Eigen::Map<Eigen::VectorXd> v(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+4*(N_-2),N_-2);
  }
    Eigen::Map<Eigen::VectorXd> a(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+5*(N_-2),N_-2);
  }
    Eigen::Map<Eigen::VectorXd> omega(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+6*(N_-2),N_-1);
  }
    Eigen::Map<Eigen::VectorXd> jerk(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+6*(N_-2)+N_-1,N_-1);
  }
    Eigen::Map<Eigen::VectorXd> tau(){
    return Eigen::Map<Eigen::VectorXd>(xut.data()+6*(N_-2)+2*(N_-1),N_-1);
  }

  //default N=3
  States3():N_(3),xut(12){}

  // N >=3
  States3(int N):N_(N),xut(9*N-15){}
};

struct DecompConstraints {
  double start_x, start_y, start_theta, start_v = 0, start_gamma = 0,
                                        start_a = 0, start_omega = 0,
                                        start_jerk = 0;
  double end_x, end_y, end_theta, end_v = 0, end_gamma = 0, end_a = 0,
                                  end_omega = 0, end_jerk = 0;
  std::vector<MatD2f> A;
  std::vector<VecDf> b;
};

class Nmpc{
public:
  Nmpc(nav_msgs::OccupancyGrid *costmap);
  ~Nmpc();
  double Solve(std::vector<geometry_msgs::PoseStamped> &init_poses,States2 &result);

private:
  void BuildNLP(std::vector<geometry_msgs::PoseStamped> &init_poses);
  std::vector<LinearConstraint2D>& BuildConstraints(std::vector<geometry_msgs::PoseStamped> &guess_poses);
  void Map2Vec2D(nav_msgs::OccupancyGrid &costmap, vec_Vec2f &vec_map);
  void Poses2Vec2D(std::vector<geometry_msgs::PoseStamped> &init_poses, vec_Vec2f &vec2D_path);
  void GetGuess(std::vector<geometry_msgs::PoseStamped> &init_poses, States2 &vec_guess);
  void PublishPath(States2 &result);

  nav_msgs::OccupancyGrid *costmap_;
  casadi::Dict nlp_config_;
  casadi::Function solver_;
  casadi::Function cost_evaluator_;
  Params params_;

  size_t cs_rows_ = 0;
  size_t cs_cols_ = 0;
  double max_a_axis_;
  double max_b_axis_;
  std::vector<LinearConstraint2D> decomp_cs_;

  casadi::DM lb_omega_, lb_jerk_,lb_dt_;
  casadi::DM ub_omega_, ub_jerk_,ub_dt_;
  casadi::DM lb_x_, lb_y_, lb_theta_, lb_v_, lb_gamma_, lb_a_;
  casadi::DM ub_x_, ub_y_, ub_theta_, ub_v_, ub_gamma_, ub_a_;

  ros::Publisher local_path_pub_;
  ros::Publisher corridor_pub_;
  ros::Publisher gamma_pub_;
};

#endif
#endif