#include "nmpc_loader_ipopt.h"

Nmpc::Nmpc(nav_msgs::OccupancyGrid *costmap):costmap_(costmap){
    ros::NodeHandle public_nh;
    local_path_pub_ = public_nh.advertise<nav_msgs::Path>("local_path",1);
    corridor_pub_ = public_nh.advertise<decomp_ros_msgs::PolyhedronArray>("corridor",1);
    gamma_pub_ = public_nh.advertise<sensor_msgs::JointState>("/joint_states",1); 
    nlp_config_ = {{"ipopt", casadi::Dict({
                               {"linear_solver", "mumps"},
                               {"print_level", 1},
                               {"max_iter",10000}})}};
}

Nmpc::~Nmpc(){
}

void Nmpc::BuildNLP(std::vector<geometry_msgs::PoseStamped> &init_poses){
    params_.N_ = init_poses.size();

    //State Variables
    casadi::SX x = casadi::SX::sym("x", params_.N_);
    casadi::SX y = casadi::SX::sym("y", params_.N_);
    casadi::SX theta = casadi::SX::sym("theta", params_.N_);
    casadi::SX v = casadi::SX::sym("v", params_.N_);
    casadi::SX gamma = casadi::SX::sym("gamma", params_.N_);
    casadi::SX a = casadi::SX::sym("a", params_.N_);

    //Input Variables
    casadi::SX omega = casadi::SX::sym("omega", params_.N_-1);
    casadi::SX jerk = casadi::SX::sym("jerk", params_.N_-1);

    //Time  Variables
    // casadi::SX dt = casadi::SX::sym("dt", params_.N_-1);
    casadi::SX dt(params_.N_-1,1);
    casadi::SX tau = casadi::SX::sym("tau", params_.N_-1);  //同胚正变换
    dt = if_else(tau>0,0.5*tau*tau+tau+1,2/(tau*tau-2*tau+2));

    //Corridor Constraints
    std::vector<std::pair<casadi::SX,casadi::SX>> corners; //A.B.C...H    TODO BUG
    corners.emplace_back (x - params_.d1_ * cos(theta) + (params_.width_ * sin(theta)) / 2,y - params_.d1_ * sin(theta) - (params_.width_ * cos(theta)) / 2);
    corners.emplace_back(x - cos(theta) * (params_.L_ - params_.L1_) + (params_.width_ * sin(theta)) / 2,y - sin(theta) * (params_.L_ - params_.L1_) - (params_.width_ * cos(theta)) / 2);
    corners.emplace_back(x - cos(theta) * (params_.L_ - params_.L1_) - (params_.width_ * sin(theta)) / 2,y - sin(theta) * (params_.L_ - params_.L1_) + (params_.width_ * cos(theta)) / 2);
    corners.emplace_back(x - params_.d1_ * cos(theta) - (params_.width_ * sin(theta)) / 2,y - params_.d1_ * sin(theta) + (params_.width_ * cos(theta)) / 2);
    // corners.emplace_back(x - cos(gamma + theta) * (params_.L_ - params_.d2_) - params_.L_ * cos(theta) - (params_.width_ * sin(gamma + theta)) / 2,y + (params_.width_*cos(gamma + theta))/2 - sin(gamma + theta)*(params_.L_ - params_.d2_) - params_.L_*sin(theta));
    // corners.emplace_back(x - params_.L2_ * cos(gamma + theta) - params_.L_ * cos(theta) - (params_.width_ * sin(gamma + theta)) / 2,y - params_.L2_ * sin(gamma + theta) + (params_.width_ * cos(gamma + theta)) / 2 - params_.L_ * sin(theta));
    // corners.emplace_back(x - params_.L2_ * cos(gamma + theta) - params_.L_ * cos(theta) + (params_.width_ * sin(gamma + theta)) / 2,y - params_.L2_ * sin(gamma + theta) - (params_.width_ * cos(gamma + theta)) / 2 - params_.L_ * sin(theta));
    // corners.emplace_back(x - cos(gamma + theta) * (params_.L_ - params_.d2_) - params_.L_ * cos(theta) + (params_.width_ * sin(gamma + theta)) / 2,y - (params_.width_ * cos(gamma + theta)) / 2 - sin(gamma + theta) * (params_.L_ - params_.d2_) - params_.L_ * sin(theta));

    std::vector<casadi::SX> Ap,b;
    auto idx = 1;
    auto &decomp_cs = BuildConstraints(init_poses);
    for(auto &cs:decomp_cs){
        casadi::SX A(cs.A_.rows(), 2);
        for(int row = 0; row < cs.A_.rows();row++)
            for (int col = 0; col < 2;col++) A(row, col) = cs.A_(row, col);
        casadi::SX b_t(cs.b_.rows(),1);
        for(int i=0;i<cs.b_.rows();i++)b_t(i) = cs.b_(i);
            for(auto &corner:corners){
                Ap.emplace_back(mtimes(A,casadi::SX::vertcat({corner.first(idx),corner.second(idx)})));
                b.emplace_back(b_t);
            }
        idx++;
    }
    auto g = casadi::SX::vertcat(Ap)-casadi::SX::vertcat(b);
    cs_rows_ = g.size().first;
    cs_cols_ = g.size().second;
    double opti_w_corr = 100;  //TODO

    // TWO points Bound constraints
    auto end = params_.N_ - 1;
    auto g_two_point = casadi::SX::vertcat({x(0),y(0),theta(0),v(0),gamma(0),a(0),x(end),y(end),theta(end),v(end),gamma(end),a(end)});

    // Kinematic Cost
    auto prev = casadi::Slice(0, params_.N_ - 1);
    auto next = casadi::Slice(1, params_.N_);
    auto g_x = x(next) - (x(prev) + dt(prev) * v(prev) * cos(theta(prev)));
    auto g_y = y(next) - (y(prev) + dt(prev) * v(prev) * sin(theta(prev)));
    auto g_theta = theta(next) - (theta(prev) + dt(prev) * (v(prev) * tan(gamma(prev)/2) / params_.L_+omega(prev)/(cos(gamma(prev))+1)));
    auto g_v = v(next) - (v(prev) + dt(prev) * a(prev));
    auto g_a = a(next) - (a(prev) + dt(prev) * jerk(prev));
    auto g_gamma = gamma(next) - (gamma(prev) + dt(prev) * omega(prev));
    auto kinematic_cost = params_.opti_w_kin_ * sumsqr(casadi::SX::vertcat({g_x, g_y, g_theta, g_v, g_a, g_gamma}));

    // Time Cost、 Input Cost、Corridor Cost、Bound Cost、Total Cost
    double opti_w_bound = 50;
    auto time_cost = params_.opti_w_time_ * sumsqr(casadi::SX::vertcat({dt}));
    auto input_cost = params_.opti_w_uj_ * sumsqr(jerk) + params_.opti_w_uw_ * sumsqr(omega);
    auto corridor_cost = opti_w_corr*sumsqr(exp(5*g));  //TODO
    auto bound_cost = opti_w_bound*sumsqr(exp(5*casadi::SX::vertcat({params_.min_velocity_-v,v-params_.max_velocity_,
    params_.min_gamma_-gamma,gamma-params_.max_gamma_,
    params_.min_acceleration_-a,a-params_.max_acceleration_,
    params_.min_omega_-omega,omega-params_.max_omega_,
    params_.min_jerk_-jerk,jerk-params_.max_jerk_})));

    auto f_obj = kinematic_cost + input_cost + time_cost + corridor_cost;

    casadi::SX opti_x = casadi::SX::vertcat({x, y, theta, v, gamma, a, omega, jerk, tau});
    casadi::SXDict nlp = {{"x", opti_x}, {"f", f_obj}, {"g",g_two_point}};
    solver_ = nlpsol("solver", "ipopt", nlp, nlp_config_);
    cost_evaluator_ = casadi::Function("inf", {opti_x}, {kinematic_cost}, {});
}

template <int Dim>
decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs, int delta = 0){
  decomp_ros_msgs::PolyhedronArray msg;
  int i = 0;
  for (const auto &v : vs){
    if(i == delta){
        msg.polyhedrons.push_back(DecompROS::polyhedron_to_ros(v)); 
        i = -1;   
    }
    i++;
  }
  return msg;
}

std::vector<LinearConstraint2D>& Nmpc::BuildConstraints(std::vector<geometry_msgs::PoseStamped> &init_poses){
    decomp_cs_.clear();
    vec_Vec2f vec_map;
    Map2Vec2D(*costmap_,vec_map);

    vec_Vec2f vec_path;
    Poses2Vec2D(init_poses, vec_path);

    EllipsoidDecomp2D decomp_util;
    decomp_util.set_obs(vec_map);
    decomp_util.set_local_bbox(Vec2f(params_.max_a_axis_, params_.max_b_axis_));
    decomp_util.dilate(vec_path); //Set max iteration number of 10, do fix the path
    auto polys = decomp_util.get_polyhedrons();
    for(size_t i = 0; i < vec_path.size() - 1; i++) {
    const auto pt_inside = (vec_path[i] + vec_path[i+1]) / 2;
    decomp_cs_.emplace_back(pt_inside, polys[i].hyperplanes());
  }
  decomp_ros_msgs::PolyhedronArray poly_msg = polyhedron_array_to_ros(polys, 6);
  poly_msg.header.frame_id = "world";
  corridor_pub_.publish(poly_msg); 
  return decomp_cs_;
}


void Nmpc::Map2Vec2D(nav_msgs::OccupancyGrid &costmap, vec_Vec2f &vec_map){
    vec_map.clear();
    auto width = costmap.info.width;
    auto height = costmap.info.height;
    auto resolution = costmap.info.resolution;
    size_t i = 0;
    for (auto map_unit : costmap.data) {
        if (map_unit > 0){
        vec_map.emplace_back(int(i % width) * resolution, int(i / width) * resolution);
        vec_map.emplace_back(int(i % width + 1) * resolution, int(i / width) * resolution);
        vec_map.emplace_back(int(i % width) * resolution, int(i / width + 1) * resolution);
        vec_map.emplace_back(int(i % width + 1) * resolution, int(i / width + 1) * resolution);  
        }
        i++;
    }
}

void Nmpc::Poses2Vec2D(std::vector<geometry_msgs::PoseStamped> &init_poses, vec_Vec2f &vec_path){
    for(auto &pose:init_poses){
        vec_path.emplace_back(Vec2f(pose.pose.position.x,pose.pose.position.y));
    }
}

void Nmpc::GetGuess(std::vector<geometry_msgs::PoseStamped> &init_poses, States2 &vec_guess){
    vec_guess.Clear();
    vec_guess.x.emplace_back(init_poses.front().pose.position.x);  //point: 0
    vec_guess.y.emplace_back(init_poses.front().pose.position.y);
    vec_guess.v.emplace_back(0);
    vec_guess.a.emplace_back(0);
    vec_guess.theta.emplace_back(tf::getYaw(init_poses.front().pose.orientation));
    vec_guess.gamma.emplace_back(0);
    auto v = 1;
    for(auto i = 1;i< init_poses.size();i++){  // [point: 1...N-1
        auto dx = init_poses[i].pose.position.x-init_poses[i-1].pose.position.x;
        auto dy = init_poses[i].pose.position.y-init_poses[i-1].pose.position.y;
        auto dist = std::hypot(dx,dy);
        vec_guess.x.emplace_back(init_poses[i].pose.position.x);
        vec_guess.y.emplace_back(init_poses[i].pose.position.y);
        vec_guess.v.emplace_back(v);
        vec_guess.a.emplace_back(0);
        vec_guess.theta.emplace_back(tf::getYaw(init_poses[i].pose.orientation));
        vec_guess.gamma.emplace_back(0);  //TODO
        vec_guess.omega.emplace_back(0); //TODO
        vec_guess.jerk.emplace_back(0); 
        auto dt =  dist/v;
        double tau,tau1,tau2;
        if(dt>1){
            tau1=1+std::sqrt(2*dt-1);
            tau2=1-std::sqrt(2*dt-1);
            tau = tau1>0?tau1:tau2;
        }
        else {
            tau1=1+std::sqrt(2*dt-dt*dt)/dt;
            tau2=1-std::sqrt(2*dt-dt*dt)/dt;
            tau = tau1<=0?tau1:tau2;
        }
        vec_guess.tau.emplace_back(tau);
    }
    vec_guess.v.back() = 0;  // [point: N-1
}

double Nmpc::Solve(std::vector<geometry_msgs::PoseStamped> &init_poses,States2 &result){
  BuildNLP(init_poses);
  // Guess
  casadi::DMDict arg, res;
  States2 guess;
  GetGuess(init_poses, guess);
  arg["x0"] = casadi::DM::vertcat({guess.x, guess.y, guess.theta, guess.v, guess.gamma, guess.a, guess.omega, guess.jerk, guess.tau});

  // Two-point boundary value of constraints
  casadi::DM lb_two_point(12, 1),ub_two_point(12, 1);
  ub_two_point(0) = lb_two_point(0) = init_poses.front().pose.position.x;
  ub_two_point(1) = lb_two_point(1) = init_poses.front().pose.position.y;
  ub_two_point(2) = lb_two_point(2) = tf::getYaw(init_poses.front().pose.orientation);
  ub_two_point(3) = lb_two_point(3) = 0;
  ub_two_point(4) = lb_two_point(4) = 0;
  ub_two_point(5) = lb_two_point(5) = 0;
  ub_two_point(6) = lb_two_point(6) = init_poses.back().pose.position.x;
  ub_two_point(7) = lb_two_point(7) = init_poses.back().pose.position.y;
  ub_two_point(8) = lb_two_point(8) = tf::getYaw(init_poses.back().pose.orientation);
  ub_two_point(9) = lb_two_point(9) = 0;
  ub_two_point(10) = lb_two_point(10) = 0;
  ub_two_point(11) = lb_two_point(11) = 0;

  arg["lbg"] = lb_two_point;
  arg["ubg"] = ub_two_point;

  // solve
  res = solver_(arg);
  casadi::DM opt = res.at("x");
  result.Clear();
  result.x.resize(params_.N_);
  result.y.resize(params_.N_);
  result.theta.resize(params_.N_);
  result.v.resize(params_.N_);
  result.gamma.resize(params_.N_);
  result.a.resize(params_.N_);
  result.omega.resize(params_.N_-1);
  result.jerk.resize(params_.N_-1);
  result.tau.resize(params_.N_-1);
  for (size_t i = 0; i < params_.N_; i++) {
    result.x[i] = double(opt(i, 0));
    result.y[i] = double(opt(params_.N_+ i, 0));
    result.theta[i] = double(opt(2 * params_.N_ + i, 0));
    result.v[i] = double(opt(3 * params_.N_ + i, 0));
    result.gamma[i] = double(opt(4 * params_.N_ + i, 0));
    result.a[i] = double(opt(5 * params_.N_ + i, 0));
  }
  for (size_t i = 0; i < params_.N_-1; i++) {
    result.omega[i] = double(opt(6 * (params_.N_ - 1) + i, 0));
    result.jerk[i] = double(opt(7 * (params_.N_ - 1) + i, 0));
    result.tau[i] = double(opt(8 *  (params_.N_ -1) + i, 0));
  }

  ROS_INFO("[Nmpc::Solve]Get solution successfully");
  PublishPath(result);
  
  // evaluate infeasibility cost
  std::vector<casadi::DM> arg_in = {opt};
  auto arg_out = cost_evaluator_(arg_in);
  return arg_out.front()->at(0);
}

#include <fstream>
void Nmpc::PublishPath(States2 &result){
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose_stamped;
    for (int i = 0;i<result.x.size();i++) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = result.x.at(i);
        pose_stamped.pose.position.y = result.y.at(i);
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(result.theta.at(i));
        path.poses.emplace_back(pose_stamped);
    }
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    local_path_pub_.publish(path);

    // static transform
    static tf::TransformBroadcaster broadcaster;
    sensor_msgs::JointState gamma;
    gamma.name = {"back_joint","lf_joint","rf_joint","lb_joint","rb_joint"};
    int end = result.x.size()-1;
    for(int i = 0;i < end;i++){
        double dtau = result.tau.at(i);
        double dt = casadi::if_else(dtau>0,0.5*dtau*dtau+dtau+1,2/(dtau*dtau-2*dtau+2));
        if(dt > 0){
            broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::createQuaternionFromYaw(result.theta.at(i)), tf::Vector3(result.x.at(i),result.y.at(i),0)),
                    ros::Time::now(),"map", "base_link"));

            gamma.header.stamp = ros::Time::now();
            gamma.position = {-result.gamma.at(i),0,0,0,0};
            gamma.velocity = {result.omega.at(i),0,0,0,0};
            gamma.effort = {0,0,0,0,0};
            gamma_pub_.publish(gamma);
            ros::Rate(1/dt).sleep();
            ROS_INFO("[Nmpc::PublishPath]Move to %d/%d pos.",i+1,end+1);
        }
    }
    broadcaster.sendTransform(
    tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromYaw(result.theta.at(end)), tf::Vector3(result.x.at(end),result.y.at(end),0)),
        ros::Time::now(),"map", "base_link"));
    gamma.header.stamp = ros::Time::now();
    gamma.position = {-result.gamma.at(end),0,0,0,0};
    gamma.velocity = {0,0,0,0,0};
    gamma.effort = {0,0,0,0,0};
    gamma_pub_.publish(gamma);
    ROS_INFO("[Nmpc::PublishPath]Move to %d/%d pos.",end+1,end+1);

}