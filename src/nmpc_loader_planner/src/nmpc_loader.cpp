#include "nmpc_loader.h"

Nmpc::Nmpc(nav_msgs::OccupancyGrid *costmap):costmap_(costmap){
    ros::NodeHandle public_nh;
    local_path_pub_ = public_nh.advertise<nav_msgs::Path>("local_path",1);
    corridor_pub_ = public_nh.advertise<decomp_ros_msgs::PolyhedronArray>("corridor",1);
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
    casadi::SX dt = casadi::SX::sym("dt", params_.N_-1);

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

    // Time Cost、 Input Cost、Total Cost
    auto time_cost = params_.opti_w_time_ * sumsqr(casadi::SX::vertcat({dt}));
    auto input_cost = params_.opti_w_uj_ * sumsqr(jerk) + params_.opti_w_uw_ * sumsqr(omega);
    auto f_obj = kinematic_cost + input_cost + time_cost;

    casadi::SX opti_x = casadi::SX::vertcat({x, y, theta, v, gamma, a, omega, jerk, dt});
    casadi::SXDict nlp = {{"x", opti_x}, {"f", f_obj},{"g",g}};
    solver_ = nlpsol("solver", "ipopt", nlp, nlp_config_);
    cost_evaluator_ = casadi::Function("inf", {opti_x}, {kinematic_cost}, {});
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
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
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

void Nmpc::GetGuess(std::vector<geometry_msgs::PoseStamped> &init_poses, States &vec_guess){
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
        vec_guess.dt.emplace_back(dist/v);
    }
    vec_guess.v.back() = 0;  // [point: N-1
}

double Nmpc::Solve(std::vector<geometry_msgs::PoseStamped> &init_poses,States &result){
  BuildNLP(init_poses);
  // Guess
  casadi::DMDict arg, res;
  States guess;
  GetGuess(init_poses, guess);
  arg["x0"] = casadi::DM::vertcat({guess.x, guess.y, guess.theta, guess.v, guess.gamma, guess.a, guess.omega, guess.jerk, guess.dt});

  // State 
  auto identity = casadi::DM::ones(params_.N_, 1);
  auto identity_ut = casadi::DM::ones(params_.N_-1, 1);
  casadi::DM lb_x, lb_y, lb_theta, lb_v, lb_gamma, lb_a;
  casadi::DM ub_x, ub_y, ub_theta, ub_v, ub_gamma, ub_a;
  lb_x = -casadi::inf * identity;
  ub_x = casadi::inf * identity;
  lb_y = -casadi::inf * identity;
  ub_y = casadi::inf * identity;
  lb_theta = -casadi::inf * identity;
  ub_theta = casadi::inf * identity;
  lb_v = params_.min_velocity_ * identity;
  ub_v = params_.max_velocity_ * identity;
  lb_a = params_.min_acceleration_ * identity;
  ub_a = params_.max_acceleration_ * identity;
  lb_gamma = params_.min_gamma_ * identity;
  ub_gamma = params_.max_gamma_ * identity;

  // Input 
  casadi::DM lb_omega, lb_jerk;
  casadi::DM ub_omega, ub_jerk;
  lb_omega = params_.min_omega_ * identity_ut;
  ub_omega = params_.max_omega_ * identity_ut;
  lb_jerk = params_.min_jerk_ * identity_ut;
  ub_jerk = params_.max_jerk_ * identity_ut;

  //Time 
  casadi::DM lb_dt;
  casadi::DM ub_dt;
  lb_dt = 0.0 * identity_ut;
  ub_dt = casadi::inf * identity_ut;

  // Two-point boundary value of constraints
  int end = params_.N_ - 1;
  lb_x(0) = ub_x(0) = init_poses.front().pose.position.x;
  lb_y(0) = ub_y(0) = init_poses.front().pose.position.y;
  lb_theta(0) = ub_theta(0) = tf::getYaw(init_poses.front().pose.orientation);
  lb_v(0) = ub_v(0) = 0;
  lb_gamma(0) = ub_gamma(0) = 0;
  lb_a(0) = ub_a(0) = 0;
  lb_x(end) = ub_x(end) = init_poses.back().pose.position.x;
  lb_y(end) = ub_y(end) = init_poses.back().pose.position.y;
  lb_theta(end) = ub_theta(end) = tf::getYaw(init_poses.back().pose.orientation);
  lb_v(end) = ub_v(end) = 0;
  lb_gamma(end) = ub_gamma(end) = 0;
  lb_a(end) = ub_a(end) = 0;

  // Optim Variables Boundary constraint, contains State and Input constraint.
  arg["lbx"] = casadi::DM::vertcat({lb_x, lb_y, lb_theta, lb_v, lb_gamma, lb_a, lb_omega, lb_jerk, lb_dt});
  arg["ubx"] = casadi::DM::vertcat({ub_x, ub_y, ub_theta, ub_v, ub_gamma, ub_a, ub_omega, ub_jerk, ub_dt});

  // Corridor boundary constraints
  auto identity_Ap_b = casadi::DM::ones(cs_rows_, 1);
  arg["lbg"] = -casadi::inf * identity_Ap_b;
  arg["ubg"] = 0. * identity_Ap_b;

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
  result.dt.resize(params_.N_-1);
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
    result.dt[i] = double(opt(8 *  (params_.N_ -1) + i, 0));
  }

  PublishPath(result);
  // evaluate infeasibility cost
  std::vector<casadi::DM> arg_in = {opt};
  auto arg_out = cost_evaluator_(arg_in);
  return arg_out.front()->at(0);
}

void Nmpc::PublishPath(States &result){
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
    path.header.frame_id = "world";
    local_path_pub_.publish(path);
}

