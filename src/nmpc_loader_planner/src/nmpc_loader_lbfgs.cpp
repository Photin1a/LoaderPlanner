#include "nmpc_loader_lbfgs.h"
using namespace Eigen;

Nmpc::Nmpc(nav_msgs::OccupancyGrid *costmap):costmap_(costmap){
    ros::NodeHandle public_nh;
    local_path_pub_ = public_nh.advertise<nav_msgs::Path>("local_path",1);
    corridor_pub_ = public_nh.advertise<decomp_ros_msgs::PolyhedronArray>("corridor",1);
}

Nmpc::~Nmpc(){
}

// xu:x,y,theta,gamma,v,a,omega,jerk
// dt
// g_kin
inline void Nmpc::GkinCostVec( const Eigen::VectorXd &xu,const Eigen::VectorXd &dt,Eigen::VectorXd &g_kin){
    int g_dim  = 6*params_.N_-6;
    int x_dim = params_.N_-2;
    int ut_dim = params_.N_-1;
    auto N_3 = params_.N_-3;
    auto N_2 = params_.N_-2;
    auto N_1 = params_.N_-1;

    g_kin.resize(g_dim);
    int x_s=0,y_s=x_dim,theta_s=2*x_dim,gamma_s=3*x_dim,v_s=4*x_dim,a_s=5*x_dim;
    int omega_s=6*x_dim,jerk_s=6*x_dim+ut_dim,dt_s=6*x_dim+2*ut_dim;
    Eigen::Map<const VectorXd> x_(xu.data(),x_dim);
    Eigen::Map<const VectorXd> y_(xu.data()+y_s,x_dim);
    Eigen::Map<const VectorXd> theta(xu.data()+theta_s,x_dim);
    Eigen::Map<const VectorXd> gamma(xu.data()+gamma_s,x_dim);
    Eigen::Map<const VectorXd> v(xu.data()+v_s,x_dim);
    Eigen::Map<const VectorXd> a(xu.data()+a_s,x_dim);
    Eigen::Map<const VectorXd> omega(xu.data()+omega_s,ut_dim);
    Eigen::Map<const VectorXd> jerk(xu.data()+jerk_s,ut_dim);
    Eigen::Map<VectorXd> g_x(g_kin.data(),N_1);
    Eigen::Map<VectorXd> g_y(g_kin.data()+N_1,N_1);
    Eigen::Map<VectorXd> g_theta(g_kin.data()+2*N_1,N_1);
    Eigen::Map<VectorXd> g_gamma(g_kin.data()+3*N_1,N_1);
    Eigen::Map<VectorXd> g_v(g_kin.data()+4*N_1,N_1);
    Eigen::Map<VectorXd> g_a(g_kin.data()+5*N_1,N_1);

    
    g_x(0) = x_(0)-x_start_(0)-dt(0)*x_start_(4)*cos(x_start_(2));
    g_x.middleRows(1,N_3) = x_.bottomRows(N_3)-x_.topRows(N_3)-(dt.middleRows(1,N_3).array()*v.topRows(N_3).array()*theta.topRows(N_3).array().cos()).matrix(); 
    g_x(N_2) = x_end_(0)-x_(N_3)-dt(N_2)*v(N_3)*cos(theta(N_3));

    g_y(0) = y_(0)-x_start_(1)-dt(0)*x_start_(4)*sin(x_start_(2));
    g_y.middleRows(1,N_3) = y_.bottomRows(N_3)-y_.topRows(N_3)-(dt.middleRows(1,N_3).array()*v.topRows(N_3).array()*theta.topRows(N_3).array().sin()).matrix(); 
    g_y(N_2) = x_end_(1)-y_(N_3)-dt(N_2)*v(N_3)*sin(theta(N_3));

    g_theta(0) = theta(0)-x_start_(2)-dt(0)*(x_start_(4)*tan(x_start_(3)/2)/params_.L_+omega(0)/(cos(x_start_(3))+1));
    g_theta.middleRows(1,N_3) = theta.bottomRows(N_3)-theta.topRows(N_3)-(dt.middleRows(1,N_3).array()*(v.topRows(N_3).array()*(gamma.topRows(N_3)/2).array().tan()/params_.L_+
                                                                                                           omega.middleRows(1,N_3).array()/(gamma.topRows(N_3).array().cos()+1))).matrix(); 
    g_theta(N_2) = x_end_(2)-theta(N_3)-dt(N_2)*(v(N_3)*tan(gamma(N_3)/2)/params_.L_+omega(N_2)/(cos(gamma(N_3))+1));

    g_gamma(0) = gamma(0)-x_start_(3)-dt(0)*omega(0);
    g_gamma.middleRows(1,N_3) = gamma.bottomRows(N_3)-gamma.topRows(N_3)-(dt.middleRows(1,N_3).array()*omega.middleRows(1,N_3).array()).matrix(); 
    g_gamma(N_2) = x_end_(3)-gamma(N_3)-dt(N_2)*omega(N_2);

    g_v(0) = v(0)-x_start_(4)-dt(0)*x_start_(5);
    g_v.middleRows(1,N_3) = v.bottomRows(N_3)-v.topRows(N_3)-(dt.middleRows(1,N_3).array()*a.topRows(N_3).array()).matrix(); 
    g_v(N_2) = x_end_(4)-v(N_3)-dt(N_2)*a(N_3);

    g_a(0) = a(0)-x_start_(5)-dt(0)*jerk(0);
    g_a.middleRows(1,N_3) = a.bottomRows(N_3)-a.topRows(N_3)-(dt.middleRows(1,N_3).array()*jerk.middleRows(1,N_3).array()).matrix(); 
    g_a(N_2) = x_end_(5)-a(N_3)-dt(N_2)*jerk(N_2);
}

inline double Nmpc::Dtau2dt(const double &tau){
    return tau>0?0.5*tau*tau+tau+1:2/(tau*tau-2*tau+2);
}

inline double Nmpc::Dt2dtau(const double &dt){
    double tau1,tau2;
    if(dt > 1){
        tau1=1+std::sqrt(2*dt-1);
        tau2=1-std::sqrt(2*dt-1);
        return tau1>0?tau1:tau2;
    }
    else {
        tau1=1+std::sqrt(2*dt-dt*dt)/dt;
        tau2=1-std::sqrt(2*dt-dt*dt)/dt;
        return tau1<=0?tau1:tau2;
    }
}

inline double Nmpc::DtDiff(const double &tau){
    return tau>0?tau+1:4*(1-tau)/pow((tau*tau-2*tau+2),2);
}

#include <map>
#include <string>

//return cost
inline double Nmpc::InEqConstraints(const VectorXd &xut, VectorXd &grad){
    using CornerMap = std::map<std::string, std::pair<VectorXd,VectorXd>>;
    using CornerPair = std::pair<VectorXd,VectorXd>;

    int x_dim = params_.N_-2;
    int u_dim = params_.N_-1;

    double opti_w_bound = 0.01;
    double opti_w_corridor = 0.01;

    // var bound
    auto cost_bound = 0.0;
    int var_dim = 3*x_dim+2*u_dim;
    Eigen::Map<const VectorXd> var(xut.data()+3*x_dim,var_dim);  //gamma,v,a,omega,jerk
    Eigen::Map<VectorXd> grad_var(grad.data()+3*x_dim,var_dim);
    VectorXd var_lb(var_dim),var_ub(var_dim);
    var_lb << VectorXd::Constant(x_dim,params_.min_gamma_),VectorXd::Constant(x_dim,params_.min_velocity_),VectorXd::Constant(x_dim,params_.min_acceleration_),
            VectorXd::Constant(u_dim,params_.min_omega_),VectorXd::Constant(u_dim,params_.min_jerk_);
    var_ub << VectorXd::Constant(x_dim,params_.max_gamma_),VectorXd::Constant(x_dim,params_.max_velocity_),VectorXd::Constant(x_dim,params_.max_acceleration_),
            VectorXd::Constant(u_dim,params_.max_omega_),VectorXd::Constant(u_dim,params_.max_jerk_);
    auto varl = var_lb-var;
    auto varu = var-var_ub;

    cost_bound += opti_w_bound*varl.unaryExpr(&Nmpc::InEqPenalty).sum();
    cost_bound += opti_w_bound*varu.unaryExpr(&Nmpc::InEqPenalty).sum();
    grad_var -= opti_w_bound*varl.unaryExpr(&Nmpc::InEqPenaltyDiff);  // dL/dvar = dvarl/dvar*dL/dvarl = -dL/dvarl
    grad_var += opti_w_bound*varu.unaryExpr(&Nmpc::InEqPenaltyDiff);  // dL/dvar = dvaru/dvar*dL/dvaru = dL/dvaru
    
    Vector2d l_A(-params_.d1_,          -params_.width_/2);
    Vector2d l_B(params_.L1_-params_.L_,-params_.width_/2);
    Vector2d l_C(params_.L1_-params_.L_, params_.width_/2);
    Vector2d l_D(-params_.d1_,           params_.width_/2);
    Vector2d l_E(params_.d2_-params_.L_, params_.width_/2);
    Vector2d l_F(-params_.L2_,           params_.width_/2);
    Vector2d l_G(-params_.L2_,          -params_.width_/2);
    Vector2d l_H(params_.d2_-params_.L_,-params_.width_/2);
    Vector2d l_O1(-params_.L_,                  0        );
    std::vector<Vector2d> corners_f={l_A,l_B,l_C,l_D};
    std::vector<Vector2d> corners_r={l_E,l_F,l_G,l_H};

    // corridor
    auto cost_corridor = 0.0;
    Vector2d sigma;  //sigma = (x,y)^T
    Matrix2d R_theta,R_theta_diff;
    Eigen::Map<const VectorXd> x(xut.data(),x_dim);
    Eigen::Map<const VectorXd> y(xut.data()+x_dim,x_dim);
    Eigen::Map<const VectorXd> theta(xut.data()+2*x_dim,x_dim);
    Eigen::Map<VectorXd> grad_sigma(grad.data(),2*x_dim);
    Eigen::Map<VectorXd> grad_theta(grad.data()+2*x_dim,x_dim);
    for(int k  = 0;k< params_.N_-2;k++){
        sigma << x(k),y(k);
        R_theta << cos(theta(k)),-sin(theta(k)),sin(theta(k)),cos(theta(k));
        R_theta_diff << -sin(theta(k)),-cos(theta(k)),cos(theta(k)),-sin(theta(k));
        for(auto &corner:corners_f){
            auto Ap_b = decomp_cs_[k].A_*(R_theta*corner+sigma)-decomp_cs_[k].b_; //g = Ap-b
            cost_corridor += opti_w_corridor*Ap_b.unaryExpr(&Nmpc::InEqPenalty).sum();  //cost = Lg1+Lg2+...   
            auto dL_inv_dAp_b  = Ap_b.unaryExpr(&Nmpc::InEqPenaltyDiff);
            auto grad_sigma_k = opti_w_corridor*decomp_cs_[k].A_.transpose()*dL_inv_dAp_b;//,grad(sigma) = dAp_b/dsigma * dL/dAp_b = A^T*dL/dAp_b
            grad_sigma(k)+=grad_sigma_k(0);  //grad_x
            grad_sigma(x_dim+k)+=grad_sigma_k(1);  //grad_y
            grad_theta(k)+=opti_w_corridor*corner.transpose()*R_theta_diff.transpose()*decomp_cs_[k].A_.transpose()*dL_inv_dAp_b; // grad(theta) = dAp_b/dtheta * dL/dAp_b = l_f^T*dR^T/dtheta*A^T * dL/dAp_b
        }
    }
    return cost_bound+cost_corridor;
}

inline double Nmpc::InEqPenalty(const double &x){
    double a0=10e-4;
    if(x<=0)return 0.0;
    else if(0<x&&x<=a0)return -0.5*pow(x,4)/pow(a0,3)+pow(x,3)/pow(a0,2);
    else return x-0.5*a0;
}

inline double Nmpc::InEqPenaltyDiff(const double &x){
    double a0=10e-4;
    if(x<=0)return 0.0;
    else if(0<x&&x<=a0)return -2*pow(x/a0,3)+3*pow(x/a0,2);
    else return 1.0;
}

// kinmatic constraints(smooth)
inline double Nmpc::EqConstraints(const VectorXd &xut, VectorXd &grad){
    int x_dim = params_.N_-2;
    int ut_dim = params_.N_-1;
    int x_s = 0,y_s = x_dim,theta_s=2*x_dim,gamma_s=3*x_dim,v_s=4*x_dim,a_s=5*x_dim;
    int omega_s=6*x_dim,jerk_s=6*x_dim+ut_dim,dt_s=6*x_dim+2*ut_dim;
    Eigen::Map<const VectorXd> x_(xut.data(),x_dim);
    Eigen::Map<const VectorXd> y_(xut.data()+y_s,x_dim);
    Eigen::Map<const VectorXd> theta(xut.data()+theta_s,x_dim);
    Eigen::Map<const VectorXd> gamma(xut.data()+gamma_s,x_dim);
    Eigen::Map<const VectorXd> v(xut.data()+v_s,x_dim);
    Eigen::Map<const VectorXd> a(xut.data()+a_s,x_dim);

    Eigen::Map<const VectorXd> omega(xut.data()+omega_s,ut_dim);
    Eigen::Map<const VectorXd> jerk(xut.data()+jerk_s,ut_dim);
    Eigen::Map<const VectorXd> dtau(xut.data()+dt_s,ut_dim);
    Eigen::Map<const VectorXd> omega_1_end(xut.data()+omega_s+1,ut_dim-1);
    Eigen::Map<const VectorXd> jerk_1_end(xut.data()+jerk_s+1,ut_dim-1);
    VectorXd dt = dtau.unaryExpr(&Nmpc::Dtau2dt);

    std::cout<<dtau.transpose()<<"\n"<<dt.transpose()<<std::endl;

    Eigen::Map<const VectorXd> dt_1_end(dt.data()+1,ut_dim-1);

    MatrixXd jacobi_dt_dtau = dtau.unaryExpr(&Nmpc::DtDiff);
    //phi = d(x,u,t)/d(x,u,tau)  *  dg/d(x,u,t) 
    MatrixXd phi_02(x_dim,ut_dim),phi_12(x_dim,ut_dim),phi_23(x_dim,ut_dim),phi_04(x_dim,ut_dim),phi_14(x_dim,ut_dim),phi_24(x_dim,ut_dim),phi_45(x_dim,ut_dim);
    MatrixXd phi_26(ut_dim,ut_dim),phi_36(ut_dim,ut_dim),phi_57(ut_dim,ut_dim),phi_08(ut_dim,ut_dim),phi_18(ut_dim,ut_dim),phi_28(ut_dim,ut_dim),phi_38(ut_dim,ut_dim),phi_48(ut_dim,ut_dim),phi_58(ut_dim,ut_dim);
    // fx = v * cos(theta) * dt
    phi_02 << MatrixXd::Zero(x_dim,1),(-v.array()*theta.array().sin()*dt_1_end.array()).matrix().asDiagonal().toDenseMatrix();
    phi_04 << MatrixXd::Zero(x_dim,1),(theta.array().cos()*dt_1_end.array()).matrix().asDiagonal().toDenseMatrix();
    VectorXd p8(x_dim+1);
    p8<<x_start_(4)*cos(x_start_(2)),(v.array()*theta.array().cos()).matrix();
    phi_08 = (jacobi_dt_dtau.array()*p8.array()).matrix().asDiagonal();  // df/dtau = dt/dtau * df/dt

    // fy = v * sin(theta) * dt
    phi_12 << MatrixXd::Zero(x_dim,1),(v.array()*theta.array().cos()*dt_1_end.array()).matrix().asDiagonal().toDenseMatrix();
    phi_14 << MatrixXd::Zero(x_dim,1),(theta.array().sin()*dt_1_end.array()).matrix().asDiagonal().toDenseMatrix();
    p8<<x_start_(4)*sin(x_start_(2)),(v.array()*theta.array().sin()).matrix();
    phi_18 = (jacobi_dt_dtau.array()*p8.array()).matrix().asDiagonal();

    // ftheta = ... * dt
    phi_23 << MatrixXd::Zero(x_dim,1),((0.5/params_.L_*v.array()*(1+(gamma/2).array().tan().square())+        //TODO
                                        omega_1_end.array()*gamma.array().sin()/(gamma.array().cos()+1).square())*
                                        dt_1_end.array()).matrix().asDiagonal().toDenseMatrix();
    phi_24 << MatrixXd::Zero(x_dim,1),((gamma/2).array().tan()/params_.L_*dt_1_end.array()).matrix().asDiagonal().toDenseMatrix();
    p8<<1/(cos(x_start_(3))+1),(1/(gamma.array().cos()+1)).matrix();
    phi_26 = (p8.array()*dt.array()).matrix().asDiagonal();
    p8<<x_start_(4)*tan(x_start_(3)/2)/params_.L_+omega(0)/(cos(x_start_(3))+1),
        (v.array()*(gamma/2).array().tan()/params_.L_+omega_1_end.array()/(gamma.array().cos()+1)).matrix();
    phi_28 = (jacobi_dt_dtau.array()*p8.array()).matrix().asDiagonal();

    phi_36 = dt.asDiagonal();
    phi_38 = (jacobi_dt_dtau.array()*omega.array()).matrix().asDiagonal();

    phi_45 << MatrixXd::Zero(x_dim,1),dt_1_end.array().matrix().asDiagonal().toDenseMatrix();
    p8<<x_start_(5),a;
    phi_48 = (jacobi_dt_dtau.array()*p8.array()).matrix().asDiagonal();

    phi_57 = dt.asDiagonal();
    phi_58 = (jacobi_dt_dtau.array()*jerk.array()).matrix().asDiagonal();

    gkin_jacobi_.block(theta_s,0,x_dim,ut_dim) = -phi_02;  //x
    gkin_jacobi_.block(theta_s,ut_dim,x_dim,ut_dim) = -phi_12;
    gkin_jacobi_.block(gamma_s,2*ut_dim,x_dim,ut_dim) = -phi_23;
    gkin_jacobi_.block(v_s,0,x_dim,ut_dim) = -phi_04;
    gkin_jacobi_.block(v_s,ut_dim,x_dim,ut_dim) = -phi_14;
    gkin_jacobi_.block(v_s,2*ut_dim,x_dim,ut_dim) = -phi_24;
    gkin_jacobi_.block(a_s,4*ut_dim,x_dim,ut_dim) = -phi_45;
    gkin_jacobi_.block(omega_s,2*ut_dim,ut_dim,ut_dim) = -phi_26;//u
    gkin_jacobi_.block(omega_s,3*ut_dim,ut_dim,ut_dim) = -phi_36;
    gkin_jacobi_.block(jerk_s,5*ut_dim,ut_dim,ut_dim) = -phi_57;
    gkin_jacobi_.block(dt_s,0,ut_dim,ut_dim) = -phi_08;
    gkin_jacobi_.block(dt_s,ut_dim,ut_dim,ut_dim) = -phi_18;//t
    gkin_jacobi_.block(dt_s,2*ut_dim,ut_dim,ut_dim) = -phi_28;
    gkin_jacobi_.block(dt_s,3*ut_dim,ut_dim,ut_dim) = -phi_38;
    gkin_jacobi_.block(dt_s,4*ut_dim,ut_dim,ut_dim) = -phi_48;
    gkin_jacobi_.block(dt_s,5*ut_dim,ut_dim,ut_dim) = -phi_58;

    Eigen::VectorXd g_kin;
    Eigen::Map<const VectorXd> xu(xut.data(),6*x_dim+2*ut_dim);
    GkinCostVec(xu,dt,g_kin);
    grad += 2*params_.opti_w_kin_*gkin_jacobi_*g_kin; //kinmatic grad  w*dg^T*g/dxut = w*dg/dxut*(2*g)
    return params_.opti_w_kin_*g_kin.squaredNorm();  //kinmatic cost
}

// xut: x1,x2,...x(N-2),y1,...y(N-2),...,...,omega(0),...,omega(N-2),jerk(0),...,jerk(N-2),dtau(0),...,dtau(N-2)
inline double Nmpc::CostFunctionCallback(void *func_data, const Eigen::VectorXd &xut, Eigen::VectorXd &grad){
    auto *opt = reinterpret_cast<Nmpc *>(func_data);   
    int x_dim = opt->params_.N_-2;
    int ut_dim = opt->params_.N_-1;

    grad.setZero();
    auto cost_kin = opt->EqConstraints(xut,grad);
    auto cost_ineq = opt->InEqConstraints(xut,grad);

    Eigen::Map<const VectorXd> uj(xut.data()+6*x_dim,ut_dim);
    Eigen::Map<const VectorXd> uw(xut.data()+6*x_dim + ut_dim,ut_dim);
    Eigen::Map<const VectorXd> dtau(xut.data()+6*x_dim+2*ut_dim,ut_dim);
    Eigen::Map<VectorXd> grad_uj(grad.data()+6*x_dim,ut_dim);
    Eigen::Map<VectorXd> grad_uw(grad.data()+6*x_dim + ut_dim,ut_dim);
    Eigen::Map<VectorXd> grad_dtau(grad.data()+6*x_dim+2*ut_dim,ut_dim);

    auto cost_ut = opt->params_.opti_w_uj_*uj.squaredNorm() + opt->params_.opti_w_uw_*uw.squaredNorm() 
                 + opt->params_.opti_w_time_ * dtau.unaryExpr(&Nmpc::Dtau2dt).sum();
    grad_uj += 2*opt->params_.opti_w_uj_*uj;
    grad_uw += 2*opt->params_.opti_w_uw_*uw;
    grad_dtau += opt->params_.opti_w_time_*dtau.unaryExpr(&Nmpc::DtDiff);
    return cost_kin + cost_ineq + cost_ut;
}


inline void Nmpc::BuildNLP(std::vector<geometry_msgs::PoseStamped> &init_poses){
    params_.N_ = init_poses.size();

    //Two points
    x_start_.resize(6);
    x_start_ << init_poses.front().pose.position.x,init_poses.front().pose.position.y,tf::getYaw(init_poses.front().pose.orientation),0,0,0;
    x_end_.resize(6);
    x_end_ << init_poses.back().pose.position.x,init_poses.back().pose.position.y,tf::getYaw(init_poses.back().pose.orientation),0,0,0;

    int x_dim = params_.N_-2;
    int ut_dim = params_.N_-1;
    MatrixXd phiE_0(x_dim,ut_dim),phi0_E(x_dim,ut_dim);
    phiE_0 << MatrixXd::Identity(x_dim,x_dim),MatrixXd::Zero(x_dim,1);
    phi0_E << MatrixXd::Zero(x_dim,1),MatrixXd::Identity(x_dim,x_dim);
    phi_E_ = phiE_0 - phi0_E;
    gkin_jacobi_ = MatrixXd::Zero(6*x_dim+3*ut_dim,6*ut_dim);
    for(int i=0;i<6;i++)gkin_jacobi_.block(i*x_dim,i*ut_dim,x_dim,ut_dim) = phi_E_;
}

inline std::vector<LinearConstraint2D>& Nmpc::BuildCorridor(std::vector<geometry_msgs::PoseStamped> &init_poses){
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

inline void Nmpc::Map2Vec2D(nav_msgs::OccupancyGrid &costmap, vec_Vec2f &vec_map){
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

inline void Nmpc::Poses2Vec2D(std::vector<geometry_msgs::PoseStamped> &init_poses, vec_Vec2f &vec_path){
    for(auto &pose:init_poses){
        vec_path.emplace_back(Vec2f(pose.pose.position.x,pose.pose.position.y));
    }
}

inline void Nmpc::GetGuess(std::vector<geometry_msgs::PoseStamped> &init_poses, States3 &vec_guess){
    vec_guess = States3(init_poses.size());

    auto x0 = init_poses.front().pose.position.x;  //point: 0
    auto y0 = init_poses.front().pose.position.y;
    auto v0 = 0;
    auto a0 = 0;
    auto theta0 = tf::getYaw(init_poses.front().pose.orientation);
    auto gamma0 = 0;
    auto v = 1;
    for(auto i = 1;i<= vec_guess.N_-2;i++){  // xut[point: 1...N-2] 
        auto dx = init_poses[i].pose.position.x-init_poses[i-1].pose.position.x;
        auto dy = init_poses[i].pose.position.y-init_poses[i-1].pose.position.y;
        auto dist = std::hypot(dx,dy);
        vec_guess.x()(i-1) = init_poses[i].pose.position.x;
        vec_guess.y()(i-1) = init_poses[i].pose.position.y;
        vec_guess.v()(i-1) = v;
        vec_guess.a()(i-1) = 0;
        vec_guess.theta()(i-1) = tf::getYaw(init_poses[i].pose.orientation);
        vec_guess.gamma()(i-1) = 0;  //TODO
        vec_guess.omega()(i-1) = 0; //TODO
        vec_guess.jerk()(i-1) = 0; 
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
        vec_guess.tau()(i-1) = tau;
    }
    vec_guess.omega()(vec_guess.N_-2) = 0;  // ut[point: N-1 
    vec_guess.jerk()(vec_guess.N_-2) = 0;
    auto dx = init_poses[vec_guess.N_-1].pose.position.x-init_poses[vec_guess.N_-2].pose.position.x;
    auto dy = init_poses[vec_guess.N_-1].pose.position.y-init_poses[vec_guess.N_-2].pose.position.y;
    auto dist = std::hypot(dx,dy);
    vec_guess.tau()(vec_guess.N_-2) = dist/v; 
}

#include "ego_utils/lbfgs.hpp"
double Nmpc::Solve(std::vector<geometry_msgs::PoseStamped> &init_poses,States3 &result){
  BuildNLP(init_poses);

  // corridor
  BuildCorridor(init_poses);

  // Guess
  States3 guess;
  GetGuess(init_poses, guess);

  // solve
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 16;//128
  lbfgs_params.past = 3; //3 
  lbfgs_params.g_epsilon = 1.0e-16;
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.delta = 1.0e-9;
  lbfgs_params.max_iterations = 120000;

  auto start_time = ros::Time::now();

  double final_cost = 0;
  int flag = lbfgs::lbfgs_optimize(
        guess.xut,
        final_cost,
        Nmpc::CostFunctionCallback,
        NULL,
        NULL,
        this,
        lbfgs_params);

  auto duration = (ros::Time::now() - start_time).toSec();
  ROS_WARN("flag: %s, duration:%f ms",lbfgs::lbfgs_strerror(flag),duration*1000);

  result = guess;
  PublishPath(result);
  
 return final_cost;
}

#include <fstream>
void Nmpc::PublishPath(States3 &result){
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = x_start_(0);  
    pose_stamped.pose.position.y = x_start_(1);
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(x_start_(3));
    path.poses.emplace_back(pose_stamped);
    for (int i = 0;i<result.x().size();i++) {
        pose_stamped.pose.position.x = result.x()(i);
        pose_stamped.pose.position.y = result.y()(i);
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(result.theta()(i));
        path.poses.emplace_back(pose_stamped);
    }
    pose_stamped.pose.position.x = x_end_(0);  
    pose_stamped.pose.position.y = x_end_(1);
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(x_end_(3));
    path.poses.emplace_back(pose_stamped);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    local_path_pub_.publish(path);

    // //print result x y theta v a gamma jerk omega dt
    // std::ofstream result_txt;
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/x.txt",std::ios::out);
    // if(!result_txt.is_open()){
    //     ROS_WARN("[Nmpc::PublishPath]:result_txt open failed!");
    // }
    // for (int i = 0;i<result.x.size();i++) {
    //     result_txt<<result.x[i]<<"\n";  
    // }
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/y.txt",std::ios::out);
    // for (int i = 0;i<result.y.size();i++) {
    //     result_txt<<result.y[i]<<"\n";  
    // }
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/theta.txt",std::ios::out);
    // for (int i = 0;i<result.theta.size();i++) {
    //     result_txt<<result.theta[i]<<"\n";  
    // }
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/v.txt",std::ios::out);
    // for (int i = 0;i<result.v.size();i++) {
    //     result_txt<<result.v[i]<<"\n";
    // }  
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/a.txt",std::ios::out);
    // for (int i = 0;i<result.a.size();i++) {
    //     result_txt<<result.a[i]<<"\n";  
    // }
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/gamma.txt",std::ios::out);
    // for (int i = 0;i<result.gamma.size();i++) {
    //     result_txt<<result.gamma[i]<<"\n";  
    // }
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/jerk.txt",std::ios::out);
    // for (int i = 0;i<result.jerk.size();i++) {
    //     result_txt<<result.jerk[i]<<"\n";  
    // }
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/omega.txt",std::ios::out);
    // for (int i = 0;i<result.omega.size();i++) {
    //     result_txt<<result.omega[i]<<"\n";  
    // }
    // result_txt.close();
    // result_txt.open("/home/photinia/Documents/LoaderPlanner/src/simple_move_base/config/dt.txt",std::ios::out);
    // for (int i = 0;i<result.dt.size();i++) {
    //     result_txt<<result.dt[i]<<"\n";  
    // }
    // result_txt.close();
}

