#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <hybrid_a_star/hybrid_a_star_flow.h>
#include "nmpc_loader.h"

enum PlanSta{
    NONE,
    PLANNING,
    SHUT_DOWN
};
class SimpleMoveBase{
public:
    SimpleMoveBase(tf2_ros::Buffer &buffer);
    ~SimpleMoveBase();

private:
    tf2_ros::Buffer &buffer_;
    nav_msgs::OccupancyGrid costmap_;

    bool has_initpose_,has_endpose_,has_map_;
    ros::Subscriber initpose_sub_, endpose_sub_, map_sub_;
    std::string initpose_topic_name_, endpose_topic_name_, map_topic_name_;
    geometry_msgs::PoseStamped initpose_, endpose_;

    PlanSta blp_type_ = PlanSta::NONE;
    PlanSta glp_type_ = PlanSta::NONE;

    double global_plan_rate_ = -1;  //TODO
    double ctl_rate_ = 100; 

    std::mutex plan_mtx_; //用来锁global_plan_
    HybridAStarFlow *kinodynamic_astar_flow_;
    std::vector<geometry_msgs::PoseStamped> *global_plan_,*next_global_plan_;
    
    std::mutex glp_mtx_;  //规划线程锁
    std::condition_variable glp_con_;
    std::thread *global_plan_thread_;
    std::thread *control_thread_;

    Nmpc *loader_controller_;

    bool ctl_permiss_=false;  //TEST
    
    void PlanThreadFcn();
    void CtlThreadFcn();
    void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initpose);
    void EndPoseCallback(const geometry_msgs::PoseStampedConstPtr &endpose);
    void MapCallback(const nav_msgs::OccupancyGrid &costmap);
};
