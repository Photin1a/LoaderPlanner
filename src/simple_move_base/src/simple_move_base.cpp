#include "simple_move_base.h"

SimpleMoveBase::SimpleMoveBase(tf2_ros::Buffer &buffer):buffer_(buffer){
    has_initpose_=false;
    has_endpose_=false;
    has_map_ = false;
    global_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    next_global_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    ros::NodeHandle public_nh;
    public_nh.param<std::string>("simple_move_base/initpose_topic_name",initpose_topic_name_,"/initialpose");
    public_nh.param<std::string>("simple_move_base/endpose_topic_name",endpose_topic_name_,"/move_base_simple/goal");
    public_nh.param<std::string>("simple_move_base/map_topic_name",map_topic_name_,"/map");

    initpose_sub_=public_nh.subscribe(initpose_topic_name_,1,&SimpleMoveBase::InitPoseCallback,this);
    endpose_sub_=public_nh.subscribe(endpose_topic_name_,1,&SimpleMoveBase::EndPoseCallback,this);
    map_sub_ = public_nh.subscribe(map_topic_name_,1,&SimpleMoveBase::MapCallback,this);
    
    kinodynamic_astar_flow_ = new HybridAStarFlow();
    global_plan_thread_ = new std::thread(&SimpleMoveBase::PlanThreadFcn,this);

    loader_controller_ = new Nmpc(&costmap_);
    control_thread_ = new std::thread(&SimpleMoveBase::CtlThreadFcn,this);
}

SimpleMoveBase::~SimpleMoveBase(){
    glp_type_=PlanSta::SHUT_DOWN;
    glp_con_.notify_all();
    global_plan_thread_->join();
    control_thread_->join();
    delete kinodynamic_astar_flow_;
    delete loader_controller_;
    delete global_plan_thread_;
    delete control_thread_;
}

void SimpleMoveBase::PlanThreadFcn(){
    while(ros::ok()){
        std::unique_lock<std::mutex> lock(glp_mtx_,std::defer_lock);
        if(!lock.try_lock())continue;
        glp_con_.wait(lock,[this](){return (PlanSta::PLANNING==glp_type_||PlanSta::SHUT_DOWN==glp_type_)?true:false;});
        if(PlanSta::SHUT_DOWN == glp_type_)return;  //退出
        if(!kinodynamic_astar_flow_->MakePlan(costmap_,initpose_,endpose_,*next_global_plan_)){
            ROS_WARN("[SimpleMoveBase::PlanThreadFcn]:Planner failed!");
        }
        else ROS_INFO("[SimpleMoveBase::PlanThreadFcn]:Global Plan successful.");
        {
            std::unique_lock<std::mutex> lock(plan_mtx_);
            auto temp_plan = global_plan_;
            global_plan_ = next_global_plan_;
            next_global_plan_ = temp_plan;
        }
        glp_type_=PlanSta::NONE;
        ctl_permiss_ = true; //TEST
    }
}

void SimpleMoveBase::CtlThreadFcn(){
    ros::Rate rate(ctl_rate_);
    States result;
    while(ros::ok()){
        if(true == ctl_permiss_ && !global_plan_->empty() && true == has_map_){
            std::unique_lock<std::mutex> lock(plan_mtx_);
            auto cost = loader_controller_->Solve(*global_plan_,result);
            ROS_INFO("[SimpleMoveBase::CtlThreadFcn]: loader_controller plan cost:%f",cost);
            //TEST
            ctl_permiss_ = false;
        }
        rate.sleep();
    }
}

void SimpleMoveBase::InitPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initpose){
    initpose_.header = initpose->header;
    initpose_.pose = initpose->pose.pose;
    has_initpose_=true;
    ROS_INFO("SimpleMoveBase::InitPoseCallback");
}

void SimpleMoveBase::EndPoseCallback(const geometry_msgs::PoseStampedConstPtr &endpose){
    endpose_ = *endpose;
    has_endpose_=true;
    if(has_initpose_&&has_endpose_&&has_map_){
        glp_type_ = PlanSta::PLANNING;
        glp_con_.notify_all();
        has_initpose_=false;
        has_endpose_=false;
    }
    ROS_INFO("SimpleMoveBase::EndPoseCallback");
}

void SimpleMoveBase::MapCallback(const nav_msgs::OccupancyGrid &costmap){
    costmap_ = costmap;
    has_map_ = true;
    ROS_INFO("SimpleMoveBase::MapCallback");
}

