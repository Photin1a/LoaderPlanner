/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "hybrid_a_star/hybrid_a_star_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

HybridAStarFlow::HybridAStarFlow() {
    ros::NodeHandle public_nh;
    double steering_angle = public_nh.param("hybrid_Astar_planner/steering_angle", 10);
    int steering_angle_discrete_num = public_nh.param("hybrid_Astar_planner/steering_angle_discrete_num", 1);
    double wheel_base = public_nh.param("hybrid_Astar_planner/wheel_base", 1.0);
    double segment_length = public_nh.param("hybrid_Astar_planner/segment_length", 1.6);
    int segment_length_discrete_num = public_nh.param("hybrid_Astar_planner/segment_length_discrete_num", 8);
    double steering_penalty = public_nh.param("hybrid_Astar_planner/steering_penalty", 1.05);
    double steering_change_penalty = public_nh.param("hybrid_Astar_planner/steering_change_penalty", 1.5);
    double reversing_penalty = public_nh.param("hybrid_Astar_planner/reversing_penalty", 2.0);
    double shot_distance = public_nh.param("hybrid_Astar_planner/shot_distance", 5.0);

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(public_nh, "/map", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(public_nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(public_nh, "/move_base_simple/goal", 1);

    path_pub_ = public_nh.advertise<nav_msgs::Path>("global_path", 1);
    searched_tree_pub_ = public_nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = public_nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);

    has_map_ = false;
}

bool HybridAStarFlow::Run(nav_msgs::OccupancyGrid *occupancygrid_map, geometry_msgs::PoseStamped initpose,geometry_msgs::PoseStamped endpose) {
    if(nullptr==occupancygrid_map){
        ROS_WARN("[HybridAStarFlow::Run]:Empty map!");
        return false;
    }
    const double map_resolution = 0.2;
    kinodynamic_astar_searcher_ptr_->Init(
            occupancygrid_map->info.origin.position.x,
            1.0 * occupancygrid_map->info.width * occupancygrid_map->info.resolution,
            occupancygrid_map->info.origin.position.y,
            1.0 * occupancygrid_map->info.height * occupancygrid_map->info.resolution,
            occupancygrid_map->info.resolution,
            map_resolution
    );

    unsigned int map_w = std::floor(occupancygrid_map->info.width / map_resolution);
    unsigned int map_h = std::floor(occupancygrid_map->info.height / map_resolution);
    for (unsigned int w = 0; w < map_w; ++w) {
        for (unsigned int h = 0; h < map_h; ++h) {
            auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                / occupancygrid_map->info.resolution);
            auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                / occupancygrid_map->info.resolution);

            if (occupancygrid_map->data[y * occupancygrid_map->info.width + x]) {
                kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
            }
        }
    }
    double start_yaw = tf::getYaw(initpose.pose.orientation);
    double goal_yaw = tf::getYaw(endpose.pose.orientation);

    Vec3d start_state = Vec3d(
            initpose.pose.position.x,
            initpose.pose.position.y,
            start_yaw
    );
    Vec3d goal_state = Vec3d(
            endpose.pose.position.x,
            endpose.pose.position.y,
            goal_yaw
    );
    auto state = false;
    if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
        path_ = kinodynamic_astar_searcher_ptr_->GetPath();
        PublishPath(path_);
        state = true;
    }
    kinodynamic_astar_searcher_ptr_->Reset();
    return state;
}

bool HybridAStarFlow::MakePlan(nav_msgs::OccupancyGrid &occupancygrid_map, geometry_msgs::PoseStamped initpose,geometry_msgs::PoseStamped endpose,std::vector<geometry_msgs::PoseStamped>& plan){    
    path_.clear();
    plan.clear();
    auto state = Run(&occupancygrid_map,initpose,endpose);
    geometry_msgs::PoseStamped pose_stamped;
    if(true==state){
        for (const auto &pose: path_) {
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = pose.x();
            pose_stamped.pose.position.y = pose.y();
            pose_stamped.pose.position.z = 0.0;

            pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());
            plan.push_back(pose_stamped);
        }
    }
    return state;
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool HybridAStarFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasStartPose() {
    return !init_pose_deque_.empty();
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}