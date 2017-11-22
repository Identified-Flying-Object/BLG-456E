//
// Created by emre on 22.11.2017.
//

#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

    GlobalPlanner::GlobalPlanner (){

    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }


    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
            costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_

            // initialize other planner parameters
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            world_model_ = new base_local_planner::CostmapModel(*costmap_);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        plan.push_back(start);
        for (int i=0; i<20; i++){
            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

            new_goal.pose.position.x = -2.5+(0.05*i);
            new_goal.pose.position.y = -3.5+(0.05*i);

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);
        }
        plan.push_back(goal);
        return true;
    }
};