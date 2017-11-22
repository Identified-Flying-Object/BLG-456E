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


    double GlobalPlanner::footprintCost(double x_i, double y_i, double z_i, double theta_i){
        if(!initialized_){
            return -1.0;
        }

        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

        if(footprint.size() < 3)
            return -1.0;

        double footprint_cost = world_model_->footprintCost(x_i, y_i, z_i, theta_i, footprint);
        return footprint_cost;
    }



    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        if(!initialized_)
            return false;



        plan.clear();
        costmap_ = costmap_ros_->getCostmap();



        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;
        double goal_z = goal.pose.position.z;
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;
        double start_z = start.pose.position.z;

        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;
        double diff_z = goal_z - start_z;
        double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

        double target_x = goal_x;
        double target_y = goal_y;
        double target_z = goal_z;

        double target_yaw = goal_yaw;


        bool done = false;
        double scale = 1.0;


        while(!done){

            if(scale < 0){
                target_x = start_x;
                target_y = start_y;
                target_z = start_z;
                break;
            }

            target_x = start_x + scale * diff_x;
            target_y = start_y + scale * diff_y;
            target_z = start_z + scale * diff_z;
            target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

            double footprint_cost = footprintCost(target_x, target_y, target_z, target_yaw);
            if(footprint_cost >= 0)
                     done = true;
        }


        plan.push_back(start);
            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

            new_goal.pose.position.x = target_x;
            new_goal.pose.position.y = target_y;
            new_goal.pose.position.z = target_z;

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();


        plan.push_back(goal);
        return done;
    }
};