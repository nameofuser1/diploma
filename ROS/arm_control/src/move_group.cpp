#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <math.h>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "Eigen/Geometry/Transform.h"
#include "Eigen/Geometry/Quaternion.h"

#include <arm_srvs/pick_pose_srv.h>

static const std::string PLANNING_GROUP = "main_group";
static const std::string PLANNING_LINK = "ee_link";

static bool plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group);
static bool move_after_pick(moveit::planning_interface::MoveGroupInterface &move_group);
static bool move_to_table_pick_pose(moveit::planning_interface::MoveGroupInterface &move_group);
static bool move_to_cell_pick_pose(moveit::planning_interface::MoveGroupInterface &,
                                   ros::ServiceClient&,
                                   uint32_t, uint32_t);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
    ros::ServiceClient pick_pose_client = node_handle.serviceClient<arm_srvs::pick_pose_srv>("cell_pick_pose_srv");

	ros::AsyncSpinner spinner(1);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface		move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface	planning_scene_interface;

    ROS_INFO("Current end-effector: %s", move_group.getEndEffector().c_str());
    ROS_INFO("Current end-effector link: %s", move_group.getEndEffectorLink().c_str());

	geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(PLANNING_LINK);
    ROS_INFO_STREAM("Current pose: " << current_pose);

	/*****************************************/
	/*	    Planning in cartesian space		 */
	/*****************************************/
    //ros::WallDuration sleep_t(3.0);
    //sleep_t.sleep();

	move_group.setPlanningTime(5);
    move_group.allowReplanning(true);

    bool success;

    success = move_to_cell_pick_pose(move_group, pick_pose_client, 9, 1);
    if(!success)
    {
        ROS_ERROR("Failed to move to cell pick position");
        return -1;
    } 

    success = move_after_pick(move_group);
    if(!success)
    {
        ROS_ERROR("Failed to move after pick");
        return -1;
    }

    success = move_to_table_pick_pose(move_group);
    if(!success)
    {
        ROS_ERROR("Failed to move to table pick position");
        return -1;
    }

	//ROS_INFO_NAMED("ur5-planning", "Planning succeded: %s", planning_succeded ? "yes" : "no");

	return 0;
}


static bool move_after_pick(moveit::planning_interface::MoveGroupInterface &move_group)
{
    std::vector<double> q = move_group.getCurrentJointValues();
    robot_state::RobotStatePtr p_robot_state = move_group.getCurrentState();
    p_robot_state->copyJointGroupPositions(p_robot_state->getRobotModel()->getJointModelGroup(move_group.getName()), q);

    ROS_INFO("Current joint values: ");
    for (auto i = q.begin(); i != q.end(); ++i)
        std::cout << *i << ' ';

    if(q.at(0) > M_PI)
    {
        q.at(0) = M_PI;
    }
    else
    {
        q.at(0) = 0.0;
    }

    move_group.setJointValueTarget(q);
    return plan_and_execute(move_group);
}


static bool move_to_table_pick_pose(moveit::planning_interface::MoveGroupInterface &move_group)
{
    Eigen::AngleAxisd ax_orientation(-0.5*3.14, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_rot(ax_orientation); // FIX

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.0;
    target_pose.position.y = -0.7;
    target_pose.position.z = 0.520;
    target_pose.orientation.x = q_rot.x();
    target_pose.orientation.y = q_rot.y();
    target_pose.orientation.z = q_rot.z();
    target_pose.orientation.w = q_rot.w();

    move_group.setPoseTarget(target_pose, PLANNING_LINK);
    return plan_and_execute(move_group);
}


static bool move_to_cell_pick_pose(moveit::planning_interface::MoveGroupInterface &move_group,
                                   ros::ServiceClient& pick_pose_service,
                                   uint32_t xi, uint32_t zi)
{
    ROS_INFO_STREAM("Planning for cell x=" << xi << "; z=" << zi << ";");

    arm_srvs::pick_pose_srv srv;
    srv.request.cell_group_id = 0;
    srv.request.xi = xi;
    srv.request.zi = zi;

    if(!pick_pose_service.call(srv))
    {
        ROS_ERROR("Failed to retrieve pick position", xi, zi);
        return false;
    }

    Eigen::AngleAxisd ax_orientation(0.5*3.14, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_rot(ax_orientation); // FIX

    geometry_msgs::Pose target_pose;
    target_pose.position.x = srv.response.pick_pose.position.x;
    target_pose.position.y = srv.response.pick_pose.position.y;
    target_pose.position.z = srv.response.pick_pose.position.z;
    target_pose.orientation.x = q_rot.x();
    target_pose.orientation.y = q_rot.y();
    target_pose.orientation.z = q_rot.z();
    target_pose.orientation.w = q_rot.w();

    move_group.setPoseTarget(target_pose, PLANNING_LINK);
    return plan_and_execute(move_group);
}


static bool plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(!success)
    {
        ROS_ERROR("Failed to plan path");
        return false;
    }

    success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
    {
        ROS_ERROR("Failed to execute trajectory");
        return false;
    }

    return true;
}


