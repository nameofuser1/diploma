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

namespace rvt = rviz_visual_tools;
static moveit_visual_tools::MoveItVisualTools* visual_tools;


static bool prepare_next_cell(moveit::planning_interface::MoveGroupInterface &move_group,
                              ros::ServiceClient&,
                              uint32_t, uint32_t);
static bool move_after_pick(moveit::planning_interface::MoveGroupInterface &move_group);
static bool move_to_table_pick_pose(moveit::planning_interface::MoveGroupInterface &move_group);
static bool move_to_cell_pick_pose(moveit::planning_interface::MoveGroupInterface &,
                                   ros::ServiceClient&,
                                   uint32_t, uint32_t);
static bool move_to_home(moveit::planning_interface::MoveGroupInterface &move_group);
static bool plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group);


static void visualize_path(moveit_msgs::RobotTrajectory &tr, robot_state::RobotState &robot_state);
static void visualize_pose(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose &pose);
static void visualize_pose(moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> q);
static void publish_state(robot_state::RobotState &pose_state);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
    ros::ServiceClient pick_pose_client = node_handle.serviceClient<arm_srvs::pick_pose_srv>("cell_pick_pose_srv");

	ros::AsyncSpinner spinner(1);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface		move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface	planning_scene_interface;

    visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
    visual_tools->loadRemoteControl();

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

    if(!move_to_home(move_group))
    {
        ROS_ERROR("Failed to move to a home position");
        return -1;
    }

    if(!prepare_next_cell(move_group, pick_pose_client, 8, 2))
    {
        ROS_ERROR_STREAM("Failed to service cell " << 9 << " " << 1);
        return -1;
    }

    if(!move_to_home(move_group))
    {
        ROS_ERROR("Failed to move to a home position");
        return -1;
    }

    if(!prepare_next_cell(move_group, pick_pose_client, 9, 1))
    {
        ROS_ERROR_STREAM("Failed to service cell " << 9 << " " << 2);
        return -1;
    }

    if(!move_to_home(move_group))
    {
        ROS_ERROR("Failed to move to a home position");
        return false;
    }

	return 0;
}


static bool prepare_next_cell(moveit::planning_interface::MoveGroupInterface &move_group,
                              ros::ServiceClient& pick_pose_client,
                              uint32_t xi, uint32_t zi)
{
    bool success;

    success = move_to_cell_pick_pose(move_group, pick_pose_client, xi, zi);
    if(!success)
    {
        ROS_ERROR("Failed to move to cell pick position");
        return false;
    }
    ROS_INFO_STREAM("Cell pick pose: " << std::endl << move_group.getCurrentPose(PLANNING_LINK));

    success = move_after_pick(move_group);
    if(!success)
    {
        ROS_ERROR("Failed to move after pick");
        return false;
    }

    ROS_INFO_STREAM("Pose after preparing to table pick: " << std::endl << move_group.getCurrentPose(PLANNING_LINK));

    success = move_to_table_pick_pose(move_group);
    if(!success)
    {
        ROS_ERROR("Failed to move to table pick position");
        return false;
    }

    ROS_INFO_STREAM("Final pose is: " << std::endl << move_group.getCurrentPose(PLANNING_LINK));
    return true;
}


static bool move_to_home(moveit::planning_interface::MoveGroupInterface &move_group)
{
    std::vector<double> q = move_group.getCurrentJointValues();
    robot_state::RobotStatePtr p_robot_state = move_group.getCurrentState();
    p_robot_state->copyJointGroupPositions(p_robot_state->getRobotModel()->getJointModelGroup(move_group.getName()), q);

    q.at(0) = 0;
    q.at(1) = -M_PI/2.0;
    q.at(2) = M_PI/2.0;
    q.at(3) = M_PI;
    q.at(4) = -M_PI/2.0;
    q.at(5) = -M_PI/2.0;

    move_group.setJointValueTarget(q);
    visualize_pose(move_group, q);
    return plan_and_execute(move_group);
}


static bool move_after_pick(moveit::planning_interface::MoveGroupInterface &move_group)
{
    std::vector<double> q = move_group.getCurrentJointValues();
    robot_state::RobotStatePtr p_robot_state = move_group.getCurrentState();
    p_robot_state->copyJointGroupPositions(p_robot_state->getRobotModel()->getJointModelGroup(move_group.getName()), q);

    ROS_INFO("Current joint values: ");
    for (auto i = q.begin(); i != q.end(); ++i)
        std::cout << *i << ' ';

    bool left_half = q.at(0) > (M_PI / 2.0);

    /*********** First motion **********/
    if(left_half)
    {
        q.at(0) = M_PI;
        q.at(4) = M_PI/2.0;
    }
    else
    {
        q.at(0) = 0.0;
        q.at(4) = -M_PI/2.0;
    }

    move_group.setJointValueTarget(q);
    visualize_pose(move_group, q);
    if(!plan_and_execute(move_group))
    {
        ROS_ERROR("Failed to perform the first motion");
        return false;
    }

    /********** Second motion **********/
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(PLANNING_LINK);
    ROS_INFO_STREAM("Pose after the first movement: " << current_pose);

    /* Target angles */
    p_robot_state->copyJointGroupPositions(p_robot_state->getRobotModel()->getJointModelGroup(move_group.getName()), q);
    q.at(0) = (left_half) ? M_PI : 0.0;
    q.at(1) = -M_PI/2.0;
    q.at(2) = M_PI/2.0;
    q.at(3) = M_PI;
    q.at(4) = (left_half) ? M_PI/2.0 : -M_PI/2.0;
    //q.at(5) = (left_half) ? -M_PI/2.0 : M_PI/2.0;

    /* Transition to Cartesian space */
    /*
    robot_model::RobotModelConstPtr robot_model  = move_group.getRobotModel();
    robot_state::RobotState kinematic_state(robot_model);

    const robot_model::JointModelGroup joint_model_group =
        *(move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));

    kinematic_state.setJointGroupPositions(&joint_model_group, q);
    Eigen::Affine3d end_effector_state = kinematic_state.getGlobalLinkTransform(PLANNING_LINK);

    Eigen::Quaterniond q_pos(end_effector_state.rotation());
    auto tr = end_effector_state.translation();

    geometry_msgs::Pose target_pose;
    target_pose.position.x = tr[0];
    target_pose.position.y = tr[1];
    target_pose.position.z = tr[2];
    target_pose.orientation.x = (fabs(q_pos.x()) >= 0.001) ? q_pos.x() : 0.0;
    target_pose.orientation.y = (fabs(q_pos.y()) >= 0.001) ? q_pos.y() : 0.0;
    target_pose.orientation.z = (fabs(q_pos.z()) >= 0.001) ? q_pos.z() : 0.0;
    target_pose.orientation.w = q_pos.w();

    ROS_INFO_STREAM("After pick second required position:" << std::endl << target_pose);
    move_group.clearPathConstraints();
    move_group.setPoseTarget(target_pose);*/

    move_group.setJointValueTarget(q);
    visualize_pose(move_group, q);
    if(!plan_and_execute(move_group))
    {
        ROS_ERROR("Failed to perform second motion");
        return false;
    }

    ROS_INFO_STREAM("After pick second actual position: " << std::endl << move_group.getCurrentPose(PLANNING_LINK));

    q.at(0) = -M_PI/2.0;
    move_group.setJointValueTarget(q);
    visualize_pose(move_group, q);

    if(!plan_and_execute(move_group))
    {
        ROS_ERROR("Failed to perform third motion");
        return false;
    }



    return true;
}


static bool move_to_table_pick_pose(moveit::planning_interface::MoveGroupInterface &move_group)
{
    /*
    Eigen::AngleAxisd ax_orientation(-0.5*3.14, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_rot(ax_orientation); // FIX

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.0;
    target_pose.position.y = -0.8-0.3;
    target_pose.position.z = 0.520;
    target_pose.orientation.x = q_rot.x();
    target_pose.orientation.y = q_rot.y();
    target_pose.orientation.z = q_rot.z();
    target_pose.orientation.w = q_rot.w();

    move_group.setPoseTarget(target_pose, PLANNING_LINK);
    visualize_pose(move_group, target_pose);

    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(PLANNING_LINK);
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = PLANNING_LINK;
    ocm.header.frame_id = current_pose.header.frame_id;
    ocm.orientation = current_pose.pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 2.0*M_PI;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);*/

    std::vector<double> q = move_group.getCurrentJointValues();
    robot_state::RobotStatePtr p_robot_state = move_group.getCurrentState();
    p_robot_state->copyJointGroupPositions(p_robot_state->getRobotModel()->getJointModelGroup(move_group.getName()), q);

    q.at(1) = 0.0;
    q.at(2) = -M_PI/6.0;
    q.at(3) = 7.0*M_PI/6.0; //5.0*M_PI/4.0;

    move_group.setJointValueTarget(q);
    visualize_pose(move_group, q);

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

    moveit_msgs::RobotTrajectory &tr = my_plan.trajectory_;
    robot_state::RobotState pose_state(*move_group.getCurrentState());
    visualize_path(tr, pose_state);

    success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
    {
        ROS_ERROR("Failed to execute trajectory");
        return false;
    }

    return true;
}


static void visualize_pose(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose &pose)
{
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState pose_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose;
    start_pose.position = pose.position;
    start_pose.orientation = pose.orientation;
    pose_state.setFromIK(joint_model_group, start_pose);

    publish_state(pose_state);
}


static void visualize_pose(moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> q)
{
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState pose_state(*move_group.getCurrentState());
    pose_state.setJointGroupPositions(joint_model_group->getName(), q);

    publish_state(pose_state);
}


static void visualize_path(moveit_msgs::RobotTrajectory &tr, robot_state::RobotState &robot_state)
{
    if(!visual_tools->publishTrajectoryPath(tr, robot_state, true))
    {
        ROS_ERROR("Failed to publish trajectory");
    }
}


static void publish_state(robot_state::RobotState &pose_state)
{
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

    // Publish arrow vector of pose
    ROS_INFO("Publishing Arrow");
    if(!visual_tools->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE))
    {
        ROS_ERROR("Failed to publish arrow");
    }

    if(!visual_tools->publishRobotState(pose_state))
    {
        ROS_ERROR("Failed to publish robot state");
    }

    if(!visual_tools->trigger())
    {
        ROS_ERROR("Failed to trigger publishing");
    }
}

