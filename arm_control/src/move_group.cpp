#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "Eigen/Geometry/Transform.h"
#include "Eigen/Geometry/Quaternion.h"

#include <map>
#include <vector>
#include <algorithm>
#include <iostream>


static const std::string PLANNING_GROUP = "arm_group";
static const std::string HOME_POSE_NAME = "home";


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	/* Loading move group */
	moveit::planning_interface::MoveGroupInterface		move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface	planning_scene;
	
	ROS_INFO_NAMED("tutorial", "Reference frame: %s",
			move_group.getPlanningFrame().c_str());

	ROS_INFO_NAMED("tutorial", "End effector link: %s",
			move_group.getEndEffectorLink().c_str());
	
	/* Loading robot model and its' kinematic state */
	robot_model::RobotModelConstPtr robot_model  = move_group.getRobotModel();
	robot_state::RobotState *kinematic_state = new robot_state::RobotState(robot_model);

	const std::string EE_NAME = move_group.getEndEffectorLink();

	const robot_model::JointModelGroup *joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	

	/* Getting HOME postition and printing it */
	std::vector<double> home_joint_values;
	const std::map<std::string, double> home_joints_map =
		move_group.getNamedTargetValues(HOME_POSE_NAME);
	
	for(auto& x: home_joints_map) {
		home_joint_values.push_back(x.second);
		ROS_INFO_STREAM(x.first << ": " << x.second);
	}

	/* Setting HOME position as target */
	bool planning_succeded = false;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	// move_group.setJointValueTarget(home_joints_map);

	/* Planning path in Joint Space to a goal position */
	//planning_succeded = move_group.plan(my_plan);
	//ROS_INFO_NAMED("ur5-planning", "Planning succeded: %s", planning_succeded ? "yes" : "no");


	/*****************************************/
	/*	    Planning in cartesian space		 */
	/*****************************************/
	ROS_INFO_STREAM("Getting transformation on " << EE_NAME);

	kinematic_state->setJointGroupPositions(joint_model_group, home_joint_values);
	Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform(EE_NAME);

	Eigen::Quaterniond q_pos(end_effector_state.rotation());
	auto tr = end_effector_state.translation();
	
	ROS_INFO_STREAM("Translation:\r\n" << end_effector_state.translation());
	ROS_INFO_STREAM("Rotation:\r\n" << end_effector_state.rotation());
	ROS_INFO_STREAM("Qauternion rotation:\r\n" << q_pos.vec());

	geometry_msgs::Pose target_pose;
	target_pose.position.x = tr[0];
	target_pose.position.y = tr[1];
	target_pose.position.z = tr[2];
	target_pose.orientation.x = q_pos.x();
	target_pose.orientation.y = q_pos.y();
	target_pose.orientation.z = q_pos.z();
	target_pose.orientation.w = q_pos.w();
	
	move_group.setPoseTarget(target_pose);
	planning_succeded = move_group.plan(my_plan);
	// move_group.execute(my_plan);

	ROS_INFO_NAMED("ur5-planning", "Planning succeded: %s", planning_succeded ? "yes" : "no");

	return 0;
}
