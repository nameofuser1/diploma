/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_group_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

	// BEGIN_TUTORIAL
	// Start
	// ^^^^^
	// Setting up to start using a planning pipeline is pretty easy.
	// Before we can load the planner, we need two objects, a RobotModel
	// and a PlanningScene.
	// We will start by instantiating a
	// `RobotModelLoader`_
	// object, which will look up
	// the robot description on the ROS parameter server and construct a
	// :moveit_core:`RobotModel` for us to use.
	//
	// .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	// Using the :moveit_core:`RobotModel`, we can construct a
	// :planning_scene:`PlanningScene` that maintains the state of
	// the world (including the robot).
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	// We can now setup the PlanningPipeline
	// object, which will use the ROS param server
	// to determine the set of request adapters and the
	// planning plugin to use
	planning_pipeline::PlanningPipelinePtr planning_pipeline(
	  new planning_pipeline::PlanningPipeline(robot_model, node_handle,
		  "/move_group/planning_plugin", "request_adapters"));

	/* Sleep a little to allow time to startup rviz, etc. */
	ros::WallDuration sleep_time(20.0);
	sleep_time.sleep();

	// Pose Goal
	// ^^^^^^^^^
	// We will now create a motion plan request for the right arm of the PR2
	// specifying the desired pose of the end-effector as input.
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.pose.position.x = 0.75;
	pose.pose.position.y = 0.75;
	pose.pose.position.z = 0.75;
	pose.pose.orientation.w = 1.0;

	// A tolerance of 0.01 m is specified in position
	// and 0.01 radians in orientation
	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	// We will create the request as a constraint using a helper function available
	// from the
	// `kinematic_constraints`_
	// package.
	//
	// .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
	req.group_name = "arm_group";
	moveit_msgs::Constraints pose_goal =
	  kinematic_constraints::constructGoalConstraints("ee_link", pose, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	// Now, call the pipeline and check whether planning was successful.
	planning_pipeline->generatePlan(planning_scene, req, res);
	/* Check that the planning was successful */
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	// Visualize the result
	// ^^^^^^^^^^^^^^^^^^^^
	ros::Publisher display_publisher =
	  node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	/* Visualize the trajectory */
	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);

	sleep_time.sleep();

	return 0;
}
