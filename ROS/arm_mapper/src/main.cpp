#include <iostream>
#include <vector>

#include "environment.hpp"

#include <ros/ros.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <arm_srvs/pick_pose_srv.h>


#define TEST_CELL_GROUP_XN  16
#define TEST_CELL_GPOUP_ZN  3
#define TEST_CELL_X_SIZE    0.350
#define TEST_CELL_Z_SIZE    0.210
#define TEST_BASE_X_SIZE    0.350
#define TEST_BASE_Z_SIZE    0.300


static void attach_collision_objects(moveit::planning_interface::PlanningSceneInterface &);


environment::CellGroup *m_cell_group;

bool cell_pick_pose_srv_handler(arm_srvs::pick_pose_srv::Request &req,
                                arm_srvs::pick_pose_srv::Response &resp)
{
    uint32_t xi = req.xi;
    uint32_t zi = req.zi;

    environment::Cell *cell = m_cell_group->getCellByXZ(xi, zi);
    if(cell == NULL)
    {
        return false;
    }

    environment::object_position_t pos = m_cell_group->getCellPickPosition(cell);
    resp.pick_pose.position.x = pos.x;
    resp.pick_pose.position.y = pos.y;
    resp.pick_pose.position.z = pos.z;

    resp.pick_pose.normal_vector.x = pos.normal[0];
    resp.pick_pose.normal_vector.y = pos.normal[1];
    resp.pick_pose.normal_vector.z = pos.normal[2];

    return true;
}


int main(int argc, char **argv)
{
    
    environment::object_position_t pos;
    pos.x = -2.8f;
    pos.y = 1.05f;
    pos.z = -0.1f;
    pos.normal[0] = 0.0f;
    pos.normal[1] = 1.0f;
    pos.normal[2] = 0.0f;

    m_cell_group = new environment::CellGroup(TEST_CELL_GROUP_XN, TEST_CELL_GPOUP_ZN,
                                              TEST_CELL_X_SIZE, TEST_CELL_Z_SIZE,
                                              TEST_BASE_X_SIZE, TEST_BASE_Z_SIZE,
                                              pos);

    ros::init(argc, argv, "Environment node init.");
    ros::NodeHandle environment_node;

    ros::WallDuration sleep(10);
    sleep.sleep();
    
    moveit::planning_interface::PlanningSceneInterface	planning_scene_interface;
    attach_collision_objects(planning_scene_interface);
    ROS_INFO("Collisions are set");

    ros::ServiceServer service = environment_node.advertiseService("cell_pick_pose_srv", cell_pick_pose_srv_handler);
    ROS_INFO("Cell pick pose service is ready!");

    ros::spin();
    return 0;
}


/**************************** COLLISIONS **********************************/

static moveit_msgs::CollisionObject create_floor_collision_object(void);
static moveit_msgs::CollisionObject create_table_collision_object(void);
static moveit_msgs::CollisionObject create_cell_group0_collision_object(void);
static moveit_msgs::CollisionObject create_cell_group1_collision_object(void);
static moveit_msgs::CollisionObject create_cell_group2_collision_object(void);
static moveit_msgs::CollisionObject create_cell_group_collision_object(uint32_t x_size, uint32_t z_size,
                                                                       geometry_msgs::Pose pose, std::string id);


const float base_dim_x = 0.350;
const float base_dim_y = 0.450;
const float base_dim_z = 0.300;

const float cell_dim_x = 0.350;
const float cell_dim_y = 0.450;
const float cell_dim_z = 0.210;


static void attach_collision_objects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(create_floor_collision_object());
    collision_objects.push_back(create_table_collision_object());
    collision_objects.push_back(create_cell_group0_collision_object());
    collision_objects.push_back(create_cell_group1_collision_object());
    collision_objects.push_back(create_cell_group2_collision_object());

    std::cout << "collision_objects.size(): " << collision_objects.size() << std::endl;
    std::cout << collision_objects[0] << std::endl;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    std::vector<std::string> obj = planning_scene_interface.getKnownObjectNames();
    if (obj.size() > 0)
    {
        ROS_INFO("-- KNOWN COLLISION OBJECTS --");
        for (int i = 0; i < obj.size(); ++i)
        {
            ROS_INFO_STREAM("" << obj[i]);
        }
    }
}


static moveit_msgs::CollisionObject create_floor_collision_object(void)
{
    moveit_msgs::CollisionObject collision_object;
    /* The header must contain a valid TF frame*/
    collision_object.header.frame_id = "world";
    /* The id of the object */
    collision_object.id = "floor";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = -0.1-0.05-0.005;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 10.0;
    primitive.dimensions[1] = 10.0;
    primitive.dimensions[2] = 0.10;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

static moveit_msgs::CollisionObject create_table_collision_object(void)
{
    moveit_msgs::CollisionObject top_surface;
    /* The header must contain a valid TF frame*/
    top_surface.header.frame_id = "world";
    /* The id of the object */
    top_surface.id = "table_top_surface";

    tf::Quaternion table_quat = tf::createQuaternionFromRPY(0.0, 0, 0);

    /* A default pose */
    geometry_msgs::Pose top_pose;
    top_pose.position.x = 0.0;
    top_pose.position.y = -0.8-0.3;
    top_pose.position.z = 0.305;
    tf::quaternionTFToMsg(table_quat, top_pose.orientation);

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive top_primitive;
    top_primitive.type = top_primitive.BOX;
    top_primitive.dimensions.resize(3);
    top_primitive.dimensions[0] = 1.0;
    top_primitive.dimensions[1] = 0.6;
    top_primitive.dimensions[2] = 0.610;

    top_surface.primitives.push_back(top_primitive);
    top_surface.primitive_poses.push_back(top_pose);
    top_surface.operation = top_surface.ADD;

    return top_surface;
}


static moveit_msgs::CollisionObject create_cell_group0_collision_object(void)
{
    const uint32_t x_size = 4;
    const uint32_t z_size = 3;
    const tf::Quaternion pose_quat = tf::createQuaternionFromRPY(0.0, 0, 0);

    geometry_msgs::Pose pose;
    pose.position.x = -0.75 - (1.4 / 2.0);
    pose.position.y = -0.8 - (cell_dim_y / 2.0);
    pose.position.z = 0.465;
    tf::quaternionTFToMsg(pose_quat, pose.orientation);

    return create_cell_group_collision_object(x_size, z_size, pose, "cell_group0");
}


static moveit_msgs::CollisionObject create_cell_group1_collision_object(void)
{
    const uint32_t x_size = 4;
    const uint32_t z_size = 3;
    const tf::Quaternion pose_quat = tf::createQuaternionFromRPY(0.0, 0, 0);

    geometry_msgs::Pose pose;
    pose.position.x = 0.75 + (1.4 / 2.0);
    pose.position.y = -0.8 - (cell_dim_y / 2.0);
    pose.position.z = 0.465;
    tf::quaternionTFToMsg(pose_quat, pose.orientation);

    return create_cell_group_collision_object(x_size, z_size, pose, "cell_group1");
}


static moveit_msgs::CollisionObject create_cell_group2_collision_object(void)
{
    const uint32_t x_size = 16;
    const uint32_t z_size = 3;
    const tf::Quaternion pose_quat = tf::createQuaternionFromRPY(0.0, 0, 0.0);

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 1.1 + (cell_dim_y / 2.0);
    pose.position.z = 0.465;
    tf::quaternionTFToMsg(pose_quat, pose.orientation);

    return create_cell_group_collision_object(x_size, z_size, pose, "cell_group2");
}


static moveit_msgs::CollisionObject create_cell_group_collision_object(uint32_t x_size, uint32_t z_size,
                                                                       geometry_msgs::Pose pose, std::string id)
{
    moveit_msgs::CollisionObject collision_object;
    /* The header must contain a valid TF frame*/
    collision_object.header.frame_id = "world";
    /* The id of the object */
    collision_object.id = id;

    float collision_box_x_size = (float)x_size * cell_dim_x;
    float collision_box_y_size = cell_dim_y;
    float collision_box_z_size = (float)z_size * cell_dim_z + base_dim_z;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = collision_box_x_size;
    primitive.dimensions[1] = collision_box_y_size;
    primitive.dimensions[2] = collision_box_z_size;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}
