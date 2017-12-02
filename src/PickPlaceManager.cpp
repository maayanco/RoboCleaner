#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/SetBool.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickClient;
typedef actionlib::SimpleActionClient<moveit_msgs::PlaceAction> PlaceClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;

//Forward declerations
void look_down();
bool set_collision_update(bool state);
moveit_msgs::PickupGoal build_pick_goal(const std::string &objectName);
bool pick_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool pick_and_place_sync_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool pick_and_place_sync1_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool gripper_cmd(GripperClient* gripperClient,double gap,double effort);
bool place_cb_gripper(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
void shut_down_cb(const std_msgs::String::ConstPtr& msg);
bool return_to_position(const std::string &posName);
std::string endPositionName;
std::string startPositionName;

ros::ServiceClient *ucClientPtr;
ros::Publisher controllerCommandPub;
ros::Subscriber shutDownSub;


int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_place_manager_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    srand((unsigned int) time(NULL));
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    std::string objectName;

    pn.param<std::string>("start_position_name", startPositionName, "pre_grasp3");
    pn.param<std::string>("end_position_name", endPositionName, "pre_grasp1");
    pn.param<std::string>("objectName", objectName, "can");

    ros::ServiceServer pickService = n.advertiseService("pick_go", &pick_cb);
    ros::ServiceServer placeService = n.advertiseService("place_go_gripper",&place_cb_gripper);
    shutDownSub = n.subscribe("shut_down_go", 10, shut_down_cb);

    //Config move group
    moveit::planning_interface::MoveGroup group("arm");
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(40.0);
    group.setNumPlanningAttempts(10);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");

    group.setStartStateToCurrentState();
    group.setNamedTarget(startPositionName);
    moveit::planning_interface::MoveGroup::Plan startPosPlan;
    if(group.plan(startPosPlan)) { //Check if plan is valid
        group.execute(startPosPlan);
        
        controllerCommandPub = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);

        ros::ServiceServer pickAndPlaceSyncPub1 = n.advertiseService("pick_and_place_sync1",&pick_and_place_sync1_cb);
        ROS_INFO("[%s]: Ready!",ros::this_node::getName().c_str());

        ROS_INFO("[%s]: Waiting for the moveit action server to come up",ros::this_node::getName().c_str());
        ros::ServiceClient ucClient = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
        ROS_INFO("[%s]: Waiting for update_collision service...",ros::this_node::getName().c_str());
        ucClient.waitForExistence();
        ucClientPtr = &ucClient;
        set_collision_update(true);

        look_down();
        
        ROS_INFO("[%s]: Looking down...",ros::this_node::getName().c_str());
        ros::ServiceServer pickAndPlaceSyncPub = n.advertiseService("pick_and_place_sync",&pick_and_place_sync_cb);
        ROS_INFO("[%s]: Ready!",ros::this_node::getName().c_str());
        ros::Duration(10.0).sleep();
    }
    else {
        ROS_ERROR("Error");
    }
    ros::waitForShutdown();
    return 0;
}

void shut_down_cb(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), msg->data.c_str());
    ros::shutdown();
}

bool gripper_cmd(GripperClient* gripperClient,double gap,double effort) {

    control_msgs::GripperCommandGoal openGoal;

    openGoal.command.position = gap;
    openGoal.command.max_effort = effort;
    gripperClient->sendGoal(openGoal);
    ROS_INFO("[%s]: Sent gripper goal",ros::this_node::getName().c_str());
    gripperClient->waitForResult();

    if(gripperClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[%s]: Gripper done",ros::this_node::getName().c_str());
        return true;
    }
    else {
        ROS_ERROR("[%s]: Gripper fault",ros::this_node::getName().c_str());
    }
    return false;
}

bool return_to_position(const std::string &posName)
{
    ROS_INFO("[%s]: Moving arm to position %s",ros::this_node::getName().c_str(),posName.c_str());
    moveit::planning_interface::MoveGroup group("arm");

    //Config move group
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(40.0);
    group.setNumPlanningAttempts(50);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");

    group.setStartStateToCurrentState();
    group.setNamedTarget(posName);
    moveit::planning_interface::MoveGroup::Plan startPosPlan;
    
    //addition - collision off
    ros::NodeHandle nnn;
    ros::ServiceClient ucClient = nnn.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = false;

    if(ucClient.call(disableColl)) {
        ROS_INFO("[%s]: update_colision response: OFF ",ros::this_node::getName().c_str());
    }
    
    
    if(group.plan(startPosPlan)) { //Check if plan is valid
        group.execute(startPosPlan);
	    ROS_INFO("[%s]: returned to arm start position",ros::this_node::getName().c_str());
        ros::Duration(10.0).sleep();

	    return true;
    }
    else {
        ROS_ERROR("[%s]: Error",ros::this_node::getName().c_str());
	    return false;
    }
  
}


moveit_msgs::PickupGoal build_pick_goal(const std::string &objectName) {
    ROS_INFO("[%s]: gonna pick object!",ros::this_node::getName().c_str());
    moveit_msgs::PickupGoal goal;
    goal.target_name = objectName;
    goal.group_name = "arm";
    goal.end_effector = "eef";
    goal.allowed_planning_time = 40.0;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.replan=true;
    goal.planning_options.replan_attempts=50;
    goal.planner_id = "RRTConnectkConfigDefault";

    goal.minimize_object_distance = true;
    moveit_msgs::Grasp g;
    g.max_contact_force = 0.01;
    g.grasp_pose.header.frame_id = goal.target_name;
    g.grasp_pose.pose.position.x = -0.04;
    g.grasp_pose.pose.position.y = 0.0;
    g.grasp_pose.pose.position.z = 0.0;
    g.grasp_pose.pose.orientation.x = 0.0;
    g.grasp_pose.pose.orientation.y = 0.0;
    g.grasp_pose.pose.orientation.z = 0.0;
    g.grasp_pose.pose.orientation.w = 1.0;

    g.pre_grasp_approach.direction.header.frame_id = "/base_footprint"; //gripper_link
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.1;
    g.pre_grasp_approach.desired_distance = 0.2;

    g.post_grasp_retreat.direction.header.frame_id = "/base_footprint"; //gripper_link
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size());
    g.pre_grasp_posture.points[0].positions[0] = 0.14;

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].positions[0] = 0.01;
    g.grasp_posture.points[0].effort.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].effort[0] = 0.4;
    goal.possible_grasps.push_back(g);
    return goal;
}

void look_down() {

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=0.0;
    q_goal[1]=0.6;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    controllerCommandPub.publish(traj);
}

bool set_collision_update(bool state){
    std_srvs::SetBool srv;
    srv.request.data=state;
    if (ucClientPtr->call(srv))
    {
        ROS_INFO("[%s]: update_colision response: %s",ros::this_node::getName().c_str(), srv.response.message.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service /find_objects_node/update_colision");
        return false;
    }

}


bool place_cb_gripper(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("[%s]: Waiting for the /gripper_controller/gripper_cmd action server to come up",ros::this_node::getName().c_str());
    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
        
    //wait for the gripper action server to come up
    while (!gripperClient.waitForServer(ros::Duration(5.0))){
       ROS_INFO("[%s]: Waiting for the /gripper_controller/gripper_cmd action server to come up",ros::this_node::getName().c_str());
    }

    ROS_INFO("[%s]: Opening gripper..",ros::this_node::getName().c_str());
    gripper_cmd(&gripperClient,0.14,0);
    sleep(5);


    ROS_INFO("[%s]: returning to original position:",ros::this_node::getName().c_str());
    return_to_position(endPositionName);
    
    res.success = true;
    res.message = "successfully placed!";
}


bool pick_and_place_sync_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    return true;
}

bool pick_and_place_sync1_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    return true;
}

bool pick_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    std::string objectName;

    pn.param<std::string>("objectName", objectName, "can");

    ros::ServiceClient ucClient = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = false;

    if(ucClient.call(disableColl)) {
        ROS_INFO("[%s]: update_colision response: OFF ",ros::this_node::getName().c_str());
    }

    PickClient pickClient("pickup", true);
    pickClient.waitForServer();

    bool res_stat=false;
    moveit_msgs::PickupGoal pickGoal = build_pick_goal(objectName);
    ROS_INFO("[%s]: sending pick goal",ros::this_node::getName().c_str());
    actionlib::SimpleClientGoalState pickStatus = pickClient.sendGoalAndWait(pickGoal);
    if(pickStatus != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[%s]: Failed to pick up object",ros::this_node::getName().c_str());
        res.success = (unsigned char) false;
        res.message = pickStatus.getText();
        res_stat=false;
        
    }
    else {
        ROS_INFO("[%s]: Successfully picked objejct",ros::this_node::getName().c_str());
        res.success= (unsigned char) true;
        res.message = pickStatus.getText();
        res_stat=true;
    }

    //enable collision updates
    std_srvs::SetBool enableColl;
    enableColl.request.data = true;
    if(ucClient.call(enableColl)) {
        ROS_INFO("[%s]: update_colision response: ON ",ros::this_node::getName().c_str());
    }


    return true;


}
