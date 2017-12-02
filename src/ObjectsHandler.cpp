#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <sstream>
#include <std_srvs/Trigger.h>
#include "tf/tf.h"


bool update_cb(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res);
bool obj_handler_sync_cb(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res);
bool switch_objs_cb(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res);
void handle_objects(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers);
std::string get_obj_id(int id);
shape_msgs::SolidPrimitive get_obj_shape(int id);
void shut_down_cb(const std_msgs::String::ConstPtr& msg);

ros::Subscriber shutDownSub;

bool update=false;
bool isSwitched=false;

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr;

int main(int argc, char **argv) {
    ros::init(argc, argv, "objects_handler_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("[%s]: waiting for pickAndPlaceSync1 node to come up...",ros::this_node::getName().c_str());
    ros::ServiceClient pickAndPlaceSync1 = n.serviceClient<std_srvs::Trigger>("pick_and_place_sync1");
    pickAndPlaceSync1.waitForExistence();

    ros::Subscriber arSub = n.subscribe("ar_pose_marker", 10, handle_objects);
    ros::Subscriber colorSub = n.subscribe("detected_objects", 10, handle_objects);
    shutDownSub = n.subscribe("shut_down_go", 10, shut_down_cb);

    ros::ServiceServer ucService = n.advertiseService("update_collision_objects", update_cb);
    ros::ServiceServer switchService = n.advertiseService("update_switch_objects", switch_objs_cb);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface_ptr=&planning_scene_interface;

    

    ros::ServiceServer objectsHandlerSyncPub = n.advertiseService("objects_handler_sync", obj_handler_sync_cb);

    ROS_INFO("objects handler is ready!");

    while (ros::ok()){

        ros::spinOnce();
    }

    return 0;
}

void shut_down_cb(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), msg->data.c_str());
    ros::shutdown();
}

std::string get_obj_id(int id) { 

    if (id==1){
      ROS_INFO("[%s]: returned can",ros::this_node::getName().c_str());
      return "can";
    }
    else if(id==2){
        ROS_INFO("[%s]: returned table",ros::this_node::getName().c_str());
        return "table";
    }
    else if(id==4){
        ROS_INFO("[%s]: returned target table",ros::this_node::getName().c_str());
        return "target_table";
    }
    else{
        ROS_INFO("[%s]: returned uneidentified object",ros::this_node::getName().c_str());
        return "unidentified_object";
    }
 }


shape_msgs::SolidPrimitive get_obj_shape(int id) {

    shape_msgs::SolidPrimitive object_primitive;
    if (id==1) {
        object_primitive.type = object_primitive.CYLINDER;
        object_primitive.dimensions.resize(2);
        object_primitive.dimensions[0] = 0.17;
        object_primitive.dimensions[1] = 0.03;
    }
    else {
        object_primitive.type = object_primitive.BOX;
        object_primitive.dimensions.resize(3);
        object_primitive.dimensions[0] = 0.3;
        object_primitive.dimensions[1] = 0.3;
        object_primitive.dimensions[2] = 0.01;
    }

    return object_primitive;
}


void handle_objects(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers)
{
	//build collisions for moveit
    std::vector<moveit_msgs::CollisionObject> colObjs;

    for (int i=0;i<markers->markers.size();i++) {

        ar_track_alvar_msgs::AlvarMarker m=markers->markers[i];

        moveit_msgs::CollisionObject obj;
        obj.header.frame_id = m.header.frame_id;
        obj.id = get_obj_id(m.id);
        obj.primitives.push_back(get_obj_shape(m.id));

        obj.primitive_poses.push_back(m.pose.pose);
        obj.operation=obj.ADD;

        colObjs.push_back(obj);

        if (update){
            planning_scene_interface_ptr->addCollisionObjects(colObjs);
        } 

    }
}

bool obj_handler_sync_cb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res) {
                   return true;
}

bool update_cb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res) {

    update=req.data;
    res.success=true;
    if (update)  res.message="update collision is ON";
    else res.message="update collision is OFF";
    return true;
}


bool switch_objs_cb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res) {
    isSwitched=true;
    update=isSwitched;
    res.success=true;
}
