#include <std_srvs/SetBool.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <sstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <vector>
#include "tf/tf.h"

ros::ServiceClient *pickClientPtr;
ros::ServiceClient *placeClientPtr;

ros::Publisher twistPub;
ros::Publisher controllerCommandPub;
ros::Subscriber targetRangeSub;
ros::Subscriber laserSub;
ros::Publisher shutDownPub;

//Forward declerations
void send_twist_msg(double linearX, double linearY, double linearZ, double angularX, double angularY, double angularZ);
void target_range_check(const geometry_msgs::Point::ConstPtr &p);
void check_emergency_stop(const sensor_msgs::LaserScan::ConstPtr &p);
void look_down();

bool shouldRotate = true;

int inputLength=540; 
int inputWidth=960;

std::string pickable_object_name;
float minimum_distance_from_obstacle;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "drive_handler_node");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.param<std::string>("pickable_object_name", pickable_object_name, "can");
    pn.param<float>("minimum_distance_from_obstacle", minimum_distance_from_obstacle, 0.1);
   
    controllerCommandPub = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);

    ros::ServiceClient pickClient = n.serviceClient<std_srvs::Trigger>("pick_go");
    ROS_INFO("[%s]: waiting for pick service to be up",ros::this_node::getName().c_str());
    pickClient.waitForExistence();
    pickClientPtr=&pickClient;

    ros::ServiceClient placeClient = n.serviceClient<std_srvs::Trigger>("place_go_gripper");
    ROS_INFO("[%s]: waiting for place service to be up",ros::this_node::getName().c_str());
    placeClient.waitForExistence();
    placeClientPtr=&placeClient;

    ROS_INFO("[%s]: waiting for ObjectsHandler node to come up...",ros::this_node::getName().c_str());
    ros::ServiceClient objHandlerClient = n.serviceClient<std_srvs::Trigger>("objects_handler_sync");
    objHandlerClient.waitForExistence();

    ROS_INFO("[%s]: waiting for ObjectsDetector node to come up...",ros::this_node::getName().c_str());
    ros::ServiceClient objDetectorClient = n.serviceClient<std_srvs::Trigger>("objects_detector_sync");
    objDetectorClient.waitForExistence();

    ROS_INFO("[%s]: waiting for pickAndPlaceSync node to come up...",ros::this_node::getName().c_str());
    ros::ServiceClient pickAndPlaceSync = n.serviceClient<std_srvs::Trigger>("pick_and_place_sync");
    pickAndPlaceSync.waitForExistence();
    
    targetRangeSub = n.subscribe("target_range_pose", 10, target_range_check);
    laserSub=n.subscribe("scan",1,check_emergency_stop);
    
    shutDownPub = n.advertise<std_msgs::String>("shut_down_go", 10);

    //initialize twist client
    twistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    //ROS_INFO("[%s]: DriveHandler is ready to receive orders!",ros::this_node::getName().c_str());
 
    ROS_INFO("[%s]: looking down..",ros::this_node::getName().c_str());

    look_down();
    ros::Duration(5.0).sleep();
    ROS_INFO("DriveHandler ready to begin!");

    //checking for pickable objs 
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> ids;
    ids.push_back(pickable_object_name);
    std::map<std::string, moveit_msgs::CollisionObject> collisionObjsMap = planning_scene_interface.getObjects(ids);
    std::map<std::string, moveit_msgs::CollisionObject>::iterator pickableObjsIterator;
    pickableObjsIterator = collisionObjsMap.find(pickable_object_name);

    

    ros::Rate rate(10.0);
    while(ros::ok()){

        while(pickableObjsIterator == collisionObjsMap.end()){
            //while a pickable obj is not found
            ros::Rate r(10.0);
            send_twist_msg(0,0,0,0,0,0.1); //rotate in place until a pickable obj is detected
            r.sleep();

            //check again for pickable objects
            collisionObjsMap = planning_scene_interface.getObjects(ids);
            pickableObjsIterator = collisionObjsMap.find(pickable_object_name);
        }

        //a pickable object was found
        std_srvs::Trigger pickSrv;
        if(pickClientPtr->call(pickSrv))
        {
            ROS_INFO("[%s]: Pick client response: %s",ros::this_node::getName().c_str(), pickSrv.response.message.c_str());
            if(pickSrv.response.success){
                ROS_INFO("[%s]: After succsessful pick - start rotating",ros::this_node::getName().c_str());
                while(shouldRotate==true){
                        ros::Rate r(1.0);
                        send_twist_msg(0,0,0,0,0,0.1); //rotate in place
                        ros::spinOnce(); //calls all the callbacks 
                        r.sleep();
                }

                //when pickableObjsIterator finishes twisting then we should place...
                std_srvs::Trigger place_srv;
                if (placeClientPtr->call(place_srv))
                {
                    if(place_srv.response.success){
                        ROS_INFO("[%s]: Placed object successfully!",ros::this_node::getName().c_str());
                        ROS_INFO("[%s]: Publish shut down message to other nodes..",ros::this_node::getName().c_str());
                        std_msgs::String shutDownMsg;
                        shutDownMsg.data = "The program has finished! shutting down..";
                        shutDownPub.publish(shutDownMsg);
                        ROS_INFO("[%s]: Shut down..",ros::this_node::getName().c_str());
                        ros::shutdown();
                    }
                    else{
                        ROS_INFO("[%s]: Failed to place object",ros::this_node::getName().c_str());
                    }
                }
                else{
                    ROS_INFO("[%s]: Failed to call place service",ros::this_node::getName().c_str());
                }
                
            }
            else{
                ROS_INFO("[%s]: Failed to pick object",ros::this_node::getName().c_str());
            }
        }
        else{
            ROS_INFO("[%s]: Failed to call pick service",ros::this_node::getName().c_str());
        }
        
    }

    ros::waitForShutdown();
    return 0;
}

void send_twist_msg(double linearX, double linearY, double linearZ, double angularX, double angularY, double angularZ){
    geometry_msgs::Twist tMsg;
    tMsg.linear.x=linearX;
    tMsg.linear.y=linearY;
    tMsg.linear.z=linearZ;
    tMsg.angular.x=angularX;
    tMsg.angular.y=angularY;
    tMsg.angular.z=angularZ;
    twistPub.publish(tMsg);

    ROS_INFO("[%s]: sending twist message..",ros::this_node::getName().c_str());
}

void target_range_check(const geometry_msgs::Point::ConstPtr &p){

    double xPar = p->x / inputWidth;
    double yPar = p->y / inputLength;

    ROS_INFO("[%s]: The relative target location x: %f, y: %f",ros::this_node::getName().c_str(),xPar, yPar);

    //Target is in range - stop rotating
    if(xPar>0.25 && xPar<0.45){
         if(yPar>0.3){
             send_twist_msg(0.3,0,0,0,0,0); //move forward!
         }
        shouldRotate=false;
    }
    
}

void check_emergency_stop(const sensor_msgs::LaserScan::ConstPtr &p){
    int vecInc = 50;
    std::vector<float> ranges = p->ranges;
    int vecLength = ranges.size();
    int midIndex = vecLength/2;
    int startIndex =midIndex-vecInc;
    int endIndex =midIndex+vecInc;
    
    for(int i=startIndex; i<endIndex; i++){
        if(ranges[i]<minimum_distance_from_obstacle){
            std_msgs::String shutDownMsg;
            shutDownMsg.data = "Emergency shutdown due to obstacle";
            shutDownPub.publish(shutDownMsg);
            ROS_INFO("Obstacle detected - emergency stop initiated ");
            ros::shutdown();
            break;
        }
    }

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
