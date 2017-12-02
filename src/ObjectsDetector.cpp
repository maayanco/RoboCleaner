#include <std_msgs/String.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <robotican_common/FindObjectDynParamConfig.h>
#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <robotican_common/switch_topic.h>
#include <std_srvs/Trigger.h>

using namespace cv;

ros::Subscriber shutDownSub;

bool find_object(Mat inputPcl, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
bool find_target(Mat inputPcl, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputPcl);
void shut_down_cb(const std_msgs::String::ConstPtr& msg);
void target_msg_cb(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);
void obj_msg_cb(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);
bool obj_detector_sync_cb(robotican_common::switch_topic::Request &req, robotican_common::switch_topic::Response &res);

tf::TransformListener *listenerPtr;
int object_id;


std::string depth_topic;
bool have_object=false;
bool found_target_table=false;
bool picked_up_obj=false;

ros::Publisher objPub;
image_transport::Publisher resultImgPub;
image_transport::Publisher targetPub;
ros::Publisher objPosePub;
ros::Publisher targetPosePub;
ros::Publisher targetRelativeRangePosePub;

//red
int minH=3,maxH=160;
int minS=70,maxS=255;

int minV=10,maxV=255;
int minA=200,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

int inv_H=1;


int main(int argc, char **argv) {

    ros::init(argc, argv, "objects_detector_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
	
    pn.param<int>("object_id", object_id, 1);
    pn.param<std::string>("depth_topic1", depth_topic, "/kinect2/qhd/points");

    

    image_transport::ImageTransport imgTransport(pn);

    //initialize publishers of images to topics
    resultImgPub = imgTransport.advertise("result", 1);
    targetPub = imgTransport.advertise("result_table",1);

    ROS_INFO("[%s]: waiting for pickAndPlaceSync1 node to come up...",ros::this_node::getName().c_str());
    ros::ServiceClient pickAndPlaceSync1 = n.serviceClient<std_srvs::Trigger>("pick_and_place_sync1");
    pickAndPlaceSync1.waitForExistence();
    
    //subscribe to messages from "/kinect2/qhd/points"
    ros::Subscriber pblSub = n.subscribe(depth_topic, 1, cloud_cb);
    shutDownSub = n.subscribe("shut_down_go", 10, shut_down_cb);
    ROS_INFO_STREAM(depth_topic);
    //set objPub as publisher of ar_track_alvar_msgs to the "detected_objects" topic
    objPub=n.advertise<ar_track_alvar_msgs::AlvarMarkers>("detected_objects", 2, true);
    //set objPosePub as publisher of Pose msgs to "object_pose" topic
    objPosePub=pn.advertise<geometry_msgs::PoseStamped>("object_pose",10);
    targetPosePub = pn.advertise<geometry_msgs::PoseStamped>("target_table_pose",10);
    //publishes the pose of the target for in-range check
    targetRelativeRangePosePub = n.advertise<geometry_msgs::Point>("target_range_pose", 1000);
	//convert depth cam tf to base foot print tf (moveit work with base footprint tf)
    //create a tf listener (receives transformation msgs)
    tf::TransformListener listener;
    listenerPtr=&listener;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pointSub;
    pointSub.subscribe(pn, "object_pose", 10);
    //messageFilter accumulates the tf messages from the "object_pose" until it can transform them to base_footprint
    tf::MessageFilter<geometry_msgs::PoseStamped> tfObjFilter(pointSub, listener, "base_footprint", 10);
    //when a msg passes the messageFilter the callback that will be invoked is obj_msg_cb
    tfObjFilter.registerCallback( boost::bind(obj_msg_cb, _1) );
    //transforming msgs from "object_pose" topic to "base_footprint" frame.
    //tfObjFilter waits until such transformation is successful and then the obj_msg_cb is called
    //now the same for the target table!
    message_filters::Subscriber<geometry_msgs::PoseStamped> targetSub;
    targetSub.subscribe(pn, "target_table_pose", 10);
    //messageFilter accumulates the tf messages from the "object_pose" until it can transform them to base_footprint
    tf::MessageFilter<geometry_msgs::PoseStamped> tfFilterTarget(targetSub, listener, "base_footprint", 10);
    //when a msg passes the messageFilter the callback that will be invoked is obj_msg_cb
    tfFilterTarget.registerCallback( boost::bind(target_msg_cb, _1) );
    //transforming msgs from "object_pose" topic to "base_footprint" frame.
    //tfFilterTarget waits until such transformation is successful and then the obj_msg_cb is called!
    //"objects_detector_sync" topic is advertised, when it receives a msg the "obj_detector_sync_cb" cb is called
    ros::ServiceServer objectsDetectorSyncPub = n.advertiseService("objects_detector_sync", &obj_detector_sync_cb);

    ros::Rate r(10);
    ROS_INFO("[%s]: Ready to find objects!",ros::this_node::getName().c_str());
    while (ros::ok()) {
        ros::spinOnce(); //calls all the callbacks waiting to be called at this point
        r.sleep();
    }

    ros::waitForShutdown();

    return 0;
}


void shut_down_cb(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), msg->data.c_str());
    ros::shutdown();
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputPcl) {

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    //convert ros point cloud msg to pcl point cloud 
    pcl::fromROSMsg (*inputPcl, cloud); 
    //create projection image from p.c.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (cloudp->empty()) {
        ROS_WARN("empty cloud");
        return;
    }
    
	//creating new ros sensor msg - picture is relative to depth camera tf
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*inputPcl, *image_msg);
    image_msg->header.stamp = inputPcl->header.stamp;
    image_msg->header.frame_id = inputPcl->header.frame_id;
	
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    Mat result=cv_ptr->image;
    Mat result2;
    result.copyTo(result2);
    
    Point3d obj;
    Point3d obj2;

    //finding the object and the target 
    have_object= find_object(result,cloudp,&obj,inputPcl->header.frame_id);
    found_target_table = find_target(result2,cloudp,&obj2,inputPcl->header.frame_id);

    waitKey(1);

    if (have_object) {
	    //publish geometry msg containing object's pose
        geometry_msgs::PoseStamped objPose;
        objPose.header.frame_id=inputPcl->header.frame_id;
        objPose.header.stamp=ros::Time::now();
        objPose.pose.position.x =obj.x;
        objPose.pose.position.y = obj.y;
        objPose.pose.position.z = obj.z;
        objPose.pose.orientation.w=1;
        objPosePub.publish(objPose);
    }

    if (found_target_table) {
	    //publish geometry msg containing target's pose
        geometry_msgs::PoseStamped targetPose;
        targetPose.header.frame_id=inputPcl->header.frame_id;
        targetPose.header.stamp=ros::Time::now();
        targetPose.pose.position.x =obj2.x;
        targetPose.pose.position.y = obj2.y;
        targetPose.pose.position.z = obj2.z;
        targetPose.pose.orientation.w=1;
        targetPosePub.publish(targetPose);
    }

}



void obj_msg_cb(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
	//get object location msg
    try
    {
        geometry_msgs::PoseStamped base_object_pose;
        listenerPtr->transformPose("base_footprint", *point_ptr, base_object_pose);
        base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

		//simulate alvar msgs, to get tf
        ar_track_alvar_msgs::AlvarMarkers msg;
        msg.header.stamp=base_object_pose.header.stamp;
        msg.header.frame_id="base_footprint";

        ar_track_alvar_msgs::AlvarMarker m;
        //the obj
        m.id=object_id;
        m.header.stamp=base_object_pose.header.stamp;
        m.header.frame_id="base_footprint";
        m.pose=base_object_pose;
        msg.markers.push_back(m);

        //publish table (underneath the obj)
        m.pose.pose.position.z-=0.1;
        m.id=2;
        msg.markers.push_back(m);
        
        objPub.publish(msg);

    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}


void target_msg_cb(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
	//get object location msg
    try
    {
        geometry_msgs::PoseStamped base_object_pose;
        listenerPtr->transformPose("base_footprint", *point_ptr, base_object_pose);
        base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

		//simulate alvar msgs, to get tf
        ar_track_alvar_msgs::AlvarMarkers msg;
        msg.header.stamp=base_object_pose.header.stamp;
        msg.header.frame_id="base_footprint";

        ar_track_alvar_msgs::AlvarMarker m;
        //target (trash can)
        m.id=object_id;
        m.header.stamp=base_object_pose.header.stamp;
        m.header.frame_id="base_footprint";
        m.pose=base_object_pose;
        m.pose.pose.position.z-=0.1;
        m.id=4;
        msg.markers.push_back(m);

        objPub.publish(msg);

    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}


bool find_target(Mat inputPcl, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp,Point3d *pr,std::string frame) {

    Mat hsv,filtered,bw,mask;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=frame;
	//use hsv colores - no light sensitivity
    cvtColor(inputPcl,hsv,CV_BGR2HSV); //make hsv the hsv colored version of inputPcl..

    inRange(hsv,cv::Scalar(50, 80, 40), cv::Scalar(80, 255, 255),mask);

    //convert to bw image, gaussian - blur, morphologic actions
    mask.copyTo(bw); //make bw copy of mask

	//get image contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	//get largest contour
    double largest_area=0;
    int largest_contour_index=0;
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0>largest_area){
            largest_area=area0;
            largest_contour_index=i;
        }
    }

    bool status=false;
    if ((largest_area>minA)&&(largest_area<maxA)) {

        //draw contours and details about object
        drawContours(inputPcl, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);

        //finding the center of the mass of the object
        Moments mu=moments(contours[largest_contour_index], true ); 
        Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

        //calculating relative pose as to input image size
	    double xPar = mc.x / inputPcl.cols;
	    double yPar = mc.y / inputPcl.cols;

        //draw a circle
        circle( inputPcl, mc, 4, Scalar(0,0,255), -1, 8, 0 );
        int pcl_index = ((int)(mc.y)*inputPcl.cols) + (int)(mc.x);
        circle( inputPcl, mc, 8, Scalar(0,255,0), -1, 8, 0 );

        //set the coordinates of the object 
        pr->x=cloudp->points[pcl_index].x;
        pr->y=cloudp->points[pcl_index].y;
        pr->z=cloudp->points[pcl_index].z;
        char str[100];
        //if we were unable to calculate either x,y,z coordinates then we report "failure" - failed to locate object
        if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) {
            sprintf(str,"NaN");
            status=false;
        }
        else {
            //set str to be the string representing the coordinates of the middle of the object!
            sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
            status=true;
        }
        //put str on the MAT image
        putText( inputPcl, str, mc, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,255,255), 1, 8);

        //publish the relative pose 
        geometry_msgs::Point target_range_point;
        target_range_point.x = mc.x;
        target_range_point.y = mc.y;
        
        targetRelativeRangePosePub.publish(target_range_point);

    }

    //publish the inputPcl image with all the drawings
    out_msg.image    = inputPcl;
    out_msg.encoding = "bgr8";
    targetPub.publish(out_msg.toImageMsg());

    

    return status;
}


bool find_object(Mat inputPcl, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp,Point3d *pr,std::string frame) {

    Mat hsv,filtered,bw,mask;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=frame;
	//use hsv colores - no light sensitivity
    cvtColor(inputPcl,hsv,CV_BGR2HSV); //make hsv the hsv colored version of inputPcl..


    if (inv_H) {
		//edges of spectrom - red colore
        Mat lower_hue_range;
        Mat upper_hue_range;
        inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_hue_range);
        inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_hue_range);

        // Combine the above two images
        addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, mask);
    }
    else{
        //if not red use (middle of spectrom):
        inRange(hsv,Scalar(minH,minS,minV),Scalar(maxH,maxS,maxV),mask);
    }

    //mask is an image (MAT obj) that includes only the red pixels..


    //convert to bw image, gaussian - blur, morphologic actions
    mask.copyTo(bw); //make bw copy of mask


	//get image contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	//get largest contour
    double largest_area=0;
    int largest_contour_index=0;
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0>largest_area){
            largest_area=area0;
            largest_contour_index=i;
        }
    }
    
    bool status=false;
    if ((largest_area>minA)&&(largest_area<maxA)) {

        //draw contours and details about object
        drawContours(inputPcl, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);

        //finding the center of the mass of the object
        Moments mu=moments(contours[largest_contour_index], true ); 
        Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        //draws a circle
        circle( inputPcl, mc, 4, Scalar(0,0,255), -1, 8, 0 );
        int pcl_index = ((int)(mc.y)*inputPcl.cols) + (int)(mc.x);
        circle( inputPcl, mc, 8, Scalar(0,255,0), -1, 8, 0 );

        //the coordinates of the object 
        pr->x=cloudp->points[pcl_index].x;
        pr->y=cloudp->points[pcl_index].y;
        pr->z=cloudp->points[pcl_index].z;
        
        char str[100];
        //if we were unable to calculate either x,y,z coordinates then we report "failure" - failed to locate object
        if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) {
            sprintf(str,"NaN");
            status=false;
        }
        else {
            //set str to be the string representing the coordinates of the middle of the object!
            sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
            status=true;
        }
        //put str on the MAT image
        putText( inputPcl, str, mc, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,255,255), 1, 8);

    }

    //publish the inputPcl image with all the drawings  
    out_msg.image    = inputPcl;
    out_msg.encoding = "bgr8";
    resultImgPub.publish(out_msg.toImageMsg());

    return status;
}


bool obj_detector_sync_cb(robotican_common::switch_topic::Request &req, robotican_common::switch_topic::Response &res) {
    return true;
}

