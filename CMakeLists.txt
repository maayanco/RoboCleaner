cmake_minimum_required(VERSION 2.8.3)
project(robocleaner)


find_package(catkin REQUIRED COMPONENTS
        roscpp
	rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        image_transport
        cv_bridge
        pcl_conversions
        pcl_ros
        tf
        dynamic_reconfigure
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        kdl_parser
        actionlib
        pr2_controllers_msgs
        tf_conversions
        trajectory_msgs
        gazebo_ros
        ar_track_alvar
        message_generation
        )
find_package(gazebo REQUIRED)



 #add_message_files(
 #  FILES
 #  
 #)

  add_service_files(
    FILES
    switch_topic.srv
  )


 generate_messages(
   DEPENDENCIES
   std_msgs 
 )

generate_dynamic_reconfigure_options(
        cfg/FindObjectDynParam.cfg
        #   cfg/DynReconf2.cfg
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs dynamic_reconfigure
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        gazebo_ros
        DEPENDS gazebo
)



include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
)



add_executable(PickPlaceManager src/PickPlaceManager.cpp)
target_link_libraries(PickPlaceManager ${catkin_LIBRARIES})
add_dependencies(PickPlaceManager robocleaner_generate_messages_cpp)

add_executable(DriveHandler src/DriveHandler.cpp)
target_link_libraries(DriveHandler ${catkin_LIBRARIES})
add_dependencies(DriveHandler robocleaner_generate_messages_cpp)

add_executable(ObjectsDetector src/ObjectsDetector.cpp)
target_link_libraries(ObjectsDetector ${catkin_LIBRARIES})
add_dependencies(ObjectsDetector robocleaner_generate_messages_cpp)

add_executable(ObjectsHandler src/ObjectsHandler.cpp)
target_link_libraries(ObjectsHandler ${catkin_LIBRARIES})
add_dependencies(ObjectsHandler robocleaner_generate_messages_cpp)

