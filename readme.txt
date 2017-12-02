
roslaunch robotican_armadillo armadillo.launch lidar:=true kinect2:=true sr300:=true moveit:=true gmapping:=true move_base:=true world_name:="`rospack find robotican_common`/worlds/objects_on_table_box4.world" gazebo:=true

roslaunch robocleaner robocleaner_united.launch


------------------------------------------------------------------------------------------------------------------------------------
seperate launch files for testing:

roslaunch robocleaner pick_up_object.launch

roslaunch robocleaner find_objects.launch

roslaunch robocleaner drive_to_basket.launch


--------------------------------------------------------------------------------------------------------------------------------------

Useful requests for testing:
rosservice call /pick_go "{}"
rosservice call /place_go "{}"

--------------------------------------------------------------------------------------------------------------------------------------

setup env:
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash



