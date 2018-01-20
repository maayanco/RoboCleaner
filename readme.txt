Click to view demo:
[![Link to demo](https://img.youtube.com/vi/HzSgDpK-Uvk/0.jpg)](https://www.youtube.com/watch?v=HzSgDpK-Uvk)

The armadillo robot locates the can on the table and picks it up.
It then begins to rotate in place until locating the trash.
When the trash is located it rotates until the can is in the range of the trash (and adjustes position otherwise), finally dropping the can.


To run the package:
roslaunch robocleaner robocleaner_united.launch

------------------------------------------------------------------------------------------------------------------------------------
seperate launch files for testing:

roslaunch robocleaner pick_up_object.launch

roslaunch robocleaner find_objects.launch

roslaunch robocleaner drive_to_basket.launch





