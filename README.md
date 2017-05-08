# CarmenCoverterToRosBag

1. Purpose: create converter from carmen log file to bag file. Program realized by using ROS API.

2. Description of program: supported old and new laser carmen log formats, any correct carmen log file can be converted.

3. Test: in datasets there is a test carmen log file(mit-csail-3rd-floor-2005-12-17-run4.flaser_original.log). To test it, you need call cmake then make, or catkin_make in correct catkin workspace. Then call CarmenCoverterToRosBag with -i(input filename) and -o(output filename) keys in terminal.
