#!/bin/bash


echo "startup!!!!!"


source ~/CCF/DJI/dji_ws/devel/setup.sh
#{
#gnome-terminal -t "DJI_SDK" -x bash -c "roslaunch dji_sdk sdk.launch;exec bash"
#}&
 
#sleep 3s
#{
#gnome-terminal -t "ZED" -x bash -c "roslaunch zed_wrapper zed.launch;exec bash"
#}&

#sleep 4s
#{
#gnome-terminal -t "Flight_Control" -x bash -c "rosrun zed_hovering_uav flight_control;exec bash"
#}&

gnome-terminal --window -e 'bash -c "roslaunch dji_sdk sdk.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source /home/qjny/CCF/loam_ws/devel/setup.sh && roslaunch loam_velodyne loam_velodyne.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; source /home/qjny/CCF/velodyne_ws/devel/setup.sh && roslaunch velodyne_pointcloud VLP16_points.launch calibration:=/home/qjny/CCF/velodyne_ws/VLP-16.yaml; exec bash"' \
