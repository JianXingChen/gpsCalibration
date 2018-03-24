#!/bin/bash

<<!
    User Parameters:
    
    Input filenames:
    1. bag_input_filename:
       input point cloud bag file

    Output filenames:
    1. feature_output_filename:
       featute data in cells

!
bag_input_filename="./data/gnss_lidar_points.bag"
feature_output_filename="./data/featureMap.txt"

#command
roslaunch featureMap featureMap.launch bag_input_filename:=${bag_input_filename} feature_output_filename:=${feature_output_filename}
