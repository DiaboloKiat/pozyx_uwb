#!/bin/bash

if [ "$1" = "base" ]
then
    PROJECT=pozyx_uwb
    REPO=pozyx_uwb
elif [ "$1" = "project_seadrone" ]
then
    PROJECT=project_seadrone/catkin_ws/src/pozyx_uwb
    REPO=project_seadrone
else
    echo "Please enter your project"
    return 0
fi

cd ~/$PROJECT
git checkout master

############################## submodules ####################################
cd ~/$PROJECT/sensors/vision_opencv
git checkout melodic

cd ~/$PROJECT/sensors/apriltags_ros
git checkout indigo-devel

cd ~/$PROJECT/sensors/realsense-ros
git checkout 2.2.15

cd ~/$REPO