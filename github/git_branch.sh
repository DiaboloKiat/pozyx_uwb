#!/bin/bash

if [ "$1" = "base" ]
then
    PROJECT=pozyx_uwb
    REPO=pozyx_uwb
elif [ "$1" = "project_seadrone" ]
then
    PROJECT=project_seadrone/catkin_ws/src/pozyx_uwb
    REPO=project_seadrone
elif [ "$1" = "master_thesis" ]
then
    PROJECT=master_thesis/catkin_ws/src/pozyx_uwb
    REPO=master_thesis
else
    echo "Please enter your project"
    return 0
fi

cd ~/$PROJECT
git checkout master

############################## submodules ####################################


cd ~/$REPO