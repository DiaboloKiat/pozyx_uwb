#! /bin/bash

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

BRANCH=master
echo "---------------------------------------------------------------------------------------------------"
echo "---------------------------------------pull pozyx_uwb----------------------------------------------"
echo "---------------------------------------------------------------------------------------------------"
cd ~/$PROJECT
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in pozyx_uwb. Aborting"
   return 1
fi


cd ~/$REPO
return 0