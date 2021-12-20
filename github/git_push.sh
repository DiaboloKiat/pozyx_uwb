#!/bin/bash

git config --global user.name "DiaboloKiat"
git config --global user.email "DiaboloKiat@gmail.com"

git status
git checkout master
echo "Enter your message"
read message

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

BRANCH=master
echo "---------------------------------------------------------------------------------------------------"
source ~/$PROJECT/github/git_branch.sh $1
echo "---------------------------------------------------------------------------------------------------"
source ~/$PROJECT/github/git_pull.sh $1

PULLSTAT=$?
if [ "$PULLSTAT" -gt 0 ] ; then
   echo "There is conflict. Aborting"
   cd $cur_path/
   return
fi
echo "---------------------------------------------------------------------------------------------------"
echo "-------------------------------------------pull success--------------------------------------------"
echo "---------------------------------------------------------------------------------------------------"


# push master

echo "---------------------------------------------------------------------------------------------------"
echo "------------------------------------------push pozyx_uwb-------------------------------------------"
echo "---------------------------------------------------------------------------------------------------"
cd ~/$PROJECT
git add -A
git commit -m "${message} on pozyx_uwb"
git push


cd ~/$REPO