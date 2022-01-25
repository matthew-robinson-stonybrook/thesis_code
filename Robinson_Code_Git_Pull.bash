#!/bin/bash
git_token = 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

echo "getting status of git"

git status

echo " "
read -p "Would you like to pull from git? (y/n) " x
echo " "

if [ $x == "y" ] 
then
   echo "! ! ! ! ! GIT USERNAME AND TOKEN FOR AUTHENTICATION ! ! ! ! !"
   echo "! ! ! ! ! GIT TOKEN EXPIRED 1/22/2022 ! ! ! ! !"
   echo "Git username: matthew-robinson-stonybrook"
   echo "Git Token (for password): ${git_token}"
   echo " "
   git pull 
else
   echo "Fine be that way."
fi
