#!/bin/bash
git_tokenp1=ghp_MqGEjfnBkWZ7Xh
git_tokenp2=obwGcIjLSHdnw03p41BQKM

echo "getting status of git"

git status

echo " "
read -p "Would you like to add all files to git? (y/n) " add
echo " "

if [ $add == "y" ] 
then
   git add .
   echo " "
   echo "Getting updated status of git"
   echo " "
   git status
   
   echo " "
   read -p "Would you like to commit and push all added files to git? (y/n) " commit
   if [ $commit == "y" ] 
   then
      read -p "Commit Message: " msg
      git commit -m "$msg"
      echo " "
      echo "Getting updated status of git"
      echo " "
      git status
      echo " "
      echo "! ! ! ! ! GIT USERNAME AND TOKEN FOR AUTHENTICATION ! ! ! ! !"
      echo "! ! ! ! ! GIT TOKEN EXPIRED 5/27/2022 ! ! ! ! !"
      echo "Git username: matthew-robinson-stonybrook"
      echo "Git Token Part1 (for password): ${git_tokenp1}"
      echo "Git Token Part2 (for password): ${git_tokenp2}"
      echo " "
      
      git push
   else
      echo "Fine be that way."
   fi
fi
