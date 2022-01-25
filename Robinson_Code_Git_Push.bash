#!/bin/bash
git_token=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

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
      echo "! ! ! ! ! GIT TOKEN EXPIRED 1/22/2022 ! ! ! ! !"
      echo "Git username: matthew-robinson-stonybrook"
      echo "Git Token (for password): ${git_token}"
      echo " "
      
      git push
   else
      echo "Fine be that way."
   fi
fi
