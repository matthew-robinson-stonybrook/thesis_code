#!/bin/bash
<<<<<<< HEAD
git_tokenp1=ghp_MqGEjfnBkWZ7Xh
git_tokenp2=obwGcIjLSHdnw03p41BQKM
=======
git_token=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
>>>>>>> bf10960e7cbba9c8c68522a51e48143f5fb06f43

echo "getting status of git"

git status

echo " "
read -p "Would you like to pull from git? (y/n) " x
echo " "

if [ $x == "y" ] 
then
   echo "! ! ! ! ! GIT USERNAME AND TOKEN FOR AUTHENTICATION ! ! ! ! !"
   echo "! ! ! ! ! GIT TOKEN EXPIRED 5/27/2022 ! ! ! ! !"
   echo "Git username: matthew-robinson-stonybrook"
   echo "Git Token Part1 (for password): ${git_tokenp1}"
   echo "Git Token Part2 (for password): ${git_tokenp2}"
   echo " "
   git pull 
else
   echo "Fine be that way."
fi
