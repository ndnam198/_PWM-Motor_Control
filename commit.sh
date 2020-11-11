#!/bin/sh
git status
timestamp=$(date +"%D %T")
message=$1
git add -A;
git commit -m "[$timestamp]: $message"
git push https://github.com/ndnam198/_PWM-Motor_Control.git master