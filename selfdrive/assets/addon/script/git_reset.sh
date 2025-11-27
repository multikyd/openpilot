#!/usr/bin/bash

cd /data/openpilot
ping -q -c 1 -w 1 google.com &> /dev/null
if [ "$?" == "0" ]; then
  REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $1}')
  if [ "$REMOVED_BRANCH" != "" ]; then
    if [ "$REMOVED_BRANCH" == "*" ]; then
      REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $2}')
    fi
    git remote prune origin --dry-run
    echo $REMOVED_BRANCH | xargs git branch -D
    sed -i "/$REMOVED_BRANCH/d" .git/config
  fi
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  git clean -d -f -f
  git fetch --all --prune
  git checkout -B $CURRENT_BRANCH origin/$CURRENT_BRANCH
  git branch --set-upstream-to=origin/$CURRENT_BRANCH $CURRENT_BRANCH

  rm -f /data/params/d/DrivingModel
  rm -f /data/openpilot/selfdrive/modeld/models/driving_*
  git -C /data/openpilot/selfdrive/modeld/models checkout driving_policy.onnx
  git -C /data/openpilot/selfdrive/modeld/models checkout driving_vision.onnx
  touch /data/ks
  sleep 1

  sudo reboot
fi