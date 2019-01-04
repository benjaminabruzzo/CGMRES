#!/bin/bash
  REMOVE=($(git status | grep deleted | awk '{ print $2}'))
    for i in "${REMOVE[@]}"
    do
      :
      git rm $i
    done

  git add --all .
  git commit -m "$1"
  git push gh ros-kinetic-ardrone
  git push bb ros-kinetic-ardrone


# This will detach your HEAD, that is, leave you with no branch checked out:
# git checkout -b old-state 862057f

# This will destroy any local modifications.
# Don't do it if you have uncommitted work you want to keep.
# git reset --hard 49f088c
