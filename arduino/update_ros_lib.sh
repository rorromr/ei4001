#!/bin/sh

bold=$(tput bold)
red=${bold}$(tput setaf 1)
reset=$(tput sgr0)

if [ ! -d ~/Arduino/libraries/ ]; then
  echo "${red}Arduino libraries directory ~/Arduino/libraries/ not found...${reset}"
else
  echo "${bold}Deleting Arduino ROS libraries...${reset}"
  rm -rf ~/Arduino/libraries
  rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/
fi
