#!/bin/bash

bold=$(tput bold)
red=${bold}$(tput setaf 1)
reset=$(tput sgr0)

libs=("SerialDXL" "Encoder" "PID" "HBridge")

if [ ! -d ~/Arduino/libraries/ ]; then
  echo "${red}Arduino libraries directory not found...${reset}"
else
  echo "${bold}Installing Arduino libraries...${reset}"
  current_dir=${PWD}
  for i in "${libs[@]}"
  do
    echo "Installing ${i}"
    ln -s ${current_dir}/${i} ~/Arduino/libraries/${i}
  done
fi

