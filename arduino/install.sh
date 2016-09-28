#!/bin/bash

bold=$(tput bold)
red=${bold}$(tput setaf 1)
reset=$(tput sgr0)

libs=("SerialDXL" "SimpleEncoder" "PID" "HBridge" "Control" "Fin_de_Carrera" "pid_tue" "TimerThree")

if [ ! -d ~/Arduino/libraries/ ]; then
  echo "${red}Arduino libraries directory not found...${reset}"
else
  echo "${bold}Installing Arduino libraries...${reset}"
  current_dir=${PWD}
  for i in "${libs[@]}"
  do
    if [ ! -d ~/Arduino/libraries/${i} ]; then
      echo "Installing ${i}"
      ln -s ${current_dir}/${i} ~/Arduino/libraries/${i}
    else
      echo "${red}Deleting ${i}${reset}"
      rm -rf ~/Arduino/libraries/${i}
      echo "Installing ${i}"
      ln -s ${current_dir}/${i} ~/Arduino/libraries/${i}
    fi
  done
fi

