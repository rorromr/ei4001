#!/bin/sh

bold=$(tput bold)
red=${bold}$(tput setaf 1)
reset=$(tput sgr0)

if [ ! -d ~/Arduino/libraries/ ]; then
  echo "${red}Arduino libraries directory not found...${reset}"
else
  echo "${bold}Installing Arduino libraries...${reset}"
  current_dir=${PWD}
  ln -s "${current_dir}/SerialDXL" ~/Arduino/libraries/SerialDXL
fi


