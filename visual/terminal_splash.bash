#!/bin/bash


function echo_centering_str {
  the_style="$2"
  the_str=$1
  the_pad_cha=$3
  local str_len=${#the_str}
  local terminal_width=$(tput cols)
  local total_padding_len=$(( $terminal_width - $str_len ))
  local single_side_padding_len=$(( $total_padding_len / 2 ))
  pad=$(printf "$the_pad_cha%.0s" $(seq $single_side_padding_len))
  printf "${pad}${the_style}${the_str}\033[0m${pad}\n"
}

#
# ASCII art from image generator: https://asciiart.club
#

#function dockerized_snow_slpash {
#  echo " "
#  echo " "
#  echo_centering_str "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣤⣤⣤⣤⣤⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⠀⠀⠀⠀⣠⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡆⠀⣶⣦⠀⢤⣾⠟⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⠀⠀⣴⣿⣿⣿⠿⠛⠛⠛⠛⠛⠿⣿⣿⣿⣿⣿⣿⣿⡀⠈⣿⣧⠀⠋⠀⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⢀⣿⡿⠉⠀⠀⠀⠀⢀⣀⣤⣤⣀⠀⠀⠀⠉⠙⠛⠻⠿⠀⠈⠉⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⣿⠋⠀⠀⠀⠀⣴⣿⣿⣿⠟⠛⠉⠛⠿⠀⠀⠀⠀⠀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⣼⡏⠀⠀⠀⠀⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⣠⣶⣿⣿⣿⣿⣿⣿⣿⣿⣷⡀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⣿⡇⠀⠀⠀⠸⣿⣿⣿⣿⣷⣄⠀⠀⣀⣶⣿⣿⣿⣿⣿⣿⠋⠀⠀⠀⠀⠙⣿⡀⠀" "\033[1;2m" " "
##  echo_centering_str "·· NorLab dockerized-norlab ••" "\033[1;37m" "\033[0m·"
##  echo_centering_str "·· NorLab Dockerized-SNOW ••" "\033[1;37m" "\033[0m·"
#  echo_centering_str "··•· Dockerized-norlab ··•••" "\033[1;37m" "\033[0m·"
##  echo_centering_str "⠀⢾⣿⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠋⠀⠀⠀⠀⠀⠀⠀⣿⣷⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⣿⣷⡀⠀⠀⠀⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠉⠀⠀⠀⠀⠀⠀⠀⠀⣾⣿⡇⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⠘⣿⣿⣷⣤⣀⠀⠀⠀⠀⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣶⣿⣿⡿⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⠀⠘⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣶⣶⣶⣶⣶⣶⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⠀⠀⠀⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⠀⠀⠀⠀⠀⠀⠙⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠛⠉⠀⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
#  echo " "
#  echo_centering_str "https://norlab.ulaval.ca" "\033[2;37m" " "
#  echo_centering_str "https://redleader962.github.io" "\033[2;37m" " "
#  echo " "
#  echo " "
#}
#dockerized_snow_slpash


function dockerized_snow_slpash {
  echo " "
  echo " "
  echo_centering_str "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣶⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣷⣼⣿⣤⣿⡗⠀⠀⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⢀⣤⡀⣿⣿⠀⠀⠉⣿⣿⡿⠁⠀⠀⣿⡟⣀⣤⠀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⠀⠙⣻⣿⣿⣧⠀⠀⢸⣿⠀⠀⢀⣿⣿⣿⣟⠉⠀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⠘⠛⠛⠉⠉⠙⠿⣿⣾⣿⣷⣿⠟⠉⠉⠙⠛⠛⠀⠀⠀" "\033[1;2m" " "
#  echo_centering_str "·· NorLab dockerized-norlab ••" "\033[1;37m" "\033[0m·"
  echo_centering_str "·····•· Dockerized-norlab ··•••" "\033[1;37m" "\033[0m·"
  echo_centering_str "⠀⠀⠀⢠⣶⣤⣄⣀⣤⣶⣿⢿⣿⢿⣿⣶⣄⣀⣤⣤⣶⠀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⠀⣨⣿⣿⣿⡟⠁⠀⢸⣿⠀⠀⠉⣿⣿⣿⣯⣀⠀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⠈⠛⠁⣿⣿⢀⠀⣠⣿⣿⣷⡀⠀⠈⣿⣧⠉⠛⢀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⡿⢻⣿⠙⣿⡷⠀⠈⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
  echo_centering_str "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" "\033[1;2m" " "
  echo " "
  echo_centering_str "https://norlab.ulaval.ca" "\033[2;37m" " "
#  echo_centering_str "https://redleader962.github.io" "\033[2;37m" " "
  echo_centering_str "https://github.com/norlab-ulaval/dockerized-norlab" "\033[2;37m" " "
  echo " "
  echo " "
}
dockerized_snow_slpash
