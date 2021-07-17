#!/usr/bin/bash


for i in `seq 1 $1`
do
  echo kill mav${i}
  screen -S mav${i} -X quit
done
