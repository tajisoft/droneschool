#!/usr/bin/bash

param=''

for i in `seq 1 $1`
do
  echo $i
  port=`expr 14540 + $i \* 10`
  echo $port
  param+=' --master 127.0.0.1:'
  param+=$port
done

echo $param
mavproxy.py $param --aircraft day5 --console --map
