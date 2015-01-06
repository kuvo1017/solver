#!/bin/sh

for n in "res1" "res2" "res3"  
do
  rm   _stat* _accident* _error* 
  for i in "0.005" "0.01" "0.03"  "0.05" "0.075" "0.1" "0.3" "0.5"
  do
    ./advmates-calc -d $i
  done
  cd $n
  rm *
  cp ../_stat* ../_accident* ../_error* ./
done
