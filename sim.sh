#!/bin/sh

for n in "res1" "res2" "res3"  
do
  rm   _stat* _accident* _error* 
  for i in "200" "400" "600"  "800" "1000" "1200" "1400" "1600"
  do
    ./advmates-calc -d $i
  done
  cd $n
  rm *
  cp ../_stat* ../_accident* ../_error* ./
  cd ../
done
