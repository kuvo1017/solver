#!/bin/sh

for n in "res1" 
do
  rm   _stat* _accident* _error* 
  for i in "1" 
  do
    ./advmates-calc -d $i -L
    mv vehicleTrip.txt ./vehicleTrip_$i".txt"
  done
  cd $n
  rm *
  cp ../_stat* ../_accident* ../_error* ../vehicleTrip* ./
  cd ../
done
