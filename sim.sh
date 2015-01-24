#!/bin/sh

for n in "res1" "res2"  "res3"  
do
  rm   _stat* _accident* _error* 
  for i in "1e-4"  "5e-4" "5e-5" "1e-5" "5e-6"    
  do
    ./advmates-calc -d $i -L
    mv vehicleTrip.txt ./vehicleTrip_$i".txt"
  done
  cd $n
  rm *
  cp ../_stat* ../_accident* ../_error* ../vehicleTrip* ./
  cd ../
done
