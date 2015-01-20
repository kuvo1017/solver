#!/bin/sh

rm nohup.out
for n in "res1" "res2" "res3"  
do
  mkdir $n
  rm   _stat* _accident* _error* 
  for i in "100" "150" "200" 
  do
    ./advmates-calc -d $i -L
  done
  cd $n
  rm *
  cp ../_stat* ../_accident* ../_error* ./
  cd ../
  cp ../simulations/okayama-kubo/result/vehicleTrip.txt ./$n"/vehicleTrip_"$i".txt"
done
