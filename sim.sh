#!/bin/sh

rm nohup.out
for n in "res1" "res2" "res3"  
do
  mkdir $n
  rm   _stat* _accident* _error* 
  for i in "200" 
  do
    ./advmates-calc -d $i -L
  done
  cd $n
  rm *
  cp ../_stat* ../_accident* ../_error* ./
  cd ../
done
