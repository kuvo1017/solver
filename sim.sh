#!/bin/sh
rm   _stat* _accident* _error* 
for n in "res1" "res2" "res3"  
do
  for i in "0.001" "0.002" "0.003" "0.01" "0.05" "0.1" "0.3" "0.5"
  do
    ./advmates-calc -d $i
  done
  mkdir $n
  cp _stat* _accident* _error* ./$n
done
