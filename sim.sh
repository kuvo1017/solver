#!/bin/sh
for i in "0.0001" "0.0005" "0.001" "0.03" "0.05" "0.1" "0.2" "0.3" "0.05"
do
 ./advmates-calc -d $i
done
