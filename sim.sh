#!/bin/sh
for i in "0.01" "0.02" "0.05" "0.1" "0.2" "0.3" "0.5"
do
 ./advmates-calc -d $i
done
