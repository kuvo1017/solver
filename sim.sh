#!/bin/sh
for i in "0.005" "0.01" "0.03"  "0.05" "0.075" "0.1" "0.3" "0.5"
do
 ./advmates-calc -d $i
done
