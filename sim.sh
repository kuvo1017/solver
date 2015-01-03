#!/bin/sh
for i in "1e-4" "3e-4" "5e-4" "1e-3" "2e-3" "5e-3" "1e-2"
do
 ./advmates-calc -d $i
done
