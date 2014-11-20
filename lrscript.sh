#!/bin/sh
 for i in {1..3}
 do
   for rate in  0.001 0.002 0.003 0.005 0.008 0.01 0.02 0.03 0.05 0.1
   do
	./advmates-calc -d $rate
   done
   ruby ./file.rb
 done
