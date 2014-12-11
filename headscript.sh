#!/bin/sh
 for i in 1 2 3
 do
   for rate in  1.0e-7 2.0e-7 3.0e-7 
   do
	./advmates-calc -d $rate
   done
   ruby ./file.rb $i
 done
