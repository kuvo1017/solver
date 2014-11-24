#!/bin/sh
 for i in {1..3}
 do
   for rate in  0.0001 0.0002 0.0003 0.0005 
   do
	./advmates-calc -d $rate
   done
   ruby ./file.rb "$i"
 done
