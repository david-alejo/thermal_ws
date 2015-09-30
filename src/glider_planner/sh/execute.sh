#! /bin/bash

if [ $# -ne 5 ]; then
  echo "Usage: execute <min_uavs> <max_uavs> <min_test> <max_test> <test_filename>"
  exit 1;
fi
  
CONTADOR=$1

# Copy all archives to the genetic ones

until [ $CONTADOR -gt $2 ]; do
	if [ -d $CONTADOR ]; then
	  cd $CONTADOR
	  CONT=$3
	  until [ $CONT -gt $4 ]; do
	    if [ -d $CONT ]; then
	      cd $CONT
	      
	      if [ ! -f PoI ]; then
		echo Copying PoI
		cp ~/repo/glider_planner/test/IROS_2013/Map3/PoI .
	      fi
	      
	      sed -i "s*directory = ../Maps;*directory = ../../Maps;*g" $5
	      sed -i "s;max_t = 0;max_t = 1500;g" $5
	      ~/repo/glider_planner/build/multiuav_test $5 > $5.out
	      cd ..
	      let CONT+=1
	    fi
	  done
	  let CONTADOR+=1
	  cd ..
	fi
done
