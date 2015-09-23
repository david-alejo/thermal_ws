#! /bin/bash

if [ $# -ne 6 ]; then
  echo "Usage: execute <min_uavs> <max_uavs> <n_tests> <test_in> <test_out> <out_directory>"
  exit 1;
fi
  
CONTADOR=$1

until [ $CONTADOR -gt $2 ]; do
	cd $CONTADOR
	CONT=0
	
	/home/sinosuke/repo/glider_planner/build/multiuav_util $4 $4_ --different_ccii $3
	until [ $CONT -ge $3 ]; do
	  let CONT_2=$CONT+1 
# 	  echo Current cont: $CONT Param 4: $4 Cont 2: $CONT_2 Contador $CONTADOR
	  DIRECTORY=$6/$CONTADOR/$CONT_2
	  if [ ! -d $DIRECTORY ]; then
	    mkdir $DIRECTORY
	  fi
	  echo "cp ${4}_${CONT} $DIRECTORY/$5"
	  cp ${4}_${CONT} $6/$CONTADOR/$CONT_2/$5
	  let CONT+=1
	done
	let CONTADOR+=1
	cd ..
done

