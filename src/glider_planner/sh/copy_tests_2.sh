#! /bin/bash

if [ $# -ne 6 ]; then
  echo "Usage: execute <min_uavs> <max_uavs> <min_test> <max_test> <test_in> <test_out>"
  exit 1;
fi
  
CONTADOR=$1

until [ $CONTADOR -gt $2 ]; do
	cd $CONTADOR
	CONT=$3
	until [ $CONT -gt $4 ]; do
	  let CONT+=1
	  echo Current cont: $CONT Param 4: $4
	  if [ ! -d $CONT ]; then
	    mkdir $CONT
	  fi
	  
	  cd $CONT
	  let CONT+=-1
	  echo "copy ${5}_${CONT}_${CONTADOR} $6"
	  cp ${5}_${CONT}_${CONTADOR} $6
	  cd ..
	  let CONT+=1
	done
	let CONTADOR+=1
	cd ..
done
