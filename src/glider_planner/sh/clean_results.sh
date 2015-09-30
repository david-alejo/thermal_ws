#! /bin/bash

if [ $# -lt 5 ]; then
  echo "Usage: execute <min_uavs> <max_uavs> <min_test> <max_test> <files to erase> [<more files to erase> ...] "
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
	      
	      CONT2=5
	      
	      for i in ${@:5}; do
		rm $i
	      done
	      
	      cd ..
	      let CONT+=1
	    fi
	  done
	  let CONTADOR+=1
	  cd ..
	fi
done
