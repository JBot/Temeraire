#!/bin/sh -x

rm -rf *.o

for file in temeraire_THREADS.c temeraire_UTILS.c temeraire_IK.c temeraire_GAIT.c temeraire.c
 do
  arm-angstrom-linux-gnueabi-gcc -Wall -lm -lrt -lpthread -c $file -I . 
done

#arm-angstrom-linux-gnueabi-gcc -Wall -lm -lrt -lpthread temeraire.c -o start.exe
