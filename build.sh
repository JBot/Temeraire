#!/bin/sh -x

rm -rf *.o
rm -rf *.a

#for file in temeraire_var.c temeraire_UTILS.c temeraire_THREADS.c temeraire_IK.c temeraire_GAIT.c 
# do
#  arm-angstrom-linux-gnueabi-gcc -Wall -lm -lrt -lpthread -c $file -I . 
#done

#LIBNAME_BASE=temeraire
#LIBNAME=libtemeraire.a
#arm-angstrom-linux-gnueabi-ar ruv libtemeraire.a *.o
#arm-angstrom-linux-gnueabi-ranlib libtemeraire.a

#arm-angstrom-linux-gnueabi-gcc -I . -L . -Wall -lm -lrt -lpthread -ltemeraire temeraire.c -o start.exe

arm-angstrom-linux-gnueabi-gcc -O2 -mtune=cortex-a8 -march=armv7-a -ftree-vectorize -mfpu=neon -mfloat-abi=softfp -ffast-math -fomit-frame-pointer -Wall -lm -lrt -lpthread -I /media/ROBOTS/robotsave/adc/ temeraire_var.c temeraire_UTILS.c temeraire_THREADS.c temeraire_GAIT.c temeraire_IK.c temeraire_AI.c temeraire.c -o start.exe
