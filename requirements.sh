#!/bin/bash

# Dependency
sudo apt install ros-foxy-vision-msgs

# Clone darknet
cd avt_vimba_camera/include/objectdetection
git clone https://github.com/AlexeyAB/darknet.git
cd darknet
git reset --hard fb2c3b0d6d25b7019af6c541e998d1716e56d7a7

# Modify darknet.h for objectdetection.h
if [ ! -e darknet/include/origin_darknet.h ]; then
    mv darknet/include/darknet.h darknet/include/origin_darknet.h
fi
cp darknet/include/origin_darknet.h darknet/include/darknet.h
DARKNET_H_MODIFY=$(cat ../../../darknet_modify.txt | sed ':a;N;$!ba;s|\n|\\n|g' | sed 's|/|\\/|g' )
sed -i "1112 a \\$DARKNET_H_MODIFY" darknet/include/darknet.h

# Modify Makefile
sed -i "s/GPU=0/GPU=1/g" darknet/Makefile
sed -i "s/OPENCV=0/OPENCV=1/" darknet/Makefile
sed -i "s/LIBSO=0/LIBSO=1/" darknet/Makefile
sed -i "58 a \\ARCH= -gencode arch=compute_72,code=[sm_72,compute_72]" darknet/Makefile
sed -i "59 a \\ARCH= -gencode arch=compute_72,code=[sm_87,compute_87]" darknet/Makefile

# Build
cd darknet
make -j${nproc}
