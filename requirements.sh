#!/bin/bash

# Dependency
sudo apt install ros-foxy-vision-msgs

# Clone darknet
cd avt_vimba_camera/include/objectdetection
git clone https://github.com/AlexeyAB/darknet.git
cd darknet
git reset --hard fb2c3b0d6d25b7019af6c541e998d1716e56d7a7

# Modify darknet.h for objectdetection.h
if [ ! -e include/origin_darknet.h ]; then
    mv include/darknet.h include/origin_darknet.h
fi
cp include/origin_darknet.h include/darknet.h
DARKNET_H_MODIFY=$(cat ../../../../darknet_modify.txt | sed ':a;N;$!ba;s|\n|\\n|g' | sed 's|/|\\/|g' )
sed -i "1112 a \\$DARKNET_H_MODIFY" include/darknet.h

# Modify Makefile
sed -i "s/GPU=0/GPU=1/g" Makefile
sed -i "s/CUDNN=0/CUDNN=1/g" Makefile
sed -i "s/OPENCV=0/OPENCV=1/g" Makefile
sed -i "s/LIBSO=0/LIBSO=1/g" Makefile
sed -i "60 a \\ARCH= -gencode arch=compute_72,code=[sm_72,compute_72]" Makefile
sed -i "61 a \\ARCH= -gencode arch=compute_87,code=[sm_87,compute_87]" Makefile

# Build
make -j${nproc}
