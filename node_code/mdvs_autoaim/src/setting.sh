#!/bin/bash
cp /etc/apt/sources.list.bak /etc/apt/sources.list
cat > /etc/apt/sources.list <<EOF
deb-src http://archive.ubuntu.com/ubuntu xenial main restricted #Added by software-properties 
deb http://mirrors.aliyun.com/ubuntu/ xenial main restricted 
deb-src http://mirrors.aliyun.com/ubuntu/ xenial main restricted multiverse universe 
deb http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted 
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted multiverse universe 
deb http://mirrors.aliyun.com/ubuntu/ xenial universe 
deb http://mirrors.aliyun.com/ubuntu/ xenial-updates universe 
deb http://mirrors.aliyun.com/ubuntu/ xenial multiverse 
deb http://mirrors.aliyun.com/ubuntu/ xenial-updates multiverse 
deb http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse 
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse 
deb http://archive.canonical.com/ubuntu xenial partner 
deb-src http://archive.canonical.com/ubuntu xenial partner 
deb http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted 
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted multiverse universe 
deb http://mirrors.aliyun.com/ubuntu/ xenial-security universe 
deb http://mirrors.aliyun.com/ubuntu/ xenial-security multiverse 
EOF
apt-get update
apt-get upgrade --fix-missing -y


sudo apt-get install libgflags-dev -y
cd /home/star/HEU_Vision_2020_MDVS/glog
./configure
make -j 24
make install
ldconfig



