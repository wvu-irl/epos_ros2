#!/bin/bash

cd ~/
mkdir epos_temp
cd epos_temp
wget https://www.maxongroup.us/medias/sys_master/root/8884354678814/EPOS-Linux-Library-En.zip
unzip EPOS-Linux-Library-En.zip
cd EPOS_Linux_Library
chmod +x install.sh
sudo ./install.sh
cd ~/
rm -rf epos_temp
