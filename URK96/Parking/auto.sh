
#! /bin/bash

cp -vf /home/urk96/EmbeddedAutoDrivingCar/Code/TestParking/testParking.c ./testParking.c
sudo ifconfig eno1 down
echo "Successful down"
sleep 1
sudo ifconfig eno1 up
echo "Successful up"
sleep 2
make clean
make
echo "send file by scp"
sleep 4
scp testParking root@10.10.70.4:/home/root/urk96