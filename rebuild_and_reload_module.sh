#!/bin/bash

# remove original igc driver
sudo modprobe -r igc

# build and install atemsys kernel pci driver
cd "src/atemsys"
make clean
make modules                        # build   kernal pci driver
sudo insmod atemsys.ko loglevel=0   # install kernel pci driver
cd "../../"

# build and run igc user space I225-V ethernet server
make clean                          # clean
make                                # compile
sudo ./igc_user_space               # run

# remove atemsys kernel module
sudo rmmod atemsys