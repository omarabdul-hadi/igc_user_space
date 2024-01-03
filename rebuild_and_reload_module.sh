#!/bin/bash

# remove original igc driver
sudo modprobe -r igc

# build and install atemsys kernel pci driver
cd "src/atemsys"
make clean
make modules                        # build   kernal pci driver
sudo insmod atemsys.ko loglevel=0   # install kernel pci driver

# build and run igc user space I225-V ethernet server
cd "../../"
make clean                          # clean
make                                # compile
sudo ./igc_user_space               # run
