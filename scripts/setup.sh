#!/bin/bash

# initializes pigpio and disables memory limits for the camera

echo "running setup!"
sudo -S pigpiod
echo 0 | sudo -S tee /sys/module/usbcore/parameters/usbfs_memory_mb
