#!/bin/bash

echo "remap the device serial port(ttyUSBX) to buslinker2"
sudo cp buslinker2.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish"