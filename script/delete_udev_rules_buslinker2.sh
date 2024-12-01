#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to buslinker2"
echo "sudo rm   /etc/udev/rules.d/buslinker2.rules"
sudo rm   /etc/udev/rules.d/buslinker2.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"