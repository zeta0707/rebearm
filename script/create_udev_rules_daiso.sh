#!/bin/bash

echo "remap video* of Daiso camera to rebecam"
sudo cp daiso.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish"