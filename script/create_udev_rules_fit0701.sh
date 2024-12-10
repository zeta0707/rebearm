#!/bin/bash

echo "remap video* of DFRbot to fit0701"
sudo cp fit0701.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish"