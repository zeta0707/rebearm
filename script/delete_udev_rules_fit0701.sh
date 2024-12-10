#!/bin/bash

echo "delete remap video* of DFRbot to fit0701"
echo "sudo rm  /etc/udev/rules.d/fit0701.rules"
sudo rm   /etc/udev/rules.d/fit0701.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"