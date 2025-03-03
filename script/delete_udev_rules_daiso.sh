#!/bin/bash

echo "delete remap video*"
echo "sudo rm  /etc/udev/rules.d/daiso.rules"
sudo rm   /etc/udev/rules.d/daiso.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"