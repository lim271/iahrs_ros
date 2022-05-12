#!/bin/bash

sudo cp `rospack find iahrs_ros`/scripts/iahrs.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart