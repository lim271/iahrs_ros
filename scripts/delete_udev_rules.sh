#!/bin/bash

sudo rm /etc/udev/rules.d/iahrs.rules
sudo service udev reload
sudo service udev restart