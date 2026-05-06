#!/bin/bash

sudo cp nexus_arm_tty.rules /etc/udev/rules.d/

sudo chmod +x /etc/udev/rules.d/nexus_arm_tty.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
