#!/bin/bash

devices=(/dev/ttyACM*)

if [ ! -e "${devices[0]}" ]; then
  echo "No ttyACM device found"
  exit 1
fi

for dev in "${devices[@]}"; do
  echo "Found ttyACM device: $dev"
  udevadm info -a -n "$dev" | grep idVendor  | awk 'NR==1 {print substr($0, 10,23)}'
  udevadm info -a -n "$dev" | grep idProduct | awk 'NR==1 {print substr($0, 10,23)}'
  udevadm info -a -n "$dev" | grep serial    | awk 'NR==1 {print substr($0, 10,24)}'
done