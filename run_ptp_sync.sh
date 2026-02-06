#!/bin/bash

cd /home/scanar/scan_ar/ || exit

echo "Starting PTP4L Master..."
sudo ptp4l -S -f gptp_master.conf -m