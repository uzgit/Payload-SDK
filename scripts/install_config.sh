#!/bin/bash

apt-get update
apt-get install -y vim git mlocate tree htop dnsmasq cmake ffmpeg libavcodec-dev libavformat-dev libavfilter-dev libopus-dev libusb-1.0-0-dev libopencv-dev

sudo updatedb

ln -s /usr/lib/arm-linux-gnueabihf/libopus.a /usr/local/lib/libopus.a

sed -i 's/rootwait/rootwait modules-load=dwc2,g_ether/' /boot/cmdline.txt

echo "dtoverlay=dwc2"  >> /boot/config.txt
echo "dtoverlay=uart0" >> /boot/config.txt
echo "enable_uart=1"   >> /boot/config.txt
