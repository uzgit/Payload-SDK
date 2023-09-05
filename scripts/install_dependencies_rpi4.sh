#!/bin/bash

apt-get update
apt-get install vim git mlocate tree htop dnsmasq cmake ffmpeg libavcodec-dev libavformat-dev libavfilter-dev libopus-dev libusb-1.0-0-dev libopencv-dev

sudo updatedb

ln -s /usr/lib/arm-linux-gnueabihf/libopus.a /usr/local/lib/libopus.a
