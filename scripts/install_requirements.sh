#!/bin/bash

cd "$(dirname "$0")"
sudo apt-get install -y can-utils  \
                        iputils-ping    \
                        iproute2        \
                        net-tools       \
                        tcpdump         \
                        nmap            \
                        socat           \
                        kmod

pip install -r requirements.txt
