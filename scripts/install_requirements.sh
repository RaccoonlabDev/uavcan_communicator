#!/bin/bash

cd "$(dirname "$0")"
apt-get install -y can-utils
pip install -r requirements.txt
