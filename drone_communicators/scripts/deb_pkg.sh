#!/bin/bash
bloom-generate rosdebian
fakeroot debian/rules binary
rm -rf obj-x86_64-linux-gnu/ debian/
