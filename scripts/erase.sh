#!/usr/bin/env bash
idf.py -p /dev/ttyUSB${1:-0} -b 2000000 erase_flash