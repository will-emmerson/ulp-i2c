#!/usr/bin/env bash
#rm -rf /build
idf.py fullclean
rm -rf /cmake-build-debug
ccache --clear
