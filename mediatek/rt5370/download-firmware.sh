#!/bin/sh
#
# TODO: write a real download script

set -e

curl -o rt2870.zip https://fuchsia-build.storage.googleapis.com/firmware/ralink/$(cat rt2870.sha1)
unzip -o rt2870.zip rt2870.bin
