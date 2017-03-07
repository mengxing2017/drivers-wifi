#!/bin/sh
#
# TODO: write a real download script

set -e

readonly SCRIPT_ROOT="$(cd $(dirname ${BASH_SOURCE[0]} ) && pwd)"
cd "${SCRIPT_ROOT}"

curl -o rt2870.zip https://fuchsia-build.storage.googleapis.com/firmware/ralink/$(cat rt2870.sha1)
unzip -o rt2870.zip rt2870.bin
