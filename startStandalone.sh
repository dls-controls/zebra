#!/bin/bash
HERE="$(dirname "$0")"
# This is the PV prefix, change this if you have more than one zebra on the network
PREFIX="MYZEBRA"
gnome-terminal -e "${HERE}/iocs/test/bin/linux-x86_64/sttest.sh ${PREFIX}" &
"${HERE}/zebraGui/zebraGui" "${PREFIX}" &
