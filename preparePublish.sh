#!/bin/bash
make -C iocs/test
rm -rf zebraGui
cp -rf /dls_sw/work/tools/RHEL6-x86_64/zebraGui/prefix zebraGui
cp -rf /dls_sw/work/tools/RHEL6-x86_64/qt_static/prefix/qml/* zebraGui/
cp -rf /dls_sw/work/tools/RHEL6-x86_64/zebraGui/zebraGui zebraGui/src

