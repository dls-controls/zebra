#!/bin/bash
make -C iocs/test
rm -rf zebraGui
cp -rf /dls_sw/prod/tools/RHEL6-x86_64/zebraGui/1-0/prefix zebraGui
cp -rf /dls_sw/prod/tools/RHEL6-x86_64/zebraGui/1-0/zebraGui zebraGui/src

