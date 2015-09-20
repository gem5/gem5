#!/usr/bin/env bash
./compile.sh
# Go to http://www.gem5.org/Download/, and download the following two full-system stuffs.
# Extract x86Dist.tar.gz into ~/Tools/GEM5/ before executing the following commands.
# Extract linux-bigswap2.img from m5_system_2.0b3.tar.bz2 into ~/Tools/GEM5/x86Dist/disks/.
./mount.sh
./run_fs.sh
./run_mcpat.sh