#!/usr/bin/env bash
sudo mkdir -p ~/system
#sudo mount -o loop,offset=32256 ~/Tools/GEM5/system/disks/linux-x86.img ~/system
#sudo mount -o loop,offset=32256 ~/Tools/GEM5/system/disks/x86root-parsec.img ~/system
sudo mount -o loop,offset=32256 ~/Tools/GEM5/system/disks/linux-parsec-2-1-m5-with-test-inputs.img ~/system
#sudo mkdir -p ~/system/hello
#sudo cp -r tests/test-progs/hello/bin/x86/linux/* ~/system/hello
#sudo umount -l ~/system/