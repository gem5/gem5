#!/usr/bin/env bash
sudo mkdir -p ~/x86_disk
sudo mount -o loop,offset=32256 ~/Tools/GEM5/x86Dist/disks/linux-x86.img ~/x86_disk
sudo mkdir -p ~/x86_disk/hello
sudo cp -r tests/test-progs/hello/bin/x86/linux/* ~/x86_disk/hello
sudo umount -l ~/x86_disk/