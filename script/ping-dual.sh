#!/bin/bash
#  --debug-flags=ArmIRQ,Interrupt,GIC,SimpleCPU,Ethernet \
    # --debug-start=3565690000000 --debug-end=3565700000000 \
# --debug-start=3565600000000 --debug-end=3565700000000 \
    # --debug-flags=Interrupt,GIC,Ethernet,ArmIRQ,Fetch,Faults,Ethernet,Interrupt,GIC,Faults,EthernetSM, \
# --debug-start=3565651000000 --debug-end=3565700000000 \
# --debug-start=3565696852500 --debug-end=3565696882000 \
# --debug-start=3565696838500 --debug-end=3565696884000 \
/research/hzhuo2/gem5/build/ARM/gem5.opt --listener-mode on \
    -d /research/hzhuo2/gem5/m5out/ping-new \
    /research/hzhuo2/gem5/configs/tail/arm_dual.py \
    --num-cores=1 --cpu=atomic\
    --bootloader=/research/hzhuo2/gem5-images/arm-prebuild/binaries/boot_v2.arm64 \
    --kernel=/research/hzhuo2/gem5-images/arm-prebuild/binaries/vmlinux.arm64 \
    --disk-image=/research/hzhuo2/gem5-images/arm-prebuild/ubuntu-18.04-arm64-tailbench.img \
    --server-script=/research/hzhuo2/gem5/configs/boot/ping-server.rcS \
    --client-script=/research/hzhuo2/gem5/configs/boot/ping-client.rcS \

    # --debug-flags=ArmIRQ,Interrupt,GIC,SimpleCPU \
    # --debug-start=541071000000 --debug-end=541072000000 \
    # --restore=/research/hzhuo2/gem5/m5out/ping-dual/cpt.499230474500 \
    # --maxinsts=20000000

    # --debug-start=3565700073500 --debug-end=3565700687250 \ for minor