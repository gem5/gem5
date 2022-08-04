/research/hzhuo2/gem5/build/ARM/gem5_native.opt --listener-mode on \
    -d /research/hzhuo2/gem5/m5out/silo-gen \
    /research/hzhuo2/gem5/configs/tail/arm_dual.py \
    --num-cores=1 --cpu=atomic\
    --bootloader=/research/hzhuo2/gem5-images/arm-prebuild/binaries/boot_v2.arm64 \
    --kernel=/research/hzhuo2/gem5-images/arm-build/vmlinux.arm64.5.15 \
    --disk-image=/research/hzhuo2/gem5-images/arm-prebuild/ubuntu-18.04-arm64-tailbench.img \
    --server-script=/research/hzhuo2/gem5/configs/boot/silo-server.rcS \
    --client-script=/research/hzhuo2/gem5/configs/boot/silo-client.rcS