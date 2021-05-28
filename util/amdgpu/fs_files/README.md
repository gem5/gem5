The utility scripts provided in this directly can be used to recreate the
files needed to run Full System amdgpu. Golden versions of these files are
provided in the gem5-resources repository.

These scripts are intended to be run on a Linux system with a discrete AMD
GPU card running the same kernel version as the disk image which will be used
for Full System simulations. To ensure the kernel versions match, it is
recommended that the user uses the `dd` utility to make a copy of the disk
image to a secondary physical disk and then boot from that disk image.

# MMIO Trace collection

The script `collect_mmio_trace.sh` collects an MMIO trace. For this to work,
MMIO tracing must be enabled in the kernel. Using the disk image approach
above and/or using recent kernel versions based on Ubuntu, this is enabled
by default.

Before collecting the trace, the amdgpu module must be blacklisted such that
we can enable tracing before the modprobe occurs and disabled once modprobe
is complete.

This can be done using either:
    `echo 'blacklist amdgpu' >> /etc/modprobe.d/blacklist.conf`
or
    add `modprobe.blacklist=amdgpu` to kernel boot command (e.g., in grub)
And then reboot.

The script will take care of the proper options and run the mmiotrace
commands. The output trace will likely contain unrelated from other MMIOs
and therefore will be around 500MB in size. You will want to reboot again
as mmiotrace disables SMP.

# GPU ROM Dump

The script `dump_gpu_rom.sh` dumps the GPU ROM. The GPU ROM on x86 resides
in the memory range 0xC0000 - 0xDFFFF. The script simply uses the `dd`
command to copy this range from /dev/mem. The amdgpu driver must be loaded
in order for the ROM to be written to this region, so run `modprobe` first.
