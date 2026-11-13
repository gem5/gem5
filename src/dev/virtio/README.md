# VirtIO 9P UFS Backend

gem5 can build an optional VirtIO 9P backend backed by npfs/ufs. This backend
exports a host directory directly to the guest and serializes the npfs state in
gem5 checkpoints, which allows a 9P-mounted root filesystem to survive
checkpoint and restore.

Build npfs first, then pass its source directory to SCons:

```sh
scons -j8 build/ARM/gem5.opt NPFS_PATH=/path/to/npfs
```

`NPFS_PATH` must point at a tree containing `include/`, `libnpfs/libnpfs.a`,
and `libufs/libufs.a`. Builds without `NPFS_PATH` still include the existing
VirtIO 9P proxy classes, but the `VirtIO9PUfs` SimObject is not available.

Only exporting a host directory requires this backend. `--vio-9p` on its own
still uses the diod server and works in any build.

For ARM full-system boots with `configs/example/arm/fs_bigLITTLE.py`, use a
host directory as the guest root filesystem with:

```sh
build/ARM/gem5.opt configs/example/arm/fs_bigLITTLE.py \
    --kernel /path/to/vmlinux \
    --bootloader /path/to/boot.arm64 \
    --bootloader /path/to/boot.arm \
    --rootfs-dir /path/to/root-directory
```

`--rootfs-dir` exports the directory through VirtIO 9P, sets the 9P mount tag
to `/dev/root`, and adds the kernel arguments needed for a 9P root mount:
`rootfstype=9p` and `rootflags=trans=virtio,version=9p2000.L`.

Use `--vio-9p-root DIR` to export a directory without making it the guest root,
and `--vio-9p-tag TAG` to override the mount tag. Pass
`--debug-flags=VIO9PUfs` for verbose output from the embedded npfs/ufs server.
