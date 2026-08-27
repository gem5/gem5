# Copyright (c) 2026 The Regents of The University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from pathlib import Path

from gem5.resources.resource import obtain_resource

GPUFS_RESOURCE_VERSION = "1.0.0"

GPUFS_BOOT_SERVICES = (
    "multipathd.service",
    "snapd.service",
    "snapd.seeded.service",
    "systemd-networkd.service",
    "systemd-networkd-wait-online.service",
    "systemd-resolved.service",
    "systemd-timesyncd.service",
    "thermald.service",
    "unattended-upgrades.service",
)


def obtain_gpu_fs_resources(resource_directory: Path):
    """Obtain the GPUFS disk image and the kernel its driver was built for."""

    resource_directory.mkdir(parents=True, exist_ok=True)
    resource_kwargs = {
        "resource_directory": str(resource_directory),
        "resource_version": GPUFS_RESOURCE_VERSION,
    }
    return (
        obtain_resource("x86-ubuntu-24.04-gpu-img", **resource_kwargs),
        obtain_resource("x86-linux-kernel-6.8.0-gpu", **resource_kwargs),
    )


def gpu_fs_boot_options():
    """Mask guest services unrelated to offline GPU smoke coverage."""

    return tuple(f"systemd.mask={service}" for service in GPUFS_BOOT_SERVICES)
