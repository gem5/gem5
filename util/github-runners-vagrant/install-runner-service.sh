#!/bin/bash

# Copyright (c) 2023 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Run inside the guest as root after copying the runner scripts.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
install -m 0644 gem5-runner.service /etc/systemd/system/gem5-runner.service
# Vagrant normally mounts 9p over SSH only. Persist its actual device tag so
# an ordinary guest reboot has the cache before the runner service starts.
if mountpoint -q /gem5-resource-cache; then
    cache_tag=$(findmnt -n -t 9p -o SOURCE --target /gem5-resource-cache)
    [[ "$cache_tag" =~ ^[a-zA-Z0-9._-]+$ ]]
    # Loading the filesystem alone does not load its virtio transport early
    # enough for a boot-time mount on Ubuntu 22.04.
    printf '%s\n' 9pnet_virtio 9p > /etc/modules-load.d/gem5-resource-cache.conf
    mount_unit=$(systemd-escape --path --suffix=mount /gem5-resource-cache)
    cat > "/etc/systemd/system/$mount_unit" <<EOF
[Unit]
Description=gem5 shared resource cache
Wants=systemd-modules-load.service
After=systemd-modules-load.service
Before=gem5-runner.service

[Mount]
What=$cache_tag
Where=/gem5-resource-cache
Type=9p
Options=trans=virtio,version=9p2000.L,cache=none,msize=1048576

[Install]
WantedBy=local-fs.target
EOF
    systemctl daemon-reload
    systemctl enable "$mount_unit"
fi
systemctl daemon-reload
# Image builders omit credentials and leave the service disabled.
if [[ -f /etc/gem5-runner.env ]]; then
    systemctl enable --now gem5-runner.service
fi
