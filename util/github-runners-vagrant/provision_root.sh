#!/usr/bin/env bash

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

# All privileged operations here run inside the guest, never on the host.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
# Include new kernel dependencies when updating the base box.
apt-get --with-new-pkgs upgrade -y
apt-get install -y linux-generic bash build-essential clang-format git git-lfs \
    jq libffi-dev libssl-dev nkf python3 python3-dev python3-pip python3-venv \
    shellcheck tree wget yamllint zstd ca-certificates curl gnupg lsb-release \
    cpu-checker

install -m 0755 -d /etc/apt/keyrings
curl --fail --show-error --silent --retry 3 \
    https://download.docker.com/linux/ubuntu/gpg \
    -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
# Replace the previous version of this provisioner's source definition.
rm -f /etc/apt/sources.list.d/docker.list
cat > /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release; echo "$VERSION_CODENAME")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin
usermod -aG docker vagrant
systemctl enable --now docker

# Re-provisioning a disk which already fills its volume group is harmless.
root_device=$(findmnt -n -o SOURCE /)
if lvs "$root_device" >/dev/null 2>&1; then
    volume_group=$(lvs --noheadings -o vg_name "$root_device" | xargs)
    free_extents=$(vgs --noheadings -o vg_free_count "$volume_group" | xargs)
    if (( free_extents > 0 )); then
        lvextend --resizefs -l +100%FREE "$root_device"
    fi
fi
apt-get clean
