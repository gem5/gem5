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

NUM_RUNNERS=3
RUNNER_PREFIX_PREFIX="$(hostname)"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export VAGRANT_HOME=${SCRIPT_DIR}

param="up"
if [[ $# -ge 1 ]]; then
    param=$1
    if [[ "${param}" != "destroy" ]] && [[ "${param}" != "shutdown" ]]; then
        echo "Only valid parameters are 'destroy' and 'shutdown' to destroy all VMs or shutdown all VMs"
        exit 1
    fi
fi


for (( i=1; i<=NUM_RUNNERS; i++ )); do
    sed -i "s/HOSTNAME=.*/HOTNAME=\"${RUNNER_PREFIX}-${i}\"/g" Vagrantfile
    if [[ "${param}" == "destroy" ]]; then
        VAGRANT_VAGRANTFILE=Vagrantfile vagrant destroy -f
    elif [[ "${param}" == "shutdown" ]]; then
        VAGRANT_VAGRANTFILE=Vagrantfile vagrant halt -f
    else
        VAGRANT_VAGRANTFILE=Vagrantfile vagrant up --provider=libvirt
    fi
done
