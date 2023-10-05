#!/bin/bash

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
