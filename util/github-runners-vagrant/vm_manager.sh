#!/bin/bash

NUM_RUNNERS=20
NUM_BUILDERS=3
RUNNER_PREFIX="$(hostname)-runner-"
BUILDER_PREFIX="$(hostname)-builder-"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export VAGRANT_HOME=${SCRIPT_DIR}

param="up"
if [[ $# -ge 1 ]]; then
    param=$1
    if [[ "${param}" != "destroy" ]]; then
        echo "Only valid parameter is 'destroy', to destroy all VMs"
        exit 1
    fi
fi


for (( i=1; i<=NUM_RUNNERS; i++ )); do
    sed -i "s/  config.vm.define.*/  config.vm.define \"${RUNNER_PREFIX}${i}\"/g" Vagrantfile-runner
    sed -i "s/  config.vm.hostname.*/  config.vm.hostname = \"${RUNNER_PREFIX}${i}\"/g" Vagrantfile-runner
    if [[ "${param}" == "destroy" ]]; then
        VAGRANT_VAGRANTFILE=Vagrantfile-runner vagrant destroy -f
    else
        VAGRANT_VAGRANTFILE=Vagrantfile-runner vagrant up --provider=libvirt
    fi
done

for (( i=1; i<=NUM_BUILDERS; i++ )); do
    sed -i "s/  config.vm.define.*/  config.vm.define \"${BUILDER_PREFIX}${i}\"/g" Vagrantfile-builder
    sed -i "s/  config.vm.hostname.*/  config.vm.hostname = \"${BUILDER_PREFIX}${i}\"/g" Vagrantfile-builder
    if [[ "${param}" == "destroy" ]]; then
        VAGRANT_VAGRANTFILE=Vagrantfile-builder vagrant destroy -f
    else
        VAGRANT_VAGRANTFILE=Vagrantfile-builder vagrant up --provider=libvirt
    fi
done
