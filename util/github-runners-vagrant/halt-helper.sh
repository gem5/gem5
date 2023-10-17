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

# This script will try to safely halt each VM specified in the Vagrantfile.
# A VM is skipped if it is currently running a job and returned to after
# attempted shutdowns on the other VMs. This cycle continues indefinitely until
# all the runners are shutdown.
#
# This script is usefull as the VMs occasionally need to be halted to apply
# patches and do maintenance. This script allows us to do this without
# interrupting any jobs that may be running.

while true; do
    # This will list all the VMs still running. If there are no VM's running,
    # we infer all have been shutdown and we exit the script. Otherwise, we
    # iterate over he VMs in an attempt to shut them down.
    active=$(vagrant status | grep running | tr -s ' ' | cut -d ' ' -f1)
    if [ "$active" == "" ]; then
        echo "All VMs have been shutdown. Exiting."
        exit 0
    fi
    echo "The following VMs are still running:"
    echo "${active}"

    for virtm in $active
    do
        # This script will first list the contents of the "_diag" directory.
        # This directory hosts the github action runner job logs. Each job
        # is logged to a seperate file in the directpry. This script then
        # sort these files by name. The last file in this sorted list is  the
        # most recent file and therefore for the most recent job. We can sort
        # them in this was because their filenames are appended with UTC
        # timestamps.
        #
        # One one job ever runs at a time on a GitHub runner so if there is any
        # job running, it be being logged in the most recent file in the
        # "_diag" directory.
        #
        # If the job has completed the last line in the file will contain the
        # string "Job completed.". This script checks for this and, if found,
        # we assume there are no jobs running safely run `vagrant halt` to
        # shutdown the VM. If the job is still running we print a message
        # saying the job is still running and will return to it on the next
        # iteration of the loop.
        echo "Inspecting \"${virtm}\"..."
        vagrant ssh $virtm -c 'ls _diag | sort | tail -1 | xargs -I % cat "_diag/%" | tail -1 | grep -q "Job completed"'
        status=$?
        if [[ ${status} == 0 ]]; then
            echo "${virtm} is Idle. Attempting shutdown"
            vagrant halt ${virtm} && echo "${virtm} successfully halted" || echo "${virtm} experience a failure halting"
        else
            echo "${virtm} is Busy. Skipping for now."
        fi
    done
    # Sleep here for 20 seconds just to ensure all the VMs have time
    # to shutdown.
    sleep 20
done
