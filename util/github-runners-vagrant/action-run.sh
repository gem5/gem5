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

set -x

# No argument checking here, this is run directly in the Vagrantfile.
PERSONAL_ACCESS_TOKEN="$1"
GITHUB_ORG="$2"
LABELS="$3"
WORK_DIR="_work"

# This checks there isn't another instance of this script running.
# If this script is run twice then more than one runner can be active in the
# VM and this causes problems.
if [[ `pgrep -f $0` != "$$" ]]; then
    echo "Another instance of shell already exist! Exiting"
    exit
fi

# If the tarball isn't here then download it and extract it.
# Note: we don't delete the tarball, we use it to check if we've already
# downloaded it and extracted it.
if [ ! -f "actions-runner-linux-x64-2.304.0.tar.gz" ]; then
    wget https://github.com/actions/runner/releases/download/v2.304.0/actions-runner-linux-x64-2.304.0.tar.gz
    tar xzf ./actions-runner-linux-x64-2.304.0.tar.gz
fi

# An infinite loop to re-configure and re-run the runner after each job.
while true; do
    # 1. Obtain the registration token.
    token_curl=$(curl -L \
    		      -X POST \
    		      -H "Accept: application/vnd.github+json" \
    		      -H "Authorization: Bearer ${PERSONAL_ACCESS_TOKEN}" \
    		      -H "X-GitHub-Api-Version: 2022-11-28" \
    		      https://api.github.com/orgs/${GITHUB_ORG}/actions/runners/registration-token)

    token=$(echo ${token_curl} | jq -r '.token')

    # 2. Configure the runner.
    ./config.sh --unattended \
                --url https://github.com/${GITHUB_ORG} \
                --ephemeral \
                --replace \
                --work "${WORK_DIR}" \
                --name "$(hostname)" \
                --labels "${LABELS}" \
                --token ${token}

    # 3. Run the runner.
    ./run.sh # This will complete with the runner being destroyed

    # 4. Cleanup the machine
    rm -rf "${WORK_DIR}"
    docker system prune --force --volumes
done
