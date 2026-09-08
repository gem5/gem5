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

# This loop owns one dedicated VM. A drain finishes the current job first.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
pat="${PERSONAL_ACCESS_TOKEN:-${1:-}}"
unset PERSONAL_ACCESS_TOKEN
export GITHUB_ORG="${GITHUB_ORG:-${2:-}}"
LABELS="${RUNNER_LABELS:-${3:-}}"
state_dir="$PWD/runner-state"
mkdir -p "$state_dir"
exec 9>"$state_dir/lock"
flock -n 9 || exit 0

state() {
    printf '%s %s\n' "$(date -u +%FT%TZ)" "$*" | tee "$state_dir/status.tmp"
    mv "$state_dir/status.tmp" "$state_dir/status"
}
quarantine() {
    state "quarantined: $*"
    touch "$state_dir/quarantine"
    exit 78
}
trap 'quarantine "controller failed at line $LINENO"' ERR
if [[ -e "$state_dir/quarantine" ]]; then
    state "quarantined: clear runner-state/quarantine after repair"
    exit 78
fi
[[ -n "$pat" && -n "$GITHUB_ORG" ]] || \
    quarantine "missing registration configuration"
[[ -x ./config.sh && -x ./run.sh ]] || \
    quarantine "runner package missing; run provision_runner.sh"

api_token() {
    curl --fail --silent --show-error --retry 3 --connect-timeout 15 \
        --max-time 90 -X POST \
        -H "Accept: application/vnd.github+json" \
        -H "Authorization: Bearer $pat" \
        -H "X-GitHub-Api-Version: 2022-11-28" \
        "https://api.github.com/orgs/$GITHUB_ORG/actions/runners/$1-token" |
        jq -er '.token | select(type == "string" and length > 0)'
}

healthy() {
    local attempt
    for attempt in 1 2 3; do
        if ./runner-health.sh; then
            return 0
        fi
        state "health check failed (attempt $attempt of 3)"
        [[ "$attempt" == 3 ]] || sleep 15
    done
    return 1
}

failures=0
while true; do
    if [[ -e "$state_dir/drain" ]]; then
        state drained
        exit 0
    fi
    state checking
    healthy || quarantine "pre-registration health check failed"
    if [[ -f .runner ]]; then
        removal_token=$(api_token remove) || \
            quarantine "could not obtain removal token"
        ./config.sh remove --token "$removal_token" || \
            quarantine "could not remove previous registration"
        unset removal_token
    fi
    # Transient API failures leave the VM offline and retry with a delay.
    if ! registration_token=$(api_token registration); then
        state "registration API unavailable; retrying in 180 seconds"
        sleep 180
        continue
    fi
    if [[ -e "$state_dir/drain" ]]; then
        state drained
        exit 0
    fi
    args=(--unattended --url "https://github.com/$GITHUB_ORG" --ephemeral
          --replace --work _work --name "$(hostname)")
    [[ -z "$LABELS" ]] || args+=(--labels "$LABELS")
    ./config.sh "${args[@]}" --token "$registration_token" || \
        quarantine "runner configuration failed"
    unset registration_token
    state listening
    result=0
    ./run.sh || result=$?
    state "cleaning; listener exit=$result"
    timeout 900 ./runner-cleanup.py || quarantine "post-job cleanup failed"
    healthy || quarantine "post-cleanup health check failed"
    if (( result != 0 )); then
        failures=$((failures + 1))
        (( failures < 3 )) || quarantine "three consecutive listener failures"
    else
        failures=0
    fi
    state ready
    [[ -e "$state_dir/drain" ]] || sleep 180
done
