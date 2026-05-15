#!/usr/bin/env bash
# Copyright (c) 2025 Polydoros Petrakis
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

# gem5_debug_ctl.sh — runtime debug flag control for a running gem5 process.
#
# gem5 registers a SIGRTMIN+1 handler at simulation start. This script writes
# a command file to /tmp/gem5_debug_<pid>.cmd and sends SIGRTMIN+1 to trigger
# processing. Commands are executed in the main event loop, so they take effect
# at the next sim tick after delivery (no mid-instruction races).
#
# Usage:
#   gem5_debug_ctl.sh <pid> <cmd> [<cmd> ...]
#
# Commands:
#   +FLAG           Enable flag (stays on until explicitly disabled)
#   +FLAG:Nt        Enable flag, auto-disable after N ticks
#   -FLAG           Disable flag immediately
#   trace:on        Global trace master enable  (all enabled flags emit output)
#   trace:off       Global trace master disable (flags unchanged, output silent)
#
# Examples (<PID> e.g. 12345):
#   gem5_debug_ctl.sh <PID> +Rename
#   gem5_debug_ctl.sh <PID> +O3CPU:100000t
#   gem5_debug_ctl.sh <PID> -Fetch trace:off
#   gem5_debug_ctl.sh <PID> +Rename:100000t +Fetch:100000t trace:on
#
# To list available flag names:
#   <gem5_binary> --debug-help 2>&1 | grep -i <keyword>
#
# Confirmation lines (gem5_debug_ctl: ...) appear in the sim's stdout/log.

set -euo pipefail

usage() {
    sed -n '/^# Usage:/,/^[^#]/{ /^[^#]/d; s/^# \{0,1\}//p }' "$0"
    exit 1
}

if [[ $# -lt 2 ]]; then
    usage
fi

PID="$1"
shift

# Verify the target is a running gem5 process.
if ! kill -0 "${PID}" 2>/dev/null; then
    echo "Error: no process with PID ${PID}" >&2
    exit 1
fi
if ! ps -p "${PID}" -o comm= 2>/dev/null | grep -q 'gem5'; then
    echo "Warning: PID ${PID} does not appear to be a gem5 process" >&2
fi

CMD_FILE="/tmp/gem5_debug_${PID}.cmd"

# Write commands one per line (overwrite any leftover file from a prior run).
printf '%s\n' "$@" > "${CMD_FILE}"

# SIGRTMIN+1 is used to avoid SIGRTMIN which is reserved for KVM.
kill -RTMIN+1 "${PID}"

echo "Sent ${#} command(s) to gem5 PID ${PID}." \
     "Check the sim log for 'gem5_debug_ctl:' confirmation lines."
