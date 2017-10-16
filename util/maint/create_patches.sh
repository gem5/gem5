#!/bin/bash
#
# Copyright (c) 2016 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
#
# Authors: Andreas Sandberg
#

set -e

REL_SCRIPT_DIR=`dirname "$0"`
SCRIPT_NAME=`basename "$0"`
SCRIPT_DIR=$(cd "$REL_SCRIPT_DIR" && echo "$(pwd -P)")
MSG_FILTER="$SCRIPT_DIR"/upstream_msg_filter.sed

PATCH_DIR="./patches/"
UPSTREAM="upstream/master"

usage()
{
    cat <<EOF
$SCRIPT_NAME [OPTION]...  [BRANCH]
Format a patch series suitable for upstream consumption.

Options:
  -u BRANCH      Upstream branch
  -d DIR         Patch directory
  -h             Show this help string.

This script creates a series of patches suitable from upstream
consumption from a git branch. By default, the script works on the
currently checked out branch (HEAD). When invoked, the script executes
the following operations in order:

  1. Rebase the patches in the current branch onto the upstream
     branch.
  2. Filter commit messages.
  3. Generate a set of patches in git format.
EOF
}

branch_exists()
{
    git rev-parse --verify -q "$1" > /dev/null
}

while getopts ":u:d:h" OPT; do
    case $OPT in
        d)
            PATCH_DIR="$OPTARG"
            ;;
        u)
            UPSTREAM="$OPTARG"
            ;;
        h)
            usage
            exit 0
            ;;

        \?)
            echo "$0: invalid option --  '$OPTARG'" >&2
            echo "Try '$0 -h' for more information." >&2
            exit 1
            ;;
        :)
            echo "$0: option requires an argument -- '$OPTARG'" >&2
            exit 1
            ;;
        *)
            echo "Unhandled getopt return:" >&2
            echo "OPT: $OPT" >&2
            echo "OPTARG: $OPTARG" >&2
            exit 1
    esac
done


shift $((OPTIND - 1))

BRANCH="${1:-HEAD}"

if ! branch_exists "$BRANCH"; then
    echo "Error: Patch branch '$BRANCH' doesn't exist" 1>&2
    exit 2
fi

if ! branch_exists "$UPSTREAM"; then
    echo "Error: Upstream branch '$UPSTREAM' doesn't exist." 1>&2
    exit 2
fi

SHA_PATCHES=`git rev-parse "$BRANCH"`
OLD_BRANCH=`git symbolic-ref --short -q HEAD`
SHA_UPSTREAM=`git rev-parse "$UPSTREAM"`

echo "Upstream branch: $UPSTREAM"
echo "Patch directory: $PATCH_DIR"

echo "Preparing detached head..."
git checkout -q --detach "$SHA_PATCHES"

# Create an exit trap to checkout the old branch when we're done
exit_trap() {
    git checkout -q "$OLD_BRANCH"
}
trap exit_trap EXIT

echo "Rebasing onto upstream master..."
git rebase "$UPSTREAM"

echo "Filtering commit messages..."
git filter-branch -f \
    --msg-filter "$MSG_FILTER" \
    "$SHA_UPSTREAM"..HEAD > /dev/null

echo "Creating patches..."
git format-patch -p -o "$PATCH_DIR" "$UPSTREAM"
