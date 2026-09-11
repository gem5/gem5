#!/bin/bash
# Copyright 2025 Google, Inc.
# All Rights Reserved.
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

INPUT=$1
TARGET_NAME=$2
OUTPUT_DIR=$3
SANDBOX=$(pwd)

if [ "${INPUT:0:1}" = "/" ]
then
  _input=$INPUT
else
  _input=$SANDBOX/$INPUT
fi
tmpdir=$(mktemp -d)
input_file=$(basename $_input)
pushd $tmpdir
cat <<EOF > SConstruct
AddOption('--no-colors', dest='use_colors', action='store_false')
AddOption('--verbose', action='store_true')
from gem5_scons.builders import Blob
env = Environment(tools=[Blob])
env['BUILDDIR'] = ''
cc, hh = env.Blob('$TARGET_NAME', '$input_file')
EOF
ln -s $SANDBOX/site_scons .
ln -s $_input .
PYTHONPATH=$SANDBOX/src/python:$SANDBOX/build_tools scons
popd
mv $tmpdir/$TARGET_NAME.cc $tmpdir/$TARGET_NAME.hh $OUTPUT_DIR
rm -rf $tmpdir
