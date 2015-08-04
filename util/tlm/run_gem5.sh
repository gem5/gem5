#!/bin/bash
# Copyright (c) 2015, University of Kaiserslautern
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
# OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# Authors: Matthias Jung

# Color Definition:
RCol='\e[0m'; # Text Reset
BGre='\e[1;31m';
echo -e "\n${BGre}Create gem5 Configuration${RCol}\n"

../../build/ARM/gem5.opt ../../configs/example/fs.py        \
--tlm-memory=memory                                         \
--cpu-type=timing                                           \
--num-cpu=1                                                 \
--mem-type=SimpleMemory                                     \
--mem-size=512MB                                            \
--mem-channels=1                                            \
--caches --l2cache                                          \
--machine-type=VExpress_EMM                                 \
--dtb-filename=vexpress.aarch32.ll_20131205.0-gem5.1cpu.dtb \
--kernel=vmlinux.aarch32.ll_20131205.0-gem5

echo -e "\n${BGre}Run gem5 ${RCol}\n"

time ./gem5.opt.sc m5out/config.ini -o 2147483648 
