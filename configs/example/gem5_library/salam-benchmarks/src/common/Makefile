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
# Authors: 	Matthias Jung
#			Frederik Lauer

CROSS_COMPILE_DIR = /usr/bin

ARCH          ?= arm
CROSS_COMPILE ?= arm-none-eabi-
#CROSS_COMPILE_DIR = /home/samerogers/aarch64-toolchain/bin

#ARCH          ?= aarch64
#CROSS_COMPILE ?= aarch64-elf-

AS           = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)as
LD           = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)ld
CC           = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)gcc
CXX          = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)g++
CPP          = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)cpp
AR           = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)ar
NM           = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)nm
STRIP        = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)strip
OBJCOPY      = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)objcopy
OBJDUMP      = $(CROSS_COMPILE_DIR)/$(CROSS_COMPILE)objdump

CFLAGS       = -g -O3 -lunistd -I .
CXXFLAGS     = -g -O3 -lunistd -I . -fno-exceptions
ASFLAGS      = -EL
LNK_OPT      = -nostartfiles
LNK_SCRIPT   = boot.ld
#LNK_SCRIPT   = start64.ld
LNK_FILE_OPT = -nostartfiles -Xlinker -T$(LNK_SCRIPT)
