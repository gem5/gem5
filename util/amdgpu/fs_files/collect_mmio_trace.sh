# Copyright (c) 2021 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# This script collects the MMIO trace during initialization of GPU driver.
# Before running this, we need to disable the drm module in use.
# For AMD GPUs do either:
#     echo 'blacklist amdgpu' >> /etc/modprobe.d/blacklist.conf
# or
#     add 'modprobe.blacklist=amdgpu' to your kernel command
# And then reboot.

# First load the drm_kms_helper with fbdev_emulation=0
modprobe drm_kms_helper fbdev_emulation=0

# Path to store the trace
TRACE_PATH="MMIO.trace"

# Choose propoer size to make sure you won’t miss any events
BUFFER_SIZE=8000

DRM_MODULE="amdgpu"
DRM_MODULE_ARGS="ip_block_mask=0xff"

# Setup MMIO tracer
echo $BUFFER_SIZE > /sys/kernel/debug/tracing/buffer_size_kb
echo mmiotrace > /sys/kernel/debug/tracing/current_tracer

# Re-direct the output to a file, process in background
cat /sys/kernel/debug/tracing/trace_pipe > $TRACE_PATH &
# Save the process ID, as we need to kill it later on
PID=$!

# Insert the module, start driver initialization
echo "Running: modprobe -v $DRM_MODULE $DRM_MODULE_ARGS"
modprobe -v $DRM_MODULE $DRM_MODULE_ARGS

# Kill the background process after module is loaded
kill $PID
