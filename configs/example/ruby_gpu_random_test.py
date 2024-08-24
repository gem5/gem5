# Copyright (c) 2018-2021 Advanced Micro Devices, Inc.
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

import argparse
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

addToPath("../")

from common import Options
from ruby import Ruby

#
# Add the ruby specific and protocol specific options
#
parser = argparse.ArgumentParser()
Options.addNoISAOptions(parser)
Ruby.define_options(parser)

# GPU Ruby tester options
parser.add_argument(
    "--cache-size",
    default="small",
    choices=["small", "large"],
    help="Cache sizes to use. Small encourages races between \
                        requests and writebacks. Large stresses write-through \
                        and/or write-back GPU caches.",
)
parser.add_argument(
    "--system-size",
    default="small",
    choices=["small", "medium", "large"],
    help="This option defines how many CUs, CPUs and cache \
                        components in the test system.",
)
parser.add_argument(
    "--address-range",
    default="small",
    choices=["small", "large"],
    help="This option defines the number of atomic \
                        locations that affects the working set's size. \
                        A small number of atomic locations encourage more \
                        races among threads. The large option stresses cache \
                        resources.",
)
parser.add_argument(
    "--episode-length",
    default="short",
    choices=["short", "medium", "long"],
    help="This option defines the number of LDs and \
                        STs in an episode. The small option encourages races \
                        between the start and end of an episode. The long \
                        option encourages races between LDs and STs in the \
                        same episode.",
)
parser.add_argument(
    "--test-length",
    type=int,
    default=1,
    help="The number of episodes to be executed by each \
                        wavefront. This determines the maximum number, i.e., \
                        val X #WFs, of episodes to be executed in the test.",
)
parser.add_argument(
    "--debug-tester",
    action="store_true",
    help="This option will turn on DRF checker",
)
parser.add_argument(
    "--random-seed",
    type=int,
    default=0,
    help="Random seed number. Default value (i.e., 0) means \
                        using runtime-specific value",
)
parser.add_argument("--log-file", type=str, default="gpu-ruby-test.log")
parser.add_argument(
    "--num-dmas",
    type=int,
    default=None,
    help="The number of DMA engines to use in tester config.",
)

args = parser.parse_args()

#
# Set up cache size - 2 options
#   0: small cache
#   1: large cache
#
if args.cache_size == "small":
    args.tcp_size = "256B"
    args.tcp_assoc = 2
    args.tcc_size = "1kB"
    args.tcc_assoc = 2
elif args.cache_size == "large":
    args.tcp_size = "256kB"
    args.tcp_assoc = 16
    args.tcc_size = "1024kB"
    args.tcc_assoc = 16

#
# Set up system size - 3 options
#
if args.system_size == "small":
    # 1 CU, 1 CPU, 1 SQC, 1 Scalar
    args.wf_size = 1
    args.wavefronts_per_cu = 1
    args.num_cpus = 1
    n_DMAs = 1
    args.cu_per_sqc = 1
    args.cu_per_scalar_cache = 1
    args.num_compute_units = 1
elif args.system_size == "medium":
    # 4 CUs, 4 CPUs, 1 SQCs, 1 Scalars
    args.wf_size = 16
    args.wavefronts_per_cu = 4
    args.num_cpus = 4
    n_DMAs = 2
    args.cu_per_sqc = 4
    args.cu_per_scalar_cache = 4
    args.num_compute_units = 4
elif args.system_size == "large":
    # 8 CUs, 4 CPUs, 1 SQCs, 1 Scalars
    args.wf_size = 32
    args.wavefronts_per_cu = 4
    args.num_cpus = 4
    n_DMAs = 4
    args.cu_per_sqc = 4
    args.cu_per_scalar_cache = 4
    args.num_compute_units = 8

# Number of DMA engines
if not (args.num_dmas is None):
    n_DMAs = args.num_dmas
    # currently the tester does not support requests returned as
    # aliased, thus we need num_dmas to be 0 for it
    if not (args.num_dmas == 0):
        print("WARNING: num_dmas != 0 not supported with VIPER")

#
# Set address range - 2 options
#   level 0: small
#   level 1: large
# Each location corresponds to a 4-byte piece of data
#
args.mem_size = "1024MiB"
if args.address_range == "small":
    num_atomic_locs = 10
    num_regular_locs_per_atomic_loc = 10000
elif args.address_range == "large":
    num_atomic_locs = 100
    num_regular_locs_per_atomic_loc = 100000

#
# Set episode length (# of actions per episode) - 3 options
#   0: 10 actions
#   1: 100 actions
#   2: 500 actions
#
if args.episode_length == "short":
    eps_length = 10
elif args.episode_length == "medium":
    eps_length = 100
elif args.episode_length == "long":
    eps_length = 500

#
# Set Ruby and tester deadlock thresholds. Ruby's deadlock detection is the
# primary check for deadlocks. The tester's deadlock threshold detection is
# a secondary check for deadlock. If there is a bug in RubyPort that causes
# a packet not to return to the tester properly, the tester will issue a
# deadlock panic. We set cache_deadlock_threshold < tester_deadlock_threshold
# to detect deadlock caused by Ruby protocol first before one caused by the
# coalescer. Both units are in Ticks
#
args.cache_deadlock_threshold = 1e8
tester_deadlock_threshold = 1e9

# For now we're testing only GPU protocol, so we force num_cpus to be 0
args.num_cpus = 0

# Number of CUs
n_CUs = args.num_compute_units

# Set test length, i.e., number of episodes per wavefront * #WFs.
# Test length can be 1x#WFs, 10x#WFs, 100x#WFs, ...
n_WFs = n_CUs * args.wavefronts_per_cu
max_episodes = args.test_length * n_WFs

# Number of SQC and Scalar caches
assert n_CUs % args.cu_per_sqc == 0
n_SQCs = n_CUs // args.cu_per_sqc
args.num_sqc = n_SQCs

assert args.cu_per_scalar_cache != 0
n_Scalars = n_CUs // args.cu_per_scalar_cache
args.num_scalar_cache = n_Scalars

#
# Create GPU Ruby random tester
#
tester = ProtocolTester(
    cus_per_sqc=args.cu_per_sqc,
    cus_per_scalar=args.cu_per_scalar_cache,
    wavefronts_per_cu=args.wavefronts_per_cu,
    workitems_per_wavefront=args.wf_size,
    num_atomic_locations=num_atomic_locs,
    num_normal_locs_per_atomic=num_regular_locs_per_atomic_loc,
    max_num_episodes=max_episodes,
    episode_length=eps_length,
    debug_tester=args.debug_tester,
    random_seed=args.random_seed,
    log_file=args.log_file,
)

#
# Create a gem5 system. Note that the memory object isn't actually used by the
# tester, but is included to ensure the gem5 memory size == Ruby memory size
# checks. The system doesn't have real CPUs or CUs. It just has a tester that
# has physical ports to be connected to Ruby
#
system = System(
    cpu=tester,
    mem_ranges=[AddrRange(args.mem_size)],
    cache_line_size=args.cacheline_size,
    mem_mode="timing",
)

system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

#
# Command processor is not needed for the tester since we don't run real
# kernels. Setting it to zero disables the VIPER protocol from creating
# a command processor and its caches.
#
args.num_cp = 0

#
# Make generic DMA sequencer for Ruby to use
#
if n_DMAs > 0:
    dma_devices = [TesterDma()] * n_DMAs
    system.piobus = IOXBar()
    for _, dma_device in enumerate(dma_devices):
        dma_device.pio = system.piobus.mem_side_ports
    system.dma_devices = dma_devices

#
# Create the Ruby system
#
# the ruby tester reuses num_cpus to specify the
# number of cpu ports connected to the tester object, which
# is stored in system.cpu. because there is only ever one
# tester object, num_cpus is not necessarily equal to the
# size of system.cpu
cpu_list = [system.cpu] * args.num_cpus
Ruby.create_system(
    args,
    full_system=False,
    system=system,
    dma_ports=system.dma_devices if n_DMAs > 0 else [],
    cpus=cpu_list,
)

#
# The tester is most effective when randomization is turned on and
# artifical delay is randomly inserted on messages
#
system.ruby.randomization = True

# Assert that we got the right number of Ruby ports
assert len(system.ruby._cpu_ports) == n_CUs + n_SQCs + n_Scalars

#
# Attach Ruby ports to the tester in the order:
#               cpu_sequencers,
#               vector_coalescers,
#               sqc_sequencers,
#               scalar_sequencers
#
# Note that this requires the protocol to create sequencers in this order
#
print("Attaching ruby ports to the tester")
for i, ruby_port in enumerate(system.ruby._cpu_ports):
    ruby_port.no_retry_on_stall = True
    ruby_port.using_ruby_tester = True

    # piobus is only created if there are DMAs
    if n_DMAs > 0:
        ruby_port.mem_request_port = system.piobus.cpu_side_ports

    if i < n_CUs:
        tester.cu_vector_ports = ruby_port.in_ports
        tester.cu_token_ports = ruby_port.gmTokenPort
        tester.max_cu_tokens = 4 * n_WFs
    elif i < (n_CUs + n_SQCs):
        tester.cu_sqc_ports = ruby_port.in_ports
    else:
        tester.cu_scalar_ports = ruby_port.in_ports

    i += 1

#
# Attach DMA ports. Since Ruby.py doesn't return these they need to be found.
# Connect tester's request port to each DMA sequencer's in_ports. This assumes
# the protocol names these system.dma_cntrl<#>.
#
dma_ports = []
for i in range(n_DMAs):
    dma_cntrl = getattr(system, "dma_cntrl" + str(i))
    dma_ports.append(dma_cntrl.dma_sequencer.in_ports)
tester.dma_ports = dma_ports

#
# Common variables for all types of threads
#
thread_clock = SrcClockDomain(
    clock="1GHz", voltage_domain=system.voltage_domain
)
g_thread_idx = 0

#
# No CPU threads are used for GPU tester
#
tester.cpu_threads = []

#
# Create DMA threads
#
dma_threads = []
print("Creating %i DMAs" % n_DMAs)
for dma_idx in range(n_DMAs):
    dma_threads.append(
        DmaThread(
            thread_id=g_thread_idx,
            num_lanes=1,
            clk_domain=thread_clock,
            deadlock_threshold=tester_deadlock_threshold,
        )
    )
    g_thread_idx += 1
tester.dma_threads = dma_threads

#
# Create GPU wavefronts
#
wavefronts = []
print(
    "Creating %i WFs attached to %i CUs"
    % (n_CUs * tester.wavefronts_per_cu, n_CUs)
)
for cu_idx in range(n_CUs):
    for wf_idx in range(tester.wavefronts_per_cu):
        wavefronts.append(
            GpuWavefront(
                thread_id=g_thread_idx,
                cu_id=cu_idx,
                num_lanes=args.wf_size,
                clk_domain=thread_clock,
                deadlock_threshold=tester_deadlock_threshold,
            )
        )
        g_thread_idx += 1
tester.wavefronts = wavefronts

#
# Run simulation
#
root = Root(full_system=False, system=system)

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency("1ns")

# Instantiate configuration
m5.instantiate()

# Simulate until tester completes
exit_event = m5.simulate()

print("Exiting tick: ", m5.curTick())
print("Exiting because ", exit_event.getCause())
