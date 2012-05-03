# Copyright (c) 2006-2008 The Regents of The University of Michigan
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
#
# Authors: Lisa Hsu

import m5
from m5.defines import buildEnv
from m5.objects import *
from Benchmarks import *

def addCommonOptions(parser):
    # system options
    parser.add_option("--cpu-type", type="choice", default="atomic",
                      choices = ["atomic", "timing", "detailed", "inorder",
                                 "arm_detailed"],
                      help = "type of cpu to run with")
    parser.add_option("--checker", action="store_true");
    parser.add_option("-n", "--num-cpus", type="int", default=1)
    parser.add_option("--caches", action="store_true")
    parser.add_option("--l2cache", action="store_true")
    parser.add_option("--fastmem", action="store_true")
    parser.add_option("--clock", action="store", type="string", default='2GHz')
    parser.add_option("--num-dirs", type="int", default=1)
    parser.add_option("--num-l2caches", type="int", default=1)
    parser.add_option("--num-l3caches", type="int", default=1)
    parser.add_option("--l1d_size", type="string", default="64kB")
    parser.add_option("--l1i_size", type="string", default="32kB")
    parser.add_option("--l2_size", type="string", default="2MB")
    parser.add_option("--l3_size", type="string", default="16MB")
    parser.add_option("--l1d_assoc", type="int", default=2)
    parser.add_option("--l1i_assoc", type="int", default=2)
    parser.add_option("--l2_assoc", type="int", default=8)
    parser.add_option("--l3_assoc", type="int", default=16)
    parser.add_option("--cacheline_size", type="int", default=64)
    parser.add_option("--ruby", action="store_true")

    # Run duration options
    parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                      metavar="T", help="Stop after T ticks")
    parser.add_option("--maxtime", type="float")
    parser.add_option("-I", "--maxinsts", action="store", type="int",
                      default=None, help="""Total number of instructions to
                                            simulate (default: run forever)""")
    parser.add_option("--work-item-id", action="store", type="int",
                      help="the specific work id for exit & checkpointing")
    parser.add_option("--work-begin-cpu-id-exit", action="store", type="int",
                      help="exit when work starts on the specified cpu")
    parser.add_option("--work-end-exit-count", action="store", type="int",
                      help="exit at specified work end count")
    parser.add_option("--work-begin-exit-count", action="store", type="int",
                      help="exit at specified work begin count")
    parser.add_option("--init-param", action="store", type="int", default=0,
                      help="""Parameter available in simulation with m5
                              initparam""")

    # Checkpointing options
    ###Note that performing checkpointing via python script files will override
    ###checkpoint instructions built into binaries.
    parser.add_option("--take-checkpoints", action="store", type="string",
        help="<M,N> will take checkpoint at cycle M and every N cycles thereafter")
    parser.add_option("--max-checkpoints", action="store", type="int",
        help="the maximum number of checkpoints to drop", default=5)
    parser.add_option("--checkpoint-dir", action="store", type="string",
        help="Place all checkpoints in this absolute directory")
    parser.add_option("-r", "--checkpoint-restore", action="store", type="int",
        help="restore from checkpoint <N>")
    parser.add_option("--checkpoint-at-end", action="store_true",
                      help="take a checkpoint at end of run")
    parser.add_option("--work-begin-checkpoint-count", action="store", type="int",
                      help="checkpoint at specified work begin count")
    parser.add_option("--work-end-checkpoint-count", action="store", type="int",
                      help="checkpoint at specified work end count")
    parser.add_option("--work-cpus-checkpoint-count", action="store", type="int",
                      help="checkpoint and exit when active cpu count is reached")
    parser.add_option("--restore-with-cpu", action="store", type="choice",
                      default="atomic", choices = ["atomic", "timing",
                                                   "detailed", "inorder"],
                      help = "cpu type for restoring from a checkpoint")


    # CPU Switching - default switch model goes from a checkpoint
    # to a timing simple CPU with caches to warm up, then to detailed CPU for
    # data measurement
    parser.add_option("-s", "--standard-switch", action="store_true",
        help="switch from timing CPU to Detailed CPU")
    parser.add_option("-w", "--warmup", action="store", type="int",
        help="if -s, then this is the warmup period.  else, this is ignored",
        default=5000000000)
    parser.add_option("-p", "--prog-interval", type="int",
        help="CPU Progress Interval")

    # Fastforwarding and simpoint related materials
    parser.add_option("-W", "--warmup-insts", action="store", type="int",
        default=None,
        help="Warmup period in total instructions (requires --standard-switch)")
    parser.add_option("--bench", action="store", type="string", default=None,
        help="base names for --take-checkpoint and --checkpoint-restore")
    parser.add_option("-F", "--fast-forward", action="store", type="string",
        default=None,
        help="Number of instructions to fast forward before switching")
    parser.add_option("-S", "--simpoint", action="store_true", default=False,
        help="""Use workload simpoints as an instruction offset for
                --checkpoint-restore or --take-checkpoint.""")
    parser.add_option("--at-instruction", action="store_true", default=False,
        help="""Treat value of --checkpoint-restore or --take-checkpoint as a
                number of instructions.""")

def addSEOptions(parser):
    # Benchmark options
    parser.add_option("-c", "--cmd", default="",
                      help="The binary to run in syscall emulation mode.")
    parser.add_option("-o", "--options", default="",
                      help="""The options to pass to the binary, use " "
                              around the entire string""")
    parser.add_option("-i", "--input", default="",
                      help="Read stdin from a file.")
    parser.add_option("--output", default="",
                      help="Redirect stdout to a file.")
    parser.add_option("--errout", default="",
                      help="Redirect stderr to a file.")

def addFSOptions(parser):
    # Simulation options
    parser.add_option("--timesync", action="store_true",
            help="Prevent simulated time from getting ahead of real time")

    # System options
    parser.add_option("--kernel", action="store", type="string")
    parser.add_option("--script", action="store", type="string")
    parser.add_option("--frame-capture", action="store_true",
            help="Stores changed frame buffers from the VNC server to compressed "\
            "files in the gem5 output directory")

    if buildEnv['TARGET_ISA'] == "arm":
        parser.add_option("--bare-metal", action="store_true",
                   help="Provide the raw system without the linux specific bits")
        parser.add_option("--machine-type", action="store", type="choice",
                choices=ArmMachineType.map.keys(), default="RealView_PBX")
    # Benchmark options
    parser.add_option("--dual", action="store_true",
                      help="Simulate two systems attached with an ethernet link")
    parser.add_option("-b", "--benchmark", action="store", type="string",
                      dest="benchmark",
                      help="Specify the benchmark to run. Available benchmarks: %s"\
                      % DefinedBenchmarks)

    # Metafile options
    parser.add_option("--etherdump", action="store", type="string", dest="etherdump",
                      help="Specify the filename to dump a pcap capture of the" \
                      "ethernet traffic")

    # Disk Image Options
    parser.add_option("--disk-image", action="store", type="string", default=None,
                      help="Path to the disk image to use.")

    # Memory Size Options
    parser.add_option("--mem-size", action="store", type="string", default=None,
                      help="Specify the physical memory size (single memory)")
