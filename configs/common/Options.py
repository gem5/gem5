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

# system options
parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("--inorder", action="store_true")
parser.add_option("-n", "--num-cpus", type="int", default=1)
parser.add_option("--caches", action="store_true")
parser.add_option("--l2cache", action="store_true")
parser.add_option("--fastmem", action="store_true")
parser.add_option("--clock", action="store", type="string", default='1GHz')
parser.add_option("--num-dirs", type="int", default=1)
parser.add_option("--num-l2caches", type="int", default=1)
parser.add_option("--topology", type="string", default="Crossbar",
                  help="check src/mem/ruby/network/topologies for complete set")
parser.add_option("--mesh-rows", type="int", default=1,
                  help="the number of rows in the mesh topology")
parser.add_option("--garnet-network", type="string", default=None,
                  help="'fixed'|'flexible'")
parser.add_option("--numa-high-bit", type="int", default=None,
                  help="high order address bit to use for numa mapping")

# ruby sparse memory options
parser.add_option("--use-map", action="store_true", default=False)
parser.add_option("--map-levels", type="int", default=4)

# Run duration options
parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                  metavar="T",
                  help="Stop after T ticks")
parser.add_option("--maxtime", type="float")
parser.add_option("--maxinsts", type="int")
parser.add_option("--prog_intvl", type="int")


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


# CPU Switching - default switch model goes from a checkpoint
# to a timing simple CPU with caches to warm up, then to detailed CPU for
# data measurement
parser.add_option("-s", "--standard-switch", action="store_true",
    help="switch from timing CPU to Detailed CPU")
parser.add_option("-w", "--warmup", action="store", type="int",
    help="if -s, then this is the warmup period.  else, this is ignored",
    default=5000000000)
parser.add_option("--profile", help="CPU profile interval")

# Fastforwarding and simpoint related materials
parser.add_option("-W", "--warmup-insts", action="store", type="int",
    default=None,
    help="Warmup period in total instructions (requires --standard-switch)")
parser.add_option("-I", "--max-inst", action="store", type="int", default=None,
    help="Total number of instructions to simulate (default: run forever)")
parser.add_option("--bench", action="store", type="string", default=None,
    help="base names for --take-checkpoint and --checkpoint-restore")
parser.add_option("-F", "--fast-forward", action="store", type="string",
    default=None,
    help="Number of instructions to fast forward before switching")
parser.add_option("-S", "--simpoint", action="store_true", default=False,
    help="""Use workload simpoints as an instruction offset for
--checkpoint-restore or --take-checkpoint.""")
parser.add_option("--at-instruction", action="store_true", default=False,
    help="""Treate value of --checkpoint-restore or --take-checkpoint as a
number of instructions.""")
