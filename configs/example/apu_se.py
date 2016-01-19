#
#  Copyright (c) 2015 Advanced Micro Devices, Inc.
#  All rights reserved.
#
#  For use for simulation and test purposes only
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors
#  may be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Sooraj Puthoor
#

import optparse, os, re
import math
import glob
import inspect

import m5
from m5.objects import *
from m5.util import addToPath

addToPath('../ruby')
addToPath('../common')
addToPath('../topologies')

import Options
import Ruby
import Simulation
import GPUTLBOptions, GPUTLBConfig

########################## Script Options ########################
def setOption(parser, opt_str, value = 1):
    # check to make sure the option actually exists
    if not parser.has_option(opt_str):
        raise Exception("cannot find %s in list of possible options" % opt_str)

    opt = parser.get_option(opt_str)
    # set the value
    exec("parser.values.%s = %s" % (opt.dest, value))

def getOption(parser, opt_str):
    # check to make sure the option actually exists
    if not parser.has_option(opt_str):
        raise Exception("cannot find %s in list of possible options" % opt_str)

    opt = parser.get_option(opt_str)
    # get the value
    exec("return_value = parser.values.%s" % opt.dest)
    return return_value

# Adding script options
parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)

parser.add_option("--cpu-only-mode", action="store_true", default=False,
                  help="APU mode. Used to take care of problems in "\
                       "Ruby.py while running APU protocols")
parser.add_option("-k", "--kernel-files",
                  help="file(s) containing GPU kernel code (colon separated)")
parser.add_option("-u", "--num-compute-units", type="int", default=1,
                  help="number of GPU compute units"),
parser.add_option("--num-cp", type="int", default=0,
                  help="Number of GPU Command Processors (CP)")
parser.add_option("--benchmark-root", help="Root of benchmark directory tree")

# not super important now, but to avoid putting the number 4 everywhere, make
# it an option/knob
parser.add_option("--cu-per-sqc", type="int", default=4, help="number of CUs" \
                  "sharing an SQC (icache, and thus icache TLB)")
parser.add_option("--simds-per-cu", type="int", default=4, help="SIMD units" \
                  "per CU")
parser.add_option("--wf-size", type="int", default=64,
                  help="Wavefront size(in workitems)")
parser.add_option("--sp-bypass-path-length", type="int", default=4, \
                  help="Number of stages of bypass path in vector ALU for Single Precision ops")
parser.add_option("--dp-bypass-path-length", type="int", default=4, \
                  help="Number of stages of bypass path in vector ALU for Double Precision ops")
# issue period per SIMD unit: number of cycles before issuing another vector
parser.add_option("--issue-period", type="int", default=4, \
                  help="Number of cycles per vector instruction issue period")
parser.add_option("--glbmem-wr-bus-width", type="int", default=32, \
                  help="VGPR to Coalescer (Global Memory) data bus width in bytes")
parser.add_option("--glbmem-rd-bus-width", type="int", default=32, \
                  help="Coalescer to VGPR (Global Memory) data bus width in bytes")
# Currently we only support 1 local memory pipe
parser.add_option("--shr-mem-pipes-per-cu", type="int", default=1, \
                  help="Number of Shared Memory pipelines per CU")
# Currently we only support 1 global memory pipe
parser.add_option("--glb-mem-pipes-per-cu", type="int", default=1, \
                  help="Number of Global Memory pipelines per CU")
parser.add_option("--wfs-per-simd", type="int", default=10, help="Number of " \
                  "WF slots per SIMD")

parser.add_option("--vreg-file-size", type="int", default=2048,
                  help="number of physical vector registers per SIMD")
parser.add_option("--bw-scalor", type="int", default=0,
                  help="bandwidth scalor for scalability analysis")
parser.add_option("--CPUClock", type="string", default="2GHz",
                  help="CPU clock")
parser.add_option("--GPUClock", type="string", default="1GHz",
                  help="GPU clock")
parser.add_option("--cpu-voltage", action="store", type="string",
                  default='1.0V',
                  help = """CPU  voltage domain""")
parser.add_option("--gpu-voltage", action="store", type="string",
                  default='1.0V',
                  help = """CPU  voltage domain""")
parser.add_option("--CUExecPolicy", type="string", default="OLDEST-FIRST",
                  help="WF exec policy (OLDEST-FIRST, ROUND-ROBIN)")
parser.add_option("--xact-cas-mode", action="store_true",
                  help="enable load_compare mode (transactional CAS)")
parser.add_option("--SegFaultDebug",action="store_true",
                 help="checks for GPU seg fault before TLB access")
parser.add_option("--FunctionalTLB",action="store_true",
                 help="Assumes TLB has no latency")
parser.add_option("--LocalMemBarrier",action="store_true",
                 help="Barrier does not wait for writethroughs to complete")
parser.add_option("--countPages", action="store_true",
                 help="Count Page Accesses and output in per-CU output files")
parser.add_option("--TLB-prefetch", type="int", help = "prefetch depth for"\
                  "TLBs")
parser.add_option("--pf-type", type="string", help="type of prefetch: "\
                  "PF_CU, PF_WF, PF_PHASE, PF_STRIDE")
parser.add_option("--pf-stride", type="int", help="set prefetch stride")
parser.add_option("--numLdsBanks", type="int", default=32,
                  help="number of physical banks per LDS module")
parser.add_option("--ldsBankConflictPenalty", type="int", default=1,
                  help="number of cycles per LDS bank conflict")


Ruby.define_options(parser)

#add TLB options to the parser
GPUTLBOptions.tlb_options(parser)

(options, args) = parser.parse_args()

# The GPU cache coherence protocols only work with the backing store
setOption(parser, "--access-backing-store")

# if benchmark root is specified explicitly, that overrides the search path
if options.benchmark_root:
    benchmark_path = [options.benchmark_root]
else:
    # Set default benchmark search path to current dir
    benchmark_path = ['.']

########################## Sanity Check ########################

# Currently the gpu model requires ruby
if buildEnv['PROTOCOL'] == 'None':
    fatal("GPU model requires ruby")

# Currently the gpu model requires only timing or detailed CPU
if not (options.cpu_type == "timing" or
   options.cpu_type == "detailed"):
    fatal("GPU model requires timing or detailed CPU")

# This file can support multiple compute units
assert(options.num_compute_units >= 1)

# Currently, the sqc (I-Cache of GPU) is shared by
# multiple compute units(CUs). The protocol works just fine
# even if sqc is not shared. Overriding this option here
# so that the user need not explicitly set this (assuming
# sharing sqc is the common usage)
n_cu = options.num_compute_units
num_sqc = int(math.ceil(float(n_cu) / options.cu_per_sqc))
options.num_sqc = num_sqc # pass this to Ruby

########################## Creating the GPU system ########################
# shader is the GPU
shader = Shader(n_wf = options.wfs_per_simd,
                clk_domain = SrcClockDomain(
                    clock = options.GPUClock,
                    voltage_domain = VoltageDomain(
                        voltage = options.gpu_voltage)))

# GPU_RfO(Read For Ownership) implements SC/TSO memory model.
# Other GPU protocols implement release consistency at GPU side.
# So, all GPU protocols other than GPU_RfO should make their writes
# visible to the global memory and should read from global memory
# during kernal boundary. The pipeline initiates(or do not initiate)
# the acquire/release operation depending on this impl_kern_boundary_sync
# flag. This flag=true means pipeline initiates a acquire/release operation
# at kernel boundary.
if buildEnv['PROTOCOL'] == 'GPU_RfO':
    shader.impl_kern_boundary_sync = False
else:
    shader.impl_kern_boundary_sync = True

# Switching off per-lane TLB by default
per_lane = False
if options.TLB_config == "perLane":
    per_lane = True

# List of compute units; one GPU can have multiple compute units
compute_units = []
for i in xrange(n_cu):
    compute_units.append(ComputeUnit(cu_id = i, perLaneTLB = per_lane,
                                     num_SIMDs = options.simds_per_cu,
                                     wfSize = options.wf_size,
                                     spbypass_pipe_length = options.sp_bypass_path_length,
                                     dpbypass_pipe_length = options.dp_bypass_path_length,
                                     issue_period = options.issue_period,
                                     coalescer_to_vrf_bus_width = \
                                     options.glbmem_rd_bus_width,
                                     vrf_to_coalescer_bus_width = \
                                     options.glbmem_wr_bus_width,
                                     num_global_mem_pipes = \
                                     options.glb_mem_pipes_per_cu,
                                     num_shared_mem_pipes = \
                                     options.shr_mem_pipes_per_cu,
                                     n_wf = options.wfs_per_simd,
                                     execPolicy = options.CUExecPolicy,
                                     xactCasMode = options.xact_cas_mode,
                                     debugSegFault = options.SegFaultDebug,
                                     functionalTLB = options.FunctionalTLB,
                                     localMemBarrier = options.LocalMemBarrier,
                                     countPages = options.countPages,
                                     localDataStore = \
                                     LdsState(banks = options.numLdsBanks,
                                              bankConflictPenalty = \
                                              options.ldsBankConflictPenalty)))
    wavefronts = []
    vrfs = []
    for j in xrange(options.simds_per_cu):
        for k in xrange(shader.n_wf):
            wavefronts.append(Wavefront(simdId = j, wf_slot_id = k))
        vrfs.append(VectorRegisterFile(simd_id=j,
                              num_regs_per_simd=options.vreg_file_size))
    compute_units[-1].wavefronts = wavefronts
    compute_units[-1].vector_register_file = vrfs
    if options.TLB_prefetch:
        compute_units[-1].prefetch_depth = options.TLB_prefetch
        compute_units[-1].prefetch_prev_type = options.pf_type

    # attach the LDS and the CU to the bus (actually a Bridge)
    compute_units[-1].ldsPort = compute_units[-1].ldsBus.slave
    compute_units[-1].ldsBus.master = compute_units[-1].localDataStore.cuPort

# Attach compute units to GPU
shader.CUs = compute_units

########################## Creating the CPU system ########################
options.num_cpus = options.num_cpus

# The shader core will be whatever is after the CPU cores are accounted for
shader_idx = options.num_cpus

# The command processor will be whatever is after the shader is accounted for
cp_idx = shader_idx + 1
cp_list = []

# List of CPUs
cpu_list = []

# We only support timing mode for shader and memory
shader.timing = True
mem_mode = 'timing'

# create the cpus
for i in range(options.num_cpus):
    cpu = None
    if options.cpu_type == "detailed":
        cpu = DerivO3CPU(cpu_id=i,
                         clk_domain = SrcClockDomain(
                             clock = options.CPUClock,
                             voltage_domain = VoltageDomain(
                                 voltage = options.cpu_voltage)))
    elif options.cpu_type == "timing":
        cpu = TimingSimpleCPU(cpu_id=i,
                              clk_domain = SrcClockDomain(
                                  clock = options.CPUClock,
                                  voltage_domain = VoltageDomain(
                                      voltage = options.cpu_voltage)))
    else:
        fatal("Atomic CPU not supported/tested")
    cpu_list.append(cpu)

# create the command processors
for i in xrange(options.num_cp):
    cp = None
    if options.cpu_type == "detailed":
        cp = DerivO3CPU(cpu_id = options.num_cpus + i,
                        clk_domain = SrcClockDomain(
                            clock = options.CPUClock,
                            voltage_domain = VoltageDomain(
                                voltage = options.cpu_voltage)))
    elif options.cpu_type == 'timing':
        cp = TimingSimpleCPU(cpu_id=options.num_cpus + i,
                             clk_domain = SrcClockDomain(
                                 clock = options.CPUClock,
                                 voltage_domain = VoltageDomain(
                                     voltage = options.cpu_voltage)))
    else:
        fatal("Atomic CPU not supported/tested")
    cp_list = cp_list + [cp]

########################## Creating the GPU dispatcher ########################
# Dispatcher dispatches work from host CPU to GPU
host_cpu = cpu_list[0]
dispatcher = GpuDispatcher()

########################## Create and assign the workload ########################
# Check for rel_path in elements of base_list using test, returning
# the first full path that satisfies test
def find_path(base_list, rel_path, test):
    for base in base_list:
        if not base:
            # base could be None if environment var not set
            continue
        full_path = os.path.join(base, rel_path)
        if test(full_path):
            return full_path
    fatal("%s not found in %s" % (rel_path, base_list))

def find_file(base_list, rel_path):
    return find_path(base_list, rel_path, os.path.isfile)

executable = find_path(benchmark_path, options.cmd, os.path.exists)
# it's common for a benchmark to be in a directory with the same
# name as the executable, so we handle that automatically
if os.path.isdir(executable):
    benchmark_path = [executable]
    executable = find_file(benchmark_path, options.cmd)
if options.kernel_files:
    kernel_files = [find_file(benchmark_path, f)
                    for f in options.kernel_files.split(':')]
else:
    # if kernel_files is not set, see if there's a unique .asm file
    # in the same directory as the executable
    kernel_path = os.path.dirname(executable)
    kernel_files = glob.glob(os.path.join(kernel_path, '*.asm'))
    if kernel_files:
        print "Using GPU kernel code file(s)", ",".join(kernel_files)
    else:
        fatal("Can't locate kernel code (.asm) in " + kernel_path)

# OpenCL driver
driver = ClDriver(filename="hsa", codefile=kernel_files)
for cpu in cpu_list:
    cpu.workload = LiveProcess(executable = executable,
                               cmd = [options.cmd] + options.options.split(),
                               drivers = [driver])
for cp in cp_list:
    cp.workload = host_cpu.workload

########################## Create the overall system ########################
# Full list of processing cores in the system. Note that
# dispatcher is also added to cpu_list although it is
# not a processing element
cpu_list = cpu_list + [shader] + cp_list + [dispatcher]

# creating the overall system
# notice the cpu list is explicitly added as a parameter to System
system = System(cpu = cpu_list,
                mem_ranges = [AddrRange(options.mem_size)],
                cache_line_size = options.cacheline_size,
                mem_mode = mem_mode)
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                   voltage_domain = system.voltage_domain)

# configure the TLB hierarchy
GPUTLBConfig.config_tlb_hierarchy(options, system, shader_idx)

# create Ruby system
system.piobus = IOXBar(width=32, response_latency=0,
                       frontend_latency=0, forward_latency=0)
Ruby.create_system(options, None, system)
system.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock,
                                    voltage_domain = system.voltage_domain)

# attach the CPU ports to Ruby
for i in range(options.num_cpus):
    ruby_port = system.ruby._cpu_ports[i]

    # Create interrupt controller
    system.cpu[i].createInterruptController()

    # Connect cache port's to ruby
    system.cpu[i].icache_port = ruby_port.slave
    system.cpu[i].dcache_port = ruby_port.slave

    ruby_port.mem_master_port = system.piobus.slave
    if buildEnv['TARGET_ISA'] == "x86":
        system.cpu[i].interrupts[0].pio = system.piobus.master
        system.cpu[i].interrupts[0].int_master = system.piobus.slave
        system.cpu[i].interrupts[0].int_slave = system.piobus.master

# attach CU ports to Ruby
# Because of the peculiarities of the CP core, you may have 1 CPU but 2
# sequencers and thus 2 _cpu_ports created. Your GPUs shouldn't be
# hooked up until after the CP. To make this script generic, figure out
# the index as below, but note that this assumes there is one sequencer
# per compute unit and one sequencer per SQC for the math to work out
# correctly.
gpu_port_idx = len(system.ruby._cpu_ports) \
               - options.num_compute_units - options.num_sqc
gpu_port_idx = gpu_port_idx - options.num_cp * 2

wavefront_size = options.wf_size
for i in xrange(n_cu):
    # The pipeline issues wavefront_size number of uncoalesced requests
    # in one GPU issue cycle. Hence wavefront_size mem ports.
    for j in xrange(wavefront_size):
        system.cpu[shader_idx].CUs[i].memory_port[j] = \
                  system.ruby._cpu_ports[gpu_port_idx].slave[j]
    gpu_port_idx += 1

for i in xrange(n_cu):
    if i > 0 and not i % options.cu_per_sqc:
        print "incrementing idx on ", i
        gpu_port_idx += 1
    system.cpu[shader_idx].CUs[i].sqc_port = \
            system.ruby._cpu_ports[gpu_port_idx].slave
gpu_port_idx = gpu_port_idx + 1

# attach CP ports to Ruby
for i in xrange(options.num_cp):
    system.cpu[cp_idx].createInterruptController()
    system.cpu[cp_idx].dcache_port = \
                system.ruby._cpu_ports[gpu_port_idx + i * 2].slave
    system.cpu[cp_idx].icache_port = \
                system.ruby._cpu_ports[gpu_port_idx + i * 2 + 1].slave
    system.cpu[cp_idx].interrupts[0].pio = system.piobus.master
    system.cpu[cp_idx].interrupts[0].int_master = system.piobus.slave
    system.cpu[cp_idx].interrupts[0].int_slave = system.piobus.master
    cp_idx = cp_idx + 1

# connect dispatcher to the system.piobus
dispatcher.pio = system.piobus.master
dispatcher.dma = system.piobus.slave

################# Connect the CPU and GPU via GPU Dispatcher ###################
# CPU rings the GPU doorbell to notify a pending task
# using this interface.
# And GPU uses this interface to notify the CPU of task completion
# The communcation happens through emulated driver.

# Note this implicit setting of the cpu_pointer, shader_pointer and tlb array
# parameters must be after the explicit setting of the System cpu list
shader.cpu_pointer = host_cpu
dispatcher.cpu = host_cpu
dispatcher.shader_pointer = shader
dispatcher.cl_driver = driver

########################## Start simulation ########################

root = Root(system=system, full_system=False)
m5.ticks.setGlobalFrequency('1THz')
if options.abs_max_tick:
    maxtick = options.abs_max_tick
else:
    maxtick = m5.MaxTick

# Benchmarks support work item annotations
Simulation.setWorkCountOptions(system, options)

# Checkpointing is not supported by APU model
if (options.checkpoint_dir != None or
    options.checkpoint_restore != None):
    fatal("Checkpointing not supported by apu model")

checkpoint_dir = None
m5.instantiate(checkpoint_dir)

# Map workload to this address space
host_cpu.workload[0].map(0x10000000, 0x200000000, 4096)

exit_event = m5.simulate(maxtick)
print "Ticks:", m5.curTick()
print 'Exiting because ', exit_event.getCause()
sys.exit(exit_event.getCode())
