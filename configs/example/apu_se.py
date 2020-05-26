# Copyright (c) 2015 Advanced Micro Devices, Inc.
# All rights reserved.
#
# For use for simulation and test purposes only
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

from __future__ import print_function
from __future__ import absolute_import

import optparse, os, re, getpass
import math
import glob
import inspect

import m5
from m5.objects import *
from m5.util import addToPath

addToPath('../')

from ruby import Ruby

from common import Options
from common import Simulation
from common import GPUTLBOptions, GPUTLBConfig

import hsaTopology
from common import FileSystemConfig

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
parser.add_option("-u", "--num-compute-units", type="int", default=4,
                  help="number of GPU compute units"),
parser.add_option("--num-cp", type="int", default=0,
                  help="Number of GPU Command Processors (CP)")
parser.add_option("--benchmark-root", help="Root of benchmark directory tree")

# not super important now, but to avoid putting the number 4 everywhere, make
# it an option/knob
parser.add_option("--cu-per-sqc", type="int", default=4, help="number of CUs" \
                  "sharing an SQC (icache, and thus icache TLB)")
parser.add_option('--cu-per-scalar-cache', type='int', default=4,
                  help='Number of CUs sharing a scalar cache')
parser.add_option("--simds-per-cu", type="int", default=4, help="SIMD units" \
                  "per CU")
parser.add_option('--cu-per-sa', type='int', default=4,
                  help='Number of CUs per shader array. This must be a '
                  'multiple of options.cu-per-sqc and options.cu-per-scalar')
parser.add_option('--sa-per-complex', type='int', default=1,
                  help='Number of shader arrays per complex')
parser.add_option('--num-gpu-complexes', type='int', default=1,
                  help='Number of GPU complexes')
parser.add_option("--wf-size", type="int", default=64,
                  help="Wavefront size(in workitems)")
parser.add_option("--sp-bypass-path-length", type="int", default=4, \
                  help="Number of stages of bypass path in vector ALU for "
                  "Single Precision ops")
parser.add_option("--dp-bypass-path-length", type="int", default=4, \
                  help="Number of stages of bypass path in vector ALU for "
                  "Double Precision ops")
# issue period per SIMD unit: number of cycles before issuing another vector
parser.add_option("--issue-period", type="int", default=4, \
                  help="Number of cycles per vector instruction issue period")
parser.add_option("--glbmem-wr-bus-width", type="int", default=32, \
                  help="VGPR to Coalescer (Global Memory) data bus width "
                  "in bytes")
parser.add_option("--glbmem-rd-bus-width", type="int", default=32, \
                  help="Coalescer to VGPR (Global Memory) data bus width in "
                  "bytes")
# Currently we only support 1 local memory pipe
parser.add_option("--shr-mem-pipes-per-cu", type="int", default=1, \
                  help="Number of Shared Memory pipelines per CU")
# Currently we only support 1 global memory pipe
parser.add_option("--glb-mem-pipes-per-cu", type="int", default=1, \
                  help="Number of Global Memory pipelines per CU")
parser.add_option("--wfs-per-simd", type="int", default=10, help="Number of " \
                  "WF slots per SIMD")

parser.add_option("--registerManagerPolicy", type="string", default="static",
                  help="Register manager policy")
parser.add_option("--vreg-file-size", type="int", default=2048,
                  help="number of physical vector registers per SIMD")
parser.add_option("--vreg-min-alloc", type="int", default=4,
                  help="Minimum number of registers that can be allocated "
                  "from the VRF. The total number of registers will be "
                  "aligned to this value.")

parser.add_option("--sreg-file-size", type="int", default=2048,
                  help="number of physical vector registers per SIMD")
parser.add_option("--sreg-min-alloc", type="int", default=4,
                  help="Minimum number of registers that can be allocated "
                  "from the SRF. The total number of registers will be "
                  "aligned to this value.")

parser.add_option("--bw-scalor", type="int", default=0,
                  help="bandwidth scalor for scalability analysis")
parser.add_option("--CPUClock", type="string", default="2GHz",
                  help="CPU clock")
parser.add_option("--gpu-clock", type="string", default="1GHz",
                  help="GPU clock")
parser.add_option("--cpu-voltage", action="store", type="string",
                  default='1.0V',
                  help = """CPU  voltage domain""")
parser.add_option("--gpu-voltage", action="store", type="string",
                  default='1.0V',
                  help = """CPU  voltage domain""")
parser.add_option("--CUExecPolicy", type="string", default="OLDEST-FIRST",
                  help="WF exec policy (OLDEST-FIRST, ROUND-ROBIN)")
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
parser.add_options("--lds-size", type="int", default=65536,
                   help="Size of the LDS in bytes")
parser.add_option('--fast-forward-pseudo-op', action='store_true',
                  help = 'fast forward using kvm until the m5_switchcpu'
                  ' pseudo-op is encountered, then switch cpus. subsequent'
                  ' m5_switchcpu pseudo-ops will toggle back and forth')
parser.add_option("--num-hw-queues", type="int", default=10,
                  help="number of hw queues in packet processor")

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
if not (options.cpu_type == "TimingSimpleCPU" or
   options.cpu_type == "DerivO3CPU"):
    fatal("GPU model requires TimingSimpleCPU or DerivO3CPU")

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
num_scalar_cache = int(math.ceil(float(n_cu) / options.cu_per_scalar_cache))
options.num_scalar_cache = num_scalar_cache

print('Num SQC = ', num_sqc, 'Num scalar caches = ', num_scalar_cache,
      'Num CU = ', n_cu)

########################## Creating the GPU system ########################
# shader is the GPU
shader = Shader(n_wf = options.wfs_per_simd,
                clk_domain = SrcClockDomain(
                    clock = options.gpu_clock,
                    voltage_domain = VoltageDomain(
                        voltage = options.gpu_voltage)))

# GPU_RfO(Read For Ownership) implements SC/TSO memory model.
# Other GPU protocols implement release consistency at GPU side.
# So, all GPU protocols other than GPU_RfO should make their writes
# visible to the global memory and should read from global memory
# during kernal boundary. The pipeline initiates(or do not initiate)
# the acquire/release operation depending on these impl_kern_launch_rel
# and impl_kern_end_rel flags.  The flag=true means pipeline initiates
# a acquire/release operation at kernel launch/end.
# VIPER protocols (GPU_VIPER, GPU_VIPER_Region and GPU_VIPER_Baseline)
# are write-through based, and thus only imple_kern_launch_acq needs to
# set.
if buildEnv['PROTOCOL'] == 'GPU_RfO':
    shader.impl_kern_launch_acq = False
    shader.impl_kern_end_rel = False
elif (buildEnv['PROTOCOL'] != 'GPU_VIPER' or
        buildEnv['PROTOCOL'] != 'GPU_VIPER_Region' or
        buildEnv['PROTOCOL'] != 'GPU_VIPER_Baseline'):
    shader.impl_kern_launch_acq = True
    shader.impl_kern_end_rel = False
else:
    shader.impl_kern_launch_acq = True
    shader.impl_kern_end_rel = True

# Switching off per-lane TLB by default
per_lane = False
if options.TLB_config == "perLane":
    per_lane = True

# List of compute units; one GPU can have multiple compute units
compute_units = []
for i in range(n_cu):
    compute_units.append(ComputeUnit(cu_id = i, perLaneTLB = per_lane,
                                     num_SIMDs = options.simds_per_cu,
                                     wf_size = options.wf_size,
                                     spbypass_pipe_length = \
                                     options.sp_bypass_path_length,
                                     dpbypass_pipe_length = \
                                     options.dp_bypass_path_length,
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
                                     debugSegFault = options.SegFaultDebug,
                                     functionalTLB = options.FunctionalTLB,
                                     localMemBarrier = options.LocalMemBarrier,
                                     countPages = options.countPages,
                                     localDataStore = \
                                     LdsState(banks = options.numLdsBanks,
                                              bankConflictPenalty = \
                                              options.ldsBankConflictPenalty,
                                              size = options.lds_size)))
    wavefronts = []
    vrfs = []
    vrf_pool_mgrs = []
    srfs = []
    srf_pool_mgrs = []
    for j in xrange(options.simds_per_cu):
        for k in xrange(shader.n_wf):
            wavefronts.append(Wavefront(simdId = j, wf_slot_id = k,
                                        wf_size = options.wf_size))
        vrf_pool_mgrs.append(SimplePoolManager(pool_size = \
                                               options.vreg_file_size,
                                               min_alloc = \
                                               options.vreg_min_alloc))

        vrfs.append(VectorRegisterFile(simd_id=j, wf_size=options.wf_size,
                                       num_regs=options.vreg_file_size))

        srf_pool_mgrs.append(SimplePoolManager(pool_size = \
                                               options.sreg_file_size,
                                               min_alloc = \
                                               options.vreg_min_alloc))
        srfs.append(ScalarRegisterFile(simd_id=j, wf_size=options.wf_size,
                                       num_regs=options.sreg_file_size))

    compute_units[-1].wavefronts = wavefronts
    compute_units[-1].vector_register_file = vrfs
    compute_units[-1].scalar_register_file = srfs
    compute_units[-1].register_manager = \
        RegisterManager(policy=options.registerManagerPolicy,
                        vrf_pool_managers=vrf_pool_mgrs,
                        srf_pool_managers=srf_pool_mgrs)
    if options.TLB_prefetch:
        compute_units[-1].prefetch_depth = options.TLB_prefetch
        compute_units[-1].prefetch_prev_type = options.pf_type

    # attach the LDS and the CU to the bus (actually a Bridge)
    compute_units[-1].ldsPort = compute_units[-1].ldsBus.slave
    compute_units[-1].ldsBus.master = compute_units[-1].localDataStore.cuPort

# Attach compute units to GPU
shader.CUs = compute_units

########################## Creating the CPU system ########################
# The shader core will be whatever is after the CPU cores are accounted for
shader_idx = options.num_cpus

# The command processor will be whatever is after the shader is accounted for
cp_idx = shader_idx + 1
cp_list = []

# List of CPUs
cpu_list = []

CpuClass, mem_mode = Simulation.getCPUClass(options.cpu_type)
if CpuClass == AtomicSimpleCPU:
    fatal("AtomicSimpleCPU is not supported")
if mem_mode != 'timing':
    fatal("Only the timing memory mode is supported")
shader.timing = True

if options.fast_forward and options.fast_forward_pseudo_op:
    fatal("Cannot fast-forward based both on the number of instructions and"
          " on pseudo-ops")
fast_forward = options.fast_forward or options.fast_forward_pseudo_op

if fast_forward:
    FutureCpuClass, future_mem_mode = CpuClass, mem_mode

    CpuClass = X86KvmCPU
    mem_mode = 'atomic_noncaching'
    # Leave shader.timing untouched, because its value only matters at the
    # start of the simulation and because we require switching cpus
    # *before* the first kernel launch.

    future_cpu_list = []

    # Initial CPUs to be used during fast-forwarding.
    for i in range(options.num_cpus):
        cpu = CpuClass(cpu_id = i,
                       clk_domain = SrcClockDomain(
                           clock = options.CPUClock,
                           voltage_domain = VoltageDomain(
                               voltage = options.cpu_voltage)))
        cpu_list.append(cpu)

        if options.fast_forward:
            cpu.max_insts_any_thread = int(options.fast_forward)

if fast_forward:
    MainCpuClass = FutureCpuClass
else:
    MainCpuClass = CpuClass

# CPs to be used throughout the simulation.
for i in range(options.num_cp):
    cp = MainCpuClass(cpu_id = options.num_cpus + i,
                      clk_domain = SrcClockDomain(
                          clock = options.CPUClock,
                          voltage_domain = VoltageDomain(
                              voltage = options.cpu_voltage)))
    cp_list.append(cp)

# Main CPUs (to be used after fast-forwarding if fast-forwarding is specified).
for i in range(options.num_cpus):
    cpu = MainCpuClass(cpu_id = i,
                       clk_domain = SrcClockDomain(
                           clock = options.CPUClock,
                           voltage_domain = VoltageDomain(
                               voltage = options.cpu_voltage)))
    if fast_forward:
        cpu.switched_out = True
        future_cpu_list.append(cpu)
    else:
        cpu_list.append(cpu)

host_cpu = cpu_list[0]

hsapp_gpu_map_vaddr = 0x200000000
hsapp_gpu_map_size = 0x1000
hsapp_gpu_map_paddr = int(Addr(options.mem_size))

# HSA kernel mode driver
gpu_driver = GPUComputeDriver(filename="kfd")

# Creating the GPU kernel launching components: that is the HSA
# packet processor (HSAPP), GPU command processor (CP), and the
# dispatcher.
gpu_hsapp = HSAPacketProcessor(pioAddr=hsapp_gpu_map_paddr,
                               numHWQueues=options.num_hw_queues)
dispatcher = GPUDispatcher()
gpu_cmd_proc = GPUCommandProcessor(hsapp=gpu_hsapp,
                                   dispatcher=dispatcher)
gpu_driver.device = gpu_cmd_proc
shader.dispatcher = dispatcher
shader.gpu_cmd_proc = gpu_cmd_proc

# Create and assign the workload Check for rel_path in elements of
# base_list using test, returning the first full path that satisfies test
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
# It's common for a benchmark to be in a directory with the same
# name as the executable, so we handle that automatically
if os.path.isdir(executable):
    benchmark_path = [executable]
    executable = find_file(benchmark_path, options.cmd)

if options.env:
    with open(options.env, 'r') as f:
        env = [line.rstrip() for line in f]
else:
    env = ['LD_LIBRARY_PATH=%s' % ':'.join([
               os.getenv('ROCM_PATH','/opt/rocm')+'/lib',
               os.getenv('HCC_HOME','/opt/rocm/hcc')+'/lib',
               os.getenv('HSA_PATH','/opt/rocm/hsa')+'/lib',
               os.getenv('HIP_PATH','/opt/rocm/hip')+'/lib',
               os.getenv('ROCM_PATH','/opt/rocm')+'/libhsakmt/lib',
               os.getenv('ROCM_PATH','/opt/rocm')+'/miopen/lib',
               os.getenv('ROCM_PATH','/opt/rocm')+'/miopengemm/lib',
               os.getenv('ROCM_PATH','/opt/rocm')+'/hipblas/lib',
               os.getenv('ROCM_PATH','/opt/rocm')+'/rocblas/lib',
               "/usr/lib/x86_64-linux-gnu"
           ]),
           'HOME=%s' % os.getenv('HOME','/'),
           "HSA_ENABLE_INTERRUPT=0"]

process = Process(executable = executable, cmd = [options.cmd]
                  + options.options.split(), drivers = [gpu_driver], env = env)

for cpu in cpu_list:
    cpu.createThreads()
    cpu.workload = process

for cp in cp_list:
    cp.workload = host_cpu.workload

if fast_forward:
    for i in range(len(future_cpu_list)):
        future_cpu_list[i].workload = cpu_list[i].workload
        future_cpu_list[i].createThreads()

########################## Create the overall system ########################
# List of CPUs that must be switched when moving between KVM and simulation
if fast_forward:
    switch_cpu_list = \
        [(cpu_list[i], future_cpu_list[i]) for i in range(options.num_cpus)]

# Full list of processing cores in the system.
cpu_list = cpu_list + [shader] + cp_list

# creating the overall system
# notice the cpu list is explicitly added as a parameter to System
system = System(cpu = cpu_list,
                mem_ranges = [AddrRange(options.mem_size)],
                cache_line_size = options.cacheline_size,
                mem_mode = mem_mode)
if fast_forward:
    system.future_cpu = future_cpu_list
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                   voltage_domain = system.voltage_domain)

if fast_forward:
    have_kvm_support = 'BaseKvmCPU' in globals()
    if have_kvm_support and buildEnv['TARGET_ISA'] == "x86":
        system.vm = KvmVM()
        for i in range(len(host_cpu.workload)):
            host_cpu.workload[i].useArchPT = True
            host_cpu.workload[i].kvmInSE = True
    else:
        fatal("KvmCPU can only be used in SE mode with x86")

# configure the TLB hierarchy
GPUTLBConfig.config_tlb_hierarchy(options, system, shader_idx)

# create Ruby system
system.piobus = IOXBar(width=32, response_latency=0,
                       frontend_latency=0, forward_latency=0)
dma_list = [gpu_hsapp, gpu_cmd_proc]
Ruby.create_system(options, None, system, None, dma_list, None)
system.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock,
                                    voltage_domain = system.voltage_domain)
gpu_cmd_proc.pio = system.piobus.master
gpu_hsapp.pio = system.piobus.master

for i, dma_device in enumerate(dma_list):
    exec('system.dma_cntrl%d.clk_domain = system.ruby.clk_domain' % i)

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
        if fast_forward:
            system.cpu[i].itb.walker.port = ruby_port.slave
            system.cpu[i].dtb.walker.port = ruby_port.slave

# attach CU ports to Ruby
# Because of the peculiarities of the CP core, you may have 1 CPU but 2
# sequencers and thus 2 _cpu_ports created. Your GPUs shouldn't be
# hooked up until after the CP. To make this script generic, figure out
# the index as below, but note that this assumes there is one sequencer
# per compute unit and one sequencer per SQC for the math to work out
# correctly.
gpu_port_idx = len(system.ruby._cpu_ports) \
               - options.num_compute_units - options.num_sqc \
               - options.num_scalar_cache
gpu_port_idx = gpu_port_idx - options.num_cp * 2

wavefront_size = options.wf_size
for i in range(n_cu):
    # The pipeline issues wavefront_size number of uncoalesced requests
    # in one GPU issue cycle. Hence wavefront_size mem ports.
    for j in range(wavefront_size):
        system.cpu[shader_idx].CUs[i].memory_port[j] = \
                  system.ruby._cpu_ports[gpu_port_idx].slave[j]
    system.cpu[shader_idx].CUs[i].gmTokenPort = \
            system.ruby._cpu_ports[gpu_port_idx].gmTokenPort
    gpu_port_idx += 1

for i in range(n_cu):
    if i > 0 and not i % options.cu_per_sqc:
        print("incrementing idx on ", i)
        gpu_port_idx += 1
    system.cpu[shader_idx].CUs[i].sqc_port = \
            system.ruby._cpu_ports[gpu_port_idx].slave
gpu_port_idx = gpu_port_idx + 1

for i in xrange(n_cu):
    if i > 0 and not i % options.cu_per_scalar_cache:
        print("incrementing idx on ", i)
        gpu_port_idx += 1
    system.cpu[shader_idx].CUs[i].scalar_port = \
        system.ruby._cpu_ports[gpu_port_idx].slave
gpu_port_idx = gpu_port_idx + 1

# attach CP ports to Ruby
for i in range(options.num_cp):
    system.cpu[cp_idx].createInterruptController()
    system.cpu[cp_idx].dcache_port = \
                system.ruby._cpu_ports[gpu_port_idx + i * 2].slave
    system.cpu[cp_idx].icache_port = \
                system.ruby._cpu_ports[gpu_port_idx + i * 2 + 1].slave
    system.cpu[cp_idx].interrupts[0].pio = system.piobus.master
    system.cpu[cp_idx].interrupts[0].int_master = system.piobus.slave
    system.cpu[cp_idx].interrupts[0].int_slave = system.piobus.master
    cp_idx = cp_idx + 1

################# Connect the CPU and GPU via GPU Dispatcher ##################
# CPU rings the GPU doorbell to notify a pending task
# using this interface.
# And GPU uses this interface to notify the CPU of task completion
# The communcation happens through emulated driver.

# Note this implicit setting of the cpu_pointer, shader_pointer and tlb array
# parameters must be after the explicit setting of the System cpu list
if fast_forward:
    shader.cpu_pointer = future_cpu_list[0]
else:
    shader.cpu_pointer = host_cpu

########################## Start simulation ########################

redirect_paths = [RedirectPath(app_path = "/proc",
                               host_paths =
                                ["%s/fs/proc" % m5.options.outdir]),
                  RedirectPath(app_path = "/sys",
                               host_paths =
                                ["%s/fs/sys"  % m5.options.outdir]),
                  RedirectPath(app_path = "/tmp",
                               host_paths =
                                ["%s/fs/tmp"  % m5.options.outdir])]

system.redirect_paths = redirect_paths

root = Root(system=system, full_system=False)

hsaTopology.createHsaTopology(options)

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

if options.fast_forward:
    print("Switch at instruction count: %d" % cpu_list[0].max_insts_any_thread)

exit_event = m5.simulate(maxtick)

if options.fast_forward:
    if exit_event.getCause() == "a thread reached the max instruction count":
        m5.switchCpus(system, switch_cpu_list)
        print("Switched CPUS @ tick %s" % (m5.curTick()))
        m5.stats.reset()
        exit_event = m5.simulate(maxtick - m5.curTick())
elif options.fast_forward_pseudo_op:
    while exit_event.getCause() == "switchcpu":
        # If we are switching *to* kvm, then the current stats are meaningful
        # Note that we don't do any warmup by default
        if type(switch_cpu_list[0][0]) == FutureCpuClass:
            print("Dumping stats...")
            m5.stats.dump()
        m5.switchCpus(system, switch_cpu_list)
        print("Switched CPUS @ tick %s" % (m5.curTick()))
        m5.stats.reset()
        # This lets us switch back and forth without keeping a counter
        switch_cpu_list = [(x[1], x[0]) for x in switch_cpu_list]
        exit_event = m5.simulate(maxtick - m5.curTick())

print("Ticks:", m5.curTick())
print('Exiting because ', exit_event.getCause())

sys.exit(exit_event.getCode())
