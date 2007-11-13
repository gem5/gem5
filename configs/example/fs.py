# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
# Authors: Ali Saidi

import optparse, os, sys

import m5
from m5.objects import *
m5.AddToPath('../common')
from FSConfig import *
from SysPaths import *
from Benchmarks import *
import Simulation
from Caches import *

if not m5.build_env['FULL_SYSTEM']:
    m5.panic("This script requires full-system mode (ALPHA_FS).")

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)

parser = optparse.OptionParser()

# System options
parser.add_option("--kernel", action="store", type="string")
parser.add_option("--script", action="store", type="string")

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

execfile(os.path.join(config_root, "common", "Options.py"))

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# driver system CPU is always simple... note this is an assignment of
# a class, not an instance.
DriveCPUClass = AtomicSimpleCPU
drive_mem_mode = 'atomic'

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

TestCPUClass.clock = '2GHz'
DriveCPUClass.clock = '2GHz'

if options.benchmark:
    try:
        bm = Benchmarks[options.benchmark]
    except KeyError:
        print "Error benchmark %s has not been defined." % options.benchmark
        print "Valid benchmarks are: %s" % DefinedBenchmarks
        sys.exit(1)
else:
    if options.dual:
        bm = [SysConfig(), SysConfig()]
    else:
        bm = [SysConfig()]

if m5.build_env['TARGET_ISA'] == "alpha":
    test_sys = makeLinuxAlphaSystem(test_mem_mode, bm[0])
elif m5.build_env['TARGET_ISA'] == "mips":
    test_sys = makeLinuxMipsSystem(test_mem_mode, bm[0])
elif m5.build_env['TARGET_ISA'] == "sparc":
    test_sys = makeSparcSystem(test_mem_mode, bm[0])
elif m5.build_env['TARGET_ISA'] == "x86":
    test_sys = makeX86System(test_mem_mode, bm[0])
else:
    m5.panic("incapable of building non-alpha or non-sparc full system!")

if options.kernel is not None:
    test_sys.kernel = binary(options.kernel)

if options.script is not None:
    test_sys.readfile = options.script

np = options.num_cpus

if options.l2cache:
    test_sys.l2 = L2Cache(size = '2MB')
    test_sys.tol2bus = Bus()
    test_sys.l2.cpu_side = test_sys.tol2bus.port
    test_sys.l2.mem_side = test_sys.membus.port

test_sys.cpu = [TestCPUClass(cpu_id=i) for i in xrange(np)]

if options.caches:
    test_sys.bridge.filter_ranges_a=[AddrRange(0, Addr.max)]
    test_sys.bridge.filter_ranges_b=[AddrRange(0, size='8GB')]
    test_sys.iocache = IOCache(mem_side_filter_ranges=[AddrRange(0, Addr.max)],
                       cpu_side_filter_ranges=[AddrRange(0x8000000000, Addr.max)])
    test_sys.iocache.cpu_side = test_sys.iobus.port
    test_sys.iocache.mem_side = test_sys.membus.port

for i in xrange(np):
    if options.caches:
        test_sys.cpu[i].addPrivateSplitL1Caches(L1Cache(size = '32kB'),
                                                L1Cache(size = '64kB'))
    if options.l2cache:
        test_sys.cpu[i].connectMemPorts(test_sys.tol2bus)
    else:
        test_sys.cpu[i].connectMemPorts(test_sys.membus)

    if options.fastmem:
        test_sys.cpu[i].physmem_port = test_sys.physmem.port

if m5.build_env['TARGET_ISA'] == 'mips':
        #CP0 Configuration
        TestCPUClass.CP0_PRId_CompanyOptions = 0
        TestCPUClass.CP0_PRId_CompanyID = 1
        TestCPUClass.CP0_PRId_ProcessorID = 147
        TestCPUClass.CP0_PRId_Revision = 0

        #CP0 Interrupt Control
        TestCPUClass.CP0_IntCtl_IPTI = 7
        TestCPUClass.CP0_IntCtl_IPPCI = 7

        # Config Register
        #TestCPUClass.CP0_Config_K23 = 0 # Since TLB
        #TestCPUClass.CP0_Config_KU = 0 # Since TLB
        TestCPUClass.CP0_Config_BE = 0 # Little Endian
        TestCPUClass.CP0_Config_AR = 1 # Architecture Revision 2
        TestCPUClass.CP0_Config_AT = 0 # MIPS32
        TestCPUClass.CP0_Config_MT = 1 # TLB MMU
        #TestCPUClass.CP0_Config_K0 = 2 # Uncached

        #Config 1 Register
        TestCPUClass.CP0_Config1_M = 1 # Config2 Implemented
        TestCPUClass.CP0_Config1_MMU = 63 # TLB Size
        # ***VERY IMPORTANT***
        # Remember to modify CP0_Config1 according to cache specs
        # Examine file ../common/Cache.py
        TestCPUClass.CP0_Config1_IS = 1 # I-Cache Sets Per Way, 16KB cache, i.e., 1 (128)
        TestCPUClass.CP0_Config1_IL = 5 # I-Cache Line Size, default in Cache.py is 64, i.e 5
        TestCPUClass.CP0_Config1_IA = 1 # I-Cache Associativity, default in Cache.py is 2, i.e, a value of 1
        TestCPUClass.CP0_Config1_DS = 2 # D-Cache Sets Per Way (see below), 32KB cache, i.e., 2
        TestCPUClass.CP0_Config1_DL = 5 # D-Cache Line Size, default is 64, i.e., 5
        TestCPUClass.CP0_Config1_DA = 1 # D-Cache Associativity, default is 2, i.e. 1
        TestCPUClass.CP0_Config1_C2 = 0 # Coprocessor 2 not implemented(?)
        TestCPUClass.CP0_Config1_MD = 0 # MDMX ASE not implemented in Mips32
        TestCPUClass.CP0_Config1_PC = 1 # Performance Counters Implemented
        TestCPUClass.CP0_Config1_WR = 0 # Watch Registers Implemented
        TestCPUClass.CP0_Config1_CA = 0 # Mips16e NOT implemented
        TestCPUClass.CP0_Config1_EP = 0 # EJTag Not Implemented
        TestCPUClass.CP0_Config1_FP = 0 # FPU Implemented

        #Config 2 Register
        TestCPUClass.CP0_Config2_M = 1 # Config3 Implemented
        TestCPUClass.CP0_Config2_TU = 0 # Tertiary Cache Control
        TestCPUClass.CP0_Config2_TS = 0 # Tertiary Cache Sets Per Way
        TestCPUClass.CP0_Config2_TL = 0 # Tertiary Cache Line Size
        TestCPUClass.CP0_Config2_TA = 0 # Tertiary Cache Associativity
        TestCPUClass.CP0_Config2_SU = 0 # Secondary Cache Control
        TestCPUClass.CP0_Config2_SS = 0 # Secondary Cache Sets Per Way
        TestCPUClass.CP0_Config2_SL = 0 # Secondary Cache Line Size
        TestCPUClass.CP0_Config2_SA = 0 # Secondary Cache Associativity


        #Config 3 Register
        TestCPUClass.CP0_Config3_M = 0 # Config4 Not Implemented
        TestCPUClass.CP0_Config3_DSPP = 1 # DSP ASE Present
        TestCPUClass.CP0_Config3_LPA = 0 # Large Physical Addresses Not supported in Mips32
        TestCPUClass.CP0_Config3_VEIC = 0 # EIC Supported
        TestCPUClass.CP0_Config3_VInt = 0 # Vectored Interrupts Implemented
        TestCPUClass.CP0_Config3_SP = 0 # Small Pages Supported (PageGrain reg. exists)
        TestCPUClass.CP0_Config3_MT = 0 # MT Not present
        TestCPUClass.CP0_Config3_SM = 0 # SmartMIPS ASE Not implemented
        TestCPUClass.CP0_Config3_TL = 0 # TraceLogic Not implemented

        #SRS Ctl - HSS
        TestCPUClass.CP0_SrsCtl_HSS = 3 # Four shadow register sets implemented


        #TestCPUClass.tlb = TLB()
        #TestCPUClass.UnifiedTLB = 1

if len(bm) == 2:
    if m5.build_env['TARGET_ISA'] == 'alpha':
        drive_sys = makeLinuxAlphaSystem(drive_mem_mode, bm[1])
    elif m5.build_env['TARGET_ISA'] == 'mips':
        drive_sys = makeLinuxMipsSystem(drive_mem_mode, bm[1])
    elif m5.build_env['TARGET_ISA'] == 'sparc':
        drive_sys = makeSparcSystem(drive_mem_mode, bm[1])
    elif m5.build.env['TARGET_ISA'] == 'x86':
        drive_sys = makeX86System(drive_mem_mode, bm[1])
    drive_sys.cpu = DriveCPUClass(cpu_id=0)
    drive_sys.cpu.connectMemPorts(drive_sys.membus)
    if options.fastmem:
        drive_sys.cpu.physmem_port = drive_sys.physmem.port
    if options.kernel is not None:
        drive_sys.kernel = binary(options.kernel)

    root = makeDualRoot(test_sys, drive_sys, options.etherdump)
elif len(bm) == 1:
    root = Root(system=test_sys)
else:
    print "Error I don't know how to create more than 2 systems."
    sys.exit(1)

Simulation.run(options, root, test_sys, FutureClass)
