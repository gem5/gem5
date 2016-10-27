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
#  Author: Steve Reinhardt
#

from ClockedObject import ClockedObject
from Device import DmaDevice
from m5.defines import buildEnv
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from MemObject import MemObject
from Process import EmulatedDriver
from Bridge import Bridge
from LdsState import LdsState

class PrefetchType(Enum): vals = [
    'PF_CU',
    'PF_PHASE',
    'PF_WF',
    'PF_STRIDE',
    'PF_END',
    ]

class VectorRegisterFile(SimObject):
    type = 'VectorRegisterFile'
    cxx_class = 'VectorRegisterFile'
    cxx_header = 'gpu-compute/vector_register_file.hh'

    simd_id = Param.Int(0, 'SIMD ID associated with this VRF')
    num_regs_per_simd = Param.Int(2048, 'number of vector registers per SIMD')
    wfSize = Param.Int(64, 'Wavefront size (in work items)')
    min_alloc = Param.Int(4, 'min number of VGPRs allocated per WF')

class Wavefront(SimObject):
    type = 'Wavefront'
    cxx_class = 'Wavefront'
    cxx_header = 'gpu-compute/wavefront.hh'

    simdId = Param.Int('SIMD id (0-ComputeUnit.num_SIMDs)')
    wf_slot_id = Param.Int('wavefront id (0-ComputeUnit.max_wfs)')
    wfSize = Param.Int(64, 'Wavefront size (in work items)')

class ComputeUnit(MemObject):
    type = 'ComputeUnit'
    cxx_class = 'ComputeUnit'
    cxx_header = 'gpu-compute/compute_unit.hh'

    wavefronts = VectorParam.Wavefront('Number of wavefronts')
    wfSize = Param.Int(64, 'Wavefront size (in work items)')
    num_SIMDs = Param.Int(4, 'number of SIMD units per CU')

    spbypass_pipe_length = Param.Int(4, 'vector ALU Single Precision bypass '\
                                        'latency')

    dpbypass_pipe_length = Param.Int(8, 'vector ALU Double Precision bypass '\
                                        'latency')

    issue_period = Param.Int(4, 'number of cycles per issue period')
    num_global_mem_pipes = Param.Int(1,'number of global memory pipes per CU')
    num_shared_mem_pipes = Param.Int(1,'number of shared memory pipes per CU')
    n_wf = Param.Int(1, 'Number of wavefront slots per SIMD')
    mem_req_latency = Param.Int(9, "Latency for request from the cu to ruby. "\
                                "Represents the pipeline to reach the TCP and "\
                                "specified in GPU clock cycles")
    mem_resp_latency = Param.Int(9, "Latency for responses from ruby to the "\
                                 "cu. Represents the pipeline between the TCP "\
                                 "and cu as well as TCP data array access. "\
                                 "Specified in GPU clock cycles")
    system = Param.System(Parent.any, "system object")
    cu_id = Param.Int('CU id')
    vrf_to_coalescer_bus_width = Param.Int(32, "VRF->Coalescer data bus width "\
                                           "in bytes")
    coalescer_to_vrf_bus_width = Param.Int(32, "Coalescer->VRF data bus width "\
                                           "in bytes")

    memory_port = VectorMasterPort("Port to the memory system")
    translation_port = VectorMasterPort('Port to the TLB hierarchy')
    sqc_port = MasterPort("Port to the SQC (I-cache")
    sqc_tlb_port = MasterPort("Port to the TLB for the SQC (I-cache)")
    perLaneTLB = Param.Bool(False, "enable per-lane TLB")
    prefetch_depth = Param.Int(0, "Number of prefetches triggered at a time"\
                               "(0 turns off prefetching)")
    prefetch_stride = Param.Int(1, "Fixed Prefetch Stride (1 means next-page)")
    prefetch_prev_type = Param.PrefetchType('PF_PHASE', "Prefetch the stride "\
                                            "from last mem req in lane of "\
                                            "CU|Phase|Wavefront")
    execPolicy = Param.String("OLDEST-FIRST", "WF execution selection policy");
    xactCasMode = Param.Bool(False, "Behavior of xact_cas_load magic instr.");
    debugSegFault = Param.Bool(False, "enable debugging GPU seg faults")
    functionalTLB = Param.Bool(False, "Assume TLB causes no delay")

    localMemBarrier = Param.Bool(False, "Assume Barriers do not wait on "\
                                        "kernel end")

    countPages = Param.Bool(False, "Generate per-CU file of all pages touched "\
                                   "and how many times")
    global_mem_queue_size = Param.Int(256, "Number of entries in the global "
                                      "memory pipeline's queues")
    local_mem_queue_size = Param.Int(256, "Number of entries in the local "
                                      "memory pipeline's queues")
    ldsBus = Bridge() # the bridge between the CU and its LDS
    ldsPort = MasterPort("The port that goes to the LDS")
    localDataStore = Param.LdsState("the LDS for this CU")

    vector_register_file = VectorParam.VectorRegisterFile("Vector register "\
                                                          "file")
    out_of_order_data_delivery = Param.Bool(False, "enable OoO data delivery"
                                            " in the GM pipeline")

class Shader(ClockedObject):
    type = 'Shader'
    cxx_class = 'Shader'
    cxx_header = 'gpu-compute/shader.hh'

    CUs = VectorParam.ComputeUnit('Number of compute units')
    n_wf = Param.Int(1, 'Number of wavefront slots per SIMD')
    impl_kern_boundary_sync = Param.Bool(True, """Insert acq/rel packets into
                                                  ruby at kernel boundaries""")
    separate_acquire_release = Param.Bool(False,
        """Do ld_acquire/st_release generate separate requests for the
        acquire and release?""")
    globalmem = Param.MemorySize('64kB', 'Memory size')
    timing = Param.Bool(False, 'timing memory accesses')

    cpu_pointer = Param.BaseCPU(NULL, "pointer to base CPU")
    translation = Param.Bool(False, "address translation");

class ClDriver(EmulatedDriver):
    type = 'ClDriver'
    cxx_header = 'gpu-compute/cl_driver.hh'
    codefile = VectorParam.String('code file name(s)')

class GpuDispatcher(DmaDevice):
    type = 'GpuDispatcher'
    cxx_header = 'gpu-compute/dispatcher.hh'
    # put at 8GB line for now
    pio_addr = Param.Addr(0x200000000, "Device Address")
    pio_latency = Param.Latency('1ns', "Programmed IO latency")
    shader_pointer = Param.Shader('pointer to shader')
    translation_port = MasterPort('Port to the dispatcher TLB')
    cpu = Param.BaseCPU("CPU to wake up on kernel completion")

    cl_driver = Param.ClDriver('pointer to driver')

class MemType(Enum): vals = [
    'M_U8',
    'M_U16',
    'M_U32',
    'M_U64',
    'M_S8',
    'M_S16',
    'M_S32',
    'M_S64',
    'M_F16',
    'M_F32',
    'M_F64',
    ]

class StorageClassType(Enum): vals = [
    'SC_SPILL',
    'SC_GLOBAL',
    'SC_SHARED',
    'SC_PRIVATE',
    'SC_READONLY',
    'SC_KERNARG',
    'SC_NONE',
    ]

class RegisterType(Enum): vals = [
    'RT_VECTOR',
    'RT_SCALAR',
    'RT_CONDITION',
    'RT_HARDWARE',
    'RT_NONE',
    ]
