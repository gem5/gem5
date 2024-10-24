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

import m5
from m5.objects import *


def createGPU(system, args):
    shader = Shader(
        n_wf=args.wfs_per_simd,
        cu_per_sqc=args.cu_per_sqc,
        timing=True,
        clk_domain=system.clk_domain,
    )

    # VIPER GPU protocol implements release consistency at GPU side. So,
    # we make their writes visible to the global memory and should read
    # from global memory during kernal boundary. The pipeline initiates
    # (or do not initiate) the acquire/release operation depending on
    # these impl_kern_launch_rel and impl_kern_end_rel flags. The flag=true
    # means pipeline initiates a acquire/release operation at kernel launch/end
    # VIPER protocol is write-through based, and thus only impl_kern_launch_acq
    # needs to set.
    shader.impl_kern_launch_acq = True
    shader.impl_kern_end_rel = False

    # Switching off per-lane TLB by default
    per_lane = False
    if args.TLB_config == "perLane":
        per_lane = True

    # List of compute units; one GPU can have multiple compute units
    compute_units = []
    for i in range(args.num_compute_units):
        compute_units.append(
            ComputeUnit(
                cu_id=i,
                perLaneTLB=per_lane,
                num_SIMDs=args.simds_per_cu,
                wf_size=args.wf_size,
                spbypass_pipe_length=args.sp_bypass_path_length,
                dpbypass_pipe_length=args.dp_bypass_path_length,
                issue_period=args.issue_period,
                coalescer_to_vrf_bus_width=args.glbmem_rd_bus_width,
                vrf_to_coalescer_bus_width=args.glbmem_wr_bus_width,
                num_global_mem_pipes=args.glb_mem_pipes_per_cu,
                num_shared_mem_pipes=args.shr_mem_pipes_per_cu,
                n_wf=args.wfs_per_simd,
                execPolicy=args.CUExecPolicy,
                localMemBarrier=args.LocalMemBarrier,
                countPages=args.countPages,
                memtime_latency=args.memtime_latency,
                max_cu_tokens=args.max_cu_tokens,
                vrf_lm_bus_latency=args.vrf_lm_bus_latency,
                mem_req_latency=args.mem_req_latency,
                mem_resp_latency=args.mem_resp_latency,
                scalar_mem_req_latency=args.scalar_mem_req_latency,
                scalar_mem_resp_latency=args.scalar_mem_resp_latency,
                localDataStore=LdsState(
                    banks=args.numLdsBanks,
                    bankConflictPenalty=args.ldsBankConflictPenalty,
                    size=args.lds_size,
                ),
            )
        )

        wavefronts = []
        vrfs = []
        vrf_pool_mgrs = []
        srfs = []
        rfcs = []
        srf_pool_mgrs = []
        for j in range(args.simds_per_cu):
            for k in range(shader.n_wf):
                wavefronts.append(
                    Wavefront(simdId=j, wf_slot_id=k, wf_size=args.wf_size)
                )

            if args.reg_alloc_policy == "simple":
                vrf_pool_mgrs.append(
                    SimplePoolManager(
                        pool_size=args.vreg_file_size,
                        min_alloc=args.vreg_min_alloc,
                    )
                )
                srf_pool_mgrs.append(
                    SimplePoolManager(
                        pool_size=args.sreg_file_size,
                        min_alloc=args.vreg_min_alloc,
                    )
                )
            elif args.reg_alloc_policy == "dynamic":
                vrf_pool_mgrs.append(
                    DynPoolManager(
                        pool_size=args.vreg_file_size,
                        min_alloc=args.vreg_min_alloc,
                    )
                )
                srf_pool_mgrs.append(
                    DynPoolManager(
                        pool_size=args.sreg_file_size,
                        min_alloc=args.vreg_min_alloc,
                    )
                )

            vrfs.append(
                VectorRegisterFile(
                    simd_id=j,
                    wf_size=args.wf_size,
                    num_regs=args.vreg_file_size,
                )
            )

            srfs.append(
                ScalarRegisterFile(
                    simd_id=j,
                    wf_size=args.wf_size,
                    num_regs=args.sreg_file_size,
                )
            )
            rfcs.append(
                RegisterFileCache(
                    simd_id=j, cache_size=args.register_file_cache_size
                )
            )

        compute_units[-1].wavefronts = wavefronts
        compute_units[-1].vector_register_file = vrfs
        compute_units[-1].scalar_register_file = srfs
        compute_units[-1].register_file_cache = rfcs
        compute_units[-1].register_manager = RegisterManager(
            policy=args.registerManagerPolicy,
            vrf_pool_managers=vrf_pool_mgrs,
            srf_pool_managers=srf_pool_mgrs,
        )
        if args.TLB_prefetch:
            compute_units[-1].prefetch_depth = args.TLB_prefetch
            compute_units[-1].prefetch_prev_type = args.pf_type

        # Attach the LDS and the CU to the bus (actually a Bridge)
        compute_units[-1].ldsPort = compute_units[-1].ldsBus.cpu_side_port
        compute_units[-1].ldsBus.mem_side_port = compute_units[
            -1
        ].localDataStore.cuPort

    # Attach compute units to GPU
    shader.CUs = compute_units

    shader.cpu_pointer = system.cpu[0]
    shader.eventq_index = 0
    shader.set_parent(system, "Shader")

    return shader


def connectGPU(system, args):
    system.pc.south_bridge.gpu = AMDGPUDevice(pci_func=0, pci_dev=8, pci_bus=0)

    system.pc.south_bridge.gpu.checkpoint_before_mmios = (
        args.checkpoint_before_mmios
    )

    system.pc.south_bridge.gpu.device_name = args.gpu_device

    if args.gpu_device == "MI100":
        system.pc.south_bridge.gpu.DeviceID = 0x738C
        system.pc.south_bridge.gpu.SubsystemVendorID = 0x1002
        system.pc.south_bridge.gpu.SubsystemID = 0x0C34
    elif args.gpu_device == "MI200":
        system.pc.south_bridge.gpu.DeviceID = 0x740F
        system.pc.south_bridge.gpu.SubsystemVendorID = 0x1002
        system.pc.south_bridge.gpu.SubsystemID = 0x0C34
    elif args.gpu_device == "MI300X":
        system.pc.south_bridge.gpu.DeviceID = 0x740F
        system.pc.south_bridge.gpu.SubsystemVendorID = 0x1002
        system.pc.south_bridge.gpu.SubsystemID = 0x0C34
    elif args.gpu_device == "Vega10":
        system.pc.south_bridge.gpu.DeviceID = 0x6863
    else:
        m5.util.panic(f"Unknown GPU device: {args.gpu_device}")

    # Use the gem5 default of 0x280 OR'd  with 0x10 which tells Linux there is
    # a PCI capabilities list to travse.
    system.pc.south_bridge.gpu.Status = 0x0290

    # The PCI capabilities are like a linked list. The list has a memory
    # offset and a capability type ID read by the OS. Make the first
    # capability at 0x80 and set the PXCAP (PCI express) capability to
    # that address. Mark the type ID as PCI express.
    # We leave the next ID of PXCAP blank to end the list.
    system.pc.south_bridge.gpu.PXCAPBaseOffset = 0x80
    system.pc.south_bridge.gpu.CapabilityPtr = 0x80
    system.pc.south_bridge.gpu.PXCAPCapId = 0x10

    # Set bits 7 and 8 in the second PCIe device capabilities register which
    # reports support for PCIe atomics for 32 and 64 bits respectively.
    # Bit 9 for 128-bit compare and swap is not set because the amdgpu driver
    # does not check this.
    system.pc.south_bridge.gpu.PXCAPDevCap2 = 0x00000180

    # Set bit 6 to enable atomic requestor, meaning this device can request
    # atomics from other PCI devices.
    system.pc.south_bridge.gpu.PXCAPDevCtrl2 = 0x00000040
