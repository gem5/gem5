"""SimObject .py files to dynamically load during param generation.

These files define SimObject subclasses scattered across the gem5 source tree
(outside src/python/). During aggregate SimObject param generation, they are
compiled and registered with the embedded Python importer so that m5.objects
auto-discovery finds all SimObject classes.

The module path for each file is m5.objects.<stem> where <stem> is the
filename without the .py extension.

Files are split by ISA/feature to avoid generating param structs whose
Params::create() methods reference constructors from disabled subsystem
libraries. The cc_import(alwayslink=True) whole-archive approach forces
ALL param objects to be linked, so any param referencing an uncompiled
SimObject constructor would cause an undefined symbol error.

Optional subsystem SimObjects (FastModel, KVM, SystemC, SST, DRAMSim2,
DRAMSim3, DRAMSys, capstone, protobuf-dependent, Ruby) are excluded from
BASE because their class headers transitively include external library
headers that may not be available. Feature-gated in BUILD.bazel:
  DRAMSim3 -> dramsim3_enabled, TrafficGen -> protobuf_enabled.
BaseSemihosting is gated on semihosting_isa_enabled (ARM or RISC-V)
via select() in BUILD.bazel. Others should be added back via select()
when properly gated.
"""

# ISA-independent SimObject .py files (always compiled)
SIMOBJECT_PY_FILES_BASE = [
    # arch/generic
    "src/arch/generic/BaseISA.py",
    "src/arch/generic/BaseInterrupts.py",
    "src/arch/generic/BaseMMU.py",
    "src/arch/generic/BaseTLB.py",
    "src/arch/generic/InstDecoder.py",
    # base
    "src/base/Graphics.py",
    "src/base/filters/BloomFilters.py",
    "src/base/vnc/Vnc.py",
    # cpu (ISA-independent)
    "src/cpu/BaseCPU.py",
    "src/cpu/CPUTracers.py",
    "src/cpu/CpuCluster.py",
    "src/cpu/DummyChecker.py",
    "src/cpu/FuncUnit.py",
    "src/cpu/StaticInstFlags.py",
    "src/cpu/TimingExpr.py",
    "src/cpu/checker/CheckerCPU.py",
    "src/cpu/minor/BaseMinorCPU.py",
    "src/cpu/minor/MinorCPU.py",
    "src/cpu/o3/BaseO3CPU.py",
    "src/cpu/o3/BaseO3Checker.py",
    "src/cpu/o3/FUPool.py",
    "src/cpu/o3/FuncUnitConfig.py",
    "src/cpu/o3/IQUnit.py",
    "src/cpu/o3/O3CPU.py",
    "src/cpu/o3/O3Checker.py",
    "src/cpu/o3/SMT.py",
    "src/cpu/o3/probe/SimpleTrace.py",
    "src/cpu/pred/BranchPredictor.py",
    "src/cpu/probes/InstTracker.py",
    "src/cpu/probes/PcCountTracker.py",
    "src/cpu/simple/AtomicSimpleCPU.py",
    "src/cpu/simple/BaseAtomicSimpleCPU.py",
    "src/cpu/simple/BaseNonCachingSimpleCPU.py",
    "src/cpu/simple/BaseSimpleCPU.py",
    "src/cpu/simple/BaseTimingSimpleCPU.py",
    "src/cpu/simple/NonCachingSimpleCPU.py",
    "src/cpu/simple/TimingSimpleCPU.py",
    "src/cpu/simple/probes/LooppointAnalysis.py",
    "src/cpu/simple/probes/SimPoint.py",
    "src/cpu/testers/memtest/MemTest.py",
    "src/cpu/testers/spatter_gen/SpatterGen.py",
    "src/cpu/testers/traffic_gen/BaseTrafficGen.py",
    "src/cpu/testers/traffic_gen/GUPSGen.py",
    "src/cpu/testers/traffic_gen/PyTrafficGen.py",
    # dev (ISA-independent)
    "src/dev/BadDevice.py",
    "src/dev/Device.py",
    "src/dev/Platform.py",
    "src/dev/i2c/I2C.py",
    "src/dev/net/Ethernet.py",
    "src/dev/pci/CopyEngine.py",
    "src/dev/pci/PciDevice.py",
    "src/dev/pci/PciHost.py",
    "src/dev/pci/PciUpstream.py",
    "src/dev/ps2/PS2.py",
    "src/dev/qemu/QemuFwCfg.py",
    "src/dev/serial/Serial.py",
    "src/dev/serial/Terminal.py",
    "src/dev/serial/Uart.py",
    "src/dev/storage/DiskImage.py",
    "src/dev/storage/Ide.py",
    "src/dev/storage/SimpleDisk.py",
    "src/dev/virtio/VirtIO.py",
    "src/dev/virtio/VirtIO9P.py",
    "src/dev/virtio/VirtIOBlock.py",
    "src/dev/virtio/VirtIOConsole.py",
    "src/dev/virtio/VirtIORng.py",
    # learning_gem5
    "src/learning_gem5/part2/HelloObject.py",
    "src/learning_gem5/part2/SimpleCache.py",
    "src/learning_gem5/part2/SimpleMemobj.py",
    "src/learning_gem5/part2/SimpleObject.py",
    # mem
    "src/mem/AbstractMemory.py",
    "src/mem/AddrMapper.py",
    "src/mem/Bridge.py",
    "src/mem/CfiMemory.py",
    "src/mem/CommMonitor.py",
    "src/mem/DRAMInterface.py",
    "src/mem/ExternalMaster.py",
    "src/mem/ExternalSlave.py",
    "src/mem/HBMCtrl.py",
    "src/mem/HMCController.py",
    "src/mem/HeteroMemCtrl.py",
    "src/mem/MemChecker.py",
    "src/mem/MemCtrl.py",
    "src/mem/MemDelay.py",
    "src/mem/MemInterface.py",
    "src/mem/NVMInterface.py",
    "src/mem/PortTerminator.py",
    "src/mem/SerialLink.py",
    "src/mem/SharedMemoryServer.py",
    "src/mem/SimpleMemory.py",
    "src/mem/SysBridge.py",
    "src/mem/ThreadBridge.py",
    "src/mem/XBar.py",
    "src/mem/cache/Cache.py",
    "src/mem/cache/compressors/Compressors.py",
    "src/mem/cache/prefetch/Prefetcher.py",
    "src/mem/cache/replacement_policies/ReplacementPolicies.py",
    "src/mem/cache/tags/Tags.py",
    "src/mem/cache/tags/indexing_policies/IndexingPolicies.py",
    "src/mem/cache/tags/partitioning_policies/PartitioningPolicies.py",
    "src/mem/probes/BaseMemProbe.py",
    "src/mem/probes/MemFootprintProbe.py",
    "src/mem/probes/StackDistProbe.py",
    "src/mem/qos/QoSMemCtrl.py",
    "src/mem/qos/QoSMemSinkCtrl.py",
    "src/mem/qos/QoSMemSinkInterface.py",
    "src/mem/qos/QoSPolicy.py",
    "src/mem/qos/QoSTurnaround.py",
    # sim
    "src/sim/ClockDomain.py",
    "src/sim/ClockedObject.py",
    "src/sim/DVFSHandler.py",
    "src/sim/InstTracer.py",
    "src/sim/PowerDomain.py",
    "src/sim/PowerState.py",
    "src/sim/Process.py",
    "src/sim/RedirectPath.py",
    "src/sim/Root.py",
    "src/sim/SignalPort.py",
    "src/sim/SubSystem.py",
    "src/sim/System.py",
    "src/sim/TickedObject.py",
    "src/sim/VoltageDomain.py",
    "src/sim/Workload.py",
    "src/sim/power/MathExprPowerModel.py",
    "src/sim/power/PowerModel.py",
    "src/sim/power/PowerModelState.py",
    "src/sim/power/ThermalDomain.py",
    "src/sim/power/ThermalModel.py",
    "src/sim/probe/Probe.py",
    # sst
    "src/sst/OutgoingRequestBridge.py",
]

# X86 ISA-specific SimObject .py files
SIMOBJECT_PY_FILES_X86 = [
    "src/arch/x86/X86CPU.py",
    "src/arch/x86/X86Decoder.py",
    "src/arch/x86/X86FsWorkload.py",
    "src/arch/x86/X86ISA.py",
    "src/arch/x86/X86LocalApic.py",
    "src/arch/x86/X86MMU.py",
    "src/arch/x86/X86NativeTrace.py",
    "src/arch/x86/X86SeWorkload.py",
    "src/arch/x86/X86TLB.py",
    "src/arch/x86/bios/ACPI.py",
    "src/arch/x86/bios/E820.py",
    "src/arch/x86/bios/IntelMP.py",
    "src/arch/x86/bios/SMBios.py",
    "src/dev/x86/Cmos.py",
    "src/dev/x86/I8042.py",
    "src/dev/x86/I82094AA.py",
    "src/dev/x86/I8237.py",
    "src/dev/x86/I8254.py",
    "src/dev/x86/I8259.py",
    "src/dev/x86/Pc.py",
    "src/dev/x86/PcSpeaker.py",
    "src/dev/x86/SouthBridge.py",
    "src/dev/x86/X86Ide.py",
    "src/dev/x86/X86QemuFwCfg.py",
]

# ARM ISA-specific SimObject .py files
SIMOBJECT_PY_FILES_ARM = [
    "src/arch/arm/ArmCPU.py",
    "src/arch/arm/ArmDecoder.py",
    "src/arch/arm/ArmFsWorkload.py",
    "src/arch/arm/ArmISA.py",
    "src/arch/arm/ArmInterrupts.py",
    "src/arch/arm/ArmMMU.py",
    "src/arch/arm/ArmNativeTrace.py",
    "src/arch/arm/ArmPMU.py",
    "src/arch/arm/ArmSeWorkload.py",
    "src/arch/arm/ArmSemihosting.py",
    "src/arch/arm/ArmSystem.py",
    "src/arch/arm/ArmTLB.py",
    "src/arch/arm/tracers/TarmacTrace.py",
    "src/dev/arm/AbstractNVM.py",
    "src/dev/arm/Display.py",
    "src/dev/arm/Doorbell.py",
    "src/dev/arm/EnergyCtrl.py",
    "src/dev/arm/FlashDevice.py",
    "src/dev/arm/GenericTimer.py",
    "src/dev/arm/Gic.py",
    "src/dev/arm/Mpam.py",
    "src/dev/arm/NoMali.py",
    "src/dev/arm/RealView.py",
    "src/dev/arm/SMMUv3.py",
    "src/dev/arm/UFSHostDevice.py",
    "src/dev/arm/VirtIOMMIO.py",
    "src/dev/arm/css/MHU.py",
    "src/dev/arm/css/Scmi.py",
    "src/dev/arm/css/Scp.py",
]

# MIPS ISA-specific SimObject .py files
SIMOBJECT_PY_FILES_MIPS = [
    "src/arch/mips/MipsCPU.py",
    "src/arch/mips/MipsDecoder.py",
    "src/arch/mips/MipsISA.py",
    "src/arch/mips/MipsInterrupts.py",
    "src/arch/mips/MipsMMU.py",
    "src/arch/mips/MipsSeWorkload.py",
    "src/arch/mips/MipsTLB.py",
    "src/dev/mips/Malta.py",
]

# Power ISA-specific SimObject .py files
SIMOBJECT_PY_FILES_POWER = [
    "src/arch/power/PowerCPU.py",
    "src/arch/power/PowerDecoder.py",
    "src/arch/power/PowerISA.py",
    "src/arch/power/PowerInterrupts.py",
    "src/arch/power/PowerMMU.py",
    "src/arch/power/PowerSeWorkload.py",
    "src/arch/power/PowerTLB.py",
]

# RISC-V ISA-specific SimObject .py files
SIMOBJECT_PY_FILES_RISCV = [
    "src/arch/riscv/PMAChecker.py",
    "src/arch/riscv/PMP.py",
    "src/arch/riscv/RiscvCPU.py",
    "src/arch/riscv/RiscvDecoder.py",
    "src/arch/riscv/RiscvFsWorkload.py",
    "src/arch/riscv/RiscvISA.py",
    "src/arch/riscv/RiscvInterrupts.py",
    "src/arch/riscv/RiscvMMU.py",
    "src/arch/riscv/RiscvSeWorkload.py",
    "src/arch/riscv/RiscvSemihosting.py",
    "src/arch/riscv/RiscvSystem.py",
    "src/arch/riscv/RiscvTLB.py",
    "src/dev/lupio/LupioBLK.py",
    "src/dev/lupio/LupioIPI.py",
    "src/dev/lupio/LupioPIC.py",
    "src/dev/lupio/LupioRNG.py",
    "src/dev/lupio/LupioRTC.py",
    "src/dev/lupio/LupioSYS.py",
    "src/dev/lupio/LupioTMR.py",
    "src/dev/lupio/LupioTTY.py",
    "src/dev/riscv/Clint.py",
    "src/dev/riscv/HiFive.py",
    "src/dev/riscv/LupV.py",
    "src/dev/riscv/Plic.py",
    "src/dev/riscv/PlicDevice.py",
    "src/dev/riscv/RTC.py",
    "src/dev/riscv/RiscvVirtIOMMIO.py",
]

# SPARC ISA-specific SimObject .py files
SIMOBJECT_PY_FILES_SPARC = [
    "src/arch/sparc/SparcCPU.py",
    "src/arch/sparc/SparcDecoder.py",
    "src/arch/sparc/SparcFsWorkload.py",
    "src/arch/sparc/SparcISA.py",
    "src/arch/sparc/SparcInterrupts.py",
    "src/arch/sparc/SparcMMU.py",
    "src/arch/sparc/SparcNativeTrace.py",
    "src/arch/sparc/SparcSeWorkload.py",
    "src/arch/sparc/SparcTLB.py",
    "src/dev/sparc/T1000.py",
]

# Ruby-specific SimObject .py files (gated on ruby_enabled)
SIMOBJECT_PY_FILES_RUBY = [
    "src/mem/ruby/network/BasicLink.py",
    "src/mem/ruby/network/BasicRouter.py",
    "src/mem/ruby/network/MessageBuffer.py",
    "src/mem/ruby/network/Network.py",
    "src/mem/ruby/network/fault_model/FaultModel.py",
    "src/mem/ruby/network/garnet/GarnetLink.py",
    "src/mem/ruby/network/garnet/GarnetNetwork.py",
    "src/mem/ruby/network/simple/SimpleLink.py",
    "src/mem/ruby/network/simple/SimpleNetwork.py",
    "src/mem/ruby/slicc_interface/Controller.py",
    "src/mem/ruby/structures/DirectoryMemory.py",
    "src/mem/ruby/structures/RubyCache.py",
    "src/mem/ruby/structures/RubyPrefetcher.py",
    "src/mem/ruby/structures/WireBuffer.py",
    "src/mem/ruby/system/RubySystem.py",
    "src/mem/ruby/system/Sequencer.py",
    # cpu testers gated on Ruby
    "src/cpu/testers/directedtest/RubyDirectedTester.py",
    "src/cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTraffic.py",
    "src/cpu/testers/rubytest/RubyTester.py",
]

# Ruby GPU-specific SimObject .py files (gated on ruby + gpu)
SIMOBJECT_PY_FILES_RUBY_GPU = [
    "src/mem/ruby/system/GPUCoalescer.py",
    "src/mem/ruby/system/VIPERCoalescer.py",
    "src/mem/ruby/system/VIPERSequencer.py",
    # cpu testers gated on Ruby + GPU
    "src/cpu/testers/gpu_ruby_test/CpuThread.py",
    "src/cpu/testers/gpu_ruby_test/DmaThread.py",
    "src/cpu/testers/gpu_ruby_test/GpuWavefront.py",
    "src/cpu/testers/gpu_ruby_test/ProtocolTester.py",
    "src/cpu/testers/gpu_ruby_test/TesterDma.py",
    "src/cpu/testers/gpu_ruby_test/TesterThread.py",
]

# GPU-specific SimObject .py files
SIMOBJECT_PY_FILES_GPU = [
    "src/arch/amdgpu/common/X86GPUTLB.py",
    "src/arch/amdgpu/vega/VegaGPUTLB.py",
    "src/dev/amdgpu/AMDGPU.py",
    "src/dev/hsa/HSADevice.py",
    "src/gpu-compute/GPU.py",
    "src/gpu-compute/GPUStaticInstFlags.py",
    "src/gpu-compute/LdsState.py",
]

# -- Path-to-label conversion for declaring .py files as Bazel inputs --
# Sorted by depth (deepest first) for longest-prefix matching.
_OVERLAY_PACKAGES = [
    "src/arch/amdgpu",
    "src/arch/arm",
    "src/arch/generic",
    "src/arch/isa_parser",
    "src/arch/mips",
    "src/arch/null",
    "src/arch/power",
    "src/arch/riscv",
    "src/arch/sparc",
    "src/arch/x86",
    "src/dev/amdgpu",
    "src/dev/arm",
    "src/dev/hsa",
    "src/dev/lupio",
    "src/dev/mips",
    "src/dev/riscv",
    "src/dev/sparc",
    "src/dev/x86",
    "src/learning_gem5/part2",
    "src/mem/ruby/protocol/chi/tlm",
    "src/mem/ruby/protocol",
    "src/mem/ruby",
    "src/mem/slicc",
    "src/arch",
    "src/base",
    "src/cpu",
    "src/dev",
    "src/gpu-compute",
    "src/kern",
    "src/mem",
    "src/proto",
    "src/python",
    "src/sim",
    "src/sst",
    "src/systemc",
    "src/test_objects",
    "src",
]

def simobject_path_to_label(path):
    """Convert a SimObject .py file path to a Bazel label.

    Maps e.g. "src/sim/Root.py" -> "//src/sim:Root.py",
              "src/cpu/o3/FUPool.py" -> "//src/cpu:o3/FUPool.py".
    """
    for pkg in _OVERLAY_PACKAGES:
        if path.startswith(pkg + "/"):
            rest = path[len(pkg) + 1:]
            return "//{}:{}".format(pkg, rest)
    return "//:" + path

def simobject_paths_to_labels(paths):
    """Convert a list of SimObject .py file paths to Bazel labels."""
    return [simobject_path_to_label(p) for p in paths]
