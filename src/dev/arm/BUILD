load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)
# ==============================================================================
# Debug Flags
# ==============================================================================

gem5_debug_flag(
    name = "debug_AMBA",
    desc = "",
    flag = "AMBA",
)

gem5_debug_flag(
    name = "debug_FlashDevice",
    desc = "",
    flag = "FlashDevice",
)

gem5_debug_flag(
    name = "debug_HDLcd",
    desc = "",
    flag = "HDLcd",
)

gem5_debug_flag(
    name = "debug_PL111",
    desc = "",
    flag = "PL111",
)

gem5_debug_flag(
    name = "debug_GICV2M",
    desc = "",
    flag = "GICV2M",
)

gem5_debug_flag(
    name = "debug_Pl050",
    desc = "",
    flag = "Pl050",
)

gem5_debug_flag(
    name = "debug_GIC",
    desc = "",
    flag = "GIC",
)

gem5_debug_flag(
    name = "debug_ITS",
    desc = "",
    flag = "ITS",
)

gem5_debug_flag(
    name = "debug_RVCTRL",
    desc = "",
    flag = "RVCTRL",
)

gem5_debug_flag(
    name = "debug_SMMUv3",
    desc = "",
    flag = "SMMUv3",
)

gem5_debug_flag(
    name = "debug_SMMUv3Hazard",
    desc = "",
    flag = "SMMUv3Hazard",
)

gem5_debug_flag(
    name = "debug_Sp805",
    desc = "",
    flag = "Sp805",
)

gem5_debug_flag(
    name = "debug_EnergyCtrl",
    desc = "",
    flag = "EnergyCtrl",
)

gem5_debug_flag(
    name = "debug_FVPBasePwrCtrl",
    desc = "",
    flag = "FVPBasePwrCtrl",
)

gem5_debug_flag(
    name = "debug_UFSHostDevice",
    desc = "",
    flag = "UFSHostDevice",
)

gem5_debug_flag(
    name = "debug_VGIC",
    desc = "",
    flag = "VGIC",
)

gem5_debug_flag(
    name = "debug_NoMali",
    desc = "",
    flag = "NoMali",
)

gem5_cc_library(
    name = "display",
    srcs = ["display.cc"],
    hdrs = ["display.hh"],
    deps = [
        ":display_sim_object",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "Display_py",
    srcs = ["Display.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "display_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
    sim_object_header = ":display",
    sim_object_module = "src.dev.arm.Display",
    sim_object_name = "Display",
    sim_object_py_lib = ":Display_py",
)

# NOTE: Need only ArmInterruptPin and ArmPPI (for python)
py_library(
    name = "Gic_py",
    srcs = [
        "Gic.py",
    ],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_enum_generator(
    name = "enum_ArmInterruptType",
    enum_module = "src.dev.arm.Gic",
    enum_name = "ArmInterruptType",
    enum_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "arm_interrupt_pin_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":enum_ArmInterruptType_hh",
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object",
        "//src/dev:platform",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":base_gic",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "ArmInterruptPin",
    sim_object_py_lib = ":Gic_py",
)

py_library(
    name = "RealView_py",
    srcs = [
        "RealView.py",
    ],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:defines",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ArmSystem",
        "//src/python/m5/objects:CfiMemory",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Display",
        "//src/python/m5/objects:EnergyCtrl",
        "//src/python/m5/objects:Ethernet",
        "//src/python/m5/objects:GenericTimer",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/objects:Graphics",
        "//src/python/m5/objects:Ide",
        "//src/python/m5/objects:KvmGic",
        "//src/python/m5/objects:MHU",
        "//src/python/m5/objects:PS2",
        "//src/python/m5/objects:PciDevice",
        "//src/python/m5/objects:PciHost",
        "//src/python/m5/objects:PciUpstream",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:SMMUv3",
        "//src/python/m5/objects:Scmi",
        "//src/python/m5/objects:SimpleMemory",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/objects:Terminal",
        "//src/python/m5/objects:Uart",
        "//src/python/m5/objects:VirtIOMMIO",
        "//src/python/m5/objects:Vnc",
        "//src/python/m5/objects:VoltageDomain",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "real_view_sim_object",
    extra_param_deps = [
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":realview",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "RealView",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "fvp_base_pwr_ctrl_sim_object",
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":fvp_base_pwr_ctrl",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "FVPBasePwrCtrl",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "gic_v2_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":enum_ArmInterruptType_hh",
        "//src/dev:platform_sim_object",
        "//src/dev/arm:base_gic_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":gic_v2",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "GicV2",
    sim_object_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "arm_spi_sim_object",
    extra_param_deps = [
        ":arm_interrupt_pin_sim_object",
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":base_gic",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "ArmSPI",
    sim_object_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "arm_ppi_sim_object",
    extra_param_deps = [
        ":arm_interrupt_pin_sim_object",
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":base_gic",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "ArmPPI",
    sim_object_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "arm_sig_interrupt_pin_sim_object",
    extra_param_deps = [
        ":arm_interrupt_pin_sim_object",
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":base_gic",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "ArmSigInterruptPin",
    sim_object_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "base_gic_sim_object",
    extra_param_deps = [
        "//src/dev:pio_device_sim_object",
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object",
        "//src/dev:platform",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":base_gic",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "BaseGic",
    sim_object_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "gicv3_its_sim_object",
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":gic_v3_its",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "Gicv3Its",
    sim_object_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "gicv3_sim_object",
    extra_param_deps = [
        ":base_gic_sim_object",
        ":gicv3_its_sim_object",
        ":arm_interrupt_pin_sim_object",
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object_hh",
        ":gic_v3_its_hh",
        ":base_gic",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":gic_v3",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "Gicv3",
    sim_object_py_lib = ":Gic_py",
)

py_library(
    name = "GenericTimer_py",
    srcs = [
        "GenericTimer.py",
    ],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/params",
        "//src/python/m5/util",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "system_counter_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":generic_timer",
    sim_object_module = "src.dev.arm.GenericTimer",
    sim_object_name = "SystemCounter",
    sim_object_py_lib = ":GenericTimer_py",
)

gem5_sim_object(
    name = "generic_timer_sim_object",
    extra_param_deps = [
        ":system_counter_sim_object",
        ":arm_ppi_sim_object",
        "//src/arch/arm:arm_system_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object_hh",
        ":base_gic",
        "//src/arch/arm:system",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:ArmSystem",
    ],
    sim_object_header = ":generic_timer",
    sim_object_module = "src.dev.arm.GenericTimer",
    sim_object_name = "GenericTimer",
    sim_object_py_lib = ":GenericTimer_py",
)

gem5_sim_object(
    name = "generic_timer_frame_sim_object",
    extra_param_deps = [
        ":system_counter_sim_object",
        ":arm_spi_sim_object",
        "//src/dev:pio_device_sim_object",
        "//src/sim:sim_object_hh",
        ":base_gic",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":generic_timer",
    sim_object_module = "src.dev.arm.GenericTimer",
    sim_object_name = "GenericTimerFrame",
    sim_object_py_lib = ":GenericTimer_py",
)

gem5_sim_object(
    name = "generic_timer_mem_sim_object",
    extra_param_deps = [
        ":system_counter_sim_object",
        ":generic_timer_frame_sim_object",
        ":arm_spi_sim_object",
        "//src/dev:pio_device_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":generic_timer",
    sim_object_module = "src.dev.arm.GenericTimer",
    sim_object_name = "GenericTimerMem",
    sim_object_py_lib = ":GenericTimer_py",
)

gem5_cc_library(
    name = "generic_timer_miscregs_types",
    hdrs = ["generic_timer_miscregs_types.hh"],
    deps = [
        "//src/base:bitunion",
    ],
)

gem5_cc_library(
    name = "gic_v3_cpu_interface_hh",
    hdrs = ["gic_v3_cpu_interface.hh"],
    deps = [
        ":gic_v3_hh",
        "//src/arch/arm:isa_device_hh",
    ],
)

gem5_cc_library(
    name = "gic_v3_cpu_interface",
    srcs = ["gic_v3_cpu_interface.cc"],
    hdrs = ["gic_v3_cpu_interface.hh"],
    deps = [
        ":debug_GIC",
        ":gic_v3",
        ":gic_v3_distributor",
        ":gic_v3_redistributor",
        "//src/arch/arm:faults_hh",
        "//src/arch/arm:isa_device_hh",
        "//src/arch/arm:isa_hh",
    ],
)

gem5_cc_library(
    name = "gic_v3_distributor",
    srcs = ["gic_v3_distributor.cc"],
    hdrs = ["gic_v3_distributor.hh"],
    deps = [
        ":debug_GIC",
        ":gic_v3_cpu_interface_hh",
        ":gic_v3_hh",
        ":gic_v3_redistributor_hh",
        "//src/base:addr_range",
        "//src/base:compiler",
        "//src/base:intmath",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "gic_v3_redistributor",
    srcs = ["gic_v3_redistributor.cc"],
    hdrs = ["gic_v3_redistributor.hh"],
    deps = [
        ":debug_GIC",
        ":gic_v3_cpu_interface_hh",
        ":gic_v3_distributor",
        ":gic_v3_hh",
        "//src/arch/arm:faults_hh",
        "//src/arch/arm:isa_device_hh",
        "//src/arch/arm:isa_hh",
        "//src/arch/arm:utility",
        "//src/base:addr_range",
        "//src/base:compiler",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "gic_v3_redistributor_hh",
    hdrs = ["gic_v3_redistributor.hh"],
    deps = [
        ":gic_v3_hh",
        "//src/base:addr_range",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "gic_v3",
    srcs = ["gic_v3.cc"],
    hdrs = ["gic_v3.hh"],
    deps = [
        ":base_gic",
        ":debug_GIC",
        ":gic_v3_cpu_interface_hh",
        ":gic_v3_distributor",
        ":gic_v3_its",
        ":gic_v3_redistributor_hh",
        ":gicv3_sim_object",
        "//src/arch/arm:interrupts_hh",
        "//src/arch/arm:types",
        "//src/cpu:base",
        "//src/dev:platform",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Interrupt",
    ],
)

gem5_cc_library(
    name = "gic_v3_hh",
    hdrs = ["gic_v3.hh"],
    deps = [
        ":base_gic",
        ":gicv3_sim_object",
        "//src/arch/arm:interrupts_hh",
        "//src/arch/arm:types",
    ],
)

gem5_cc_library(
    name = "base_gic",
    srcs = ["base_gic.cc"],
    hdrs = ["base_gic.hh"],
    deps = [
        ":arm_interrupt_pin_sim_object",
        ":arm_ppi_sim_object",
        ":arm_sig_interrupt_pin_sim_object",
        ":arm_spi_sim_object",
        ":base_gic_sim_object",
        ":debug_GIC",
        ":enum_ArmInterruptType_hh",
        ":realview_hh",
        "//src/arch/arm:system_hh",
        "//src/cpu:thread_context_hh",
        "//src/dev:intpin",
        "//src/dev:io_device",
    ],
)

gem5_cc_library(
    name = "realview_hh",
    hdrs = ["realview.hh"],
    deps = [
        "//src/dev:platform",
    ],
)

gem5_cc_library(
    name = "gic_v2",
    srcs = ["gic_v2.cc"],
    hdrs = ["gic_v2.hh"],
    deps = [
        "debug_GIC",
        ":base_gic",
        ":gic_v2_sim_object",
        "//src/arch/arm:interrupts_hh",
        "//src/base:addr_range",
        "//src/base:bitunion",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:base",
        "//src/dev:io_device",
        "//src/dev:platform",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
        "//src/sim:debug_IPI",
        "//src/sim:debug_Interrupt",
    ],
)

gem5_sim_object(
    name = "v_gic_sim_object",
    extra_deps = [
        ":base_gic",
        "//src/dev:platform",
        "//src/sim:sim_object",
    ],
    extra_param_deps = [
        ":base_gic_sim_object",
        "//src/dev:platform_sim_object",
        "//src/dev:pio_device_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":vgic",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "VGic",
    sim_object_py_lib = ":Gic_py",
)

gem5_cc_library(
    name = "vgic_hh",
    hdrs = ["vgic.hh"],
    deps = [
        "//src/base:addr_range",
        "//src/base:bitunion",
        "//src/dev:io_device",
        "//src/dev:platform",
    ],
)

gem5_cc_library(
    name = "vgic",
    srcs = ["vgic.cc"],
    hdrs = ["vgic.hh"],
    deps = [
        ":base_gic",
        ":debug_VGIC",
        ":v_gic_sim_object",
        ":vgic_hh",
        "//src/arch/arm:interrupts_hh",
        "//src/base:addr_range",
        "//src/base:bitunion",
        "//src/base:trace",
        "//src/cpu:base",
        "//src/dev:io_device",
        "//src/dev:platform",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
    ],
)

gem5_sim_object(
    name = "gicv2m_frame_sim_object",
    extra_deps = [
        ":gic_v2m",
        "//src/sim:sim_object",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
    sim_object_header = ":gic_v2m_hh",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "Gicv2mFrame",
    sim_object_py_lib = ":Gic_py",
)

gem5_sim_object(
    name = "gicv2m_sim_object",
    extra_deps = [
        ":base_gic",
        ":gic_v2m",
        "//src/sim:sim_object",
    ],
    extra_param_deps = [
        ":base_gic_sim_object",
        ":gicv2m_frame_sim_object",
        "//src/dev:pio_device_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
    ],
    sim_object_header = ":gic_v2m_hh",
    sim_object_module = "src.dev.arm.Gic",
    sim_object_name = "Gicv2m",
    sim_object_py_lib = ":Gic_py",
)

gem5_cc_library(
    name = "gic_v2m_hh",
    hdrs = ["gic_v2m.hh"],
    deps = [
        ":base_gic",
        "//src/base:bitunion",
        "//src/dev:io_device",
        "//src/dev:platform",
    ],
)

gem5_cc_library(
    name = "gic_v2m",
    srcs = ["gic_v2m.cc"],
    hdrs = ["gic_v2m.hh"],
    deps = [
        ":base_gic",
        ":debug_GICV2M",
        ":gic_v2m_hh",
        ":gicv2m_frame_sim_object",
        ":gicv2m_sim_object",
        "//src/base:bitunion",
        "//src/base:intmath",
        "//src/dev:io_device",
        "//src/dev:platform",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
    ],
)

gem5_cc_library(
    name = "generic_timer",
    srcs = ["generic_timer.cc"],
    hdrs = ["generic_timer.hh"],
    deps = [
        ":base_gic",
        ":generic_timer_frame_sim_object",
        ":generic_timer_mem_sim_object",
        ":generic_timer_miscregs_types",
        ":generic_timer_sim_object",
        ":system_counter_sim_object",
        "//bazel/generated/config",
        "//src/arch/arm:isa_device_hh",
        "//src/arch/arm:page_size",
        "//src/arch/arm:system",
        "//src/arch/arm:utility",
        "//src/arch/arm/regs:misc_hh",
        "//src/base:addr_range",
        "//src/base:bitunion",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:base",
        "//src/cpu/kvm:vm_hh",  # Update if enabling KVM
        "//src/dev:io_device",
        "//src/dev:platform",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:debug_Timer",
        "//src/sim:drain",
        "//src/sim:eventq",
        "//src/sim:serialize",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "fvp_base_pwr_ctrl",
    srcs = ["fvp_base_pwr_ctrl.cc"],
    hdrs = ["fvp_base_pwr_ctrl.hh"],
    deps = [
        ":debug_FVPBasePwrCtrl",
        ":fvp_base_pwr_ctrl_sim_object",
        "//src/arch/arm:faults_hh",
        "//src/arch/arm:system_hh",
        "//src/arch/arm:utility_hh",
        "//src/base:bitunion",
        "//src/cpu:base",
        "//src/cpu:thread_context_hh",
        "//src/dev:io_device",
        "//src/mem:packet_access",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "gic_v3_its",
    srcs = ["gic_v3_its.cc"],
    hdrs = ["gic_v3_its.hh"],
    deps = [
        ":debug_GIC",
        ":debug_ITS",
        ":gic_v3_distributor",
        ":gic_v3_hh",
        ":gic_v3_redistributor_hh",
        ":gicv3_its_sim_object",
        "//src/base:addr_range",
        "//src/base:bitunion",
        "//src/base:coroutine",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:dma_device",
        "//src/mem:debug_AddrRanges",
        "//src/mem:packet_access",
        "//src/sim:debug_Drain",
    ],
)

gem5_cc_library(
    name = "gic_v3_its_hh",
    hdrs = ["gic_v3_its.hh"],
    deps = [
        "//src/base:addr_range",
        "//src/base:bitunion",
        "//src/base:coroutine",
        "//src/base:types",
        "//src/dev:dma_device",
    ],
)

gem5_cc_library(
    name = "realview",
    srcs = ["realview.cc"],
    hdrs = ["realview.hh"],
    deps = [
        ":base_gic",
        ":real_view_sim_object",
        "//src/base:logging",
        "//src/dev:platform",
    ],
)

gem5_cc_library(
    name = "amba_hh",
    hdrs = ["amba.hh"],
    deps = [
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "amba_device_hh",
    hdrs = ["amba_device.hh"],
    deps = [
        ":base_gic",
        "//src/dev:dma_device",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "amba_device",
    srcs = ["amba_device.cc"],
    hdrs = ["amba_device.hh"],
    deps = [
        ":amba_device_hh",
        ":amba_dma_device_sim_object",
        ":amba_fake_hh",
        ":amba_int_device_sim_object",
        ":amba_pio_device_sim_object",
        ":base_gic",
        ":debug_AMBA",
        "//src/base:trace",
        "//src/dev:dma_device",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "a9scu",
    srcs = ["a9scu.cc"],
    hdrs = ["a9scu.hh"],
    deps = [
        ":a9_scu_sim_object",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "pci_host",
    srcs = ["pci_host.cc"],
    hdrs = ["pci_host.hh"],
    deps = [
        ":base_gic",
        ":enum_ArmPciIntRouting_hh",
        ":generic_arm_pci_host_sim_object",
        "//src/dev/pci:host",
    ],
)

gem5_cc_library(
    name = "rv_ctrl",
    srcs = ["rv_ctrl.cc"],
    hdrs = ["rv_ctrl.hh"],
    deps = [
        ":debug_RVCTRL",
        ":real_view_ctrl_sim_object",
        ":real_view_osc_sim_object",
        ":real_view_temperature_sensor_sim_object",
        "//src/base:bitunion",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:clock_domain",
        "//src/sim:sim_object",
        "//src/sim:system",
        "//src/sim:voltage_domain",
        "//src/sim/power:thermal_model",
    ],
)

gem5_cc_library(
    name = "ssc",
    srcs = ["ssc.cc"],
    hdrs = ["ssc.hh"],
    deps = [
        ":sys_sec_ctrl_sim_object",
        "//src/dev:io_device",
        "//src/dev:reg_bank",
    ],
)

gem5_cc_library(
    name = "amba_fake_hh",
    hdrs = ["amba_fake.hh"],
    deps = [
        ":amba_device_hh",
    ],
)

gem5_cc_library(
    name = "amba_fake",
    srcs = ["amba_fake.cc"],
    hdrs = ["amba_fake.hh"],
    deps = [
        ":amba_device",
        ":amba_fake_hh",
        ":amba_fake_sim_object",
        ":debug_AMBA",
        "//src/base:trace",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "pl011",
    srcs = ["pl011.cc"],
    hdrs = ["pl011.hh"],
    deps = [
        ":amba_device",
        ":base_gic",
        ":pl011_sim_object",
        "//src/base:trace",
        "//src/dev:intpin",
        "//src/dev:io_device",
        "//src/dev/serial:debug_Uart",
        "//src/dev/serial:uart",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
        "//src/sim:sim_exit",
    ],
)

gem5_cc_library(
    name = "timer_sp804",
    srcs = ["timer_sp804.cc"],
    hdrs = ["timer_sp804.hh"],
    deps = [
        ":amba_device",
        ":base_gic",
        ":sp804_sim_object",
        "//src/base:bitunion",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:intpin",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
        "//src/sim:debug_Timer",
        "//src/sim:eventq",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "watchdog_sp805",
    srcs = ["watchdog_sp805.cc"],
    hdrs = ["watchdog_sp805.hh"],
    deps = [
        ":amba_device",
        ":debug_Sp805",
        ":sp805_sim_object",
        "//src/base:logging",
        "//src/dev:intpin",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "watchdog_generic",
    srcs = ["watchdog_generic.cc"],
    hdrs = ["watchdog_generic.hh"],
    deps = [
        ":base_gic",
        ":generic_timer",
        ":generic_watchdog_sim_object",
        "//src/dev:io_device",
    ],
)

gem5_cc_library(
    name = "timer_cpulocal",
    srcs = ["timer_cpulocal.cc"],
    hdrs = ["timer_cpulocal.hh"],
    deps = [
        ":base_gic",
        ":cpu_local_timer_sim_object",
        "//src/arch/arm:system",
        "//src/base:bitunion",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:intpin",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
        "//src/sim:debug_Timer",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "rtc_pl031",
    srcs = ["rtc_pl031.cc"],
    hdrs = ["rtc_pl031.hh"],
    deps = [
        ":amba_device",
        ":pl031_sim_object",
        "//src/base:intmath",
        "//src/base:time",
        "//src/base:trace",
        "//src/dev:intpin",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
        "//src/sim:debug_Timer",
    ],
)

gem5_cc_library(
    name = "kmi",
    srcs = ["kmi.cc"],
    hdrs = ["kmi.hh"],
    deps = [
        ":amba_device",
        ":debug_Pl050",
        ":pl050_sim_object",
        "//src/base:trace",
        "//src/base/vnc:vnc",
        "//src/dev:intpin",
        "//src/dev/ps2:device",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "pl111",
    srcs = ["pl111.cc"],
    hdrs = ["pl111.hh"],
    deps = [
        ":amba_device",
        ":base_gic",
        ":debug_PL111",
        ":pl111_sim_object",
        "//src/base:bmpwriter",
        "//src/base:framebuffer",
        "//src/base:output",
        "//src/base:trace",
        "//src/base/vnc:vnc",
        "//src/dev/serial:debug_Uart",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:serialize",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "hdlcd",
    srcs = ["hdlcd.cc"],
    hdrs = ["hdlcd.hh"],
    deps = [
        ":amba_device",
        ":base_gic",
        ":debug_HDLcd",
        ":hd_lcd_sim_object",
        "//src/base:compiler",
        "//src/base:enum_ImageFormat_hh",
        "//src/base:framebuffer",
        "//src/base:imgwriter",
        "//src/base:imgwriter_hh",
        "//src/base:output",
        "//src/base:trace",
        "//src/base/vnc:vnc",
        "//src/dev:pixelpump",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Checkpoint",
        "//src/sim:serialize",
        "//src/sim:system",
    ],
)

gem5_enum_generator(
    name = "enum_ArmPciIntRouting",
    enum_module = "src.dev.arm.RealView",
    enum_name = "ArmPciIntRouting",
    enum_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "amba_pio_device_sim_object",
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:ArmSystem",
        "//src/python/m5/objects:Ethernet",
    ],
    sim_object_header = ":amba_device",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "AmbaPioDevice",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "amba_int_device_sim_object",
    extra_deps = [
        ":base_gic",
    ],
    extra_param_deps = [
        ":amba_pio_device_sim_object",
        ":arm_interrupt_pin_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":amba_device",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "AmbaIntDevice",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "amba_dma_device_sim_object",
    extra_deps = [
        ":base_gic",
    ],
    extra_param_deps = [
        ":arm_interrupt_pin_sim_object",
        "//src/dev:dma_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:ArmSystem",
        "//src/python/m5/objects:Ethernet",
    ],
    sim_object_header = ":amba_device",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "AmbaDmaDevice",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "a9_scu_sim_object",
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":a9scu",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "A9SCU",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "generic_arm_pci_host_sim_object",
    extra_param_deps = [
        ":enum_ArmPciIntRouting_hh",
        "//src/dev/pci:generic_pci_host_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:PciHost",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":pci_host",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "GenericArmPciHost",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "real_view_ctrl_sim_object",
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":rv_ctrl",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "RealViewCtrl",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "real_view_osc_sim_object",
    extra_deps = [
        "//src/sim:voltage_domain",
    ],
    extra_param_deps = [
        ":real_view_ctrl_sim_object",
        "//src/sim:clock_domain_sim_object",
        "//src/sim:voltage_domain_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:VoltageDomain",
    ],
    sim_object_header = ":rv_ctrl",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "RealViewOsc",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "real_view_temperature_sensor_sim_object",
    extra_deps = [
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        ":real_view_ctrl_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":rv_ctrl",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "RealViewTemperatureSensor",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "sys_sec_ctrl_sim_object",
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":ssc",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "SysSecCtrl",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "amba_fake_sim_object",
    extra_param_deps = [
        ":amba_pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":amba_fake",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "AmbaFake",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "pl011_sim_object",
    extra_deps = [
        ":base_gic",
    ],
    extra_param_deps = [
        ":arm_interrupt_pin_sim_object",
        "//src/dev/serial:uart_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:Uart",
    ],
    sim_object_header = ":pl011",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "Pl011",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "sp804_sim_object",
    extra_deps = [
        ":base_gic",
    ],
    extra_param_deps = [
        ":amba_pio_device_sim_object",
        ":arm_spi_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":timer_sp804",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "Sp804",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "sp805_sim_object",
    extra_param_deps = [
        ":amba_int_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":watchdog_sp805",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "Sp805",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "generic_watchdog_sim_object",
    extra_deps = [
        ":base_gic",
        ":generic_timer",
    ],
    extra_param_deps = [
        ":arm_interrupt_pin_sim_object",
        ":system_counter_sim_object",
        "//src/dev:pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:GenericTimer",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":watchdog_generic",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "GenericWatchdog",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "cpu_local_timer_sim_object",
    extra_deps = [
        ":base_gic",
    ],
    extra_param_deps = [
        ":arm_ppi_sim_object",
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":timer_cpulocal",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "CpuLocalTimer",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "pl031_sim_object",
    extra_param_deps = [
        ":amba_int_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":rtc_pl031",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "PL031",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "pl050_sim_object",
    extra_deps = [
        "//src/dev/ps2:device",
    ],
    extra_param_deps = [
        ":amba_int_device_sim_object",
        "//src/dev/ps2:ps2_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:PS2",
    ],
    sim_object_header = ":kmi",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "Pl050",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "pl111_sim_object",
    extra_deps = [
        "//src/base/vnc:vnc",
    ],
    extra_param_deps = [
        ":amba_dma_device_sim_object",
        "//src/base/vnc:vnc_input_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Display",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:Vnc",
    ],
    sim_object_header = ":pl111",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "Pl111",
    sim_object_py_lib = ":RealView_py",
)

gem5_sim_object(
    name = "hd_lcd_sim_object",
    extra_deps = [
        ":display",
        "//src/base/vnc:vnc",
        "//src/sim:clock_domain_hh",
    ],
    extra_param_deps = [
        ":amba_dma_device_sim_object",
        ":display_sim_object",
        "//src/base:enum_ImageFormat_hh",
        "//src/base/vnc:vnc_input_sim_object",
        "//src/sim:clock_domain_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Display",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:Vnc",
    ],
    sim_object_header = ":hdlcd",
    sim_object_module = "src.dev.arm.RealView",
    sim_object_name = "HDLcd",
    sim_object_py_lib = ":RealView_py",
)

py_library(
    name = "EnergyCtrl_py",
    srcs = [
        "EnergyCtrl.py",
    ],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "energy_ctrl_sim_object",
    extra_deps = [
        "//src/sim:dvfs_handler",
    ],
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/sim:dvfs_handler_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:DVFSHandler",
    ],
    sim_object_header = ":energy_ctrl",
    sim_object_module = "src.dev.arm.EnergyCtrl",
    sim_object_name = "EnergyCtrl",
    sim_object_py_lib = ":EnergyCtrl_py",
)

gem5_cc_library(
    name = "energy_ctrl_hh",
    hdrs = ["energy_ctrl.hh"],
    deps = [
        "//src/dev:io_device",
    ],
)

gem5_cc_library(
    name = "energy_ctrl",
    srcs = ["energy_ctrl.cc"],
    hdrs = ["energy_ctrl.hh"],
    deps = [
        ":debug_EnergyCtrl",
        ":energy_ctrl_hh",
        ":energy_ctrl_sim_object",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:dvfs_handler",
        "//src/sim:serialize",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "VirtIOMMIO_py",
    srcs = ["VirtIOMMIO.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/objects:VirtIO",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "mmio_virt_io_sim_object",
    extra_deps = [
        ":base_gic",
        "//src/dev/virtio:base",
    ],
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/dev/arm:arm_interrupt_pin_sim_object",
        "//src/dev/virtio:virt_io_device_base_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/objects:VirtIO",
    ],
    sim_object_header = ":vio_mmio",
    sim_object_module = "src.dev.arm.VirtIOMMIO",
    sim_object_name = "MmioVirtIO",
    sim_object_py_lib = ":VirtIOMMIO_py",
)

gem5_cc_library(
    name = "vio_mmio",
    srcs = ["vio_mmio.cc"],
    hdrs = ["vio_mmio.hh"],
    deps = [
        ":base_gic",
        ":debug_AMBA",
        ":mmio_virt_io_sim_object",
        "//src/base:logging",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/dev/virtio:base",
        "//src/dev/virtio:debug_VIOIface",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

py_library(
    name = "Doorbell_py",
    srcs = ["Doorbell.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "doorbell_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":doorbell",
    sim_object_module = "src.dev.arm.Doorbell",
    sim_object_name = "Doorbell",
    sim_object_py_lib = ":Doorbell_py",
)

gem5_cc_library(
    name = "doorbell",
    hdrs = ["doorbell.hh"],
    deps = [
        ":doorbell_sim_object",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "SMMUv3_py",
    srcs = ["SMMUv3.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "smm_uv3_device_interface_sim_object",
    extra_deps = [
        ":smmu_v3_headers",
    ],
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockedObject",
    ],
    sim_object_header = ":smmu_v3_deviceifc",
    sim_object_module = "src.dev.arm.SMMUv3",
    sim_object_name = "SMMUv3DeviceInterface",
    sim_object_py_lib = ":SMMUv3_py",
)

gem5_sim_object(
    name = "smm_uv3_sim_object",
    extra_deps = [
        ":base_gic",
        ":smmu_v3_headers",
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        ":smm_uv3_device_interface_sim_object",
        "//src/base:addr_range",
        "//src/dev/arm:arm_spi_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":smmu_v3",
    sim_object_module = "src.dev.arm.SMMUv3",
    sim_object_name = "SMMUv3",
    sim_object_py_lib = ":SMMUv3_py",
)

gem5_cc_library(
    name = "smmu_v3_defs",
    srcs = ["smmu_v3_defs.cc"],
    hdrs = ["smmu_v3_defs.hh"],
    deps = [
        "//src/base:bitunion",
    ],
)

gem5_cc_library(
    name = "smmu_v3_ports_hh",
    hdrs = ["smmu_v3_ports.hh"],
    deps = [
        ":smmu_v3_defs",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem:qport",
        "//src/mem:tport",
    ],
)

gem5_cc_library(
    name = "smmu_v3_ports",
    srcs = ["smmu_v3_ports.cc"],
    deps = [
        ":smmu_v3_headers",
        ":smmu_v3_ports_hh",
        "//src/base:logging",
    ],
)

gem5_cc_library(
    name = "smmu_v3_headers",
    hdrs = [
        "smmu_v3.hh",
        "smmu_v3_caches.hh",
        "smmu_v3_cmdexec.hh",
        "smmu_v3_deviceifc.hh",
        "smmu_v3_events.hh",
        "smmu_v3_proc.hh",
        "smmu_v3_transl.hh",
    ],
    deps = [
        ":smm_uv3_device_interface_sim_object",
        ":smm_uv3_sim_object",
        ":smmu_v3_defs",
        ":smmu_v3_ports_hh",
        "//src/arch/arm:isa",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:coroutine",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:dma_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/mem/cache/tags:base",
        "//src/sim:clocked_object",
        "//src/sim:eventq",
    ],
)

# Alias for compatibility if needed, or used by implementation
alias(
    name = "smmu_v3_hh",
    actual = ":smmu_v3_headers",
)

gem5_cc_library(
    name = "smmu_v3_cmdexec",
    srcs = ["smmu_v3_cmdexec.cc"],
    deps = [
        ":debug_SMMUv3",
        ":smmu_v3_headers",
        "//src/base:bitfield",
    ],
)

gem5_cc_library(
    name = "smmu_v3_proc",
    srcs = ["smmu_v3_proc.cc"],
    deps = [
        ":debug_SMMUv3",
        ":debug_SMMUv3Hazard",
        ":smmu_v3_headers",
        "//src/base:coroutine",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "smmu_v3_caches",
    srcs = ["smmu_v3_caches.cc"],
    deps = [
        ":debug_SMMUv3",
        ":smmu_v3_headers",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/sim:stats",
    ],
)

gem5_cc_library(
    name = "smmu_v3_deviceifc",
    srcs = ["smmu_v3_deviceifc.cc"],
    deps = [
        ":debug_SMMUv3",
        ":smm_uv3_device_interface_sim_object",
        ":smmu_v3_headers",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "smmu_v3_events",
    srcs = ["smmu_v3_events.cc"],
    deps = [
        ":smmu_v3_headers",
    ],
)

gem5_cc_library(
    name = "smmu_v3_transl",
    srcs = ["smmu_v3_transl.cc"],
    deps = [
        ":amba_hh",
        ":base_gic",
        ":debug_SMMUv3",
        ":debug_SMMUv3Hazard",
        ":smmu_v3_headers",
        "//src/arch/arm:pagetable",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "smmu_v3",
    srcs = ["smmu_v3.cc"],
    deps = [
        ":base_gic",
        ":debug_SMMUv3",
        ":smm_uv3_sim_object",
        ":smmu_v3_caches",
        ":smmu_v3_cmdexec",
        ":smmu_v3_deviceifc",
        ":smmu_v3_events",
        ":smmu_v3_headers",
        ":smmu_v3_ports",
        ":smmu_v3_proc",
        ":smmu_v3_transl",
        "//src/base:bitfield",
        "//src/base:cast",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:dma_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:clocked_object",
        "//src/sim:debug_Checkpoint",
        "//src/sim:system",
    ],
)
