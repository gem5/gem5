load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

py_library(
    name = "power_init",
    srcs = ["__init__.py"],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_ThermalDomain",
    desc = "",
    flag = "ThermalDomain",
)

gem5_cc_library(
    name = "power_model_hh",
    hdrs = ["power_model.hh"],
    deps = [
        ":enum_PMType_hh",
        ":power_model_sim_object",
        ":power_model_state_sim_object",
        "//src/base:statistics",
        "//src/base:temperature",
        "//src/base:types",
        "//src/sim:sim_object_hh",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "power_model",
    srcs = ["power_model.cc"],
    hdrs = ["power_model.hh"],
    deps = [
        ":enum_PMType_hh",
        ":power_model_hh",
        ":power_model_sim_object",
        ":power_model_state_sim_object",
        "//src/base:statistics",
        "//src/base:temperature",
        "//src/sim:clocked_object_hh",
        "//src/sim:sim_object_hh",
        "//src/sim:sub_system_hh",
        "//src/sim/probe",
    ],
)

py_library(
    name = "PowerModelState_py",
    srcs = ["PowerModelState.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

# Generator for PowerModelState
gem5_sim_object(
    name = "power_model_state_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    sim_object_header = ":power_model",  # Placeholder
    sim_object_module = "src.sim.power.PowerModelState",
    sim_object_name = "PowerModelState",
    sim_object_py_lib = ":PowerModelState_py",
)

py_library(
    name = "PowerModel_py",
    srcs = ["PowerModel.py"],
    deps = [
        ":PowerModelState_py",
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_PMType",
    enum_module = "src.sim.power.PowerModel",
    enum_name = "PMType",
    enum_py_lib = ":PowerModel_py",
    extra_tool_deps = [
        "//src/python/m5/objects:SubSystem",
    ],
)

# Generator for PowerModel
gem5_sim_object(
    name = "power_model_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        "//src/sim:sub_system_hh",
        "//src/sim/probe",
    ],
    extra_param_deps = [
        ":enum_PMType_hh",
        "//src/base:temperature",
        "//src/sim:sim_object_sim_object",
        "//src/sim/power:power_model_state_sim_object",
        "//src/sim:sub_system_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/objects:PowerModelState",
    ],
    sim_object_header = ":power_model",
    sim_object_module = "src.sim.power.PowerModel",
    sim_object_name = "PowerModel",
    sim_object_py_lib = ":PowerModel_py",
)

gem5_cc_library(
    name = "thermal_entity",
    hdrs = ["thermal_entity.hh"],
    deps = [
        "//src/sim:sim_object_hh",
    ],
)

py_library(
    name = "ThermalDomain_py",
    srcs = ["ThermalDomain.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "thermal_domain_hh",
    hdrs = ["thermal_domain.hh"],
    deps = [
        ":thermal_domain_sim_object",
        ":thermal_entity",
        ":thermal_node",
        "//src/base:statistics_hh",
        "//src/base:temperature",
        "//src/sim:clocked_object_hh",
        "//src/sim:sim_object_hh",
        "//src/sim:sub_system_hh",
    ],
)

gem5_cc_library(
    name = "thermal_domain",
    srcs = ["thermal_domain.cc"],
    hdrs = ["thermal_domain.hh"],
    deps = [
        ":debug_ThermalDomain",
        ":thermal_domain_hh",
        ":thermal_domain_sim_object",
        ":thermal_entity",
        ":thermal_model_hh",
        ":thermal_node",
        "//src/base:statistics_hh",
        "//src/base:temperature",
        "//src/base:types",
        "//src/sim:clocked_object_hh",
        "//src/sim:linear_solver",
        "//src/sim:sim_object_hh",
        "//src/sim:sub_system_hh",
        "//src/sim/probe",
    ],
)

# Generator for ThermalDomain
gem5_sim_object(
    name = "thermal_domain_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/base:temperature",
        "//src/sim:sim_object_sim_object",
    ],
    sim_object_header = ":thermal_domain",
    sim_object_module = "src.sim.power.ThermalDomain",
    sim_object_name = "ThermalDomain",
    sim_object_py_lib = ":ThermalDomain_py",
)

gem5_cc_library(
    name = "thermal_node",
    srcs = ["thermal_node.cc"],
    hdrs = ["thermal_node.hh"],
    deps = [
        ":thermal_node_sim_object",
        "//src/base:temperature",
        "//src/base:types",
        "//src/sim:sim_object_hh",
    ],
)

py_library(
    name = "ThermalModel_py",
    srcs = ["ThermalModel.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:ThermalDomain",
        "//src/python/m5/params",
    ],
)

# Generator for ThermalNode
gem5_sim_object(
    name = "thermal_node_sim_object",
    extra_deps = [
        "//src/base:temperature",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ThermalDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:PowerState",
    ],
    sim_object_header = ":thermal_node",
    sim_object_module = "src.sim.power.ThermalModel",
    sim_object_name = "ThermalNode",
    sim_object_py_lib = ":ThermalModel_py",
)

# Generator for ThermalResistor
gem5_sim_object(
    name = "thermal_resistor_sim_object",
    extra_deps = [
        "//src/base:temperature",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ThermalDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:PowerState",
    ],
    sim_object_header = ":thermal_model",
    sim_object_module = "src.sim.power.ThermalModel",
    sim_object_name = "ThermalResistor",
    sim_object_py_lib = ":ThermalModel_py",
)

# Generator for ThermalCapacitor
gem5_sim_object(
    name = "thermal_capacitor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/base:temperature",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ThermalDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:PowerState",
    ],
    sim_object_header = ":thermal_model",
    sim_object_module = "src.sim.power.ThermalModel",
    sim_object_name = "ThermalCapacitor",
    sim_object_py_lib = ":ThermalModel_py",
)

# Generator for ThermalReference
gem5_sim_object(
    name = "thermal_reference_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/base:temperature",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ThermalDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:PowerState",
    ],
    sim_object_header = ":thermal_model",
    sim_object_module = "src.sim.power.ThermalModel",
    sim_object_name = "ThermalReference",
    sim_object_py_lib = ":ThermalModel_py",
)

# Generator for ThermalModel
gem5_sim_object(
    name = "thermal_model_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        "//src/base:temperature",
        ":thermal_domain_hh",
    ],
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        ":thermal_domain_sim_object",
        ":thermal_node_sim_object",
        ":thermal_resistor_sim_object",
        ":thermal_capacitor_sim_object",
        ":thermal_reference_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ThermalDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:PowerState",
    ],
    sim_object_header = ":thermal_model",
    sim_object_module = "src.sim.power.ThermalModel",
    sim_object_name = "ThermalModel",
    sim_object_py_lib = ":ThermalModel_py",
)

gem5_cc_library(
    name = "thermal_model_hh",
    hdrs = ["thermal_model.hh"],
    deps = [
        ":thermal_capacitor_sim_object",
        ":thermal_domain_hh",
        ":thermal_entity",
        ":thermal_model_sim_object",
        ":thermal_node",
        ":thermal_reference_sim_object",
        ":thermal_resistor_sim_object",
        "//src/base:statistics_hh",
        "//src/base:temperature",
        "//src/base:types",
        "//src/sim:clocked_object_hh",
        "//src/sim:linear_solver",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "thermal_model",
    srcs = ["thermal_model.cc"],
    hdrs = ["thermal_model.hh"],
    deps = [
        ":thermal_capacitor_sim_object",
        ":thermal_domain",
        ":thermal_entity",
        ":thermal_model_hh",
        ":thermal_model_sim_object",
        ":thermal_node",
        ":thermal_reference_sim_object",
        ":thermal_resistor_sim_object",
        "//src/base:statistics_hh",
        "//src/base:temperature",
        "//src/base:types",
        "//src/sim:clocked_object",
        "//src/sim:linear_solver",
        "//src/sim:sim_object_hh",
    ],
)
