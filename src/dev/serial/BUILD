load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_Terminal",
    desc = "",
    flag = "Terminal",
)

gem5_debug_flag(
    name = "debug_TerminalVerbose",
    desc = "",
    flag = "TerminalVerbose",
)

gem5_debug_flag(
    name = "debug_Uart",
    desc = "",
    flag = "Uart",
)

gem5_cc_library(
    name = "serial",
    srcs = ["serial.cc"],
    hdrs = ["serial.hh"],
    deps = [
        ":serial_device_sim_object",
        ":serial_null_device_sim_object",
        "//src/base:logging",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "Serial_py",
    srcs = ["Serial.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "serial_device_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":serial",
    sim_object_module = "src.dev.serial.Serial",
    sim_object_name = "SerialDevice",
    sim_object_py_lib = ":Serial_py",
)

gem5_sim_object(
    name = "serial_null_device_sim_object",
    extra_param_deps = [
        ":serial_device_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":serial",
    sim_object_module = "src.dev.serial.Serial",
    sim_object_name = "SerialNullDevice",
    sim_object_py_lib = ":Serial_py",
)

py_library(
    name = "Uart_py",
    srcs = ["Uart.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:defines",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Serial",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "uart_sim_object",
    extra_deps = [
        ":serial",
        "//src/dev:platform",
    ],
    extra_param_deps = [
        "//src/dev:basic_pio_device_sim_object",
        "//src/dev:platform_sim_object",
        "//src/dev/serial:serial_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:Serial",
    ],
    sim_object_header = ":uart",
    sim_object_module = "src.dev.serial.Uart",
    sim_object_name = "Uart",
    sim_object_py_lib = ":Uart_py",
)

gem5_sim_object(
    name = "simple_uart_sim_object",
    extra_param_deps = [
        ":uart_sim_object",
        "//src/sim:enum_ByteOrder_hh",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
    ],
    sim_object_header = ":simple",
    sim_object_module = "src.dev.serial.Uart",
    sim_object_name = "SimpleUart",
    sim_object_py_lib = ":Uart_py",
)

gem5_sim_object(
    name = "uart8250_sim_object",
    extra_deps = [
        ":uart",
    ],
    extra_param_deps = [
        ":uart_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
    ],
    sim_object_header = ":uart8250",
    sim_object_module = "src.dev.serial.Uart",
    sim_object_name = "Uart8250",
    sim_object_py_lib = ":Uart_py",
)

gem5_cc_library(
    name = "uart",
    srcs = ["uart.cc"],
    hdrs = ["uart.hh"],
    deps = [
        ":serial",
        ":uart_sim_object",
        "//src/base:callback",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/dev:platform",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "simple",
    srcs = ["simple.cc"],
    hdrs = ["simple.hh"],
    deps = [
        ":simple_uart_sim_object",
        ":uart",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:sim_exit",
    ],
)

gem5_cc_library(
    name = "uart8250",
    srcs = ["uart8250.cc"],
    hdrs = ["uart8250.hh"],
    deps = [
        ":debug_Uart",
        ":uart",
        ":uart8250_sim_object",
        "//src/base:bitunion",
        "//src/base:inifile",
        "//src/base:logging",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/dev:platform",
        "//src/dev:reg_bank",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:serialize",
    ],
)

py_library(
    name = "Terminal_py",
    srcs = ["Terminal.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:Serial",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_TerminalDump",
    enum_module = "src.dev.serial.Terminal",
    enum_name = "TerminalDump",
    enum_py_lib = ":Terminal_py",
)

gem5_sim_object(
    name = "terminal_sim_object",
    extra_param_deps = [
        ":enum_TerminalDump_hh",
        ":serial_device_sim_object",
        "//src/base:socket",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Serial",
    ],
    sim_object_header = ":terminal",
    sim_object_module = "src.dev.serial.Terminal",
    sim_object_name = "Terminal",
    sim_object_py_lib = ":Terminal_py",
)

gem5_cc_library(
    name = "terminal",
    srcs = ["terminal.cc"],
    hdrs = ["terminal.hh"],
    deps = [
        ":debug_Terminal",
        ":debug_TerminalVerbose",
        ":debug_Uart",  # Used in terminal.cc
        ":enum_TerminalDump_hh",
        ":serial",
        ":terminal_sim_object",
        ":uart",  # Includes dev/serial/uart.hh in terminal.cc
        "//src/base:atomicio",
        "//src/base:callback",
        "//src/base:circlebuf",
        "//src/base:logging",
        "//src/base:output",
        "//src/base:pollevent",
        "//src/base:socket",
        "//src/base:trace",
        "//src/dev:platform",
        "//src/sim:sim_object_hh",
    ],
)
