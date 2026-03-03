"""Public API for gem5 custom build rules.

Import rules from this file rather than individual .bzl files:
    load("//rules:defs.bzl", "gem5_debug_flag", "gem5_blob")
"""

load("//rules:debug_flag.bzl",
     _gem5_debug_flag = "gem5_debug_flag",
     _gem5_compound_flag = "gem5_compound_flag",
     _gem5_debug_format_flag = "gem5_debug_format_flag")
load("//rules:blob.bzl", _gem5_blob = "gem5_blob")
load("//rules:switching_header.bzl", _gem5_switching_headers = "gem5_switching_headers")
load("//rules:config_header.bzl",
     _gem5_config_header = "gem5_config_header",
     _gem5_config_header_string = "gem5_config_header_string",
     _gem5_config_header_namespace = "gem5_config_header_namespace")
load("//rules:isa_desc.bzl", _gem5_isa_desc = "gem5_isa_desc")
load("//rules:slicc.bzl", _gem5_slicc_protocol = "gem5_slicc_protocol")
load("//rules:py_source.bzl",
     _gem5_py_source = "gem5_py_source",
     _gem5_embedded_python_library = "gem5_embedded_python_library",
     _gem5_simobject_pysources = "gem5_simobject_pysources")
load("//rules:sim_object.bzl",
     _gem5_sim_object = "gem5_sim_object",
     _gem5_sim_object_aggregate = "gem5_sim_object_aggregate")
load("//rules:gen_defines.bzl", _gem5_gen_defines = "gem5_gen_defines")
load("//rules:gem5_binary.bzl", _gem5_binary = "gem5_binary")

gem5_debug_flag = _gem5_debug_flag
gem5_compound_flag = _gem5_compound_flag
gem5_debug_format_flag = _gem5_debug_format_flag
gem5_blob = _gem5_blob
gem5_switching_headers = _gem5_switching_headers
gem5_config_header = _gem5_config_header
gem5_config_header_string = _gem5_config_header_string
gem5_config_header_namespace = _gem5_config_header_namespace
gem5_isa_desc = _gem5_isa_desc
gem5_slicc_protocol = _gem5_slicc_protocol
gem5_py_source = _gem5_py_source
gem5_embedded_python_library = _gem5_embedded_python_library
gem5_simobject_pysources = _gem5_simobject_pysources
gem5_sim_object = _gem5_sim_object
gem5_sim_object_aggregate = _gem5_sim_object_aggregate
gem5_gen_defines = _gem5_gen_defines
gem5_binary = _gem5_binary
