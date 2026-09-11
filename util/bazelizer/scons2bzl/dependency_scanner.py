# Copyright 2025 Google, Inc.
# All Rights Reserved.
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

"""Utility functions to retrieve dependent files for various file types."""

import logging
import os
import re
from collections.abc import Iterator

from scons2bzl import (
    defines,
    types,
)


def scan_includes(
    src: types.Path, hdrs: list[types.Path] = tuple()
) -> Iterator[str]:
    """Input a C/C++ file and get included files relevant to Bazel.

    Note that some include files needs to be excluded even they are listed in
    `src`.

    Args:
        src: Path to the source file to scan.
        hdrs: List of paths to the accompanying headers of the source.

    Yields:
        The string specified by the `#include` directive.
    """
    base = types.AbsPath(src.base, base=src.base)
    if not os.path.isfile(src.abs):  # `src` is a generated file
        assert src.abs.endswith("sim/tags.cc")
        return
    includes = []
    with open(src.abs) as f:
        for l in f:
            if match := re.match(r'.*#include ["<](.*)[">]', l):
                includes.append(match.group(1))

    for inc in includes:
        first_layer = inc.split("/", 1)[0]
        is_ext_or_gen = not os.path.isdir(base.append("src", first_layer).abs)
        is_gen = first_layer in ["params", "config", "debug", "enums"]
        if is_ext_or_gen and not is_gen:
            for lib_pattern in defines.ext_lib_map:
                if inc.startswith(lib_pattern):
                    yield inc
                    break
            continue
        if base.append("src", inc) in hdrs:
            continue
        if base.append("src", inc) == src:  # Self include
            continue
        yield inc


BuildAction = dict[str, str | tuple | dict[str, bool]]
BuildActions = list[BuildAction]


def scan_sconscript(
    sconscript: types.Path, allow_exec: bool
) -> dict[str, BuildActions]:
    """Input a SConscript and get declarations of Gem5 build actions.

    This function looks for `Source()`, `SimObject()`, `DebugFlag()` and the like
    from the input sconscript.  The list of these declarations along with
    information required by Bazel will then be returned.  The current
    implementation is extremely lengthy and highly relying on heuristics (and
    hacks).  Waiting to be refactored.

    Currently implemented: Source(), SimObject(), DebugFlag(), DebugFormatFlag(),
      CompoundFlag().

    Args:
        sconscript: Path to the sconscript to scan.
        allow_exec: Instead of implementing a parser, this function leverages
          python's `exec()` to collect declarations by injecting fake names to the
          execution scope.  This switch is simply a safe guard to inform the user
          on the fact that this is going to cause the input to be executed.

    Returns:
        A dict mapping declaration groups to lists of declaration instances.  For
        example:

        {'Source': [],
         'SimObject': [],
         'DebugFlag': []}

        TODO(hchsiao): add some declarations to the example

    Raises:
        NotImplementedError: Certain SConscripts are still not supported.
        RuntimeError: Will be triggered on `exec()` if not `allow_exec`.
    """
    sconscript = sconscript.abs
    # Hardcode (hack) dependencies if failed to parse
    if sconscript.endswith("src/arch/SConscript"):
        return {
            "Source": [],
            "SimObject": [],
            "DebugFormatFlag": [],
            "DebugFlag": [
                {
                    "description": "Decoder debug output",
                    "flags": (),
                    "format_flag": False,
                    "name": "Decoder",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": (
                        "Information about faults, exceptions, interrupts, etc"
                    ),
                    "flags": (),
                    "format_flag": False,
                    "name": "Faults",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "TLBVerbose",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "IntRegs",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "FloatRegs",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "VecRegs",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "VecPredRegs",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "MatRegs",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "CCRegs",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
                {
                    "description": "",
                    "flags": (),
                    "format_flag": False,
                    "name": "MiscRegs",
                    "tags": ("gem5 lib",),
                    "type": "DebugFlag",
                    "condition": {},
                },
            ],
            "CompoundFlag": [
                {
                    "description": "",
                    "flags": (
                        "IntRegs",
                        "FloatRegs",
                        "VecRegs",
                        "VecPredRegs",
                        "MatRegs",
                        "CCRegs",
                        "MiscRegs",
                    ),
                    "format_flag": False,
                    "name": "Registers",
                    "tags": (),
                    "type": "CompoundFlag",
                    "condition": {},
                },
            ],
        }

    class ReturnExceptionError(Exception):
        pass

    class NameProxy:
        """The variable surrogate to make up `exec()` scope."""

        def __init__(self, names):
            if isinstance(names, str):
                names = [names]
            self.names = names

        @property
        def name(self):
            return self.names[0]

        def __call__(self, *args, **kwargs):
            if self.name == "with_tag":
                return "XXX"
            elif self.name == "CompoundFlag":
                assert all([k in ["desc", "tags"] for k in kwargs])
                if len(args) == 2:
                    assert isinstance(args[0], str)
                    assert isinstance(args[1], list)
                    assert not args[1] or isinstance(args[1][0], str)
                    name, flags = args
                    desc, tags = "", []
                    if "desc" in kwargs:
                        desc = kwargs["desc"]
                    if "tags" in kwargs:
                        tags = kwargs["tags"]
                elif len(args) == 3:
                    assert isinstance(args[0], str)
                    assert isinstance(args[1], list)
                    assert not args[1] or isinstance(args[1][0], str)
                    assert isinstance(args[2], str)
                    name, flags, desc = args
                    tags = []
                    if "tags" in kwargs:
                        tags = kwargs["tags"]
                else:
                    raise NotImplementedError()
                spec = {
                    "type": "CompoundFlag",
                    "name": name,
                    "flags": tuple(flags),
                    "tags": tuple(tags),
                    "format_flag": False,
                    "description": desc,
                }
                logging.debug("CompoundFlag entry registered: %s", str(spec))
                declarations.append(tuple(sorted(spec.items())))
            elif self.name == "DebugFormatFlag":
                assert not kwargs
                assert len(args) == 2  # name, desc
                spec = {
                    "type": "DebugFormatFlag",
                    "name": args[0],
                    "flags": tuple(),
                    "tags": tuple(),
                    "format_flag": True,
                    "description": args[1],
                }
                logging.debug(
                    "DebugFormatFlag entry registered: %s", str(spec)
                )
                declarations.append(tuple(sorted(spec.items())))
            elif self.name == "DebugFlag":
                assert all([k in ["tags", "add_tags"] for k in kwargs])
                if len(args) > 1:
                    assert len(args) == 2  # name, desc
                    name, desc = args
                else:
                    assert len(args) == 1  # name
                    name, desc = args[0], ""
                tags = ["gem5 lib"]
                if "tags" in kwargs:
                    tags = []
                    if isinstance(kwargs["tags"], str):
                        tags.append(kwargs["tags"])
                    elif isinstance(kwargs["tags"], tuple):
                        tags += list(kwargs["tags"])
                    elif isinstance(kwargs["tags"], list):
                        tags += kwargs["tags"]
                    else:
                        assert False
                if "add_tags" in kwargs:
                    if isinstance(kwargs["add_tags"], str):
                        tags.append(kwargs["add_tags"])
                    elif isinstance(kwargs["add_tags"], list):
                        tags += kwargs["add_tags"]
                    else:
                        assert False
                spec = {
                    "type": "DebugFlag",
                    "name": name,
                    "flags": tuple(),
                    "tags": tuple(tags),
                    "format_flag": False,
                    "description": desc,
                }
                logging.debug("DebugFlag entry registered: %s", str(spec))
                declarations.append(tuple(sorted(spec.items())))
            elif self.name == "SimObject":
                assert all(
                    [k in ["tags", "sim_objects", "enums"] for k in kwargs]
                )
                assert len(args) == 1
                tags = ["gem5 lib"]
                enums = []
                sim_objs = []
                if "tags" in kwargs:
                    tags = []
                    if isinstance(kwargs["tags"], str):
                        tags.append(kwargs["tags"])
                    elif isinstance(kwargs["tags"], tuple):
                        tags += list(kwargs["tags"])
                    elif isinstance(kwargs["tags"], list):
                        tags += kwargs["tags"]
                    else:
                        assert False
                if "enums" in kwargs:
                    assert isinstance(kwargs["enums"], list)
                    enums += kwargs["enums"]
                if "sim_objects" in kwargs:
                    assert isinstance(kwargs["sim_objects"], list)
                    sim_objs += kwargs["sim_objects"]
                spec = {
                    "type": "SimObject",
                    "src": args[0],
                    "tags": tuple(tags),
                    "enums": tuple(enums),
                    "sim_objects": tuple(sim_objs),
                }
                logging.debug("SimObject entry registered: %s", str(spec))
                declarations.append(tuple(sorted(spec.items())))
            elif self.name == "Source":
                assert all(
                    [
                        k in ["tags", "add_tags", "append", "tag_gem5_lib"]
                        for k in kwargs
                    ]
                )
                assert len(args) == 1
                if "tag_gem5_lib" in kwargs and not kwargs["tag_gem5_lib"]:
                    tags = []
                else:
                    tags = ["gem5 lib"]
                appends = {}
                if "tags" in kwargs:
                    tags = []
                    if isinstance(kwargs["tags"], str):
                        tags.append(kwargs["tags"])
                    elif isinstance(kwargs["tags"], tuple):
                        tags += list(kwargs["tags"])
                    elif isinstance(kwargs["tags"], list):
                        tags += kwargs["tags"]
                    else:
                        assert False
                if "add_tags" in kwargs:
                    if isinstance(kwargs["add_tags"], str):
                        tags.append(kwargs["add_tags"])
                    elif isinstance(kwargs["add_tags"], list):
                        tags += kwargs["add_tags"]
                    else:
                        assert False
                if "append" in kwargs:
                    assert isinstance(kwargs["append"], dict)
                    appends.update(kwargs["append"])
                if args[0] is None:
                    return  # m5ImporterCode declaration (hardcoded separately)
                spec = {
                    "type": "Source",
                    "src": args[0],
                    "tags": tuple(tags),
                    "appends": tuple(sorted(appends.items())),
                }
                logging.debug("Source entry registered: %s", str(spec))
                declarations.append(tuple(sorted(spec.items())))
            elif self.name == "Return":
                raise ReturnExceptionError()
            else:
                pass  # assert false

    class EnvProxy(dict):
        """The `env` surrogate to make up `exec()` scope."""

        def __init__(self, default_config):
            self["GCC"] = True
            self["BACKTRACE_IMPL"] = "glibc"
            self["CONF"] = {
                "TARGET_GPU_ISA": "XXX",
                "KVM_ISA": "XXX",
            }
            self.config_switches = [
                ("HAVE_DRAMSYS", []),
                ("HAVE_DRAMSIM", []),
                ("HAVE_DRAMSIM3", []),
                ("USE_PYTHON", []),
                ("RUBY", ["CONF"]),
                ("BUILD_ISA", ["CONF"]),
                ("HAVE_TUNTAP", ["CONF"]),
                ("HAVE_HDF5", ["CONF"]),
                ("HAVE_PROTOBUF", ["CONF"]),
                ("HAVE_PNG", ["CONF"]),
                ("USE_POWER_ISA", ["CONF"]),
                ("USE_MIPS_ISA", ["CONF"]),
                ("USE_ARM_ISA", ["CONF"]),
                ("USE_X86_ISA", ["CONF"]),
                ("USE_ARM_FASTMODEL", ["CONF"]),
                ("USE_RISCV_ISA", ["CONF"]),
                ("USE_SPARC_ISA", ["CONF"]),
                ("RUBY_PROTOCOL_MESI_Three_Level_HTM", ["CONF"]),
                ("USE_CAPSTONE", ["CONF"]),
                ("USE_KVM", ["CONF"]),
                ("BUILD_GPU", ["CONF"]),
            ]
            for switch, hierarchy in self.config_switches:
                switch_parent = self
                while hierarchy:
                    switch_parent = switch_parent[hierarchy[0]]
                    hierarchy = hierarchy[1:]
                switch_parent[switch] = default_config
                # TODO(hchsiao): remove hacks
                if switch in ["HAVE_DRAMSYS", "HAVE_DRAMSIM", "HAVE_DRAMSIM3"]:
                    switch_parent[switch] = False

        def __getattr__(self, attr):
            if attr == "Blob":
                return lambda *args: (None, None)
            elif attr == "TagImplies":
                return lambda *args: None
            elif attr == "subst":

                def _f(str_to_substitute):
                    return str_to_substitute.replace(
                        '${CONF["KVM_ISA"]}', "x86"
                    )

                return _f
            else:
                raise NotImplementedError(
                    f"Proxy for env.{attr} not implemented"
                )

    sconscript_content = []
    with open(sconscript) as f:
        sconscript_content = [
            l
            for l in f.readlines()
            if (
                (not re.match(r"^import", l))
                and (not re.match(r"^from.*import", l))
            )
        ]
    sconscript_content = "\n".join(sconscript_content)
    surrogates = {
        "with_tag": NameProxy("with_tag"),
        "with_any_tags": NameProxy("with_any_tags"),
        "Return": NameProxy("Return"),
        "Import": NameProxy("Import"),
        "Source": NameProxy("Source"),
        "SimObject": NameProxy("SimObject"),
        "GTest": NameProxy("GTest"),
        "ProtoBuf": NameProxy("ProtoBuf"),
        "Executable": NameProxy("Executable"),
        "SourceLib": NameProxy("SourceLib"),
        "CompoundFlag": NameProxy("CompoundFlag"),
        "DebugFormatFlag": NameProxy("DebugFormatFlag"),
        "TagImplies": NameProxy("TagImplies"),
        "GdbXml": NameProxy("GdbXml"),
        "DebugFlag": NameProxy("DebugFlag"),
        "PySource": NameProxy("PySource"),
        "ISADesc": NameProxy("ISADesc"),
    }
    declarations_by_mode = {
        "ALL_OFF": None,
        "ALL_ON": None,
    }
    for mode in declarations_by_mode:
        declarations = []
        if mode == "ALL_OFF":
            surrogates["env"] = EnvProxy(default_config=False)
        elif mode == "ALL_ON":
            surrogates["env"] = EnvProxy(default_config=True)
        else:
            assert False, "Not reachable"
        if not allow_exec:
            raise RuntimeError("Requires exec() for this to work")
        try:
            # pylint: disable=exec-used
            exec(sconscript_content, surrogates)
            # pylint: enable=exec-used
        except ReturnExceptionError:
            pass
        except Exception:
            logging.error("Error executing SConscript: %s", sconscript)
            raise
        declarations_by_mode[mode] = set(declarations)
    # Approximation of the config-to-declaration mapping (boolean function)
    # TODO(hchsiao): document the approach
    unconditional_declarations = (
        declarations_by_mode["ALL_ON"] & declarations_by_mode["ALL_OFF"]
    )
    conditional_declarations = (
        declarations_by_mode["ALL_ON"] | declarations_by_mode["ALL_OFF"]
    ) - unconditional_declarations
    if conditional_declarations:
        marginal_decl_by_switch = {k: {} for k in ["LEFT_ONE", "ADD_ONE"]}
        for mode in declarations_by_mode:
            if mode == "ALL_OFF":
                surrogates["env"] = EnvProxy(default_config=False)
                inverse = True
                marginal_type = "ADD_ONE"
            elif mode == "ALL_ON":
                surrogates["env"] = EnvProxy(default_config=True)
                inverse = False
                marginal_type = "LEFT_ONE"
            else:
                assert False, "Not reachable"
            for switch, hierarchy in surrogates["env"].config_switches:
                switch_parent = surrogates["env"]
                while hierarchy:
                    switch_parent = switch_parent[hierarchy[0]]
                    hierarchy = hierarchy[1:]
                switch_parent[switch] = inverse
                declarations = []
                try:
                    # pylint: disable=exec-used
                    exec(sconscript_content, surrogates)
                    # pylint: enable=exec-used
                except ReturnExceptionError:
                    pass
                except Exception:
                    logging.error("Error executing SConscript: %s", sconscript)
                    raise
                marginal_decl_by_switch[marginal_type][switch] = set(
                    declarations
                )
    all_declarations = []
    unconditional_declarations = sorted(unconditional_declarations)
    conditional_declarations = sorted(conditional_declarations)
    for decl in unconditional_declarations:
        decl = {k: v for k, v in decl}
        decl["condition"] = {}
        all_declarations.append(decl)
    for decl in conditional_declarations:
        cond = None
        for switch, hierarchy in surrogates["env"].config_switches:
            in_left1_set = decl in marginal_decl_by_switch["LEFT_ONE"][switch]
            in_add1_set = decl in marginal_decl_by_switch["ADD_ONE"][switch]
            in_all1_set = decl in declarations_by_mode["ALL_ON"]
            in_all0_set = decl in declarations_by_mode["ALL_OFF"]
            if (
                in_all1_set
                and in_add1_set
                and not in_all0_set
                and not in_left1_set
            ):
                cond = {switch: True}
                break
            if (
                in_all0_set
                and in_left1_set
                and not in_all1_set
                and not in_add1_set
            ):
                cond = {switch: False}
                break
        if cond is None:
            raise NotImplementedError()
        decl = {k: v for k, v in decl}
        decl["condition"] = cond
        all_declarations.append(decl)
    retval = {
        "Source": [],
        "SimObject": [],
        "DebugFlag": [],
        "DebugFormatFlag": [],
        "CompoundFlag": [],
    }
    for decl in all_declarations:
        if "appends" in decl:
            decl["appends"] = {k: v for k, v in decl["appends"]}
        retval[decl["type"]].append(decl)
    return retval


def scan_sim_object(
    # pylint: disable=unused-argument
    name: str,
    # pylint: enable=unused-argument
    py_file: types.Path,
    py_file_from_sim_object: dict[str, str],
    package_from_enum: dict[str, str],
    allow_exec: bool,
) -> tuple[list[str], list[str]]:
    """Input a SimObject parameter file and get its dependencies.

    The current implementation is extremely lengthy and highly relying on
    heuristics (and hacks).  Waiting to be refactored.

    TODO(hchsiao): The current implementation will lump all SimObjects in the
      same file as a single entity (with one slightly larger target).  This can
      be improved by making one target per SimObject so finer-grained build can
      be achieved as well as better performance.

    Args:
        name: Reserved for finer-grained target enhancement. Currently not used.
        py_file: Path to the SimObject parameter file to scan.
        py_file_from_sim_object: A map from SimObject names to the parameter file
          it's declared.  Needed for deriving the dependencies.
        package_from_enum: A map from enums declared with SimObject to the
          corresponding Bazel package.  Needed for deriving the dependencies.
        allow_exec: Instead of implementing a parser, this function leverages
          python's `exec()` to collect declarations by injecting fake names to the
          execution scope.  This switch is simply a safe guard to inform the user
          on the fact that this is going to cause the input to be executed.

    Returns:
        Two lists representing dependencies (in Bazel label form) and the list of
        `cxx_header` specified in the parameter file.  For example:

        ([
          "//path/to/package:dep_a",
          "//path/to/another/package:dep_b",
        ], [
          "my_sim_object.hh",
        ])

    Raises:
        RuntimeError: Will be triggered on `exec()` if not `allow_exec`.
    """
    # Hardcode (hack) dependencies if failed to parse
    if py_file.abs.endswith("src/python/m5/objects/SimObject.py"):
        return ["//src/base:types"], []
    if py_file.abs.endswith("src/sim/Workload.py"):
        return [
            "//src/generated/enums:KernelPanicOopsBehaviour",
            "//src/base:socket",
            "//src/generated/params:SimObject",
        ], []
    if py_file.abs.endswith("src/sim/SubSystem.py"):
        return [
            "//src/generated/params:SimObject",
            "//src/generated/params:ThermalDomain",
        ], []
    if py_file.abs.endswith("src/sim/Process.py"):
        return [
            "//src/generated/params:System",
            "//src/generated/params:SimObject",
            "//src/base:types",
        ], []
    if "x86" in py_file.abs_dirname:
        return [], []
    if "arch/arm" in py_file.abs_dirname or "dev/arm" in py_file.abs_dirname:
        return [], []
    if py_file.abs.endswith("src/cpu/o3/FUPool.py"):
        return [
            "//src/generated/params:SimObject",
            "//src/generated/params:FUDesc",
        ], []
    if py_file.abs.endswith("src/cpu/o3/BaseO3CPU.py"):
        return [
            "//src/generated/params:FUPool",
            "//src/generated/params:BranchPredictor",
            "//src/generated/params:BaseCPU",
            "//src/generated/enums:CommitPolicy",
            "//src/generated/enums:SMTQueuePolicy",
            "//src/generated/enums:SMTFetchPolicy",
            "//src/base:types",
        ], []
    if py_file.abs.endswith(
        "src/cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTraffic.py"
    ):
        return [], []

    if py_file.abs.endswith("src/cpu/o3/probe/ElasticTrace.py"):
        return [
            "//src/generated/params:ProbeListenerObject",
            "//src/base:types",
        ], []

    if not allow_exec:
        raise RuntimeError("Requires exec() for this to work")

    assert os.path.isfile(py_file.abs)
    sim_object_code = []
    cxx_headers = []
    with open(py_file.abs) as f:
        reading_head = True
        for line in f:
            if re.match(r"^class ", line):
                reading_head = False
            if reading_head:
                continue  # ignore various forms of `import`
            if sim_object_code and sim_object_code[-1].endswith("\\\n"):
                # if the previous line not terminated
                sim_object_code[-1] = sim_object_code[-1][:-2] + line
            else:
                if "cxx_header" in line:
                    cxx_header = (
                        line.replace('"', "'")
                        .replace("'", "")
                        .split("=", 1)[1]
                        .strip()
                    )
                    cxx_headers.append(cxx_header)
                    assert "(" not in cxx_header  # e.g. cxx_header = (\n
                sim_object_code.append(line)
    sim_object_code = "\n".join(sim_object_code)
    # The wording "parse" here actually means `exec()`
    parsed_deps = []
    parsed_param_deps = []
    parsed_enum_deps = []

    class FakeParam:
        """The surrogate for undefined stuff to make up `exec()` scope."""

        def __getattr__(self, name):
            if name in [
                "Counter",
                "Cycles",
                "Tick",
                "MaxTick",
                "Addr",
                "MicroPC",
                "MicroPCRomBit",
                "MaxAddr",
                "RegValue",
                "RegIndex",
                "ThreadID",
                "ContextID",
                "PortID",
                "Falut",
                "NoFault",
                "FaultBase",
            ]:
                parsed_deps.append("//src/base:types")
            elif name in py_file_from_sim_object:
                parsed_param_deps.append(name)
            elif name in package_from_enum:
                parsed_enum_deps.append(name)
            elif name in ["AddrRange"]:
                parsed_deps.append("//src/base:addr_range")
            elif name in ["Temperature"]:
                parsed_deps.append("//src/base:temperature")
            elif name in ["HostSocket"]:
                parsed_deps.append("//src/base:socket")
            elif name in ["EthernetAddr"]:
                parsed_deps.append("//src/base:inet")
            elif name in ["PcCountPair"]:
                parsed_deps.append("//src/cpu/probes:pc_count_pair")

            # pylint: disable=unused-argument
            def _callable(*args):
                pass

            # pylint: enable=unused-argument

            return _callable

        def __call__(self, *args):
            pass

    class FakeSelf:

        def __getattr__(self, name):
            return 1

    class FakeScope(dict):
        """The surrogate for undefined stuff to make up `exec()` scope."""

        def __init__(self):
            # pylint: disable=unused-argument
            def _callable(**kwargs):
                pass

            # pylint: disable=invalid-name
            def cxxMethod(*args, **kwargs):
                return lambda _: None

            # pylint: enable=invalid-name
            # pylint: enable=unused-argument

            self["cxxMethod"] = cxxMethod
            self["default_tracer"] = None
            self["ResetResponsePort"] = lambda _: None
            self["IntSinkPin"] = lambda _: None
            self["IntSourcePin"] = lambda _: None
            self["VectorIntSinkPin"] = lambda _: None
            self["VectorIntSourcePin"] = lambda _: None
            self["PyBindMethod"] = lambda _: None
            self["Self"] = FakeSelf()
            self["Parent"] = FakeSelf()
            for sim_object in py_file_from_sim_object.keys():
                self[sim_object] = _callable
            self["RiscvUart8250"] = (
                _callable  # SimObject not in sim_objects=[...]
            )
            self["Param"] = FakeParam()
            self["VectorParam"] = FakeParam()
            self["ResponsePort"] = FakeParam()
            self["AddrRange"] = FakeParam()
            # self["Param"] = None
            # self["VectorParam"] = None
            self["Enum"] = None
            self["ScopedEnum"] = None
            self["Bool"] = None
            self["String"] = None
            self["Float"] = None
            self["Int"] = None
            self["Unsigned"] = None
            self["Int8"] = None
            self["UInt8"] = None
            self["Int16"] = None
            self["UInt16"] = None
            self["Int32"] = None
            self["UInt32"] = None
            self["Int64"] = None
            self["UInt64"] = None
            self["Counter"] = None
            self["Addr"] = None
            self["Tick"] = None
            self["Percent"] = None
            self["TcpPort"] = None
            self["UdpPort"] = None
            self["EthernetAddr"] = None
            self["IpAddress"] = None
            self["IpNetmask"] = None
            self["IpWithPort"] = None
            self["MemorySize"] = None
            self["MemorySize32"] = None
            self["Latency"] = None
            self["Frequency"] = None
            self["Clock"] = None
            self["Voltage"] = None
            self["Current"] = None
            self["Energy"] = None
            self["Temperature"] = None
            self["NetworkBandwidth"] = None
            self["MemoryBandwidth"] = None
            # self["AddrRange"] = None
            self["MaxAddr"] = None
            self["MaxTick"] = None
            self["AllMemory"] = None
            self["Time"] = None
            self["NextEthernetAddr"] = None
            self["NULL"] = None
            self["Port"] = None
            self["RequestPort"] = lambda _: None
            # self["ResponsePort"] = None
            self["MasterPort"] = None
            self["SlavePort"] = lambda _: None
            self["VectorPort"] = None
            self["VectorRequestPort"] = lambda _: None
            self["VectorResponsePort"] = lambda _: None
            self["VectorMasterPort"] = None
            self["VectorSlavePort"] = None
            self["DeprecatedParam"] = lambda _1, _2: None
            self["PcCountPair"] = None

        def __getitem__(self, name):
            class FakeSimObject:

                def __init__(self, *args, **kwargs):
                    pass

            if name == "add_citation":
                return lambda _1, _2: None
            elif name == "buildEnv":
                return {"HAVE_TUNTAP": True}
            elif name in ["Enum", "ScopedEnum"]:
                pass
            else:
                parsed_param_deps.append(name)
            return FakeSimObject

    try:
        # pylint: disable=exec-used
        exec(sim_object_code, FakeScope())
        # pylint: enable=exec-used
    except Exception:
        logging.error("Error executing python file: %s", py_file.abs)
        raise
    altered_param_deps = []
    for param_dep in parsed_param_deps:
        if param_dep not in py_file_from_sim_object:
            continue
        dep_py_file = py_file_from_sim_object[param_dep]
        if dep_py_file == os.path.basename(py_file.abs):
            continue
        altered_param_deps.append(
            f"//src/generated/params:{dep_py_file[:-3]}_py"
        )
    altered_enum_deps = [
        f"//src/generated/enums:{dep}" for dep in parsed_enum_deps
    ]
    return (
        list(set(parsed_deps + altered_param_deps + altered_enum_deps)),
        cxx_headers,
    )
