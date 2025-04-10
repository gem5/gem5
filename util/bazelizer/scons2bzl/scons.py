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

"""Class definitions to mimic the hierarchy of SConstruct and SConscript.

Typical usage example:

  scons = SConstruct(sconstruct)  # walks through SConscripts under the hood
  scons.bazelize(True)  # populates BUILD files by reading many files
  scons.finalize()  # finalize generated BUILD files
"""

import logging
import os

import scons2bzl.dependency_scanner as depscan
from scons2bzl import (
    io,
    workarounds,
)
from scons2bzl.defines import (
    config_setting_from_tag,
    ext_lib_map,
    hardcoded_contents,
)
from scons2bzl.types import (
    AbsPath,
    Path,
    RelPath,
)


def _path_to_label(path, target_prefix=""):
    """Converts a path to a Bazel label."""
    # Handle circular dependency workarounds
    if path in workarounds.hdrs_cut_circular_dep:
        target_prefix = "workaround_that_cuts_circular_dependency__"

    # External library mappings
    for pattern, target in ext_lib_map.items():
        if path.startswith(pattern):
            return target

    # Mapping for generated files under specific directories
    generated_directories = {
        "params/": "//src/generated/params",
        "enums/": "//src/generated/enums",
        "debug/": "//src/generated/debug",
        "config/": "//src/generated/config",
    }
    for directory, label_prefix in generated_directories.items():
        if path.startswith(directory):
            sim_object, _ = os.path.splitext(os.path.basename(path))
            return f"{label_prefix}:{sim_object}"

    # Mapping for source files under 'src/'
    if path.endswith((".cc", ".hh", ".h")):
        package, file = path.rsplit("/", 1)
        file_name, _ = os.path.splitext(file)
        target = target_prefix + file_name.replace(".", "_")
        return f"//src/{package}:{target}"

    # Handle unknown paths
    raise NotImplementedError(f"No label mapping found for path: {path}")


def _process_appends(appends):
    retval = {"copts": []}
    for flag_group, flag in appends.items():
        if flag_group in ["CXXFLAGS"]:
            bazel_opt_name = "copts"
        else:
            raise NotImplementedError(
                f"Do not know how to handle {flag_group}"
            )
        retval[bazel_opt_name].append(flag)
    return retval


class SConscript:
    """Mimics the hierarchy of a SConscript."""

    def __init__(self, path, gem5_home, allow_exec):
        if not allow_exec:
            raise RuntimeError(
                "Requires SConscript execution for this to work"
            )

        self.declarations = depscan.scan_sconscript(path, allow_exec)
        self.path = path

        self.obj_targets = []

        # Export data
        self.includes = []
        self.sim_objects_from_py_file = {}
        self.py_file_from_sim_object = {}
        self.package_from_enum = {}
        self.tags_from_labels = {}
        self.cond_from_labels = {}

        self.gem5_home = gem5_home

        for spec in self.declarations["Source"]:
            src = self.path.sibling(spec["src"])
            assert src.ext == ".cc"
            label = _path_to_label(os.path.relpath(src.rel, "src"))
            self.tags_from_labels[label] = spec["tags"]
            self.cond_from_labels[label] = spec["condition"]

    def _get_precondition_from_tags(self, tags):
        tag = tags[0].replace(
            " ", "_"
        )  # TODO(hchsiao): handle more than 1 tag
        return config_setting_from_tag[tag]

    def get_precondition_from_cond(self, cond):
        if not cond:
            return True
        if len(cond) > 1:
            raise NotImplementedError()
        switch, value = list(cond.items())[0]
        return switch.lower() + "_is_true" if value else "_is_false"

    def get_precondition_from_label(self, label):
        tags_config_setting, cond_config_setting = True, True
        if label in self.global_tags_from_labels:
            l_tags = self.global_tags_from_labels[label]
            tags_config_setting = self._get_precondition_from_tags(l_tags)
        if label in self.global_cond_from_labels:
            l_cond = self.global_cond_from_labels[label]
            cond_config_setting = self.get_precondition_from_cond(l_cond)
        if tags_config_setting == cond_config_setting:
            return tags_config_setting
        else:  # TODO(hchsiao): remove hack for merge path
            return workarounds.merge_config_settings(
                tags_config_setting, cond_config_setting
            )

    def bazelize(self, all_tags_from_labels, all_cond_from_labels):
        """Performs BUILD file generation."""
        self.global_tags_from_labels = all_tags_from_labels
        self.global_cond_from_labels = all_cond_from_labels

        for decl_type, specs in self.declarations.items():
            if decl_type in ["DebugFlag", "DebugFormatFlag", "CompoundFlag"]:
                handler_name = "handle_debugflag_variations"
            else:
                handler_name = f"handle_{decl_type.lower()}"
            handler = getattr(self, handler_name)
            for spec in specs:
                handler(spec)

        self._generate_obj_labels()

    def _generate_obj_labels(self):
        """Generates labels for object targets and updates the BUILD file.

        Objects will be added to the :lib target with a SConscript adjacent to
        the belonging BUILD file instead of the nearest BUILD file.
        For example, "//src/kern/freebsd:events" is added to "//src/kern:lib"
        instead of "//src/kern/freebsd:lib".
        """
        build_file = self.path.sibling(Path.BUILD_FILE)
        labels = []
        # TODO(hchsiao): Extra care on `obj_targets` ordering is
        # required if we want to drop `alwayslink = True` in `cc_library`.
        for target in self.obj_targets:
            label = f"//{target}"
            config_setting = self.get_precondition_from_label(label)
            if isinstance(config_setting, str):
                labels.append(
                    "] + select({\n"
                    f'"//src/generated/flags:{config_setting}": ["//{target}"],\n'
                    '"//conditions:default": [],\n'
                    "}) + ["
                )
            elif config_setting:
                labels.append(f'"//{target}",')
        io.update_build(build_file, "OBJS_GOES_HERE", "\n".join(labels))

    def handle_source(self, spec):
        """Handles a Source(...) declaration in SConscript."""
        src = self.path.sibling(spec["src"])
        appends = _process_appends(spec["appends"])
        name = src.no_ext_basename
        cc_name = f"{name}.cc"
        hh_name = f"{name}.hh"
        cc = src.sibling(cc_name)
        hh = src.sibling(hh_name)
        has_header = os.path.isfile(hh.abs)
        hdrs = [hh] if has_header else []
        deps = list(depscan.scan_includes(cc, hdrs))
        hdr_deps = list(depscan.scan_includes(hh)) if has_header else []
        self.includes.extend(deps + hdr_deps)

        dep_labels = self._generate_dependency_labels(deps + hdr_deps)
        package = hh.rel_dirname
        build_file = src.sibling(Path.BUILD_FILE)
        io.update_build(
            build_file,
            "TARGET_GOES_HERE",
            "\n"
            + io.TARGET_ENTRY_TEMPLATE.format(
                name=name,
                srcs=f'"{cc_name}"',
                hdrs=", ".join([f'"{t.basename}"' for t in hdrs]),
                deps="".join(dep_labels),
                prefix=os.path.relpath(package, "src"),
                copts=", ".join([f'"{opt}"' for opt in appends["copts"]]),
            ),
        )
        self.obj_targets.append(f"{package}:{name}")

    def _generate_dependency_labels(self, dependencies):
        """Generates dependency labels with conditional select statements."""
        dep_labels = []
        for inc in dependencies:
            label = _path_to_label(inc)
            quoted_label = f'"{label}"'
            config_setting = self.get_precondition_from_label(label)
            if isinstance(config_setting, str):
                quoted_label = (
                    f'] + select({{"//src/generated/flags:{config_setting}":'
                    f' [{quoted_label}], "//conditions:default": []}}) + ['
                )
            elif config_setting:
                quoted_label += ", "
            dep_labels.append(quoted_label)
        # Keep the deps ordering to avoid BUILD file alternation
        return sorted(set(dep_labels))

    @workarounds.hack_sim_object
    def handle_simobject(self, spec):
        """Handles a SimObject(...) declaration in SConscript."""
        src = self.path.sibling(spec["src"])
        enums = spec["enums"]
        cond = spec["condition"]
        sim_objects = spec["sim_objects"]
        assert src.ext == ".py"
        package = src.rel_dirname
        self.sim_objects_from_py_file[src] = sim_objects
        for sim_object in sim_objects:
            self.py_file_from_sim_object[sim_object] = src.basename
        for enum in enums:
            self.package_from_enum[enum] = package
        # TODO(hchsiao): consider tags in addition to condition
        config_setting = self.get_precondition_from_cond(cond)
        conditional = isinstance(config_setting, str)
        build_file = src.sibling(Path.BUILD_FILE)
        if conditional:
            io.update_build(
                build_file,
                "SIMOBJ_GOES_HERE",
                f'] + select({{"//src/generated/flags:{config_setting}": [',
            )
        io.update_build(
            build_file,
            "SIMOBJ_GOES_HERE",
            io.SIMOBJ_ENTRY_TEMPLATE.format(
                file=src.basename,
                sim_objects=", ".join([f'"{so}"' for so in sim_objects]),
                enums=", ".join([f'"{e}"' for e in enums]),
            ),
        )
        if conditional:
            io.update_build(
                build_file,
                "SIMOBJ_GOES_HERE",
                '], "//conditions:default": []}) + [',
            )

    def handle_debugflag_variations(self, spec):
        """Handles a DebugFlag(...) declaration or its variant in SConscript."""
        name = spec["name"]
        flags = spec["flags"]
        format_flag = spec["format_flag"]
        desc = spec["description"]
        build_file = self.gem5_home.append(
            "src/generated/debug", Path.BUILD_FILE
        )
        io.update_build(
            build_file,
            "DEBUG_FLAG_GOES_HERE",
            io.DEBUG_FLAG_ENTRY_TEMPLATE.format(
                name=name,
                desc=desc,
                flags=", ".join([f'"{f}"' for f in flags]),
                is_fmt="True" if format_flag else "False",
            ),
        )


class SConstruct:
    """Mimics the hierarchy of a SConstruct."""

    def __init__(self, sconstruct):
        gem5_home = os.path.dirname(sconstruct)
        self.gem5_home = AbsPath(gem5_home, base=gem5_home)
        self.sconscript_files = []
        self.build_files = []
        for root, _, files in os.walk(gem5_home):
            root_path = AbsPath(root, base=gem5_home)
            skip = False
            for excl in workarounds.exclude_recursive:
                if root_path.startswith(excl):
                    skip = True
                    break
            for excl in workarounds.exclude:
                if root_path == RelPath(excl, base=gem5_home):
                    skip = True
                    break
            for file in files:
                if file == Path.SCONSCRIPT and not skip:
                    self.sconscript_files.append(root_path.append(file))
                elif file == Path.BUILD_FILE:
                    self.build_files.append(root_path.append(file))

    def reset_generated(self):
        for build_file in self.build_files:
            io.remove_if_generated(build_file)

    def populate_build_files(self, allow_code_execution):
        """Populates BUILD files.

        This spreads BUILD.bazel recursively for each folders and creates basic
        compilation units, e.g. cc_library for `*.o`.
        """
        # 1st pass
        self.sconscripts = []
        tags_from_labels = {}
        cond_from_labels = {}
        for sconscript_file in self.sconscript_files:
            sconscript = SConscript(
                sconscript_file, self.gem5_home, allow_code_execution
            )
            tags_from_labels.update(sconscript.tags_from_labels)
            cond_from_labels.update(sconscript.cond_from_labels)
            self.sconscripts.append(sconscript)
        # 2nd pass
        self.includes = []
        self.py_file_from_sim_object = {}
        self.sim_objects_from_py_file = {}
        self.package_from_enum = {}
        for sconscript in self.sconscripts:
            sconscript.bazelize(tags_from_labels, cond_from_labels)
            self.includes += sconscript.includes
            self.sim_objects_from_py_file.update(
                sconscript.sim_objects_from_py_file
            )
            self.py_file_from_sim_object.update(
                sconscript.py_file_from_sim_object
            )
            self.package_from_enum.update(sconscript.package_from_enum)

    def generate_params(self, allow_code_execution):
        """Generates SimObject params.

        Creating targets for SimObject headers under `params/`.
        """
        for py_file, sim_objects in self.sim_objects_from_py_file.items():
            py_name = py_file.no_ext_basename
            package = py_file.rel_dirname
            deps, cxx_headers = depscan.scan_sim_object(
                None,
                py_file,
                self.py_file_from_sim_object,
                self.package_from_enum,
                allow_code_execution,
            )
            cc_deps = [_path_to_label(hdr) for hdr in cxx_headers]
            cc_deps += deps + [
                "//src/base:compiler",
                "//src/sim:init",
                "//src/sim:workaround_that_cuts_circular_dependency__sim_object",
            ]
            deps = sorted(set(deps))
            cc_deps = sorted(set(cc_deps))
            build_file = self.gem5_home.append(
                "src/generated/params", Path.BUILD_FILE
            )
            py_hdrs = ", ".join(
                [f'":{sim_obj}_aux_hh"' for sim_obj in sim_objects]
            )
            py_ccs = ", ".join(
                [f'":{sim_obj}_aux_cc"' for sim_obj in sim_objects]
            )
            io.update_build(
                build_file,
                "TARGET_GOES_HERE",
                "\n".join(
                    [
                        io.AUX_TARGET_TEMPLATE.format(
                            name=f"{sim_obj}_aux_hh",
                            src=f"//{package}:{sim_obj}_param_hh",
                            out=f"{sim_obj}.hh",
                        )
                        for sim_obj in sim_objects
                    ]
                    + [
                        io.AUX_TARGET_TEMPLATE.format(
                            name=f"{sim_obj}_aux_cc",
                            src=f"//{package}:{sim_obj}_param_cc",
                            out=f"{sim_obj}.cc",
                        )
                        for sim_obj in sim_objects
                    ]
                    + [
                        io.TARGET_ENTRY_TEMPLATE.format(
                            name=f"{sim_obj}",  # per child class
                            srcs="",
                            hdrs="",
                            deps=f'":{py_name}_py"',
                            prefix="params",
                            copts="",
                        )
                        for sim_obj in sim_objects
                    ]
                    + [
                        io.TARGET_ENTRY_TEMPLATE.format(
                            name=f"{py_name}_all",
                            srcs=py_ccs,
                            hdrs=py_hdrs,
                            deps=", ".join([f'"{t}"' for t in cc_deps]),
                            prefix="params",
                            copts="",
                        ),
                        io.TARGET_ENTRY_TEMPLATE.format(
                            name=f"{py_name}_py",  # hdrs only version
                            srcs="",
                            hdrs=py_hdrs,
                            deps=", ".join([f'"{t}"' for t in deps]),
                            prefix="params",
                            copts="",
                        ),
                    ]
                ),
            )
            if workarounds.sim_object_param_filter(py_name):
                io.update_build(
                    build_file, "OBJS_GOES_HERE", f'":{py_name}_all",'
                )

    def generate_enums(self):
        """Generates SimObject enums.

        Creating targets for SimObject headers under `enums/`
        """
        for enum, package in self.package_from_enum.items():
            deps = ["//src/base:compiler", "//src/sim:init", "@pybind11//:lib"]
            build_file = self.gem5_home.append(
                "src/generated/enums", Path.BUILD_FILE
            )
            io.update_build(
                build_file,
                "TARGET_GOES_HERE",
                "\n".join(
                    [
                        io.AUX_TARGET_TEMPLATE.format(
                            name=f"{enum}_aux_hh",
                            src=f"//{package}:{enum}_enum_hh",
                            out=f"{enum}.hh",
                        ),
                        io.AUX_TARGET_TEMPLATE.format(
                            name=f"{enum}_aux_cc",
                            src=f"//{package}:{enum}_enum_cc",
                            out=f"{enum}.cc",
                        ),
                        io.TARGET_ENTRY_TEMPLATE.format(
                            name=f"{enum}",
                            srcs=f'"{enum}_aux_cc"',
                            hdrs=f'"{enum}_aux_hh"',
                            deps=", ".join([f'"{t}"' for t in deps]),
                            prefix="enums",
                            copts="",
                        ),
                    ]
                ),
            )

    def detect_nested_includes(self):
        """Detects nested (indirect) includes."""
        includes = list(set(self.includes))
        non_exist_hdrs = []
        while True:
            new_nested_hdrs = []
            for hdr in includes:
                hdr_path = self.gem5_home.append("src", hdr)
                if not os.path.isfile(hdr_path.abs):
                    # `hdr` is either a generated file or not in src/
                    # nothing available for scanning at this point
                    logging.debug(
                        "%s not found while scanning nested includes", hdr
                    )
                    non_exist_hdrs.append(hdr)
                    continue
                for dep in depscan.scan_includes(hdr_path):
                    if dep not in includes:
                        new_nested_hdrs.append(dep)
            if new_nested_hdrs:
                includes = list(set(includes + new_nested_hdrs))
                new_nested_hdrs = []
            else:
                includes = set(includes)
                break
        if non_exist_hdrs:
            includes -= set(non_exist_hdrs)
        self.includes = includes

    def append_sole_headers(self):
        """Appends headers without a `.cc`.

        This step creates targets with sole header files for:
        1. to overcome circular deps (`//...:workaround_*` targets).
        2. headers without a `*.cc`.
        """
        includes = sorted(set(self.includes))
        for hdr in includes:
            hdr_path = self.gem5_home.append("src", hdr)
            package = hdr_path.rel_dirname
            if os.path.relpath(package, "src") in ["params", "enums"]:
                continue
            have_cc = os.path.isfile(
                hdr_path.sibling(f"{hdr_path.no_ext_basename}.cc").abs
            )
            if not have_cc or hdr in workarounds.hdrs_cut_circular_dep:
                label = _path_to_label(hdr)
            else:
                continue
            _, target = label.rsplit(":")
            build_file = hdr_path.sibling(Path.BUILD_FILE)
            deps = list(depscan.scan_includes(hdr_path))
            dep_labels = sorted({_path_to_label(inc) for inc in deps})
            io.update_build(
                build_file,
                "TARGET_GOES_HERE",
                "\n"
                + io.TARGET_ENTRY_TEMPLATE.format(
                    name=target,
                    srcs="",
                    hdrs=f'"{hdr_path.basename}"',
                    deps=", ".join([f'"{t}"' for t in dep_labels]),
                    prefix=os.path.relpath(package, "src"),
                    copts="",
                ),
            )

    def grow_dependency_tree(self):
        """Grows dependency tree.

        Adding subdirectory :lib and :sim_objects targets for the parent :lib.
        """
        for root, dirs, _ in os.walk(self.gem5_home.abs):
            root_path = AbsPath(root, base=self.gem5_home.abs)
            package = root_path.rel
            if not package.startswith("src"):
                continue
            build_file = root_path.append(Path.BUILD_FILE)
            for d in sorted(dirs):
                lib_excluded = (
                    f"//{package}/{d}:lib" in workarounds.exclude_lib
                )
                sim_obj_excluded = (
                    f"//{package}/{d}:sim_objects"
                    in workarounds.exclude_sim_objects
                )
                if os.path.isfile(root_path.append(d, Path.BUILD_FILE).abs):
                    prefix = "# " if lib_excluded else ""
                    io.update_build(
                        build_file,
                        "OBJS_GOES_HERE",
                        "\n".join([f'{prefix}"//{package}/{d}:lib",']),
                    )
                    prefix = "# " if sim_obj_excluded else ""
                    io.update_build(
                        build_file,
                        "SIMOBJ_LIB_GOES_HERE",
                        "\n".join([f'{prefix}"//{package}/{d}:sim_objects",']),
                    )

    def manual_contents(self):
        """Injects manually written stuff to generated files."""
        for package, insert_type in hardcoded_contents:
            content = hardcoded_contents[package, insert_type]
            build_file = self.gem5_home.append(package, Path.BUILD_FILE)
            io.update_build(build_file, insert_type, content)
        build_file = self.gem5_home.append("src/base", Path.BUILD_FILE)
        io.update_build(
            build_file,
            "TARGET_GOES_HERE",
            "\n"
            + io.TARGET_ENTRY_TEMPLATE.format(
                name="manual_date",
                srcs='"date.cc"',
                hdrs="",
                deps="",
                prefix="base",
                copts="",
            ),
        )
        build_file = self.gem5_home.append("src/proto", Path.BUILD_FILE)
        io.update_build(
            build_file,
            "TARGET_GOES_HERE",
            "\n"
            + io.TARGET_ENTRY_TEMPLATE.format(
                name="manual_pb",
                srcs="",
                hdrs="",
                deps='":inst_pb", "inst_dep_record_pb", "packet_pb"',
                prefix="proto",
                copts="",
            ),
        )

    def bazelize(self, allow_code_execution):
        """Generates BUILD file for the entire project."""
        logging.info("Reset generated BUILD.bazel files")
        self.reset_generated()
        logging.info("Walking through SConscripts and populate BUILD.bazel")
        self.populate_build_files(allow_code_execution)
        logging.info("Creating targets for SimObject headers under `params/`")
        self.generate_params(allow_code_execution)
        logging.info("Creating targets for SimObject headers under `enums/`")
        self.generate_enums()
        logging.info("Scanning nested includes")
        self.detect_nested_includes()
        logging.info("Appending sole header targets to BUILD.bazel")
        self.append_sole_headers()
        logging.info("Append manually written (hardcoded) contents")
        self.manual_contents()

    def finalize(self):
        logging.info("Adding subdirectory for :lib and :sim_objects targets")
        self.grow_dependency_tree()
