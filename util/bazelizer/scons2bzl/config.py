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

"""Gem5 config related logic of the BUILD file generator.

Typical usage example:

  configs = config.read_kconfig(gem5_home)
  config.update_build_files(gem5_home, configs)
"""

import os

import kconfiglib as kcfg
from scons2bzl import io
from scons2bzl.defines import config_headers
from scons2bzl.types import (
    AbsPath,
    Path,
)


class KconfigEnvContext:
    """Gem5 context for kconfiglib."""

    def __init__(self):
        self.old_env = os.environ.copy()

    def __enter__(self):
        # Enable all switches to get the maximum set of config options
        os.environ["HAVE_FENV"] = "y"
        os.environ["HAVE_PNG"] = "y"
        os.environ["HAVE_VALGRIND"] = "y"
        os.environ["HAVE_DEPRECATED_NAMESPACE"] = "y"
        os.environ["HAVE_POSIX_CLOCK"] = "y"
        os.environ["HAVE_HDF5"] = "y"
        os.environ["HAVE_PROTOBUF"] = "y"
        os.environ["HAVE_TUNTAP"] = "y"
        os.environ["HAVE_CAPSTONE"] = "y"

    def __exit__(self, exc_type, exc_value, traceback):
        os.environ = self.old_env


def read_kconfig(gem5_home: str) -> dict[str, str]:
    """Get the maximum set of config to type mapping by reading Kconfig files.

    Args:
        gem5_home: Path to the gem5 repository root.

    Returns:
        A dict mapping config names to the corresponding type name.  Implemented
        type names are ['bool', 'string', 'int'].  For example:

        {'have_abc': 'bool',
         'have_xyz': 'bool',
         'kvm_isa': 'string'}
    """
    gem5_home = AbsPath(gem5_home)
    type_from_config = {}

    def visit(node):
        while node:
            if not isinstance(node.item, int):
                config_type = kcfg.TYPE_TO_STR[node.item.type]
                assert config_type in ["bool", "string", "int"]
                type_from_config[node.item.name] = config_type
            if node.list:
                visit(node.list)
            node = node.next

    with KconfigEnvContext():
        kconf = kcfg.Kconfig(os.path.join(gem5_home.abs, "src/Kconfig"))
    visit(kconf.top_node)
    return type_from_config


def update_build_files(
    gem5_home: str, type_from_config: dict[str, str]
) -> None:
    """Update config related BUILD files.

    Args:
        gem5_home: Path to the gem5 repository root.
        type_from_config: Map of config names to type names.
    """
    gem5_home = AbsPath(gem5_home)
    build_file = gem5_home.append("src/generated/config", Path.BUILD_FILE)
    for config, decl in config_headers.items():
        io.update_build(build_file, "OBJS_GOES_HERE", f'":{config}",')
        io.update_build(
            build_file,
            "TARGET_GOES_HERE",
            io.CONFIG_HDR_ENTRY_TEMPLATE.format(config=config, decl=decl),
        )
    for conf_name, config_type in type_from_config.items():
        target_name = conf_name.lower()
        if config_type == "bool":
            default = "False"
        elif config_type == "int":
            default = 0
        elif config_type == "string":
            default = '""'
        # using flags/ instead of config/ because target name conflicts
        build_file = gem5_home.append("src/generated/flags", Path.BUILD_FILE)
        io.update_build(
            build_file,
            "TARGET_GOES_HERE",
            io.CONFIG_FLAG_ENTRY_TEMPLATE.format(
                config_type=config_type,
                target_name=target_name,
                default=default,
            ),
        )
        if config_type == "bool":
            io.update_build(
                build_file,
                "TARGET_GOES_HERE",
                io.CONFIG_SETTING_ENTRY_TEMPLATE.format(
                    target_name=target_name
                ),
            )
