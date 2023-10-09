# Copyright (c) 2019 Inria
# Copyright (c) 2012, 2017-2018 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
import inspect
import sys
from textwrap import TextWrapper

import m5.internal.params
import m5.objects
from gem5.runtime import get_supported_isas


class ObjectList(object):
    """Creates a list of objects that are sub-classes of a given class."""

    def _is_obj_class(self, cls):
        """Determine if a class is a a sub class of the provided base class
        that can be instantiated.
        """

        # We can't use the normal inspect.isclass because the ParamFactory
        # and ProxyFactory classes have a tendency to confuse it.
        try:
            return issubclass(cls, self.base_cls) and not cls.abstract
        except (TypeError, AttributeError):
            return False

    def get(self, name):
        """Get a sub class from a user provided class name or alias."""

        real_name = self._aliases.get(name, name)
        try:
            sub_cls = self._sub_classes[real_name]
            return sub_cls
        except KeyError:
            print(f"{name} is not a valid sub-class of {self.base_cls}.")
            raise

    def print(self):
        """Print a list of available sub-classes and aliases."""

        print(f"Available {self.base_cls} classes:")
        doc_wrapper = TextWrapper(
            initial_indent="\t\t", subsequent_indent="\t\t"
        )
        for name, cls in list(self._sub_classes.items()):
            print(f"\t{name}")

            # Try to extract the class documentation from the class help
            # string.
            doc = inspect.getdoc(cls)
            if doc:
                for line in doc_wrapper.wrap(doc):
                    print(line)

        if self._aliases:
            print("\Aliases:")
            for alias, target in list(self._aliases.items()):
                print(f"\t{alias} => {target}")

    def get_names(self):
        """Return a list of valid sub-class names and aliases."""
        return list(self._sub_classes.keys()) + list(self._aliases.keys())

    def _add_objects(self):
        """Add all sub-classes of the base class in the object hierarchy."""
        for name, cls in inspect.getmembers(m5.objects, self._is_obj_class):
            self._sub_classes[name] = cls

    def _add_aliases(self, aliases):
        """Add all aliases of the sub-classes."""
        if aliases is not None:
            for alias, target in aliases:
                if target in self._sub_classes:
                    self._aliases[alias] = target

    def __init__(self, base_cls, aliases=None):
        # Base class that will be used to determine if models are of this
        # object class
        self.base_cls = base_cls
        # Dictionary that maps names of real models to classes
        self._sub_classes = {}
        self._add_objects()

        # Filtered list of aliases. Only aliases for existing objects exist
        # in this list.
        self._aliases = {}
        self._add_aliases(aliases)


class CPUList(ObjectList):
    def _is_obj_class(self, cls):
        """Determine if a class is a CPU that can be instantiated"""

        # We can't use the normal inspect.isclass because the ParamFactory
        # and ProxyFactory classes have a tendency to confuse it.
        try:
            return super(CPUList, self)._is_obj_class(cls) and not issubclass(
                cls, m5.objects.CheckerCPU
            )
        except (TypeError, AttributeError):
            return False

    def _add_objects(self):
        super(CPUList, self)._add_objects()

        from importlib import import_module

        for isa in {
            "generic",
        } | {isa.name.lower() for isa in get_supported_isas()}:
            try:
                package = import_module(
                    ".cores." + isa, package=__name__.rpartition(".")[0]
                )
            except ImportError:
                # No timing models for this ISA
                continue

            for mod_name, module in inspect.getmembers(
                package, inspect.ismodule
            ):
                for name, cls in inspect.getmembers(
                    module, self._is_obj_class
                ):
                    self._sub_classes[name] = cls


class EnumList(ObjectList):
    """Creates a list of possible values for a given enum class."""

    def _add_objects(self):
        """Add all enum values to the ObjectList"""
        self._sub_classes = {}
        for (key, value) in list(self.base_cls.__members__.items()):
            # All Enums have a value Num_NAME at the end which we
            # do not want to include
            if not key.startswith("Num_"):
                self._sub_classes[key] = value


rp_list = ObjectList(getattr(m5.objects, "BaseReplacementPolicy", None))
bp_list = ObjectList(getattr(m5.objects, "BranchPredictor", None))
cpu_list = CPUList(getattr(m5.objects, "BaseCPU", None))
hwp_list = ObjectList(getattr(m5.objects, "BasePrefetcher", None))
indirect_bp_list = ObjectList(getattr(m5.objects, "IndirectPredictor", None))
mem_list = ObjectList(getattr(m5.objects, "AbstractMemory", None))
dram_addr_map_list = EnumList(
    getattr(m5.internal.params, "enum_AddrMap", None)
)

# Platform aliases. The platforms listed here might not be compiled,
# we make sure they exist before we add them to the platform list.
_platform_aliases_all = [("VExpress_GEM5", "VExpress_GEM5_V1")]
platform_list = ObjectList(
    getattr(m5.objects, "Platform", None), _platform_aliases_all
)


def _subclass_tester(name):
    sub_class = getattr(m5.objects, name, None)

    def tester(cls):
        return (
            sub_class is not None
            and cls is not None
            and issubclass(cls, sub_class)
        )

    return tester


is_kvm_cpu = _subclass_tester("BaseKvmCPU")
is_noncaching_cpu = _subclass_tester("NonCachingSimpleCPU")
