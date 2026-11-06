# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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


"""Install named profile children using native SimObject ownership."""

import keyword
import re

from m5.params import VectorParamDesc
from m5.proxy import (
    AttrProxy,
    Self,
)
from m5.SimObject import MetaSimObject


def _reserved_names(container):
    names = set()
    for cls in (*container.__mro__, *type(container).__mro__):
        namespace = vars(cls)
        names.update(namespace)
        for field in ("_params", "_ports", "_deprecated_params", "_children"):
            names.update(namespace.get(field, {}).keys())
        for field in ("cxx_exports", "cxx_param_exports"):
            names.update(item.name for item in namespace.get(field, ()))
    names.update(MetaSimObject.init_keywords)
    names.update(MetaSimObject.keywords)
    # MetaSimObject.__getattr__ synthesizes these without storing attributes.
    names.update(
        ("cxx_class_path", "cxx_class_name", "cxx_namespaces", "pybind_class")
    )
    names.update(vars(Self))
    for cls in type(Self).__mro__:
        names.update(vars(cls))
    # These selectors produce AnyProxy/AllProxy rather than named references.
    names.update(("any", "all"))
    return names


def install_profile(container, entry, vector_name, entries, profile_path):
    """Populate a fixed collection class before it is instantiated."""
    entries = tuple(entries)
    reserved = _reserved_names(container)
    seen = set()
    proxies = []
    for alias, _params in entries:
        context = f"{profile_path}: {container.__name__} alias {alias!r}"
        if (
            not isinstance(alias, str)
            or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", alias)
            or keyword.iskeyword(alias)
        ):
            raise ValueError(context + " is not a supported identifier")
        if alias in reserved:
            raise ValueError(context + " conflicts with the object namespace")
        if alias in seen:
            raise ValueError(context + " is duplicated")
        seen.add(alias)
        proxy = getattr(Self, alias)
        if not isinstance(proxy, AttrProxy):
            raise ValueError(context + " conflicts with the proxy namespace")
        proxies.append(proxy)

    parameter = container._params.get(vector_name)
    if (
        not isinstance(parameter, VectorParamDesc)
        or parameter.ptype is not entry
    ):
        raise ValueError(
            f"{profile_path}: {vector_name} has the wrong entry type"
        )
    if container._instantiated:
        raise ValueError(
            f"{profile_path}: install profile before instantiation"
        )

    # Construct all entries before assigning children. Native cloning copies
    # these defaults; proxies resolve replacements made before instantiate.
    children = [(alias, entry(**params)) for alias, params in entries]
    for alias, child in children:
        setattr(container, alias, child)
    setattr(container, vector_name, proxies)
