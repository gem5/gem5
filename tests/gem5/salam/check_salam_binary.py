# Copyright (c) 2026 Akanksha Chaudhari
# All rights reserved.
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

# Runs inside gem5: exits nonzero unless the binary has the SALAM objects.

import json
import sys

import m5
import m5.objects

required = ("LLVMInterface", "CommInterface", "CycleCounts", "HWInterface")
missing = [name for name in required if not hasattr(m5.objects, name)]
if missing:
    print("missing SALAM SimObjects: " + ", ".join(missing))
    sys.exit(1)
print("SALAM SimObjects present")


if len(sys.argv) > 1:
    expected = set(json.loads(sys.argv[1]))
    modules = {"m5.objects.FunctionalUnits", "m5.objects.InstConfig"}
    actual = {
        name
        for name, cls in vars(m5.objects).items()
        if isinstance(cls, type) and cls.__module__ in modules
    }
    assert actual == expected, (
        sorted(expected - actual),
        sorted(actual - expected),
    )
    root = m5.objects.Root(full_system=False)
    root.units = m5.objects.FunctionalUnits.from_profile()
    root.instructions = m5.objects.InstConfig.from_profile()
    m5.instantiate()
    constructed = set()
    for container in (root.units, root.instructions):
        for obj in (container, *container._children.values()):
            assert obj.getCCObject() is not None
            constructed.add(type(obj).__name__)
    assert constructed == expected, (
        sorted(expected - constructed),
        sorted(constructed - expected),
    )
    print(
        f"SALAM profile: {len(constructed)} registered and constructed classes"
    )
