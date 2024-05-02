# Copyright (c) 2024 The Regents of the University of California
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

"""This script is used for checking that SimObject Vectorsare
correctly parsed through to the gem5 PyStats."""

import m5
from m5.objects import (
    Root,
    ScalarStatTester,
    VectorStatTester,
)
from m5.stats.gem5stats import get_simstat

root = Root(full_system=False)
root.stat_testers = [
    ScalarStatTester(name="placeholder", value=11),
    ScalarStatTester(
        name="placeholder", value=22, description="Index 2 desc."
    ),
    ScalarStatTester(name="placeholder", value=33),
    VectorStatTester(
        name="index_4",
        values=[44, 55, 66],
        description="A SimStat Vector within a SimObject Vector.",
    ),
]

m5.instantiate()
m5.simulate()

simstat = get_simstat(root)

# 'stat_testers' is a list of SimObjects
assert hasattr(simstat, "stat_testers"), "No stat_testers attribute found."
assert len(simstat.stat_testers) == 4, "stat_testers list is not of length 3."

# Accessable by index.
simobject = simstat.stat_testers[0]

# We can directly access the statistic we're interested in and its "str"
# representation should be the same as the value we set. In this case "11.0".
assert (
    str(simobject.placeholder) == "11.0"
), "placeholder value is not 11.0 ()."

# They can also be accessed like so:
# "other_stat" is a SimObject with a single stat called "stat".
str(
    simstat["stat_testers"][3]["index_4"][0]
) == "44.0", 'simstat[3]["index_4"][0] value is not 44.'

# We can also access other stats like type and description.
assert simstat.stat_testers[1].placeholder.description == "Index 2 desc."
assert simstat.stat_testers[1].placeholder.type == "Scalar"
