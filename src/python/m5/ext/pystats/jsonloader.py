# Copyright (c) 2021 The Regents of The University of California
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
import json
from json.decoder import JSONDecodeError
from typing import IO
from typing import Union

from .group import Group
from .group import Vector
from .simstat import SimStat
from .statistic import Accumulator
from .statistic import Distribution
from .statistic import Scalar
from .statistic import Statistic


class JsonLoader(json.JSONDecoder):
    """
    Subclass of JSONDecoder that overrides 'object_hook'. Converts JSON object
    into a SimStat object.

    Usage
    -----
    ```
    from m5.ext.pystats.jsonloader import JsonLoader

    with open(path) as f:
        simstat_object = json.load(f, cls=JsonLoader)
    ```
    """

    def __init__(self):
        super().__init__(self, object_hook=self.__json_to_simstat)

    def __json_to_simstat(self, d: dict) -> Union[SimStat, Statistic, Group]:
        if "type" in d:
            if d["type"] == "Scalar":
                d.pop("type", None)
                return Scalar(**d)

            elif d["type"] == "Distribution":
                d.pop("type", None)
                return Distribution(**d)

            elif d["type"] == "Accumulator":
                d.pop("type", None)
                return Accumulator(**d)

            elif d["type"] == "Group":
                return Group(**d)

            elif d["type"] == "Vector":
                d.pop("type", None)
                d.pop("time_conversion", None)
                return Vector(d)

            else:
                raise ValueError(
                    f"SimStat object has invalid type {d['type']}",
                )
        else:
            return SimStat(**d)


def load(json_file: IO) -> SimStat:
    """
    Wrapper function that provides a cleaner interface for using the
    JsonLoader class.

    Usage
    -----
    ```
    import m5.ext.pystats as pystats

    with open(path) as f:
        pystats.jsonloader.load(f)
    ```
    """

    simstat_object = json.load(json_file, cls=JsonLoader)
    return simstat_object
