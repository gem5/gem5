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

from typing import (
    Any,
    Dict,
    List,
    Optional,
    Union,
)

from .abstract_stat import AbstractStat
from .statistic import Statistic
from .timeconversion import TimeConversion


class Group(AbstractStat):
    """
    Used to create the heirarchical stats structure. A Group object contains a
    map of labeled  Groups, Statistics, Lists of Groups, or List of Statistics.
    """

    type: Optional[str]
    time_conversion: Optional[TimeConversion]

    def __init__(
        self,
        type: Optional[str] = None,
        time_conversion: Optional[TimeConversion] = None,
        **kwargs: Dict[
            str, Union["Group", Statistic, List["Group"], List["Statistic"]]
        ],
    ):
        if type is None:
            self.type = "Group"
        else:
            self.type = type

        self.time_conversion = time_conversion

        for key, value in kwargs.items():
            setattr(self, key, value)


class SimObjectGroup(Group):
    """A group of statistics encapulated within a SimObject."""

    def __init__(self, **kwargs: Dict[str, Union[Group, Statistic]]):
        super().__init__(type="SimObject", **kwargs)


class SimObjectVectorGroup(Group):
    """A Vector of SimObject objects. I.e., that which would be constructed
    from something like `system.cpu = [DerivO3CPU(), TimingSimpleCPU()]`.
    """

    def __init__(self, value: List[AbstractStat], **kwargs: Dict[str, Any]):
        assert isinstance(value, list), "Value must be a list"
        super().__init__(type="SimObjectVector", value=value, **kwargs)

    def __getitem__(self, index: Union[int, str, float]) -> AbstractStat:
        if not isinstance(index, int):
            raise KeyError(
                f"Index {index} not found in int. Cannot index Array with "
                "non-int"
            )
        return self.value[index]

    def __iter__(self):
        return iter(self.value)

    def __len__(self):
        return len(self.value)

    def get_all_stats_of_name(self, name: str) -> List[AbstractStat]:
        """
        Get all the stats in the vector of that name. Useful for performing
        operations on all the stats of the same name in a vector.
        """
        to_return = []
        for stat in self.value:
            if hasattr(stat, name):
                to_return.append(getattr(stat, name))

        # If the name is in the format "sim.bla.whatever", we are looking for
        # the "bla.whatever" stats in the "sim" group.
        # This is messy, but it works.
        name_split = name.split(".")
        if len(name_split) == 1:
            return to_return

        if name_split[0] not in self:
            return to_return

        to_return.extend(
            self[name_split[0]].get_all_stats_of_name(".".join(name_split[1:]))
        )
        return to_return

    def __getitem__(self, item: int):
        return self.value[item]
