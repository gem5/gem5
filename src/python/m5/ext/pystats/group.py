# Copyright (c) 2025 Arm Limited
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
    Callable,
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
    values: Dict[
        str, Union["Group", Statistic, List["Group"], List["Statistic"]]
    ]
    name: Optional[str]

    def __init__(
        self,
        type: Optional[str] = None,
        name: Optional[str] = None,
        time_conversion: Optional[TimeConversion] = None,
        **kwargs: Dict[
            str, Union["Group", Statistic, List["Group"], List["Statistic"]]
        ],
    ):
        if type:
            self.type = type

        self.name = name

        self.time_conversion = time_conversion
        self.values = kwargs

    def children(
        self,
        predicate: Optional[Callable[[str], bool]] = None,
        recursive: bool = False,
    ) -> List["AbstractStat"]:
        to_return = []
        for key, obj in self.values:
            if isinstance(obj, AbstractStat):
                if (predicate and predicate(obj)) or not predicate:
                    to_return.append(obj)
                if recursive:
                    to_return = to_return + obj.children(
                        predicate=predicate, recursive=True
                    )
        return to_return

    def accept(self, visitor):
        return visitor.visit_group(self)

    def __getitem__(self, key: str) -> "AbstractStat":
        return self.values[key]

    def __getattr__(self, name):
        try:
            return self.values[name]
        except KeyError:
            raise AttributeError(name)

    def __setattr__(self, name, value):
        # Let normal attributes be handled normally
        if (
            name.startswith("_")
            or name == "values"
            or "values" not in self.__dict__
        ):
            super().__setattr__(name, value)
        else:
            self.values[name] = value


class SimObjectGroup(Group):
    """A group of statistics encapulated within a SimObject."""

    def __init__(
        self,
        name: Optional[str] = None,
        **kwargs: Dict[str, Union[Group, Statistic]],
    ):
        super().__init__(type="SimObject", name=name, **kwargs)

    def accept(self, visitor):
        return visitor.visit_simobject_group(self)


class SimObjectVectorGroup(Group):
    """A Vector of SimObject objects. I.e., that which would be constructed
    from something like `system.cpu = [DerivO3CPU(), TimingSimpleCPU()]`.
    """

    def __init__(self, children: List[AbstractStat], **kwargs: Dict[str, Any]):
        assert isinstance(children, list), "Value must be a list"
        kwargs["value"] = children
        super().__init__(type="SimObjectVector", **kwargs)

    def __iter__(self):
        return iter(self.values)

    def __len__(self):
        return len(self.values)

    def __getitem__(self, item: int):
        return self.values[item]

    def __contains__(self, item):
        if isinstance(item, int):
            return item >= 0 and item < len(self)

    def children(
        self,
        predicate: Optional[Callable[[str], bool]] = None,
        recursive: bool = False,
    ) -> List["AbstractStat"]:
        to_return = []
        for child in self.values:
            to_return = to_return + child.children(
                predicate=predicate, recursive=recursive
            )

        return to_return

    def accept(self, visitor):
        return visitor.visit_simobject_vector_group(self)
