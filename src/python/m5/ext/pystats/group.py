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
    Dict,
    List,
    Mapping,
    Optional,
    Union,
)

from .abstract_stat import AbstractStat
from .statistic import (
    Scalar,
    Statistic,
)
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


class Vector(Group):
    """
    The Vector class is used to store vector information. However, in gem5
    Vectors, in practise, hold information that is more like a dictionary of
    Scalar Values. This class may change, and may be merged into Group in
    accordance to decisions made in relation to
    https://gem5.atlassian.net/browse/GEM5-867.
    """

    def __init__(self, scalar_map: Mapping[str, Scalar]):
        super().__init__(type="Vector", time_conversion=None, **scalar_map)

    def _repr_name(self) -> str:
        return "Vector"
