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

import re
from typing import (
    Callable,
    Dict,
    Iterator,
    List,
    Mapping,
    Optional,
    Pattern,
    Union,
)

from .jsonserializable import JsonSerializable
from .statistic import Scalar, Statistic
from .timeconversion import TimeConversion


class Group(JsonSerializable):
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
        ]
    ):
        if type is None:
            self.type = "Group"
        else:
            self.type = type

        self.time_conversion = time_conversion

        for key, value in kwargs.items():
            setattr(self, key, value)

    def children(
        self, predicate: Optional[Callable[[str], bool]] = None
    ) -> Iterator[Union["Group", Statistic]]:
        """ Iterate through all of the children, optionally with a predicate

        ```
        >>> system.children(lambda _name: 'cpu' in name)
        [cpu0, cpu1, cpu2]
        ```

        :param: predicate(str) -> bool: Optional. Each child's name is passed
                to this function. If it returns true, then the child is
                yielded. Otherwise, the child is skipped.
                If not provided then all children are returned.
        """
        for attr in self.__dict__:
            # Check the provided predicate. If not a match, skip this child
            if predicate and not predicate(attr):
                continue
            obj = getattr(self, attr)
            if isinstance(obj, Group) or isinstance(obj, Statistic):
                yield obj

    def find(self, name: str) -> Iterator[Union["Group", Statistic]]:
        """ Find all stats that match the name

        This function searches all of the "children" in this group. It yields
        the set of attributes (children) that have the `name` as a substring.
        The order of the objects returned by the generator is arbitrary.

        ```
        >>> system.find('cpu')
        [cpu0, cpu1, cpu2, cpu3, other_cpu, ...]
        ```

        This is useful for performing aggregates over substats. For instance:

        ```
        >>> total_instructions = sum([cpu.exec_context.thread_0.numInsts.value
                                      for cpu in simstat.system.find('cpu')])
        100000
        ```

        :param: name: The name to search for
        """
        yield from self.children(lambda _name: _name in name)

    def find_re(
        self, regex: Union[str, Pattern]
    ) -> Iterator[Union["Group", Statistic]]:
        """ Find all stats that match the name

        This function searches all of the "children" in this group. It yields
        the set of attributes (children) that have the `name` mathing the
        regex provided. The order of the objects returned by the generator is
        arbitrary.

        ```
        >>> system.find_re('cpu[0-9]')
        [cpu0, cpu1, cpu2]
        ```
        Note: The above will not match `cpu_other`.

        :param: regex: The regular expression used to search. Can be a
                precompiled regex or a string in regex format
        """
        if isinstance(regex, str):
            pattern = re.compile(regex)
        else:
            pattern = regex
        yield from self.children(lambda _name: bool(pattern.search(_name)))


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
