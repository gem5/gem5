# Copyright (c) 2022 The Regents of The University of California
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
    Any,
    Callable,
    List,
    Optional,
    Pattern,
    Tuple,
    Union,
)

from .serializable_stat import SerializableStat


class AbstractStat(SerializableStat):
    """
    An abstract class which all PyStats inherit from.

    All PyStats are JsonSerializable.
    """

    def children(
        self,
        predicate: Optional[Callable[[str], bool]] = None,
        recursive: bool = False,
    ) -> List["AbstractStat"]:
        """Iterate through all of the children, optionally with a predicate

        .. code-block::

            >>> system.children(lambda _name: 'cpu' in name)
            [cpu0, cpu1, cpu2]


        :param predicate: Optional. Each child's name is passed to this function.
                          If it returns ``True``, then the child is yielded.
                          Otherwise, the child is skipped. If not provided then
                          all children are returned.

        Note: This is method must be implemented in AbstractStat subclasses
        which have children, otherwise it will return an empty list.
        """
        return []

    def find(self, regex: Union[str, Pattern]) -> List["AbstractStat"]:
        """Find all stats that match the name, recursively through all the
        SimStats.

        .. code-block::

            >>> system.find('cpu[0-9]')
            [cpu0, cpu1, cpu2]


        .. note::

            The above will not match ``cpu_other``.

        :param regex: The regular expression used to search. Can be a
                precompiled regex or a string in regex format.
        """
        if isinstance(regex, str):
            pattern = re.compile(regex)
        else:
            pattern = regex
        return self.children(
            lambda _name: re.match(pattern, _name), recursive=True
        )

    def _get_vector_item(self, item: str) -> Optional[Tuple[str, int, Any]]:
        """It has been the case in gem5 that SimObject vectors are stored as
        strings such as "cpu0" or "cpu1". This function splits the string into
        the SimObject name and index, (e.g.: ["cpu", 0] and ["cpu", 1]) and
        returns the item for that name and it's index. If the string cannot be
        split into a SimObject name and index, or if the SimObject does not
        exit at `Simobject[index]`, the function returns None.
        """
        regex = re.compile("[0-9]+$")
        match = regex.search(item)
        if not match:
            return None

        match_str = match.group()

        assert match_str.isdigit(), f"Regex match must be a digit: {match_str}"
        vector_index = int(match_str)
        vector_name = item[: (-1 * len(match_str))]

        if hasattr(self, vector_name):
            vector = getattr(self, vector_name)
            try:
                vector_value = vector[vector_index]
                return vector_name, vector_index, vector_value
            except KeyError:
                pass
        return None

    def __iter__(self):
        return iter(self.__dict__)

    def __getattr__(self, item: str) -> Any:
        vector_item = self._get_vector_item(item)
        if not vector_item:
            return None

        assert (
            len(vector_item) == 3
        ), f"Vector item must have 3 elements: {vector_item}"
        return vector_item[2]

    def __getitem__(self, item: str):
        return getattr(self, item)

    def __contains__(self, item: Any) -> bool:
        return (
            isinstance(item, str) and self._get_vector_item(item)
        ) or hasattr(self, item)
