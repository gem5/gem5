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
    Callable,
    List,
    Optional,
    Pattern,
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
        """

        to_return = []
        for attr in self.__dict__:
            obj = getattr(self, attr)
            if isinstance(obj, AbstractStat):
                if (predicate and predicate(attr)) or not predicate:
                    to_return.append(obj)
                if recursive:
                    to_return = to_return + obj.children(
                        predicate=predicate, recursive=True
                    )

        return to_return

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
