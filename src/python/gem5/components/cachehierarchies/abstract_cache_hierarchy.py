# Copyright (c) 2021 The Regents of the University of California
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

from abc import ABCMeta, abstractmethod

from ..boards.abstract_board import AbstractBoard

from m5.objects import SubSystem


class AbstractCacheHierarchy(SubSystem):
    __metaclass__ = ABCMeta

    def __init__(self):
        super().__init__()

    """
    A Cache Hierarchy incorporates any system components which manages
    communicaton between the processor and memory. E.g., Caches, the MemBus,
    MMU, and the MMU Cache.

    All Cache Hierarchies must have this as a base class.
    """

    @abstractmethod
    def incorporate_cache(self, board: AbstractBoard) -> None:
        """
        Incorporates the caches into a board.

        Each specific hierarchy needs to implement this function and will be
        unique for each setup.

        :param board: The board in which the cache heirarchy is to be
                      incorporated.
        """

        raise NotImplementedError

    @abstractmethod
    def is_ruby(self) -> bool:
        """
        Specifies whether this cache hierarchy is using the Ruby memory system
        or not.

        :returns: ``True`` if the cache hierarchy is ruby. Otherwise ``False``.
        """
        raise NotImplementedError

    def _post_instantiate(self):
        """Called to set up anything needed after ``m5.instantiate``."""
        pass
