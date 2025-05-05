# Copyright (c) 2024 Arm Limited
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

from abc import (
    ABCMeta,
    abstractmethod,
)
from typing import Callable

from m5.objects import (
    Root,
    SubSystem,
)
from m5.util.fdthelper import *

from ...coherence_protocol import CoherenceProtocol
from ..boards.abstract_board import AbstractBoard


class CacheNode:
    def __init__(
        self,
        name: str,
        cache: SimObject,
        next_level: "CacheNode",
        hierarchy: "AbstractCacheHierarchy",
    ):
        self.name = name
        self.cache = cache
        self.next_level = next_level
        self.hierarchy = hierarchy

        self.prev_levels = []

        # Need to assign this to a SimObject
        if cache is not None:
            setattr(hierarchy, self.name, cache)

    def add_child(self, name: str, cache: SimObject) -> "CacheNode":
        """
        Add a child node to the current node provided a cache object and
        its name. Because of the intrinsic topology of caches, children will be
        one level higher than their parent in the hierarchy.
        This means the chain of insertions to the tree will be something
        like:
        l3.add_child("l2", l2).add_child("l1", l1)

        :param name: The name of the cache
        :param cache: The cache SimObject
        :returns: The new child node being generated
        """
        new_node = CacheNode(name, cache, self, self.hierarchy)
        self.prev_levels.append(new_node)
        return new_node

    def generate_dtb_entry(self, state, level):
        node = FdtNode(f"{self.name}")
        node.append(FdtPropertyStrings("compatible", ["cache"]))
        node.append(FdtPropertyWords("cache-level", int(level)))
        node.append(FdtPropertyWords("cache-size", int(self.cache.size)))
        if self.next_level:
            node.append(
                FdtPropertyWords(
                    "next-level-cache", state.phandle(self.next_level.cache)
                )
            )

        node.appendPhandle(self.cache)
        return node


class AbstractCacheHierarchy(SubSystem):
    __metaclass__ = ABCMeta

    def __init__(self):
        super().__init__()
        self._root = CacheNode("root", None, None, self)

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

    @abstractmethod
    def get_coherence_protocol(self) -> CoherenceProtocol:
        """
        Returns the coherence protocol used in the cache hierarchy.

        :returns: The coherence protocol used in the cache hierarchy.
        """
        raise NotImplementedError

    def _pre_instantiate(self, root: Root) -> None:
        """Called in the `AbstractBoard`'s `_pre_instantiate` method. This is
        called after `connect_things`, after the creation of the root object
        (which is passed in as an argument), but before `m5.instantiate`).

        Subclasses should override this method to set up any connections.

        At present there is no general task that must be specified here and is
        default or applicable to all cache hierarchies.
        """
        pass

    def _post_instantiate(self):
        """Called to set up anything needed after ``m5.instantiate``."""
        pass

    def add_root_child(self, *args, **kwargs):
        """This adds the LLC to the root node"""
        return self._root.add_child(*args, **kwargs)

    def traverse(
        self, node: CacheNode, visit: Callable[[CacheNode, int], None]
    ) -> int:
        """
        Traverse the tree in post-order. Return the level of the
        current node passed as an argument. The method accepts
        a visit function to be called at each node

        :param node: starting node for traversal
        :param visit: visiting function to be called at each node

        :returns: level of the node passed as an argument
        """
        if not node.prev_levels:
            level = 1
        else:
            for prev in node.prev_levels:
                level = self.traverse(prev, visit)

        visit(node, level)

        return level + 1

    def generateDeviceTree(self, state):
        dt_entries = []

        def add_dt_entry(node, level):
            # Do not generate a DTB entry for the root node
            # as it does not point to a real cache (node.cache = None)
            # and for the L1I and L1D caches as their data should be
            # part of the CPU node as described by:
            # https://devicetree-specification.readthedocs.io/en/stable/devicenodes.html
            if node.cache is not None and level != 1:
                dt_entries.append(node.generate_dtb_entry(state, level))

        self.traverse(self._root, add_dt_entry)

        yield from dt_entries
