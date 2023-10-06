# -*- mode:python -*-
# Copyright (c) 2018, 2020 ARM Limited
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
# Copyright (c) 2004-2005 The Regents of The University of Michigan
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
import functools

import SCons.Script

########################################################################
# Code for adding source files of various types
#
# When specifying a source file of some type, a set of tags can be
# specified for that file.


def tag_implies(env, tag, tag_list):
    """
    Associates a tag X to a list of tags which are implied by X.

    For example, assume:
    - Each file <X>.cc is tagged with the tag "Tag <X>".
    - B.cc refers to symbols from A.cc
    - C.cc refers to symbols from B.cc
    - D.cc refers to symbols from A.cc and C.cc

    Then:
    - "Tag A" is implied by "Tag B"
    - "Tag B" is implied by "Tag C"
    - "Tag A" is transitively implied by "Tag C" (from "Tag B")
    - "Tag A" and "Tag C" are implied by "Tag D"
    - "Tag B" is transitively implied by "Tag D" (from "Tag C")
    - "Tag A" is transitively implied by "Tag D" (from transitive "Tag B")

    All of these implications are simply declared as:
        env.TagImplies("Tag B", "Tag A")
        env.TagImplies("Tag C", "Tag B")
        env.TagImplies("Tag D", ["Tag A", "Tag C"])

    So that any use of a tag will automatically include its transitive tags
    after being resolved.
    """

    env.SetDefault(_tag_implies={})
    implications = env["_tag_implies"]

    if isinstance(tag_list, str):
        tag_list = frozenset([tag_list])
    if not isinstance(tag_list, frozenset):
        tag_list = frozenset(tag_list)
    if tag in implications:
        implications[tag] |= tag_list
    else:
        implications[tag] = tag_list

    # Check if any of the tags on which the new tag depends on already
    # has a list of implications. If so, add the list to the new tag's
    # implications
    for t in tag_list:
        if t in implications:
            implications[tag] |= implications[t]

    # Check if another tag depends on this tag. If so, add this tag's
    # implications to that tag.
    for t, implied in implications.items():
        if tag in implied:
            implications[t] |= implications[tag]


def TagImpliesTool(env):
    env.AddMethod(tag_implies, "TagImplies")


def resolve_tags(env, tags):
    """
    Returns the complete set of tags implied (dependencies) by the
    supplied tags.
    """

    implications = env.SetDefault(_tag_implies={})
    implications = env["_tag_implies"]

    if isinstance(tags, str):
        tags = frozenset([tags])
    if not isinstance(tags, frozenset):
        tags = frozenset(tags)

    tags = tags.copy()
    for tag in tags:
        if tag in implications:
            tags |= implications[tag]
    return tags


class SourceFilter(object):
    factories = {}

    def __init__(self, predicate):
        self.predicate = predicate

    def __or__(self, other):
        return SourceFilter(
            lambda env, tags: self.predicate(env, tags)
            or other.predicate(env, tags),
        )

    def __and__(self, other):
        return SourceFilter(
            lambda env, tags: self.predicate(env, tags)
            and other.predicate(env, tags),
        )


def with_any_tags(*tags):
    """Return a list of sources with any of the supplied tags."""
    return SourceFilter(
        lambda env, stags: len(resolve_tags(env, tags) & stags) > 0,
    )


def with_all_tags(*tags):
    """Return a list of sources with all of the supplied tags."""
    return SourceFilter(lambda env, stags: resolve_tags(env, tags) <= stags)


def with_tag(tag):
    """Return a list of sources with the supplied tag."""
    return with_any_tags(*[tag])


def without_tags(*tags):
    """Return a list of sources without any of the supplied tags."""
    return SourceFilter(
        lambda env, stags: len(resolve_tags(env, tags) & stags) == 0,
    )


def without_tag(tag):
    """Return a list of sources without the supplied tag."""
    return without_tags(*[tag])


SourceFilter.factories.update(
    {
        "with_any_tags": with_any_tags,
        "with_all_tags": with_all_tags,
        "with_tag": with_tag,
        "without_tags": without_tags,
        "without_tag": without_tag,
    },
)


class SourceList(list):
    def apply_filter(self, env, f):
        def match(source):
            return f.predicate(env, resolve_tags(env, source.tags))

        return SourceList(filter(match, self))

    def __getattr__(self, name):
        func = SourceFilter.factories.get(name, None)
        if not func:
            raise AttributeError

        @functools.wraps(func)
        def wrapper(env, *args, **kwargs):
            return self.apply_filter(env, func(*args, **kwargs))

        return wrapper


class SourceMeta(type):
    """Meta class for source files that keeps track of all files of a
    particular type."""

    def __init__(cls, name, bases, dict):
        super(SourceMeta, cls).__init__(name, bases, dict)
        cls.all = SourceList()


class SourceItem(object, metaclass=SourceMeta):
    """Base object that encapsulates the notion of a source component for
    gem5. This specifies a set of tags which help group components into groups
    based on arbitrary properties."""

    def __init__(self, source, tags=None, add_tags=None, append=None):
        self.source = source

        if tags is None:
            tags = "gem5 lib"
        if isinstance(tags, str):
            tags = {tags}
        if not isinstance(tags, set):
            tags = set(tags)
        self.tags = tags.copy()

        if add_tags:
            if isinstance(add_tags, str):
                add_tags = {add_tags}
            if not isinstance(add_tags, set):
                add_tags = set(add_tags)
            self.tags |= add_tags

        self.append = append

        for base in type(self).__mro__:
            if issubclass(base, SourceItem):
                base.all.append(self)


class SourceFile(SourceItem):
    """Base object that encapsulates the notion of a source file.
    This includes, the source node, target node, various manipulations
    of those."""

    def __init__(self, source, tags=None, add_tags=None, append=None):
        super().__init__(source, tags=tags, add_tags=add_tags, append=append)

        tnode = SCons.Script.File(source)

        self.tnode = tnode
        self.filename = str(self.tnode)
        self.snode = tnode.srcnode()

    def static(self, env):
        if self.append:
            env = env.Clone()
            env.Append(**self.append)
        return env.StaticObject(self.tnode.abspath)

    def shared(self, env):
        if self.append:
            env = env.Clone()
            env.Append(**self.append)
        return env.SharedObject(self.tnode.abspath)


__all__ = [
    "TagImpliesTool",
    "SourceFilter",
    "SourceList",
    "SourceFile",
    "SourceItem",
    "with_any_tags",
    "with_all_tags",
    "with_tag",
    "without_tags",
    "without_tag",
]
