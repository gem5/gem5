#!/usr/bin/env python3
#
# Copyright (c) 2021 The Regents of the University of California
# Copyright (c) 2020 Arm Limited
# All rights reserved
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

import os
from typing import (
    Any,
    Dict,
    Iterator,
    Mapping,
    Optional,
    TextIO,
    Tuple,
    Union,
)

import yaml

PathOrFile = Union[TextIO, str]


class FileFormatException(Exception):
    pass


class MissingFieldException(FileFormatException):
    pass


class IllegalValueException(FileFormatException):
    pass


class Subsystem(object):
    tag: str
    description: str

    def __init__(
        self,
        tag: str,
        description: str = "",
    ):
        self.tag = tag
        self.description = description if description is not None else ""


class Tags(object):
    DEFAULT_TAGS = os.path.join(
        os.path.dirname(__file__), "../../../TAGS.yaml"
    )

    _subsystems: Dict[str, Subsystem]  # tag -> Subsystem

    def __init__(self, ydict: Mapping[str, Any]):
        self._subsystems = {}
        for tag, content in list(ydict.items()):
            self._subsystems[tag] = Tags._parse_subsystem(tag, content)

    @classmethod
    def from_file(cls, path_or_file: Optional[PathOrFile] = None) -> "Tags":

        return cls(Tags._load_tags_file(path_or_file))

    @classmethod
    def from_yaml(cls, yaml_str: str) -> "Tags":
        return cls(yaml.load(yaml_str, Loader=yaml.SafeLoader))

    @classmethod
    def _load_tags_file(
        cls, path_or_file: Optional[PathOrFile] = None
    ) -> Mapping[str, Any]:
        if path_or_file is None:
            path_or_file = cls.DEFAULT_TAGS

        if isinstance(path_or_file, str):
            with open(path_or_file, "r") as fin:
                return yaml.load(fin, Loader=yaml.SafeLoader)
        else:
            return yaml.load(path_or_file, Loader=yaml.SafeLoader)

    @classmethod
    def _parse_subsystem(cls, tag: str, ydict: Mapping[str, Any]) -> Subsystem:
        return Subsystem(
            tag,
            description=ydict.get("desc", "") if ydict is not None else "",
        )

    def __iter__(self) -> Iterator[Tuple[str, Subsystem]]:
        return iter(list(self._subsystems.items()))

    def __getitem__(self, key: str) -> Subsystem:
        return self._subsystems[key]


def _main():
    tags = Tags.from_file()
    for tag, subsys in tags:
        print(f"{tag}: {subsys.description}")
        print()


if __name__ == "__main__":
    _main()

__all__ = [
    "FileFormatException",
    "MissingFieldException",
    "IllegalValueException",
    "Status",
    "Subsystem",
    "Tags",
]
