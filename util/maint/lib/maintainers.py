#!/usr/bin/env python3
#
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

import email.utils
import enum
import os
from typing import (
    Any,
    Dict,
    Iterator,
    List,
    Mapping,
    Optional,
    Sequence,
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


class Status(enum.Enum):
    MAINTAINED = enum.auto()
    ORPHANED = enum.auto()

    @classmethod
    def from_str(cls, key: str) -> "Status":
        _status_dict = {"maintained": cls.MAINTAINED, "orphaned": cls.ORPHANED}
        return _status_dict[key]

    def __str__(self) -> str:
        return {Status.MAINTAINED: "maintained", Status.ORPHANED: "orphaned"}[
            self
        ]


class Subsystem(object):
    tag: str
    status: Status
    maintainers: List[Tuple[str, str]]  # Name, email
    description: str

    def __init__(
        self,
        tag: str,
        maintainers: Optional[Sequence[Tuple[str, str]]],
        description: str = "",
        status: Status = Status.ORPHANED,
    ):
        self.tag = tag
        self.status = status
        self.maintainers = list(maintainers) if maintainers is not None else []
        self.description = description if description is not None else ""


class Maintainers(object):
    DEFAULT_MAINTAINERS = os.path.join(
        os.path.dirname(__file__), "../../../MAINTAINERS.yaml"
    )

    _subsystems: Dict[str, Subsystem]  # tag -> Subsystem

    def __init__(self, ydict: Mapping[str, Any]):
        self._subsystems = {}
        for tag, maint in list(ydict.items()):
            self._subsystems[tag] = Maintainers._parse_subsystem(tag, maint)

    @classmethod
    def from_file(
        cls, path_or_file: Optional[PathOrFile] = None
    ) -> "Maintainers":
        return cls(Maintainers._load_maintainers_file(path_or_file))

    @classmethod
    def from_yaml(cls, yaml_str: str) -> "Maintainers":
        return cls(yaml.load(yaml_str, Loader=yaml.SafeLoader))

    @classmethod
    def _load_maintainers_file(
        cls, path_or_file: Optional[PathOrFile] = None
    ) -> Mapping[str, Any]:
        if path_or_file is None:
            path_or_file = cls.DEFAULT_MAINTAINERS

        if isinstance(path_or_file, str):
            with open(path_or_file, "r") as fin:
                return yaml.load(fin, Loader=yaml.SafeLoader)
        else:
            return yaml.load(path_or_file, Loader=yaml.SafeLoader)

    @classmethod
    def _parse_subsystem(cls, tag: str, ydict: Mapping[str, Any]) -> Subsystem:
        def required_field(name):
            try:
                return ydict[name]
            except KeyError:
                raise MissingFieldException(
                    f"{tag}: Required field '{name}' is missing"
                )

        maintainers: List[Tuple[str, str]] = []
        raw_maintainers = ydict.get("maintainers", [])
        if not isinstance(raw_maintainers, Sequence):
            raise IllegalValueException(
                f"{tag}: Illegal field 'maintainers' isn't a list."
            )
        for maintainer in raw_maintainers:
            name, address = email.utils.parseaddr(maintainer)
            if name == "" and address == "":
                raise IllegalValueException(
                    f"{tag}: Illegal maintainer field: '{maintainer}'"
                )
            maintainers.append((name, address))

        try:
            status = Status.from_str(required_field("status"))
        except KeyError:
            raise IllegalValueException(
                f"{tag}: Invalid status '{ydict['status']}'"
            )

        return Subsystem(
            tag,
            maintainers=maintainers,
            status=status,
            description=ydict.get("desc", ""),
        )

    def __iter__(self) -> Iterator[Tuple[str, Subsystem]]:
        return iter(list(self._subsystems.items()))

    def __getitem__(self, key: str) -> Subsystem:
        return self._subsystems[key]


def _main():
    maintainers = Maintainers.from_file()
    for tag, subsys in maintainers:
        print(f"{tag}: {subsys.description}")
        print(f"  Status: {subsys.status}")
        print(f"  Maintainers:")
        for maint in subsys.maintainers:
            print(f"    - {maint[0]} <{maint[1]}>")
        print()


if __name__ == "__main__":
    _main()

__all__ = [
    "FileFormatException",
    "MissingFieldException",
    "IllegalValueException",
    "Status",
    "Subsystem",
    "Maintainers",
]
