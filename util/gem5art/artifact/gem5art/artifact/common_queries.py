# Copyright (c) 2020-2021 The Regents of the University of California
# All Rights Reserved.
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
"""File contains the some helper functions with common queries for artifacts
in the ArtifactDB.
"""
from typing import Iterator

from ._artifactdb import ArtifactDB
from .artifact import Artifact


def _getByType(db: ArtifactDB, typ: str, limit: int = 0) -> Iterator[Artifact]:
    """Returns a generator of Artifacts with matching `type` from the db.

    Limit specifies the maximum number of results to return.
    """
    data = db.searchByType(typ, limit=limit)

    for d in data:
        yield Artifact(d)


def getDiskImages(db: ArtifactDB, limit: int = 0) -> Iterator[Artifact]:
    """Returns a generator of disk images (type = disk image).

    Limit specifies the maximum number of results to return.
    """

    return _getByType(db, "disk image", limit)


def getgem5Binaries(db: ArtifactDB, limit: int = 0) -> Iterator[Artifact]:
    """Returns a generator of gem5 binaries (type = gem5 binary).

    Limit specifies the maximum number of results to return.
    """

    return _getByType(db, "gem5 binary", limit)


def getLinuxBinaries(db: ArtifactDB, limit: int = 0) -> Iterator[Artifact]:
    """Returns a generator of Linux kernel binaries (type = kernel).

    Limit specifies the maximum number of results to return.
    """

    return _getByType(db, "kernel", limit)


def getByName(db: ArtifactDB, name: str, limit: int = 0) -> Iterator[Artifact]:
    """Returns all objects mathching `name` in database.

    Limit specifies the maximum number of results to return.
    """
    data = db.searchByName(name, limit=limit)

    for d in data:
        yield Artifact(d)
