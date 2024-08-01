# Copyright (c) 2022-2023 The Regents of the University of California
# Copyright (c) 2023 COSEDA Technologies GmbH
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

import hashlib
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from hashlib import _Hash


def _md5_update_from_file(filename: Path, hash: "_Hash") -> "_Hash":
    assert filename.is_file()

    if filename.stat().st_size < 1024 * 1024 * 100:
        from ..utils.progress_bar import FakeTQDM

        # if the file is less than 100MB, no need to show a progress bar.
        tqdm = FakeTQDM()
    else:
        from ..utils.progress_bar import tqdm

    with tqdm.wrapattr(
        open(str(filename), "rb"),
        "read",
        miniters=1,
        desc=f"Computing md5sum on {filename}",
        total=filename.stat().st_size,
    ) as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash.update(chunk)
    return hash


def _md5_update_from_dir(directory: Path, hash: "_Hash") -> "_Hash":
    assert directory.is_dir()
    for path in sorted(directory.iterdir(), key=lambda p: str(p).lower()):
        hash.update(path.name.encode())
        if path.is_file():
            hash = _md5_update_from_file(path, hash)
        elif path.is_dir():
            hash = _md5_update_from_dir(path, hash)
    return hash


def md5(path: Path) -> str:
    """
    Gets the md5 value of a file or directory. ``md5_file`` is used if the path
    is a file and ``md5_dir`` is used if the path is a directory. An exception
    is returned if the path is not a valid file or directory.

    :param path: The path to get the md5 of.
    """
    if path.is_file():
        return md5_file(Path(path))
    elif path.is_dir():
        return md5_dir(Path(path))
    else:
        raise Exception(f"Path '{path}' is not a valid file or directory.")


def md5_file(filename: Path) -> str:
    """
    Gives the md5 hash of a file.

    :filename: The file in which the md5 is to be calculated.
    """
    return str(_md5_update_from_file(filename, hashlib.md5()).hexdigest())


def md5_dir(directory: Path) -> str:
    """
    Gives the md5 value of a directory.

    This is achieved by getting the md5 hash of all files in the directory.

    .. note::

        The path of files are also hashed so the md5 of the directory changes
        if empty files are included or filenames are changed.
    """
    return str(_md5_update_from_dir(directory, hashlib.md5()).hexdigest())
