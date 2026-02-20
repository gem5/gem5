# Copyright (c) 2025 The Regents of the University of California
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

"""Utility for skipping file writes when content is unchanged.

When code-generation scripts unconditionally overwrite output files, the
updated mtime defeats ninja's ``restat`` optimization and triggers massive
unnecessary recompilation cascades.  ``write_if_changed()`` compares the
new content against the existing file and only writes when something
actually changed, preserving the mtime otherwise.
"""

import os
import tempfile


def write_if_changed(filepath, content):
    """Write *content* to *filepath* only if it differs from the current file.

    If *filepath* already exists and its contents are identical to
    *content*, the function returns immediately without touching the file,
    preserving its mtime for ninja ``restat``.

    When a write is necessary, it is performed atomically: the content is
    first written to a temporary file in the same directory, then moved
    into place with ``os.replace()``.
    """
    if isinstance(content, str):
        content = content.encode("utf-8")

    try:
        with open(filepath, "rb") as f:
            if f.read() == content:
                return
    except FileNotFoundError:
        pass

    dirpath = os.path.dirname(filepath) or "."
    os.makedirs(dirpath, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=dirpath)
    try:
        os.write(fd, content)
    except BaseException:
        os.close(fd)
        os.unlink(tmp)
        raise
    os.close(fd)
    try:
        os.replace(tmp, filepath)
    except BaseException:
        os.unlink(tmp)
        raise
