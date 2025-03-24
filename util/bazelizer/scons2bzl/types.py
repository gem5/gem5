# Copyright 2025 Google, Inc.
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

"""Types with commonly used methods for the BUILD file generator."""

import os


class Path:
  """Class represents a path with some handy utils."""

  def __init__(self, path, absolute=False, base=None):
    self.absolute = absolute
    if isinstance(path, str):
      pass
    elif isinstance(path, list):
      path = os.path.join(*path)
    else:
      raise TypeError()
    if base is None:
      self.base = None
    elif isinstance(base, str):
      assert os.path.isabs(base)
      self.base = os.path.normpath(base)
    else:
      raise TypeError()
    if absolute:
      self._abs_path = os.path.normpath(path)
    else:
      self._rel_path = os.path.normpath(path)

  @property
  def abs(self):
    if self.absolute:
      return self._abs_path
    else:
      assert self.base is not None
      return os.path.join(self.base, self._rel_path)

  @property
  def rel(self):
    if self.absolute:
      assert self.base is not None
      assert self._abs_path.startswith(self.base)
      return os.path.relpath(self._abs_path, self.base)
    else:
      return self._rel_path

  def sibling(self, sibling_path, from_base=False, inherit_base=True):
    """Derives the path to the adjacent file of the current path."""
    if self.absolute:
      old_dirname = self.abs_dirname
    else:
      old_dirname = self.rel_dirname
    if from_base:
      new_path = os.path.join(self.base, sibling_path)
    else:
      new_path = os.path.join(old_dirname, sibling_path)
    if from_base or inherit_base or self.base is None:
      new_base = self.base
    else:
      new_base = self.abs_dirname
    return Path(new_path, self.absolute, new_base)

  def append(self, *path):
    return self.sibling(os.path.join(self.basename, *path), False, True)

  @property
  def abs_dirname(self):
    return os.path.dirname(self.abs)

  @property
  def rel_dirname(self):
    return os.path.dirname(self.rel)

  @property
  def basename(self):
    if self.absolute:
      return os.path.basename(self._abs_path)
    else:
      return os.path.basename(self._rel_path)

  @property
  def ext(self):
    return os.path.splitext(self.basename)[1]

  @property
  def no_ext_basename(self):
    return os.path.splitext(self.basename)[0]

  def startswith(self, s):
    return self.rel.startswith(s)

  def __hash__(self):
    if not self.absolute and self.base is None:
      return hash((False, self._rel_path))
    else:
      return hash((True, self.abs))

  def __eq__(self, another):
    return hash(self) == hash(another)


AbsPath = lambda path, base=None: Path(path, True, base)
RelPath = lambda path, base=None: Path(path, False, base)

Path.BUILD_FILE = 'BUILD.bazel'
Path.SCONSCRIPT = 'SConscript'
Path.SCONSTRUCT = 'SConstruct'
