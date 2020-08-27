#!/usr/bin/env python2.7
#
# Copyright (c) 2016 ARM Limited
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

from abc import *
import os
import subprocess

from .region import *
from .style import modified_regions

class AbstractRepo(object):
    __metaclass__ = ABCMeta

    def file_path(self, fname):
        """Get the absolute path to a file relative within the repository. The
        input file name must be a valid path within the repository.

        """
        return os.path.join(self.repo_base(), fname)

    def in_repo(self, fname):
        """Check if a path points to something within the repository base. Not
        that this does not check for the presence of the object in the
        file system as it could exist in the index without being in
        the file system.

        """
        fname = os.path.abspath(fname)
        repo_path = os.path.abspath(self.repo_base())

        return os.path.commonprefix([repo_path, fname]) == repo_path

    def repo_path(self, fname):
        """Get the path of a file relative to the repository base. The input
        file name is assumed to be an absolute path or a path relative
        to the current working directory.

        """
        return os.path.relpath(fname, self.repo_base())

    def get_file(self, name):
        """Get the contents of a file in the file system using a path relative
        to the repository root.

        """
        with open(self.file_path(name), "r") as f:
            return f.read()

    @abstractmethod
    def repo_base(self):
        """Get the path to the base of the repository"""
        pass

    @abstractmethod
    def staged_files(self):
        """Get a tuple describing the files that have been staged for a
        commit: (list of new, list of modified)

        """
        pass

    @abstractmethod
    def staged_regions(self, fname, context=0):
        """Get modified regions that will be committed by the next commit
        command

        """
        pass

    @abstractmethod
    def modified_regions(self, fname, context=0):
        """Get modified regions that have been staged for commit or are
        present in the file system.

        """
        pass

class GitRepo(AbstractRepo):
    def __init__(self):
        self.git = "git"
        self._head_revision = None
        self._repo_base = None

    def repo_base(self):
        if self._repo_base is None:
            self._repo_base = subprocess.check_output(
                [ self.git, "rev-parse", "--show-toplevel" ]) \
                .decode().rstrip("\n")

        return self._repo_base

    def staged_files(self):
        added = []
        modified = []
        for action, fname in self.status(filter="MA", cached=True):
            if action == "M":
                modified.append(fname)
            elif action == "A":
                added.append(fname)

        return added, modified

    def staged_regions(self, fname, context=0):
        if self.file_status(fname, cached=True) in ("", "A", ):
            return all_regions

        old = self.file_from_head(self.repo_path(fname)).split("\n")
        new = self.file_from_index(self.repo_path(fname)).split("\n")

        return modified_regions(old, new, context=context)

    def modified_regions(self, fname, context=0):
        if self.file_status(fname) in ("", "A", ):
            return all_regions

        old = self.file_from_head(self.repo_path(fname)).split("\n")
        new = self.get_file(self.repo_path(fname)).split("\n")

        return modified_regions(old, new, context=context)


    def head_revision(self):
        if self._head_revision is not None:
            return self._head_revision

        try:
            self._head_revision = subprocess.check_output(
                [ self.git, "rev-parse", "--verify", "HEAD" ],
                stderr=subprocess.PIPE).decode().rstrip("\n")
        except subprocess.CalledProcessError:
            # Assume that the repo is empty and use the semi-magic
            # empty tree revision if git rev-parse returned an error.
            self._head_revision = "4b825dc642cb6eb9a060e54bf8d69288fbee4904"

        return self._head_revision

    def file_status(self, fname, cached=False):
        status = self.status(files=[fname], cached=cached)
        assert len(status) <= 1
        if status:
            return status[0][0]
        else:
            # No information available for the file. This usually
            # means that it hasn't been added to the
            # repository/commit.
            return ""

    def status(self, filter=None, files=[], cached=False):
        cmd = [ self.git, "diff-index", "--name-status" ]
        if cached:
            cmd.append("--cached")
        if filter:
            cmd += [ "--diff-filter=%s" % filter ]
        cmd += [ self.head_revision(), "--" ] + files
        status = subprocess.check_output(cmd).decode('utf-8').rstrip("\n")

        if status:
            return [ f.split("\t") for f in status.split("\n") ]
        else:
            return []

    def file_from_index(self, name):
        return subprocess.check_output(
            [ self.git, "show", ":%s" % (name, ) ]).decode('utf-8')

    def file_from_head(self, name):
        return subprocess.check_output(
            [ self.git, "show", "%s:%s" % (self.head_revision(), name) ]) \
            .decode('utf-8')

def detect_repo(path="."):
    """Auto-detect the revision control system used for a source code
    directory. The code starts searching for repository meta data
    directories in path and then continues towards the root directory
    until root is reached or a metadatadirectory has been found.

    Returns: List of repository helper classes that can interface with
    the detected revision control system(s).

    """

    _repo_types = (
        (".git", GitRepo),
    )

    repo_types = []
    for repo_dir, repo_class in _repo_types:
        if os.path.exists(os.path.join(path, repo_dir)):
            repo_types.append(repo_class)

    if repo_types:
        return repo_types
    else:
        parent_dir = os.path.abspath(os.path.join(path, ".."))
        if not os.path.samefile(parent_dir, path):
            return detect_repo(path=parent_dir)
        else:
            # We reached the root directory without finding a meta
            # data directory.
            return []
