# Copyright (c) 2008 The Hewlett-Packard Development Company
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
import importlib.abc
import importlib.util
import os


class ByteCodeLoader(importlib.abc.Loader):
    def __init__(self, code):
        super().__init__()
        self.code = code

    def exec_module(self, module):
        exec(self.code, module.__dict__)


# Simple importer that allows python to import data from a dict of
# code objects.  The keys are the module path, and the items are the
# filename and bytecode of the file.
class CodeImporter(object):
    def __init__(self):
        self.modules = {}
        override_var = os.environ.get("M5_OVERRIDE_PY_SOURCE", "false")
        self.override = override_var.lower() in ("true", "yes")

    def add_module(self, abspath, modpath, code):
        if modpath in self.modules:
            raise AttributeError(f"{modpath} already found in importer")

        self.modules[modpath] = (abspath, code)

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.modules:
            return None

        abspath, code = self.modules[fullname]

        if self.override and os.path.exists(abspath):
            src = open(abspath, "r").read()
            code = compile(src, abspath, "exec")

        is_package = os.path.basename(abspath) == "__init__.py"
        spec = importlib.util.spec_from_loader(
            name=fullname, loader=ByteCodeLoader(code), is_package=is_package
        )

        spec.loader_state = self.modules.keys()

        return spec


# Create an importer and add it to the meta_path so future imports can
# use it.  There's currently nothing in the importer, but calls to
# add_module can be used to add code.
def install():
    importer = CodeImporter()
    global add_module
    add_module = importer.add_module
    import sys

    sys.meta_path.insert(0, importer)

    # Injected into this module's namespace by the c++ code that loads it.
    _init_all_embedded()
