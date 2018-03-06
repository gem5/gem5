# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
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

from __future__ import print_function

import os
import sys

from slicc.parser import SLICC

usage="%prog [options] <files> ... "
version="%prog v0.4"
brief_copyright='''
Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
Copyright (c) 2009 The Hewlett-Packard Development Company
All Rights Reserved.
'''

def nprint(format, *args):
    pass

def eprint(format, *args):
    if args:
        format = format % args

    print(format, file=sys.stderr)

def main(args=None):
    import optparse

    parser = optparse.OptionParser(usage=usage, version=version,
                                   description=brief_copyright)
    parser.add_option("-d", "--debug", default=False, action="store_true",
                      help="Turn on PLY debugging")
    parser.add_option("-C", "--code-path", default="generated",
                      help="Path where C++ code output code goes")
    parser.add_option("-H", "--html-path",
                      help="Path where html output goes")
    parser.add_option("-F", "--print-files",
                      help="Print files that SLICC will generate")
    parser.add_option("--tb", "--traceback", action='store_true',
                      help="print traceback on error")
    parser.add_option("-q", "--quiet",
                      help="don't print messages")
    opts,files = parser.parse_args(args=args)

    if len(files) != 1:
        parser.print_help()
        sys.exit(2)

    output = nprint if opts.quiet else eprint

    output("SLICC v0.4")
    output("Parsing...")

    slicc = SLICC(files[0], verbose=True, debug=opts.debug, traceback=opts.tb)

    if opts.print_files:
        for i in sorted(slicc.files()):
            print('    %s' % i)
    else:
        output("Processing AST...")
        slicc.process()

        output("Writing C++ files...")
        slicc.writeCodeFiles(opts.code_path)

        if opts.html_path:
            output("Writing HTML files...")
            slicc.writeHTMLFiles(opts.html_path)

    output("SLICC is Done.")

if __name__ == "__main__":
    main()
