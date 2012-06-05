#!/usr/bin/env python

# Copyright (c) 2012 ARM Limited
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
#
# Authors: Ali Saidi
#

# This python code is used to migrate checkpoints that were created in one
# version of the simulator to newer version. As features are added or bugs are
# fixed some of the state that needs to be checkpointed can change. If you have
# many historic checkpoints that you use, manually editing them to fix them is
# both time consuming and error-prone.

# This script provides a way to migrate checkpoints to the newer repository in
# a programatic way. It can be imported into another script or used on the
# command line. From the command line the script will either migrate every
# checkpoint it finds recursively (-r option) or a single checkpoint. When a
# change is made to the gem5 repository that breaks previous checkpoints a
# from_N() method should be implemented here and the gem5CheckpointVersion
# variable in src/sim/serialize.hh should be incremented. For each version
# between the checkpoints current version and the new version the from_N()
# method will be run, passing in a ConfigParser object which contains the open
# file. As these operations can be isa specific the method can verify the isa
# and use regexes to find the correct sections that need to be updated.


import ConfigParser
import sys, os
import os.path as osp

def from_0(cpt):
    pass

# An example of a translator
def from_1(cpt):
    if cpt.get('root','isa') == 'arm':
        for sec in cpt.sections():
            import re
            # Search for all the execution contexts
            if re.search('.*sys.*\.cpu.*\.x.\..*', sec):
                # Update each one
                mr = cpt.get(sec, 'miscRegs').split()
                #mr.insert(21,0)
                #mr.insert(26,0)
                cpt.set(sec, 'miscRegs', ' '.join(str(x) for x in mr))

migrations = []
migrations.append(from_0)
migrations.append(from_1)

verbose_print = False

def verboseprint(*args):
    if not verbose_print:
        return
    for arg in args:
        print arg,
    print

def process_file(path, **kwargs):
    if not osp.isfile(path):
        import errno
        raise IOError(ennro.ENOENT, "No such file", path)

    verboseprint("Processing file %s...." % path)

    if kwargs.get('backup', True):
        import shutil
        shutil.copyfile(path, path + '.bak')

    cpt = ConfigParser.SafeConfigParser()

    # gem5 is case sensitive with paramaters
    cpt.optionxform = str

    # Read the current data
    cpt_file = file(path, 'r')
    cpt.readfp(cpt_file)
    cpt_file.close()

    # Make sure we know what we're starting from
    if not cpt.has_option('root','cpt_ver'):
        raise LookupError("cannot determine version of checkpoint")

    cpt_ver = cpt.getint('root','cpt_ver')

    # If the current checkpoint is longer than the migrations list, we have a problem
    # and someone didn't update this file
    if cpt_ver > len(migrations):
        raise ValueError("upgrade script is too old and needs updating")

    verboseprint("\t...file is at version %#x" % cpt_ver)

    if cpt_ver == len(migrations):
        verboseprint("\t...nothing to do")
        return

    # Walk through every function from now until the end fixing the checkpoint
    for v in xrange(cpt_ver,len(migrations)):
        verboseprint("\t...migrating to version %#x" %  (v + 1))
        migrations[v](cpt)
        cpt.set('root','cpt_ver', str(v + 1))

    # Write the old data back
    verboseprint("\t...completed")
    cpt.write(file(path, 'w'))


if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("usage: %prog [options] <filename or directory>")
    parser.add_option("-r", "--recurse", action="store_true",
                      help="Recurse through all subdirectories modifying "\
                           "each checkpoint that is found")
    parser.add_option("-N", "--no-backup", action="store_false",
                      dest="backup", default=True,
                      help="Do no backup each checkpoint before modifying it")
    parser.add_option("-v", "--verbose", action="store_true",
                      help="Print out debugging information as")

    (options, args) = parser.parse_args()
    if len(args) != 1:
        parser.error("You must specify a checkpoint file to modify or a "\
                     "directory of checkpoints to recursively update")

    verbose_print = options.verbose

    # Deal with shell variables and ~
    path = osp.expandvars(osp.expanduser(args[0]))

    # Process a single file if we have it
    if osp.isfile(path):
        process_file(path, **vars(options))
    # Process an entire directory
    elif osp.isdir(path):
        cpt_file = osp.join(path, 'm5.cpt')
        if options.recurse:
            # Visit very file and see if it matches
            for root,dirs,files in os.walk(path):
                for name in files:
                    if name == 'm5.cpt':
                        process_file(osp.join(root,name), **vars(options))
                for dir in dirs:
                    pass
        # Maybe someone passed a cpt.XXXXXXX directory and not m5.cpt
        elif osp.isfile(cpt_file):
            process_file(cpt_file, **vars(options))
        else:
            print "Error: checkpoint file not found at in %s " % path,
            print "and recurse not specified"
            sys.exit(1)
    sys.exit(0)

