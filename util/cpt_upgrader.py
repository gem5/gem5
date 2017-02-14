#!/usr/bin/env python2

# Copyright (c) 2012-2013,2015-2016 ARM Limited
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
#          Curtis Dunham
#

# This python code is used to migrate checkpoints that were created in one
# version of the simulator to newer version. As features are added or bugs are
# fixed some of the state that needs to be checkpointed can change. If you have
# many historic checkpoints that you use, manually editing them to fix them is
# both time consuming and error-prone.

# This script provides a way to migrate checkpoints to the newer repository in
# a programmatic way. It can be imported into another script or used on the
# command line. From the command line the script will either migrate every
# checkpoint it finds recursively (-r option) or a single checkpoint. When a
# change is made to the gem5 repository that breaks previous checkpoints an
# upgrade() method should be implemented in its own .py file and placed in
# src/util/cpt_upgraders/.  For each upgrader whose tag is not present in
# the checkpoint tag list, the upgrade() method will be run, passing in a
# ConfigParser object which contains the open file. As these operations can
# be isa specific the method can verify the isa and use regexes to find the
# correct sections that need to be updated.

# It is also possible to use this mechanism to revert prior tags.  In this
# case, implement a downgrade() method instead.  Dependencies should still
# work naturally - a tag depending on a tag with a downgrader means that it
# insists on the other tag being removed and its downgrader executed before
# its upgrader (or downgrader) can run.  It is still the case that a tag
# can only be used once.

# Dependencies between tags are expressed by two variables at the top-level
# of the upgrader script: "depends" can be either a string naming another
# tag that it depends upon or a list of such strings; and "fwd_depends"
# accepts the same datatypes but it reverses the sense of the dependency
# arrow(s) -- it expresses that that tag depends upon the tag of the current
# upgrader. This can be especially valuable when maintaining private
# upgraders in private branches.


import ConfigParser
import glob, types, sys, os
import os.path as osp

verbose_print = False

def verboseprint(*args):
    if not verbose_print:
        return
    for arg in args:
        print arg,
    print

class Upgrader:
    tag_set = set()
    untag_set = set() # tags to remove by downgrading
    by_tag = {}
    legacy = {}
    def __init__(self, filename):
        self.filename = filename
        execfile(filename, {}, self.__dict__)

        if not hasattr(self, 'tag'):
            self.tag = osp.basename(filename)[:-3]
        if not hasattr(self, 'depends'):
            self.depends = []
        elif isinstance(self.depends, str):
            self.depends = [self.depends]

        if not isinstance(self.depends, list):
            print "Error: 'depends' for %s is the wrong type" % self.tag
            sys.exit(1)

        if hasattr(self, 'fwd_depends'):
            if isinstance(self.fwd_depends, str):
                self.fwd_depends = [self.fwd_depends]
        else:
            self.fwd_depends = []

        if not isinstance(self.fwd_depends, list):
            print "Error: 'fwd_depends' for %s is the wrong type" % self.tag
            sys.exit(1)

        if hasattr(self, 'upgrader'):
            if not isinstance(self.upgrader, types.FunctionType):
                print "Error: 'upgrader' for %s is %s, not function" \
                    % (self.tag, type(self))
                sys.exit(1)
            Upgrader.tag_set.add(self.tag)
        elif hasattr(self, 'downgrader'):
            if not isinstance(self.downgrader, types.FunctionType):
                print "Error: 'downgrader' for %s is %s, not function" \
                    % (self.tag, type(self))
                sys.exit(1)
            Upgrader.untag_set.add(self.tag)
        else:
            print "Error: no upgrader or downgrader method for", self.tag
            sys.exit(1)

        if hasattr(self, 'legacy_version'):
            Upgrader.legacy[self.legacy_version] = self

        Upgrader.by_tag[self.tag] = self

    def ready(self, tags):
        for dep in self.depends:
            if dep not in tags:
                return False
        return True

    def update(self, cpt, tags):
        if hasattr(self, 'upgrader'):
            self.upgrader(cpt)
            tags.add(self.tag)
            verboseprint("applied upgrade for", self.tag)
        else:
            self.downgrader(cpt)
            tags.remove(self.tag)
            verboseprint("applied downgrade for", self.tag)

    @staticmethod
    def get(tag):
        return Upgrader.by_tag[tag]

    @staticmethod
    def load_all():
        util_dir = osp.dirname(osp.abspath(__file__))

        for py in glob.glob(util_dir + '/cpt_upgraders/*.py'):
            Upgrader(py)

        # make linear dependences for legacy versions
        i = 3
        while i in Upgrader.legacy:
            Upgrader.legacy[i].depends = [Upgrader.legacy[i-1].tag]
            i = i + 1

        # resolve forward dependencies and audit normal dependencies
        for tag, upg in Upgrader.by_tag.items():
            for fd in upg.fwd_depends:
                if fd not in Upgrader.by_tag:
                    print "Error: '%s' cannot (forward) depend on "\
                          "nonexistent tag '%s'" % (fd, tag)
                    sys.exit(1)
                Upgrader.by_tag[fd].depends.append(tag)
            for dep in upg.depends:
                if dep not in Upgrader.by_tag:
                    print "Error: '%s' cannot depend on "\
                          "nonexistent tag '%s'" % (tag, dep)
                    sys.exit(1)

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

    change = False

    # Make sure we know what we're starting from
    if cpt.has_option('root','cpt_ver'):
        cpt_ver = cpt.getint('root','cpt_ver')

        # Legacy linear checkpoint version
        # convert to list of tags before proceeding
        tags = set([])
        for i in xrange(2, cpt_ver+1):
            tags.add(Upgrader.legacy[i].tag)
        verboseprint("performed legacy version -> tags conversion")
        change = True

        cpt.remove_option('root', 'cpt_ver')
    elif cpt.has_option('Globals','version_tags'):
        tags = set((''.join(cpt.get('Globals','version_tags'))).split())
    else:
        print "fatal: no version information in checkpoint"
        exit(1)

    verboseprint("has tags", ' '.join(tags))
    # If the current checkpoint has a tag we don't know about, we have
    # a divergence that (in general) must be addressed by (e.g.) merging
    # simulator support for its changes.
    unknown_tags = tags - (Upgrader.tag_set | Upgrader.untag_set)
    if unknown_tags:
        print "warning: upgrade script does not recognize the following "\
              "tags in this checkpoint:", ' '.join(unknown_tags)

    # Apply migrations for tags not in checkpoint and tags present for which
    # downgraders are present, respecting dependences
    to_apply = (Upgrader.tag_set - tags) | (Upgrader.untag_set & tags)
    while to_apply:
        ready = set([ t for t in to_apply if Upgrader.get(t).ready(tags) ])
        if not ready:
            print "could not apply these upgrades:", ' '.join(to_apply)
            print "update dependences impossible to resolve; aborting"
            exit(1)

        for tag in ready:
            Upgrader.get(tag).update(cpt, tags)
            change = True

        to_apply -= ready

    if not change:
        verboseprint("...nothing to do")
        return

    cpt.set('Globals', 'version_tags', ' '.join(tags))

    # Write the old data back
    verboseprint("...completed")
    cpt.write(file(path, 'w'))

if __name__ == '__main__':
    from optparse import OptionParser, SUPPRESS_HELP
    parser = OptionParser("usage: %prog [options] <filename or directory>")
    parser.add_option("-r", "--recurse", action="store_true",
                      help="Recurse through all subdirectories modifying "\
                           "each checkpoint that is found")
    parser.add_option("-N", "--no-backup", action="store_false",
                      dest="backup", default=True,
                      help="Do no backup each checkpoint before modifying it")
    parser.add_option("-v", "--verbose", action="store_true",
                      help="Print out debugging information as")
    parser.add_option("--get-cc-file", action="store_true",
                      # used during build; generate src/sim/tags.cc and exit
                      help=SUPPRESS_HELP)

    (options, args) = parser.parse_args()
    verbose_print = options.verbose

    Upgrader.load_all()

    if options.get_cc_file:
        print "// this file is auto-generated by util/cpt_upgrader.py"
        print "#include <string>"
        print "#include <set>"
        print
        print "std::set<std::string> version_tags = {"
        for tag in Upgrader.tag_set:
            print "  \"%s\"," % tag
        print "};"
        exit(0)
    elif len(args) != 1:
        parser.error("You must specify a checkpoint file to modify or a "\
                     "directory of checkpoints to recursively update")

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

