#!/usr/bin/env python

# Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
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

"""
SYNOPSIS

    ./regression/regression.py ./regression/

DESCRIPTION

    Runs regression tester for McPAT.
    This tester can compile and runs McPAT on the input contained in the
    specified directory, and then compares the output to that of a prior run in
    order to ensure that specific power and area calculations do not change.

AUTHORS

    Joel Hestness <hestness@cs.wisc.edu> (while interning at AMD)
    Yasuko Eckert <yasuko.eckert@amd.com>
"""

import os
import sys
import optparse
import re

################################
# Global Variables
################################

global mcpat_binary
mcpat_binary = "../../build/mcpat/mcpat"
global optionsparser

################################
# Global Functions
################################

def run_test(testdir):
    test_passed = True
    testfiles = os.listdir(testdir)
    for testfile in testfiles:
        # For each power_region file, run McPAT on it and check the
        # output created against the regression
        if re.match("power_region.*\.xml$", testfile):
            # Get the region index of the test
            fileparts = testfile.split(".")
            region_index = fileparts[0][12:]
            regression_test = os.path.join(testdir, testfile)
            regression_output = os.path.join(
                    testdir, "region%s.out" % region_index)
            regression_correct = os.path.join(
                    testdir, "region%s.out.ref" % region_index)
            print "Running test: %s..." % regression_test
            # Run McPAT on the input
            os.system(
                    "%s -infile %s -print_level 10 > %s" %
                    (mcpat_binary, regression_test, regression_output) )
            if os.path.exists(regression_correct):
                diff = os.popen(
                        "diff %s %s" % (regression_output, regression_correct),
                        "r").read()
                if diff != "":
                    print "WARN: Differences found in %s" % regression_output
                    if options.verbose:
                        print diff
                    test_passed = False
            else:
                print "WARN: Regression test not set up: %s..." % regression_test
                print "WARN: Not able to verify test"
                test_passed = False

            if options.cleanup:
                if options.verbose:
                    print "WARN: Cleaning (deleting) regression output file: "\
                            "%s" % regression_output
                os.system("rm -f %s" % regression_output)

    if test_passed:
        print "PASSED: %s\n\n" % testdir
    else:
        print "FAILED: %s\n\n" % testdir

def has_power_region_files(testdir):
    files = os.listdir(testdir)
    for file in files:
        if "power_region" in file and ".xml" in file:
            return True

def is_valid_test_directory(testdir):
    valid_regression = True
    power_region_file_found = False

    files = os.listdir(testdir)
    for file in files:
        if "power_region" in file and ".xml" in file:
            power_region_file_found = True
            fileparts = file.split(".")
            region_index = fileparts[0][12:]
            regression_output = os.path.join(
                    testdir, "region%s.out.ref" % region_index)
            if os.path.exists(regression_output):
                if options.verbose:
                    print "Valid regression test: %s/%s" % (testdir, file)
            else:
                valid_regression = False

    return valid_regression and power_region_file_found

################################
# Execute here
################################

optionsparser = optparse.OptionParser(
        formatter = optparse.TitledHelpFormatter(),
        usage = globals()['__doc__'])
optionsparser.add_option(
        "-b", "--build", action = "store_true", default = False,
        help = "Build McPAT before running tests")
optionsparser.add_option(
        "-c", "--cleanup", action = "store_true", default = False,
        help = "Clean up the specified regression directory")
optionsparser.add_option(
        "-f", "--force", action = "store_true", default = False,
        help = "Force run regression even if directory isn't set up")
optionsparser.add_option(
        "-m", "--maketest", action = "store_true", default = False,
        help = "Set up the specified test directory")
optionsparser.add_option(
        "-v", "--verbose", action = "store_true", default = False,
        help = "Print verbose output")
(options, args) = optionsparser.parse_args()

if not os.path.exists(mcpat_binary) and not options.build:
    print "ERROR: McPAT binary does not exist: %s" % mcpat_binary
    exit(0)

if options.build:
    print "Building McPAT..."
    bin_dir = os.path.dirname(mcpat_binary)
    directory = os.path.join(bin_dir, "../../ext/mcpat")
    build_output = os.popen(
            "cd %s; make clean; make -j 8 dbg 2>&1" % directory).read()
    if "error" in build_output.lower():
        print "Error during build: %s" % build_output
        exit(0)

if len(args) < 1:
    print "ERROR: Must specify regressions directory"
    exit(0)

# check params
rootdir = args[0];
if not os.path.exists(rootdir):
    print "ERROR: Regressions directory does not exist: %s" % rootdir
    exit(0)

if options.maketest:
    # The specified rootdir must exist since we got here
    # Check if directory has tests
    list = os.listdir(rootdir)
    found_test = False
    for file in list:
        if "power_region" in file and "out" not in file and "ref" not in file:
            found_test = True
            # Prepare to run the test in order to set it up
            fileparts = file.split(".")
            region_index = fileparts[0][12:]
            regression_test = os.path.join(rootdir, file)
            regression_output = os.path.join(
                    rootdir, "region%s.out.ref" % region_index)
            if os.path.exists(regression_output):
                print "WARN: Overwriting old regression output: " \
                        "%s" % regression_output
            # Run the test to set it up
            print "Writing new regression output..."
            os.system(
                    "%s -infile %s -print_level 10 > %s" %
                    (mcpat_binary, regression_test, regression_output))

    if not found_test:
        print "ERROR: Invalid test directory: %s" % rootdir
        print "ERROR: Must contain XML file power_region*.xml"

    exit(0)

found_test = False
if has_power_region_files(rootdir):
    found_test = True
    if is_valid_test_directory(rootdir) or options.force:
        run_test(rootdir)
    else:
        print "WARN: Regression directory is not set up: %s" % rootdir
else:
    folders = os.listdir(rootdir)
    folders.sort()
    for folder in folders:
        testdir = os.path.join(rootdir, folder)
        if os.path.isdir(testdir):
            if has_power_region_files(testdir):
                found_test = True
                if is_valid_test_directory(testdir):
                    run_test(testdir)
                else:
                    if options.force:
                        print "WARN: Regression directory is not set up: " \
                                "%s" % testdir
                        print "WARN: Running test anyway: %s..." % testdir
                        run_test(testdir)
                    else:
                        print "Regression directory is not set up: %s" % testdir
            else:
                print "Not a valid test directory: %s" % testdir

if not found_test:
    print "No valid regressions found in %s" % rootdir
