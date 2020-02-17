#!/usr/bin/env python

# Copyright (c) 2018, Cornell University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or
# without modification, are permitted provided that the following
# conditions are met:
#
# Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# Neither the name of Cornell University nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
# USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import argparse
import subprocess

from multiprocessing import Pool

import clusterjob
from clusterjob import *

#-------------------------------------------------------------------------
# utility function to run a process
#-------------------------------------------------------------------------
def execute(cmd):
  try:
    return subprocess.check_output(cmd, shell=True)
  except subprocess.CalledProcessError, err:
    print "ERROR: " + err.output

#-------------------------------------------------------------------------
# Input options
#-------------------------------------------------------------------------

# general options
parser = argparse.ArgumentParser(description='run RISC-V assembly tests')
parser.add_argument('--max-tick',
                    help = 'maximum simulated tick',
                    type = int,
                    default = 10000000000)
parser.add_argument('--num-cpus',
                    help = 'number of CPUs\
                            (some tests require at least 4 CPUs)',
                    type = int,
                    default = 4)
parser.add_argument('--ruby',
                    help = 'Use Ruby memory?',
                    action = 'store_true')

args = parser.parse_args()

# convert all paths to absolute paths
test_dir = os.path.abspath('../../bin/riscv/')
test_summary_out = './test-summary.out'

#-------------------------------------------------------------------------
# gem5 variables
#-------------------------------------------------------------------------
gem5_dir = os.path.abspath('../../../../../')
gem5_bin = os.path.join(gem5_dir, 'build', 'RISCV', 'gem5.opt')
config = os.path.join(gem5_dir, 'configs', 'example', 'se.py')

# list of CPU models to be tested
cpu_models = ['AtomicSimpleCPU',
              'TimingSimpleCPU',
              'MinorCPU',
              'DerivO3CPU']

# get a list of test binaries in the given directory
tests = []
for line in execute("ls %s/*" % (test_dir)).splitlines():
  if line:
    tests.append(line.split('/')[-1])

# total number of tests to run
n_tests = len(tests) * len(cpu_models)

# make a list of jobs
job_cmds = []
job_names = []
for test in tests:
  for model in cpu_models:
    test_name = test + '-' + model
    job_names.append(test_name)
    job_cmds.append([gem5_bin,
                     '-d', 'm5out/' + test_name,
                     '--listener-mode', 'off',
                     config,
                     '-m', str(args.max_tick),
                     '--cpu-type', model,
                     '-n', str(args.num_cpus),
                     '-c', test_dir + '/' + test,
                     '--ruby' if args.ruby else '--caches',
                    ])

# execute all jobs
job_pool = Pool(processes = n_tests)
job_outputs = job_pool.map(subprocess.call, job_cmds)
job_pool.close()

# process job outputs
file = open(test_summary_out, "w")

job_outputs = zip(job_names, job_outputs)
for entry in job_outputs:
  # a negative return value indicates that the job was terminated
  # by a signal
  # a positive return value indicates that the job exited with a return
  # value
  if entry[1] < 0:
    file.write("%-50s failed - signal = %d\n" % (entry[0], -1 * entry[1]))
  elif entry[1] > 0:
    file.write("%-50s failed - status = %d\n" % (entry[0], entry[1]))
  else:
    file.write("%-50s passed\n" % (entry[0]))

file.close()
