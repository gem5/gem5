#!/usr/bin/env python2.7
'''
The main source for testlib. Ties together the default test runners and
loaders.

Discovers and runs all tests from a given root directory.
'''
from __future__ import print_function

import sys
import os

base_dir = os.path.dirname(os.path.abspath(__file__))
ext_path = os.path.join(base_dir, os.pardir, 'ext')

sys.path.insert(0, base_dir)
sys.path.insert(0, ext_path)

import testlib.main as testlib
import testlib.config as config
import testlib.helper as helper

config.basedir = helper.absdirpath(__file__)
testlib()
