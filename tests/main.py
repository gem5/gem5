#!/usr/bin/env python3
"""
The main source for testlib. Ties together the default test runners and
loaders.

Discovers and runs all tests from a given root directory.
"""

import os
import sys

os.environ["PYTHONUNBUFFERED"] = "1"

base_dir = os.path.dirname(os.path.abspath(__file__))
ext_path = os.path.join(base_dir, os.pardir, "ext")

sys.path.insert(0, base_dir)
sys.path.insert(0, ext_path)

import testlib.configuration as config
import testlib.helper as helper
import testlib.main as testlib

config.basedir = helper.absdirpath(__file__)
sys.exit(testlib())
