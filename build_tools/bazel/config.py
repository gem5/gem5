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

# TODO(hchsiao): Detect system package availability
import argparse
import os
import re
import logging

def convert_config(gem5_home, config_file):
  config_values = {}
  with open(config_file, 'r') as f:
    for line in f.readlines():
      if match := re.match(r'# ([A-Za-z0-9_]+) is not set', line):
        config_values[match.group(1)] = None
      elif match := re.match(r'([A-Za-z0-9_]+)=(.*)', line):
        config_values[match.group(1)] = match.group(2)
  # TODO(hchsiao): derive config from the detection result and .config
  fixed_config = '\n'.join([
      'build --//src/generated/flags:have_tuntap=True',
      'build --//src/generated/flags:build_isa=True',
      'build --//src/generated/flags:use_riscv_isa=True',
      'build --//build_tools/bazel:python_version=3.11',
  ] + [f'# {opt}={val}' for opt, val in config_values.items()])
  bazelrc_path = os.path.join(gem5_home, 'build/bazelrc')
  if not os.path.isdir(os.path.dirname(bazelrc_path)):
    os.mkdir(os.path.dirname(bazelrc_path))
  with open(bazelrc_path, 'w') as f:
    f.write(fixed_config)

def main():
  logging.basicConfig(level=logging.INFO)

  assert 'BUILD_WORKSPACE_DIRECTORY' in os.environ, 'invoke with `bazel run`'
  gem5_home = os.environ['BUILD_WORKSPACE_DIRECTORY']

  parser = argparse.ArgumentParser()
  parser.add_argument('config_file')
  args = parser.parse_args()

  if not os.path.isfile(args.config_file):
    raise RuntimeError(f'Cannot open {args.config_file}')

  logging.info(f'Writing config to `build/bazelrc`')
  convert_config(gem5_home, args.config_file)
  logging.info('Done')

if __name__ == '__main__':
  main()
