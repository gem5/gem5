#
# Copyright (c) 2014-2015 ARM Limited
# All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Andreas Sandberg

# Makefile fragment for executables

$(d)/%.o: $(d)/%.c
	$(CC) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<

$(d)/%.o: $(d)/%.cc
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $@ $<

$(d)/%.d: $(d)/%.c
	$(CC) -MM -MT $(<:.c=.o) $(CPPFLAGS) $(CFLAGS) -o $@ $<

$(d)/%.d: $(d)/%.c
	$(CXX) -MM -MT $(<:.cc=.o) $(CPPFLAGS) $(CXXFLAGS) -o $@ $<
