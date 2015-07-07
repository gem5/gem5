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

DOXYGEN = doxygen

GCC_VERSION := $(shell $(CC) -dumpversion | sed -e 's/\.//g')
ifeq "$(shell expr $(GCC_VERSION) \< 47)" "1"
$(error Default GCC version is too old. Please use gcc 4.7 or newer.)
endif

CPPFLAGS = -Iinclude/
CFLAGS = -fvisibility=hidden -O1 -g -Wall
CXXFLAGS = -std=c++0x $(CFLAGS)
LDFLAGS=

LIB_CPPFLAGS = $(CPPFLAGS)
LIB_CFLAGS = $(CFLAGS) -fPIC
LIB_CXXFLAGS = $(CXXFLAGS) -fPIC
LIB_LDFLAGS= $(LDFLAGS) -shared

# Default targets
ALL :=

# Test targets
ALL_TESTS :=

# Dependency includes
DEPS :=

# Files/directories to remove in the clean target
CLEAN :=

all: _all

dir:=lib
include $(dir)/Rules.mk

dir:=tests
include $(dir)/Rules.mk

_all: $(ALL)

test: $(ALL_TESTS)
	@set -e;			\
	for T in $^ ; do		\
	  echo "Running $${T}";		\
	  ./$${T};			\
	done

docs:
	$(DOXYGEN) Doxyfile

depclean:
	$(RM) $(DEPS)

clean:
	$(RM) -r $(CLEAN)
	$(RM) -r docs/html


.PHONY: all _all test depclean clean docs

# Include dependencies
-include $(MODEL_OBJS:.o=.d)
-include $(LIBMIDGARDMODEL_OBJS:.o=.d)
-include $(LIBNOMALI_OBJS:.o=.d)
