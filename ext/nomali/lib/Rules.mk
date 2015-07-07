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

sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)

NOMALI_OBJS := $(addprefix $(d)/, 	\
	gpu.o				\
	gpublock.o			\
	gpucontrol.o			\
	jobcontrol.o			\
	jobslot.o			\
	mmu.o				\
					\
	mali_midgard.o			\
	mali_t6xx.o			\
	mali_t7xx.o			\
	)

LIBNOMALI_OBJS := $(addprefix $(d)/, 	\
	nomali_api.o			\
	)

OBJS := $(NOMALI_OBJS) 		\
	$(LIBMIDGARDMODEL_OBJS)

LIBS := libnomali.so

ALL := $(ALL) $(LIBS)
DEPS := $(DEPS)	$(OBJS:.o=.d)
CLEAN := $(CLEAN) $(OBJS) $(LIBS)

include Rules.lib.mk

libnomali.so: $(NOMALI_OBJS) $(LIBNOMALI_OBJS)
	$(CXX) $(LIB_LDFLAGS) -o $@ $^

d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))
