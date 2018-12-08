## ****************************************************************************
##
##  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
##  more contributor license agreements.  See the NOTICE file distributed
##  with this work for additional information regarding copyright ownership.
##  Accellera licenses this file to you under the Apache License, Version 2.0
##  (the "License"); you may not use this file except in compliance with the
##  License.  You may obtain a copy of the License at
##
##   http://www.apache.org/licenses/LICENSE-2.0
##
##  Unless required by applicable law or agreed to in writing, software
##  distributed under the License is distributed on an "AS IS" BASIS,
##  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
##  implied.  See the License for the specific language governing
##  permissions and limitations under the License.
##
## ****************************************************************************
##
##  src/tlm_utils/Makefile.am --
##  Process this file with automake to produce a Makefile.in file.
##
##  Original Author: Alan Fitch, Doulos, 2012-03-10
##
## ****************************************************************************
##
##  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
##  changes you are making here.
##
##      Name, Affiliation, Date:
##  Description of Modification:
##
## ****************************************************************************

include $(top_srcdir)/config/Make-rules.sysc

H_FILES = \
	convenience_socket_bases.h \
	instance_specific_extensions.h \
	instance_specific_extensions_int.h \
	multi_passthrough_initiator_socket.h \
	multi_passthrough_target_socket.h \
	multi_socket_bases.h \
	passthrough_target_socket.h \
	peq_with_cb_and_phase.h \
	peq_with_get.h \
	simple_initiator_socket.h \
	simple_target_socket.h \
	tlm_quantumkeeper.h

CXX_FILES = \
	convenience_socket_bases.cpp \
	instance_specific_extensions.cpp

EXTRA_DIST += \
	README.txt

localincludedir = $(includedir)/tlm_utils
nobase_localinclude_HEADERS = $(H_FILES)

noinst_LTLIBRARIES = libtlm_utils.la
libtlm_utils_la_SOURCES = $(NO_H_FILES) $(CXX_FILES)
