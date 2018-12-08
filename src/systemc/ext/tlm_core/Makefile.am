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
##  src/tlm_core/Makefile.am --
##  Process this file with automake to produce a Makefile.in file.
##
##  Original Author: Philipp A. Hartmann, OFFIS, 2013-05-17
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
	tlm_2/tlm_version.h \
	tlm_2/tlm_2_interfaces/tlm_2_interfaces.h \
	tlm_2/tlm_2_interfaces/tlm_dmi.h \
	tlm_2/tlm_2_interfaces/tlm_fw_bw_ifs.h \
	\
	tlm_2/tlm_generic_payload/tlm_array.h \
	tlm_2/tlm_generic_payload/tlm_endian_conv.h \
	tlm_2/tlm_generic_payload/tlm_generic_payload.h \
	tlm_2/tlm_generic_payload/tlm_gp.h \
	tlm_2/tlm_generic_payload/tlm_helpers.h \
	tlm_2/tlm_generic_payload/tlm_phase.h \
	\
	tlm_2/tlm_quantum/tlm_global_quantum.h \
	tlm_2/tlm_quantum/tlm_quantum.h \
	\
	tlm_2/tlm_sockets/tlm_base_socket_if.h \
	tlm_2/tlm_sockets/tlm_initiator_socket.h \
	tlm_2/tlm_sockets/tlm_sockets.h \
	tlm_2/tlm_sockets/tlm_target_socket.h \
	\
	tlm_1/tlm_analysis/tlm_analysis.h \
	tlm_1/tlm_analysis/tlm_analysis_fifo.h \
	tlm_1/tlm_analysis/tlm_analysis_if.h \
	tlm_1/tlm_analysis/tlm_analysis_port.h \
	tlm_1/tlm_analysis/tlm_analysis_triple.h \
	tlm_1/tlm_analysis/tlm_write_if.h \
	\
	tlm_1/tlm_req_rsp/tlm_req_rsp.h \
	tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_core_ifs.h \
	tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_fifo_ifs.h \
	tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_master_slave_ifs.h \
	tlm_1/tlm_req_rsp/tlm_1_interfaces/tlm_tag.h \
	\
	tlm_1/tlm_req_rsp/tlm_adapters/tlm_adapters.h \
	\
	tlm_1/tlm_req_rsp/tlm_channels/tlm_fifo/circular_buffer.h \
	tlm_1/tlm_req_rsp/tlm_channels/tlm_fifo/tlm_fifo.h \
	tlm_1/tlm_req_rsp/tlm_channels/tlm_fifo/tlm_fifo_peek.h \
	tlm_1/tlm_req_rsp/tlm_channels/tlm_fifo/tlm_fifo_put_get.h \
	tlm_1/tlm_req_rsp/tlm_channels/tlm_fifo/tlm_fifo_resize.h \
	tlm_1/tlm_req_rsp/tlm_channels/tlm_req_rsp_channels/tlm_put_get_imp.h \
	tlm_1/tlm_req_rsp/tlm_channels/tlm_req_rsp_channels/tlm_req_rsp_channels.h \
	\
	tlm_1/tlm_req_rsp/tlm_ports/tlm_event_finder.h \
	tlm_1/tlm_req_rsp/tlm_ports/tlm_nonblocking_port.h

CXX_FILES = \
	tlm_2/tlm_generic_payload/tlm_gp.cpp \
	tlm_2/tlm_generic_payload/tlm_phase.cpp \
	tlm_2/tlm_quantum/tlm_global_quantum.cpp

EXTRA_DIST += \
	tlm_1/README.txt \
	tlm_2/README.txt

localincludedir = $(includedir)/tlm_core
nobase_localinclude_HEADERS = $(H_FILES)

noinst_LTLIBRARIES = libtlm_core.la
libtlm_core_la_SOURCES = $(NO_H_FILES) $(CXX_FILES)
