/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#ifndef ARCH_CONST_H_
#define ARCH_CONST_H_

typedef struct{
        unsigned int capacity;
        unsigned int assoc;//fully
        unsigned int blocksize;
} array_inputs;

//Do Not change, unless you want to bypass the XML interface and do not care about the default values.
//Global parameters
const int  			number_of_cores =	8;
const int  			number_of_L2s 	=	1;
const int 			number_of_L3s	=	1;
const int 			number_of_NoCs	=	1;

const double 		archi_F_sz_nm	=	90.0;
const unsigned int 	dev_type		=	0;
const double 		CLOCKRATE 		= 	1.2*1e9;
const double 		AF 				= 	0.5;
//const bool 			inorder			=	true;
const bool			embedded		=	false; //NEW

const bool 			homogeneous_cores	= 	true;
const bool 			temperature		=	360;
const int			number_cache_levels	=	3;
const int			L1_property		=	0; //private 0; coherent 1, shared 2.
const int		 	L2_property		=	2;
const bool	    	homogeneous_L2s	=	true;
const bool		    L3_property		= 	2;
const bool 			homogeneous_L3s	=	true;
const double 		Max_area_deviation	=	50;
const double	    Max_dynamic_deviation	=50; //New
const int 			opt_dynamic_power	=	1;
const int 			opt_lakage_power	=	0;
const int		 	opt_area			=	0;
const int			interconnect_projection_type	=	0;

//******************************Core Parameters
#if (inorder)
const int opcode_length			= 	8;//Niagara
const int reg_length			=	5;//Niagara
const int instruction_length	=	32;//Niagara
const int data_width			=	64;
#else
const int opcode_length			= 	8;//16;//Niagara
const int reg_length			=	7;//Niagara
const int instruction_length	=	32;//Niagara
const int data_width			=	64;
#endif


//Caches
//itlb
const int itlbsize=512;
const int itlbassoc=0;//fully
const int itlbblocksize=8;
//icache
const int icachesize=32768;
const int icacheassoc=4;
const int icacheblocksize=32;
//dtlb
const int dtlbsize=512;
const int dtlbassoc=0;//fully
const int dtlbblocksize=8;
//dcache
const int dcachesize=32768;
const int dcacheassoc=4;
const int dcacheblocksize=32;
const int dcache_write_buffers=8;

//cache controllers
//IB,
const int numIBEntries			=	64;
const int IBsize				=	64;//2*4*instruction_length/8*2;
const int IBassoc				=	0;//In Niagara it is still fully associ
const int IBblocksize			=	4;

//IFB and MIL should have the same parameters CAM
const int IFBsize=128;//
const int IFBassoc=0;//In Niagara it is still fully associ
const int IFBblocksize=4;




const int icache_write_buffers=8;

//register file RAM
const int regfilesize=5760;
const int regfileassoc=1;
const int regfileblocksize=18;
//regwin  RAM
const int regwinsize=256;
const int regwinassoc=1;
const int regwinblocksize=8;



//store buffer, lsq
const int lsqsize=512;
const int lsqassoc=0;
const int lsqblocksize=8;

//data fill queue RAM
const int dfqsize=1024;
const int dfqassoc=1;
const int dfqblocksize=16;

//outside the cores
//L2 cache bank
const int l2cachesize=262144;
const int l2cacheassoc=16;
const int l2cacheblocksize=64;

//L2 directory
const int l2dirsize=1024;
const int l2dirassoc=0;
const int l2dirblocksize=2;

//crossbar
//PCX
const int PCX_NUMBER_INPUT_PORTS_CROSSBAR = 8;
const int PCX_NUMBER_OUTPUT_PORTS_CROSSBAR = 9;
const int PCX_NUMBER_SIGNALS_PER_PORT_CROSSBAR =144;
//PCX buffer RAM
const int pcx_buffersize=1024;
const int pcx_bufferassoc=1;
const int pcx_bufferblocksize=32;
const int pcx_numbuffer=5;
//pcx arbiter
const int pcx_arbsize=128;
const int pcx_arbassoc=1;
const int pcx_arbblocksize=2;
const int pcx_numarb=5;

//CPX
const int CPX_NUMBER_INPUT_PORTS_CROSSBAR = 5;
const int CPX_NUMBER_OUTPUT_PORTS_CROSSBAR = 8;
const int CPX_NUMBER_SIGNALS_PER_PORT_CROSSBAR =150;
//CPX buffer RAM
const int cpx_buffersize=1024;
const int cpx_bufferassoc=1;
const int cpx_bufferblocksize=32;
const int cpx_numbuffer=8;
//cpx arbiter
const int cpx_arbsize=128;
const int cpx_arbassoc=1;
const int cpx_arbblocksize=2;
const int cpx_numarb=8;





const int numPhysFloatRegs=256;
const int numPhysIntRegs=32;
const int numROBEntries=192;
const int umRobs=1;

const int BTBEntries=4096;
const int BTBTagSize=16;
const int LFSTSize=1024;
const int LQEntries=32;
const int RASSize=16;
const int SQEntries=32;
const int SSITSize=1024;
const int activity=0;
const int backComSize=5;
const int cachePorts=200;
const int choiceCtrBits=2;
const int choicePredictorSize=8192;


const int commitWidth=8;
const int decodeWidth=8;
const int dispatchWidth=8;
const int fetchWidth=8;
const int issueWidth=1;
const int renameWidth=8;
//what is this forwardComSize=5??

const int globalCtrBits=2;
const int globalHistoryBits=13;
const int globalPredictorSize=8192;



const int localCtrBits=2;
const int localHistoryBits=11;
const int localHistoryTableSize=2048;
const int localPredictorSize=2048;

const double Woutdrvnandn	=30 *0.09;//(24.0 * LSCALE)
const double Woutdrvnandp	=12.5 *0.09;//(10.0 * LSCALE)
const double Woutdrvnorn	=7.5*0.09;//(6.0 * LSCALE)
const double Woutdrvnorp  =50 * 0.09;//	(40.0 * LSCALE)
const double Woutdrivern	=60*0.09;//(48.0 * LSCALE)
const double Woutdriverp	=100 * 0.09;//(80.0 * LSCALE)

/*
smtCommitPolicy=RoundRobin
smtFetchPolicy=SingleThread
smtIQPolicy=Partitioned
smtIQThreshold=100
smtLSQPolicy=Partitioned
smtLSQThreshold=100
smtNumFetchingThreads=1
smtROBPolicy=Partitioned
smtROBThreshold=100
squashWidth=8
*/

/*
prefetch_access=false
prefetch_cache_check_push=true
prefetch_data_accesses_only=false
prefetch_degree=1
prefetch_latency=10000
prefetch_miss=false
prefetch_past_page=false
prefetch_policy=none
prefetch_serial_squash=false
prefetch_use_cpu_id=true
prefetcher_size=100
prioritizeRequests=false
repl=Null


split=false
split_size=0
subblock_size=0
tgts_per_mshr=20
trace_addr=0
two_queue=false

cpu_side=system.cpu0.dcache_port
mem_side=system.tol2bus.port[2]
*/

//[system.cpu0.dtb]
//type=AlphaDT


#endif /* ARCH_CONST_H_ */
