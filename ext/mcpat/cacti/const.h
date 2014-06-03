/*****************************************************************************
 *                                McPAT/CACTI
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#ifndef __CONST_H__
#define __CONST_H__

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*  The following are things you might want to change
 *  when compiling
 */

/*
 * Address bits in a word, and number of output bits from the cache
 */

/*
was: #define ADDRESS_BITS 32
now: I'm using 42 bits as in the Power4,
since that's bigger then the 36 bits on the Pentium 4
and 40 bits on the Opteron
*/
const int ADDRESS_BITS = 42;

/*dt: In addition to the tag bits, the tags also include 1 valid bit, 1 dirty bit, 2 bits for a 4-state
  cache coherency protocoll (MESI), 1 bit for MRU (change this to log(ways) for full LRU).
  So in total we have 1 + 1 + 2 + 1 = 5 */
const int EXTRA_TAG_BITS = 5;

/* limits on the various N parameters */

const unsigned int MAXDATAN     = 512;      // maximum for Ndwl and Ndbl
const unsigned int MAXSUBARRAYS = 1048576;  // maximum subarrays for data and tag arrays
const unsigned int MAXDATASPD   = 256;      // maximum for Nspd
const unsigned int MAX_COL_MUX  = 256;



#define ROUTER_TYPES 3
#define WIRE_TYPES 6

const double Cpolywire = 0;


/* Threshold voltages (as a proportion of Vdd)
   If you don't know them, set all values to 0.5 */
#define VTHFA1         0.452
#define VTHFA2         0.304
#define VTHFA3         0.420
#define VTHFA4         0.413
#define VTHFA5         0.405
#define VTHFA6         0.452
#define VSINV          0.452
#define VTHCOMPINV     0.437
#define VTHMUXNAND     0.548  // TODO : this constant must be revisited
#define VTHEVALINV     0.452
#define VTHSENSEEXTDRV 0.438


//WmuxdrvNANDn and WmuxdrvNANDp are no longer being used but it's part of the old
//delay_comparator function which we are using exactly as it used to be, so just setting these to 0
const double WmuxdrvNANDn = 0;
const double WmuxdrvNANDp = 0;


/*===================================================================*/
/*
 * The following are things you probably wouldn't want to change.
 */

#define BIGNUM 1e30
#define INF 9999999
#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))

/* Used to communicate with the horowitz model */
#define RISE 1
#define FALL 0
#define NCH  1
#define PCH  0


#define EPSILON 0.5 //v4.1: This constant is being used in order to fix floating point -> integer
//conversion problems that were occuring within CACTI. Typical problem that was occuring was
//that with different compilers a floating point number like 3.0 would get represented as either
//2.9999....or 3.00000001 and then the integer part of the floating point number (3.0) would
//be computed differently depending on the compiler. What we are doing now is to replace
//int (x) with (int) (x+EPSILON) where EPSILON is 0.5. This would fix such problems. Note that
//this works only when x is an integer >= 0.
/*
 * Sheng thinks this is more a solution to solve the simple truncate problem
 * (http://www.cs.tut.fi/~jkorpela/round.html) rather than the problem mentioned above.
 * Unfortunately, this solution causes nasty bugs (different results when using O0 and O3).
 * Moreover, round is not correct in CACTI since when an extra fraction of bit/line is needed,
 * we need to provide a complete bit/line even the fraction is just 0.01.
 * So, in later version than 6.5 we use (int)ceil() to get double to int conversion.
 */

#define EPSILON2 0.1
#define EPSILON3 0.6


#define MINSUBARRAYROWS 16 //For simplicity in modeling, for the row decoding structure, we assume
//that each row predecode block is composed of at least one 2-4 decoder. When the outputs from the
//row predecode blocks are combined this means that there are at least 4*4=16 row decode outputs
#define MAXSUBARRAYROWS 262144 //Each row predecode block produces a max of 2^9 outputs. So
//the maximum number of row decode outputs will be 2^9*2^9
#define MINSUBARRAYCOLS 2
#define MAXSUBARRAYCOLS 262144


#define INV 0
#define NOR 1
#define NAND 2


#define NUMBER_TECH_FLAVORS 4

#define NUMBER_INTERCONNECT_PROJECTION_TYPES 2 //aggressive and conservative
//0 = Aggressive projections, 1 = Conservative projections
#define NUMBER_WIRE_TYPES 4 //local, semi-global and global
//1 = 'Semi-global' wire type, 2 = 'Global' wire type


const int dram_cell_tech_flavor = 3;


#define VBITSENSEMIN 0.08 //minimum bitline sense voltage is fixed to be 80 mV.

#define fopt 4.0

#define INPUT_WIRE_TO_INPUT_GATE_CAP_RATIO 0
#define BUFFER_SEPARATION_LENGTH_MULTIPLIER 1
#define NUMBER_MATS_PER_REDUNDANT_MAT 8

#define NUMBER_STACKED_DIE_LAYERS 1

// this variable can be set to carry out solution optimization for
// a maximum area allocation.
#define STACKED_DIE_LAYER_ALLOTED_AREA_mm2 0 //6.24 //6.21//71.5

// this variable can also be employed when solution optimization
// with maximum area allocation is carried out.
#define MAX_PERCENT_AWAY_FROM_ALLOTED_AREA 50

// this variable can also be employed when solution optimization
// with maximum area allocation is carried out.
#define MIN_AREA_EFFICIENCY 20

// this variable can be employed when solution with a desired
// aspect ratio is required.
#define STACKED_DIE_LAYER_ASPECT_RATIO 1

// this variable can be employed when solution with a desired
// aspect ratio is required.
#define MAX_PERCENT_AWAY_FROM_ASPECT_RATIO 101

// this variable can be employed to carry out solution optimization
// for a certain target random cycle time.
#define TARGET_CYCLE_TIME_ns 1000000000

#define NUMBER_PIPELINE_STAGES 4

// this can be used to model the length of interconnect
// between a bank and a crossbar
#define LENGTH_INTERCONNECT_FROM_BANK_TO_CROSSBAR 0 //3791 // 2880//micron

#define IS_CROSSBAR 0
#define NUMBER_INPUT_PORTS_CROSSBAR 8
#define NUMBER_OUTPUT_PORTS_CROSSBAR 8
#define NUMBER_SIGNALS_PER_PORT_CROSSBAR 256


#define MAT_LEAKAGE_REDUCTION_DUE_TO_SLEEP_TRANSISTORS_FACTOR 1
#define LEAKAGE_REDUCTION_DUE_TO_LONG_CHANNEL_HP_TRANSISTORS_FACTOR 1

#define PAGE_MODE 0

#define MAIN_MEM_PER_CHIP_STANDBY_CURRENT_mA 60
// We are actually not using this variable in the CACTI code. We just want to acknowledge that
// this current should be multiplied by the DDR(n) system VDD value to compute the standby power
// consumed during precharge.


const double VDD_STORAGE_LOSS_FRACTION_WORST = 0.125;
const double CU_RESISTIVITY = 0.022; //ohm-micron
const double BULK_CU_RESISTIVITY = 0.018; //ohm-micron
const double PERMITTIVITY_FREE_SPACE = 8.854e-18; //F/micron

const static uint32_t sram_num_cells_wl_stitching_ = 16;
const static uint32_t dram_num_cells_wl_stitching_ = 64;
const static uint32_t comm_dram_num_cells_wl_stitching_ = 256;
const static double num_bits_per_ecc_b_          = 8.0;

const double    bit_to_byte  = 8.0;

#define MAX_NUMBER_GATES_STAGE 20
#define MAX_NUMBER_HTREE_NODES 20
#define NAND2_LEAK_STACK_FACTOR 0.2
#define NAND3_LEAK_STACK_FACTOR 0.2
#define NOR2_LEAK_STACK_FACTOR 0.2
#define INV_LEAK_STACK_FACTOR  0.5
#define MAX_NUMBER_ARRAY_PARTITIONS 1000000

// abbreviations used in this project
// ----------------------------------
//
//  num  : number
//  rw   : read/write
//  rd   : read
//  wr   : write
//  se   : single-ended
//  sz   : size
//  F    : feature
//  w    : width
//  h    : height or horizontal
//  v    : vertical or velocity


enum ram_cell_tech_type_num {
    itrs_hp   = 0,
    itrs_lstp = 1,
    itrs_lop  = 2,
    lp_dram   = 3,
    comm_dram = 4
};

const double pppm[4]      = {1, 1, 1, 1};
const double pppm_lkg[4]  = {0, 1, 1, 0};
const double pppm_dyn[4]  = {1, 0, 0, 0};
const double pppm_Isub[4] = {0, 1, 0, 0};
const double pppm_Ig[4]   = {0, 0, 1, 0};
const double pppm_sc[4]   = {0, 0, 0, 1};



#endif
