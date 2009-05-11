/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
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
 *
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
 */

#ifndef _POWER_ARRAY_H
#define _POWER_ARRAY_H


#define SIM_ARRAY_NO_MODEL      0
#define SIM_ARRAY_READ          0
#define SIM_ARRAY_WRITE         1

#define SIM_ARRAY_RECOVER       1

/* read/write */
#define SIM_ARRAY_RW            0
/* only write */
#define SIM_ARRAY_WO            1

typedef enum {
        GENERIC_DEC =1,
        DEC_MAX_MODEL
} power_dec_model;

typedef enum {
        GENERIC_MUX =1,
        MUX_MAX_MODEL
} power_mux_model;

typedef enum {
        GENERIC_AMP =1,
        AMP_MAX_MODEL
} power_amp_model;

typedef enum {
        CACHE_RW_WORDLINE =1,
        CACHE_WO_WORDLINE,
        CAM_RW_WORDLINE,
        CAM_WO_WORDLINE,
        WORDLINE_MAX_MODEL
} power_wordline_model;

typedef enum {
        RW_BITLINE =1,
        WO_BITLINE,
        BITLINE_MAX_MODEL
} power_bitline_model;

typedef enum {
        SINGLE_BITLINE =1,
        EQU_BITLINE,
        SINGLE_OTHER,
        PRE_MAX_MODEL
} power_pre_model;

typedef enum {
        NORMAL_MEM =1,
        CAM_TAG_RW_MEM,
        CAM_TAG_WO_MEM,
        CAM_DATA_MEM,
        CAM_ATTACH_MEM,
        MEM_MAX_MODEL
} power_mem_model;

typedef enum {
        CACHE_COMPONENT =1,
        CAM_COMP,
        COMP_MAX_MODEL
} power_comp_model;

typedef enum {
        CACHE_OUTDRV =1,
        CAM_OUTDRV,
        REG_OUTDRV,
        OUTDRV_MAX_MODEL
} power_outdrv_model;



typedef struct {
        int model;
        unsigned n_bits;
        unsigned long int n_chg_output;
        unsigned long int n_chg_addr;
        unsigned long int n_chg_l1;
        double e_chg_output;
        double e_chg_addr;
        double e_chg_l1;
        unsigned n_in_1st;
        unsigned n_in_2nd;
        unsigned n_out_0th;
        unsigned n_out_1st;
        unsigned long int addr_mask;
} power_decoder;

typedef struct {
        int model;
        int share_rw;
        unsigned long int n_read;
        unsigned long int n_write;
        double e_read;
        double e_write;
        double i_leakage;
} power_wordline;

typedef struct {
        int model;
        int share_rw;
        unsigned end;
        unsigned long int n_col_write;
        unsigned long int n_col_read;
        unsigned long int n_col_sel;
        double e_col_write;
        double e_col_read;
        double e_col_sel;
        double i_leakage;
} power_bitline;

typedef struct {
        int model;
        unsigned long int n_access;
        double e_access;
} power_amp;

typedef struct {
        int model;
        unsigned n_bits;
        unsigned assoc;
        unsigned long int n_access;
        unsigned long int n_match;
        unsigned long int n_mismatch;
        unsigned long int n_miss;
        unsigned long int n_bit_match;
        unsigned long int n_bit_mismatch;
        unsigned long int n_chg_addr;
        double e_access;
        double e_match;
        double e_mismatch;
        double e_miss;
        double e_bit_match;
        double e_bit_mismatch;
        double e_chg_addr;
        unsigned long int comp_mask;
} power_comp;

typedef struct {
        int model;
        unsigned end;
        unsigned long int n_switch;
        double e_switch;
        double i_leakage;
} power_mem;

typedef struct {
        int model;
        unsigned assoc;
        unsigned long int n_mismatch;
        unsigned long int n_chg_addr;
        double e_mismatch;
        double e_chg_addr;
} power_mux;

typedef struct {
        int model;
        unsigned item_width;
        unsigned long int n_select;
        unsigned long int n_chg_data;
        unsigned long int n_out_1;
        unsigned long int n_out_0;
        double e_select;
        double e_chg_data;
        double e_out_1;
        double e_out_0;
        unsigned long int out_mask;
} power_out;

typedef struct {
        int model;
        unsigned long int n_charge;
        double e_charge;
        double i_leakage;
} power_arr_pre;


/*@
 * data type: array port state
 *
 *  - row_addr       -- input to row decoder
 *    col_addr       -- input to column decoder, if any
 *    tag_addr       -- input to tag comparator
 * $+ tag_line       -- value of tag bitline
 *  # data_line_size -- size of data_line in char
 *  # data_line      -- value of data bitline
 *
 * legend:
 *   -: only used by non-fully-associative array
 *   +: only used by fully-associative array
 *   #: only used by fully-associative array or RF array
 *   $: only used by write-through array
 *
 * NOTE:
 *   (1) *_addr may not necessarily be an address
 *   (2) data_line_size is the allocated size of data_line in simulator,
 *       which must be no less than the physical size of data line
 *   (3) each instance of module should define an instance-specific data
 *       type with non-zero-length data_line and cast it to this type
 */
typedef struct {
        unsigned long int row_addr;
        unsigned long int col_addr;
        unsigned long int tag_addr;
        unsigned long int tag_line;
        unsigned data_line_size;
        char data_line[0];
} SIM_array_port_state_t;

/*@
 * data type: array set state
 *
 *   entry           -- pointer to some entry structure if an entry is selected for
 *                      r/w, NULL otherwise
 *   entry_set       -- pointer to corresponding set structure
 * + write_flag      -- 1 if entry is already written once, 0 otherwise
 * + write_back_flag -- 1 if entry is already written back, 0 otherwise
 *   valid_bak       -- valid bit of selected entry before operation
 *   dirty_bak       -- dirty bit of selected entry, if any, before operation
 *   tag_bak         -- tag of selected entry before operation
 *   use_bak         -- use bits of all entries before operation
 *
 * legend:
 *   +: only used by fully-associative array
 *
 * NOTE:
 *   (1) entry is interpreted by modules, if some module has no "entry structure",
 *       then make sure this field is non-zero if some entry is selected
 *   (2) tag_addr may not necessarily be an address
 *   (3) each instance of module should define an instance-specific data
 *       type with non-zero-length use_bit and cast it to this type
 */
typedef struct {
        void *entry;
        void *entry_set;
        int write_flag;
        int write_back_flag;
        unsigned valid_bak;
        unsigned dirty_bak;
        unsigned long int tag_bak;
        unsigned use_bak[0];
} SIM_array_set_state_t;





// Array

typedef struct {
        power_decoder row_dec;
        power_decoder col_dec;
        power_wordline data_wordline;
        power_wordline tag_wordline;
        power_bitline data_bitline;
        power_bitline tag_bitline;
        power_mem data_mem;
        power_mem tag_mem;
        power_mem tag_attach_mem;
        power_amp data_amp;
        power_amp tag_amp;
        power_comp comp;
        power_mux mux;
        power_out outdrv;
        power_arr_pre row_dec_pre;
        power_arr_pre col_dec_pre;
        power_arr_pre data_bitline_pre;
        power_arr_pre tag_bitline_pre;
        power_arr_pre data_colsel_pre;
        power_arr_pre tag_colsel_pre;
        power_arr_pre comp_pre;
        double i_leakage;
} power_array;

typedef struct {
        //common for data and tag array
        int share_rw;
        unsigned read_ports;
        unsigned write_ports;
        unsigned n_set;
        unsigned blk_bits;
        unsigned assoc;
        int row_dec_model;
        //for data array
        unsigned data_width;
        int col_dec_model;
        int mux_model;
        int outdrv_model;
        //for tag array
        unsigned tag_addr_width;
        unsigned tag_line_width;
        int comp_model;
        //data common
        unsigned data_ndwl;
        unsigned data_ndbl;
        unsigned data_nspd;
        unsigned data_n_share_amp;
        unsigned data_end;
        int data_wordline_model;
        int data_bitline_model;
        int data_amp_model;
        int data_mem_model;
        //tag common
        unsigned tag_ndwl;
        unsigned tag_ndbl;
        unsigned tag_nspd;
        unsigned tag_n_share_amp;
        unsigned tag_end;
        unsigned tag_wordline_model;
        unsigned tag_bitline_model;
        unsigned tag_amp_model;
        unsigned tag_mem_model;
        unsigned tag_attach_mem_model;
        //precharging parameters
        int row_dec_pre_model;
        int col_dec_pre_model;
        int data_bitline_pre_model;
        int tag_bitline_pre_model;
        int data_colsel_pre_model;
        int tag_colsel_pre_model;
        int comp_pre_model;
        //derived
        unsigned n_item;
        unsigned eff_data_cols;
        unsigned eff_tag_cols;
        //flags used by prototype array model
        unsigned use_bit_width;
        unsigned valid_bit_width;
        int write_policy;
        //fields filled up during initialization
        double data_arr_width;
        double tag_arr_width;
        double data_arr_height;
        double tag_arr_height;
} power_array_info;


extern int power_array_init(power_array_info *info, power_array *arr );

extern double array_report(power_array_info *info, power_array *arr);

extern int SIM_buf_power_data_read(power_array_info *info, power_array *arr, unsigned long int data);

extern int SIM_buf_power_data_write(power_array_info *info, power_array *arr, char *data_line, char *old_data, char *new_data);

extern int SIM_array_clear_stat(power_array *arr);

extern int SIM_power_array_dec( power_array_info *info, power_array *arr, SIM_array_port_state_t *port, unsigned long int row_addr, int rw );
extern int SIM_power_array_data_read( power_array_info *info, power_array *arr, unsigned long int data );
extern int SIM_power_array_data_write( power_array_info *info, power_array *arr, SIM_array_set_state_t *set, unsigned n_item, char *data_line, char *old_data, char *new_data );
extern int power_array_tag_read( power_array_info *info, power_array *arr, SIM_array_set_state_t *set );
extern int power_array_tag_update( power_array_info *info, power_array *arr, SIM_array_port_state_t *port, SIM_array_set_state_t *set );
extern int power_array_tag_compare( power_array_info *info, power_array *arr, SIM_array_port_state_t *port, unsigned long int tag_input, unsigned long int col_addr, SIM_array_set_state_t *set );
extern int SIM_power_array_output( power_array_info *info, power_array *arr, unsigned data_size, unsigned length, void *data_out, void *data_all );

extern int SIM_array_port_state_init( power_array_info *info, SIM_array_port_state_t *port );
extern int SIM_array_set_state_init( power_array_info *info, SIM_array_set_state_t *set );

#endif




