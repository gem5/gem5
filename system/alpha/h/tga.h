/*
 * @DEC_COPYRIGHT@
 */
/*
 * HISTORY
 * $Log: tga.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:18  verghese
 * current 10/29/97
 *
 * Revision 1.1.9.2  1995/04/24  23:34:47  Jeff_Colburn
 * 	Add support for shared interrupts, ISR's now have a return value
 * 	of INTR_SERVICED or INTR_NOT_SERVICED. Changed return type from "void"
 * 	to "int" for interrupt return type in "tga_info_t".
 * 	[1995/04/24  23:24:12  Jeff_Colburn]
 *
 * Revision 1.1.7.2  1994/11/07  23:31:23  Jeff_Colburn
 * 	Added defines for putting TGA in Copy mode to support console scrolling fix.
 * 	[1994/10/26  22:32:00  Jeff_Colburn]
 *
 * Revision 1.1.5.5  1994/05/16  19:29:08  Monty_Brandenberg
 * 	Add dma_map_info_t to the info struct.
 * 	[1994/05/12  05:31:19  Monty_Brandenberg]
 *
 * Revision 1.1.5.4  1994/04/19  21:59:33  Stuart_Hollander
 * 	merge agoshw2 bl5 to gold bl10
 * 	Revision 1.1.2.4  1994/04/14  20:17:15  Monty_Brandenberg
 * 	T32-88 Framebuffer offset was incorrect causing obscure z buffering
 * 	problems in pex.
 * 	[1994/04/14  19:17:10  Monty_Brandenberg]
 *
 * Revision 1.1.5.3  1994/04/11  12:58:27  Stuart_Hollander
 * 	Revision 1.1.2.3  1994/02/28  16:53:52  Monty_Brandenberg
 * 	Removed a few magic numbers from the code and made symbolics out of
 * 	them.
 * 	[1994/02/23  21:09:10  Monty_Brandenberg]
 *
 * 	Add header files we need to define structures.  Added two new ioctls
 * 	to extract direct dma mapping info.  SFB+ and TGA ioctl structures
 * 	are now incompatible.  Fixed truecolor index and private ioctl code.
 * 	[1994/02/04  21:10:07  Monty_Brandenberg]
 *
 * 	Removed two interrupt sources as per spec.  Fixed the RAMDAC access
 * 	macros to actually work.  Imagine.
 * 	[1994/02/02  23:16:55  Monty_Brandenberg]
 *
 * 	Converted to use io_handles.  Needs work on DMA
 * 	[1994/01/07  21:14:33  Monty_Brandenberg]
 *
 * 	Added support for 24-plane option using BT463 and custom cursor chip.
 * 	Converted to fully prototyped functions.  Started conversion to
 * 	io_handle structure.
 * 	[1994/01/06  21:15:14  Monty_Brandenberg]
 *
 * Revision 1.1.5.2  1994/01/23  21:16:19  Stuart_Hollander
 * 	merge from hw2 to goldbl8
 * 	[1993/12/29  12:57:23  Stuart_Hollander]
 *
 * Revision 1.1.2.2  1993/12/21  13:42:28  Monty_Brandenberg
 * 	Initial version of the TGA device driver.
 * 	[1993/12/14  00:22:06  Monty_Brandenberg]
 *
 * $EndLog$
 */
/*
 * @(#)$RCSfile: tga.h,v $ $Revision: 1.1.1.1 $ (DEC) $Date: 1997/10/30 23:27:18 $
 */

/************************************************************************
 *									*
 *			Copyright (c) 1993 by				*
 *		Digital Equipment Corporation, Maynard, MA		*
 *			All rights reserved.				*
 *									*
 *   This software is furnished under a license and may be used and	*
 *   copied  only  in accordance with the terms of such license and	*
 *   with the  inclusion  of  the  above  copyright  notice.   This	*
 *   software  or  any  other copies thereof may not be provided or	*
 *   otherwise made available to any other person.  No title to and	*
 *   ownership of the software is hereby transferred.			*
 *									*
 *   The information in this software is subject to change  without	*
 *   notice  and should not be construed as a commitment by Digital	*
 *   Equipment Corporation.						*
 *									*
 *   Digital assumes no responsibility for the use  or  reliability	*
 *   of its software on equipment which is not supplied by Digital.	*
 *									*
 ************************************************************************/

/*
 * RAMDAC is a trademark of Brooktree Corporation
 */

#ifndef TGA_DEFINED
#define TGA_DEFINED

/*
 * Header files
 */
#if 0
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/workstation.h>
#include <sys/inputdevice.h>
#include <sys/wsdevice.h>
#include <io/common/devdriver.h>
#endif					/* 0 */

/*
 * Special offsets within the PCI configuration header
 */
#define TGA_CONFIG_PVRR_OFFSET		0x00000040
#define TGA_CONFIG_PAER_OFFSET		0x00000044

/*
 * PAER values
 */
#define TGA_CONFIG_PAER_32MB		0x00000000
#define TGA_CONFIG_PAER_64MB		0x00010000
#define TGA_CONFIG_PAER_128MB		0x00030000

/*
 * Offsets within Memory Space
 */
#define TGA_ROM_OFFSET			0x000000
#define TGA_ASIC_OFFSET			0x100000
#define TGA_RAMDAC_SETUP_OFFSET		0x1000c0
#define TGA_RAMDAC_DATA_OFFSET		0x1001f0
#define TGA_XY_REG_OFFSET		0x100074
#define TGA_VALID_REG_OFFSET		0x100070

#define TGA_0_0_FB_OFFSET		0x00200000
#define TGA_0_0_FB_SIZE			0x00200000
#define TGA_0_1_FB_OFFSET		0x00400000
#define TGA_0_1_FB_SIZE			0x00400000
#define TGA_0_3_FB_OFFSET		0x00800000
#define TGA_0_3_FB_SIZE			0x00800000
#define TGA_1_3_FB_OFFSET		0x00800000
#define TGA_1_3_FB_SIZE			0x00800000
#define TGA_1_7_FB_OFFSET		0x01000000
#define TGA_1_7_FB_SIZE			0x01000000
#define TGA_INVALID_FB_OFFSET		0
#define TGA_INVALID_FB_SIZE		0

/*
 * TGA card types
 */
#define TGA_TYPE_T801			0
#define TGA_TYPE_T802			1
#define TGA_TYPE_T822			2
#define TGA_TYPE_T844			3
#define TGA_TYPE_T3204			4
#define TGA_TYPE_T3208			5
#define TGA_TYPE_T3288			6
#define TGA_TYPE_INVALID		7
#define TGA_TYPE_NUM			8

#ifndef NDEPTHS
#define NDEPTHS 1			/* all current hardware just has one */
#define NVISUALS 1
#endif					/* NDEPTHS */

typedef unsigned char tga_pix8_t;
typedef unsigned int tga_pix32_t;
typedef unsigned int tga_reg_t;

/*
 * Window tags definitions
 */
#define TGA_TRUECOLOR_WID_INDEX		0xc
#define TGA_TRUECOLOR_WID_MASK		0xc0000000

#if 0
/*
 * Device-private ioctls
 */
#define TGA_IOCTL_PRIVATE		_IOWR('w', (0|IOC_S), tga_ioc_t)
#define TGA_IOC_LOAD_WINDOW_TAGS	0
#define TGA_IOC_ENABLE_DMA_OPS		1
#define TGA_IOC_SET_STEREO_MODE		2
#define TGA_IOC_GET_STEREO_MODE		3
#define TGA_IOC_GET_DIRECT_DMA_COUNT	4
#define TGA_IOC_GET_DIRECT_DMA_INFO	5

typedef struct {
    char windex;
    unsigned char low;
    unsigned char mid;
    unsigned char high;
} tga_window_tag_cell_t;

typedef struct {
    short ncells;
    short start;
    tga_window_tag_cell_t *p_cells;
} tga_ioc_window_tag_t;

typedef struct {
    vm_offset_t phys_base;
    vm_offset_t bus_base;
    vm_size_t map_size;
} tga_dma_map_t;

typedef struct {
    int alloc_map_num;			/* input */
    int actual_map_num;			/* output */
    tga_dma_map_t *maps;		/* input & output */
} tga_ioc_dma_info_t;

typedef struct {
    short screen;
    short cmd;
    union {
        tga_ioc_window_tag_t window_tag;
        unsigned int stereo_mode;
#define TGA_IOC_STEREO_NONE		0
#define TGA_IOC_STEREO_24		1

int direct_dma_count;
        tga_ioc_dma_info_t direct_dma_info;
    } data;
} tga_ioc_t;
#endif					/* 0 */

typedef struct {
    unsigned deep : 1;
    unsigned mbz0 : 1;
    unsigned mask : 3;
    unsigned block : 4;
    unsigned col_size : 1;
    unsigned sam_size : 1;
    unsigned parity : 1;
    unsigned write_en : 1;
    unsigned ready : 1;
    unsigned slow_dac : 1;
    unsigned dma_size : 1;
    unsigned sync_type : 1;
    unsigned mbz1 : 15;
} tga_deep_reg_t;

#define TGA_DEEP_DEEP_8PLANE		0
#define TGA_DEEP_DEEP_32PLANE		1

#define TGA_DEEP_MASK_4MB		0x00
#define TGA_DEEP_MASK_8MB		0x01
#define TGA_DEEP_MASK_16MB		0x03
#define TGA_DEEP_MASK_32MB		0x07

#define TGA_DEEP_PARITY_ODD		0
#define TGA_DEEP_PARITY_EVEN		1

#define TGA_DEEP_READY_ON_8		0
#define TGA_DEEP_READY_ON_2		1

#define TGA_DEEP_DMA_64			0
#define TGA_DEEP_DMA_128		1

typedef struct {
    unsigned s_wr_mask : 8;
    unsigned s_rd_mask : 8;
    unsigned s_test : 3;
    unsigned s_fail : 3;
    unsigned d_fail : 3;
    unsigned d_pass : 3;
    unsigned z_test : 3;
    unsigned z : 1;
} tga_stencil_mode_reg_t;

#define TGA_SM_TEST_GEQ			0x00
#define TGA_SM_TEST_TRUE		0x01
#define TGA_SM_TEST_FALSE		0x02
#define TGA_SM_TEST_LS			0x03
#define TGA_SM_TEST_EQ			0x04
#define TGA_SM_TEST_LEQ			0x05
#define TGA_SM_TEST_GT			0x06
#define TGA_SM_TEST_NEQ			0x07

#define TGA_SM_RESULT_KEEP		0x00
#define TGA_SM_RESULT_ZERO		0x01
#define TGA_SM_RESULT_REPLACE		0x02
#define TGA_SM_RESULT_INCR		0x03
#define TGA_SM_RESULT_DECR		0x04
#define TGA_SM_RESULT_INV		0x05

#define TGA_SM_Z_REPLACE		0
#define TGA_SM_Z_KEEP			1

typedef struct {
    unsigned mode : 8;
    unsigned visual : 3;
    unsigned rotate : 2;
    unsigned line : 1;
    unsigned z16 : 1;
    unsigned cap_ends : 1;
    unsigned mbz : 16;
} tga_mode_reg_t;

#define TGA_MODE_MODE_SIMPLE			0x00
#define TGA_MODE_MODE_Z_SIMPLE			0x10
#define TGA_MODE_MODE_OPA_STIP			0x01
#define TGA_MODE_MODE_OPA_FILL			0x21
#define TGA_MODE_MODE_TRA_STIP			0x05
#define TGA_MODE_MODE_TRA_FILL			0x25
#define TGA_MODE_MODE_TRA_BLK_STIP		0x0d
#define TGA_MODE_MODE_TRA_BLK_FILL		0x2d
#define TGA_MODE_MODE_OPA_LINE			0x02
#define TGA_MODE_MODE_TRA_LINE			0x06
#define TGA_MODE_MODE_CINT_TRA_LINE		0x0e
#define TGA_MODE_MODE_CINT_TRA_DITH_LINE	0x2e
#define TGA_MODE_MODE_Z_OPA_LINE		0x12
#define TGA_MODE_MODE_Z_TRA_LINE		0x16
#define TGA_MODE_MODE_Z_CINT_OPA_LINE		0x1a
#define TGA_MODE_MODE_Z_SINT_OPA		0x5a
#define TGA_MODE_MODE_Z_CINT_OPA_DITH_LINE	0x3a
#define TGA_MODE_MODE_Z_CINT_TRA_LINE		0x1e
#define TGA_MODE_MODE_Z_SINT_TRA		0x5e
#define TGA_MODE_MODE_Z_CINT_TRA_DITH_LINE	0x3e
#define TGA_MODE_MODE_COPY			0x07
#define TGA_MODE_MODE_COPY24                    0x307
#define TGA_MODE_MODE_DMA_READ			0x17
#define TGA_MODE_MODE_DMA_READ_DITH		0x37
#define TGA_MODE_MODE_DMA_WRITE			0x1f

#define TGA_MODE_VISUAL_8_PACKED		0x00
#define TGA_MODE_VISUAL_8_UNPACKED		0x01
#define TGA_MODE_VISUAL_12_LOW			0x02
#define TGA_MODE_VISUAL_12_HIGH			0x06
#define TGA_MODE_VISUAL_24			0x03

#define TGA_MODE_PM_IS_PERS			0x00800000
#define TGA_MODE_ADDR_IS_NEW			0x00400000
#define TGA_MODE_BRES3_IS_NEW			0x00200000
#define TGA_MODE_COPY_WILL_DRAIN		0x00100000

typedef struct {
    unsigned opcode : 4;
    unsigned mbz : 4;
    unsigned visual : 2;
    unsigned rotate : 2;
} tga_raster_op_t;

#define TGA_ROP_OP_CLEAR			0
#define TGA_ROP_OP_AND				1
#define TGA_ROP_OP_AND_REVERSE			2
#define TGA_ROP_OP_COPY				3
#define TGA_ROP_OP_COPY24                       0x303
#define TGA_ROP_OP_AND_INVERTED			4
#define TGA_ROP_OP_NOOP				5
#define TGA_ROP_OP_XOR				6
#define TGA_ROP_OP_OR				7
#define TGA_ROP_OP_NOR				8
#define TGA_ROP_OP_EQUIV			9
#define TGA_ROP_OP_INVERT			10
#define TGA_ROP_OP_OR_REVERSE			11
#define TGA_ROP_OP_COPY_INVERTED		12
#define TGA_ROP_OP_OR_INVERTED			13
#define TGA_ROP_OP_NAND				14
#define TGA_ROP_OP_SET				15

#define TGA_ROP_VISUAL_8_PACKED			0x00
#define TGA_ROP_VISUAL_8_UNPACKED		0x01
#define TGA_ROP_VISUAL_12			0x02
#define TGA_ROP_VISUAL_24			0x03

#define TGA_INTR_VSYNC				0x00000001
#define TGA_INTR_SHIFT_ADDR			0x00000002
#define TGA_INTR_TIMER				0x00000010
#define TGA_INTR_ALL				0x00000013
#define TGA_INTR_ENABLE_SHIFT			16

#define TGA_RAMDAC_INTERF_WRITE_SHIFT		0
#define TGA_RAMDAC_INTERF_READ0_SHIFT		16
#define TGA_RAMDAC_INTERF_READ1_SHIFT		24

#define TGA_RAMDAC_485_READ			0x01
#define TGA_RAMDAC_485_WRITE			0x00

#define TGA_RAMDAC_485_ADDR_PAL_WRITE		0x00
#define TGA_RAMDAC_485_DATA_PAL			0x02
#define TGA_RAMDAC_485_PIXEL_MASK		0x04
#define TGA_RAMDAC_485_ADDR_PAL_READ		0x06
#define TGA_RAMDAC_485_ADDR_CUR_WRITE		0x08
#define TGA_RAMDAC_485_DATA_CUR			0x0a
#define TGA_RAMDAC_485_CMD_0			0x0c
#define TGA_RAMDAC_485_ADDR_CUR_READ		0x0e
#define TGA_RAMDAC_485_CMD_1			0x10
#define TGA_RAMDAC_485_CMD_2			0x12
#define TGA_RAMDAC_485_STATUS			0x14
#define TGA_RAMDAC_485_CMD_3			0x14
#define TGA_RAMDAC_485_CUR_RAM			0x16
#define TGA_RAMDAC_485_CUR_LOW_X		0x18
#define TGA_RAMDAC_485_CUR_HIGH_X		0x1a
#define TGA_RAMDAC_485_CUR_LOW_Y		0x1c
#define TGA_RAMDAC_485_CUR_HIGH_Y		0x1e

#define TGA_RAMDAC_485_ADDR_EPSR_SHIFT		0
#define TGA_RAMDAC_485_ADDR_EPDR_SHIFT		8

#define TGA_RAMDAC_463_HEAD_MASK		0x01
#define TGA_RAMDAC_463_READ			0x02
#define TGA_RAMDAC_463_WRITE			0x00
#define TGA_RAMDAC_463_ADDR_LOW			0x00
#define TGA_RAMDAC_463_ADDR_HIGH		0x04
#define TGA_RAMDAC_463_CMD_CURS			0x08
#define TGA_RAMDAC_463_CMD_CMAP			0x0c

#define TGA_RAMDAC_463_ADDR_EPSR_SHIFT		0
#define TGA_RAMDAC_463_ADDR_EPDR_SHIFT		8

#define TGA_RAMDAC_463_CURSOR_COLOR0		0x0100
#define TGA_RAMDAC_463_CURSOR_COLOR1		0x0101
#define TGA_RAMDAC_463_COMMAND_REG_0		0x0201
#define TGA_RAMDAC_463_COMMAND_REG_1		0x0202
#define TGA_RAMDAC_463_COMMAND_REG_2		0x0203
#define TGA_RAMDAC_463_READ_MASK		0x0205
#define TGA_RAMDAC_463_BLINK_MASK		0x0209
#define TGA_RAMDAC_463_WINDOW_TYPE_TABLE	0x0300

typedef union {
    struct {
        unsigned int pixels : 9;
        unsigned int front_porch : 5;
        unsigned int sync : 7;
        unsigned int back_porch : 7;
        unsigned int ignore : 3;
        unsigned int odd : 1;
    } horizontal_setup;
    unsigned int h_setup;
} tga_horizontal_setup_t;

typedef union {
    struct {
        unsigned int scan_lines : 11;
        unsigned int front_porch : 5;
        unsigned int sync : 6;
        unsigned int back_porch : 6;
    } vertical_setup;
    unsigned int v_setup;
} tga_vertical_setup_t;

typedef volatile struct {
  tga_reg_t buffer[8];

tga_reg_t foreground;
tga_reg_t background;
tga_reg_t planemask;
tga_reg_t pixelmask;
tga_reg_t mode;
tga_reg_t rop;
tga_reg_t shift;
tga_reg_t address;

tga_reg_t bres1;
tga_reg_t bres2;
tga_reg_t bres3;
tga_reg_t brescont;
tga_reg_t deep;
tga_reg_t start;
tga_reg_t stencil_mode;
tga_reg_t pers_pixelmask;

tga_reg_t cursor_base_address;
tga_reg_t horizontal_setup;
tga_reg_t vertical_setup;

#define TGA_VERT_STEREO_EN		0x80000000
tga_reg_t base_address;
tga_reg_t video_valid;

#define TGA_VIDEO_VALID_SCANNING	0x00000001
#define TGA_VIDEO_VALID_BLANK		0x00000002
#define TGA_VIDEO_VALID_CURSOR_ENABLE	0x00000004
tga_reg_t cursor_xy;
tga_reg_t video_shift_addr;
tga_reg_t intr_status;

tga_reg_t pixel_data;
tga_reg_t red_incr;
tga_reg_t green_incr;
tga_reg_t blue_incr;
tga_reg_t z_incr_low;
tga_reg_t z_incr_high;
tga_reg_t dma_address;
tga_reg_t bres_width;

tga_reg_t z_value_low;
tga_reg_t z_value_high;
tga_reg_t z_base_address;
tga_reg_t address2;
tga_reg_t red_value;
tga_reg_t green_value;
tga_reg_t blue_value;
tga_reg_t _jnk12;

tga_reg_t ramdac_setup;
struct {
    tga_reg_t junk;
} _junk[8 * 2 - 1];

struct {
    tga_reg_t data;
} slope_no_go[8];

struct {
    tga_reg_t data;
} slope[8];

tga_reg_t bm_color_0;
tga_reg_t bm_color_1;
tga_reg_t bm_color_2;
tga_reg_t bm_color_3;
tga_reg_t bm_color_4;
tga_reg_t bm_color_5;
tga_reg_t bm_color_6;
tga_reg_t bm_color_7;

tga_reg_t c64_src;
tga_reg_t c64_dst;
tga_reg_t c64_src2;
tga_reg_t c64_dst2;
tga_reg_t _jnk45;
tga_reg_t _jnk46;
tga_reg_t _jnk47;
tga_reg_t _jnk48;

struct {
    tga_reg_t junk;
} _junk2[8 * 3];

tga_reg_t eprom_write;
tga_reg_t _res0;
tga_reg_t clock;
tga_reg_t _res1;
tga_reg_t ramdac;
tga_reg_t _res2;
tga_reg_t command_status;
tga_reg_t command_status2;

}
 tga_rec_t, *tga_ptr_t;

#if 0
typedef struct {
    ws_screen_descriptor screen;	/* MUST be first!!! */
    ws_depth_descriptor depth[NDEPTHS];
    ws_visual_descriptor visual[NVISUALS];
    ws_cursor_functions cf;
    ws_color_map_functions cmf;
    ws_screen_functions sf;
    int (*attach)();
    int (*bootmsg)();
    int (*map)();
    int (*interrupt)();
    int (*setup)();
    vm_offset_t base;
    tga_ptr_t asic;
    vm_offset_t fb;
    size_t fb_size;
    unsigned int bt485_present;
    unsigned int bits_per_pixel;
    unsigned int core_size;
    unsigned int paer_value;
    tga_reg_t deep;
    tga_reg_t head_mask;
    tga_reg_t refresh_count;
    tga_reg_t horizontal_setup;
    tga_reg_t vertical_setup;
    tga_reg_t base_address;
    caddr_t info_area;
    vm_offset_t virtual_dma_buffer;
    vm_offset_t physical_dma_buffer;
    int wt_min_dirty;
    int wt_max_dirty;
    int wt_dirty;
    tga_window_tag_cell_t wt_cell[16];	/* magic number */
    unsigned int stereo_mode;
    io_handle_t io_handle;
    dma_map_info_t p_map_info;
} tga_info_t;

#define TGA_USER_MAPPING_COUNT		4

typedef struct {
    vm_offset_t fb_alias_increment;
    vm_offset_t option_base;
    unsigned int planemask;
    vm_offset_t virtual_dma_buffer;
    vm_offset_t physical_dma_buffer;
} tga_server_info_t;
#endif					/* 0 */

typedef struct {
    unsigned char dirty_cell;
    unsigned char red;			/* only need 8 bits */
    unsigned char green;
    unsigned char blue;
} tga_bt485_color_cell_t;

typedef struct {
    volatile unsigned int *setup;
    volatile unsigned int *data;
    unsigned int head_mask;
    short fb_xoffset;
    short fb_yoffset;
    short min_dirty;
    short max_dirty;
    caddr_t reset;
    u_int mask;
} tga_bt485_type_t;

#if 0
typedef struct {
    volatile unsigned int *setup;
    volatile unsigned int *data;
    unsigned int head_mask;
    short fb_xoffset;
    short fb_yoffset;
    short min_dirty;
    short max_dirty;
    caddr_t reset;
    u_int mask;

    /*************************************************************** fields
     * above this line MUST match struct bt485type
     * exactly!***************************************************************/
    u_int unit;
    char screen_on;
    char on_off;
    char dirty_cursor;
    char dirty_colormap;
    short x_hot;
    short y_hot;
    ws_color_cell cursor_fg;
    ws_color_cell cursor_bg;
    void (*enable_interrupt)();
    u_long bits[256];
    tga_bt485_color_cell_t cells[256];
} tga_bt485_info_t;
#endif					/* 0 */

#define TGA_RAMDAC_463_WINDOW_TAG_COUNT	16
#define TGA_RAMDAC_463_CMAP_ENTRY_COUNT	528

typedef struct {
    unsigned char dirty_cell;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
} tga_bt463_color_cell_t;

typedef struct {
    unsigned char low_byte;
    unsigned char middle_byte;
    unsigned char high_byte;
    unsigned char unused;
} tga_bt463_wid_cell_t;

typedef struct {
    volatile unsigned int *setup;
    volatile unsigned int *data;
    unsigned int head_mask;
    short fb_xoffset;
    short fb_yoffset;
} tga_bt463_type_t;

#if 0
typedef struct {
    volatile unsigned int *setup;
    volatile unsigned int *data;
    unsigned int head_mask;
    short fb_xoffset;
    short fb_yoffset;
    char type;
    char screen_on;
    char dirty_colormap;
    char dirty_cursormap;
    int unit;
    void (*enable_interrupt)();
    caddr_t cursor_closure;
    ws_color_cell cursor_fg;
    ws_color_cell cursor_bg;
    short min_dirty;
    short max_dirty;
    tga_bt463_color_cell_t cells[TGA_RAMDAC_463_CMAP_ENTRY_COUNT];
} tga_bt463_info_t;
#endif					/* 0 */

typedef struct {
    volatile unsigned int *xy_reg;
    volatile unsigned int *valid;
    short fb_xoffset;
    short fb_yoffset;
} tga_curs_type_t;

#if 0
typedef struct {
    volatile unsigned int *xy_reg;
    volatile unsigned int *valid;
    short fb_xoffset;
    short fb_yoffset;
    u_int unit;
    char on_off;
    char dirty_cursor;
    char dirty_cursormap;
    short x_hot;
    short y_hot;
    short last_row;
    ws_color_cell cursor_fg;
    ws_color_cell cursor_bg;
    void (*enable_interrupt)();
    unsigned int bits[256];
} tga_curs_info_t;
#endif					/* 0 */

#endif					/* TGA_DEFINED */

