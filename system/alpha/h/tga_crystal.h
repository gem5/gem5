#define OPTION_130  0
#define OPTION_119  1
#define OPTION_108  2
#define OPTION_104  3
#define OPTION_92   4
#define OPTION_75   5
#define OPTION_74   6
#define OPTION_69   7
#define OPTION_65   8
#define OPTION_50   9
#define OPTION_40  10
#define OPTION_32  11
#define OPTION_25  12
#define OPTION_135 13
#define OPTION_110 14

#define PIXEL_BIAS 0

struct isc_data {
  unsigned char data[56];
};


typedef struct isc_data ISC;

struct monitor_data {
  int option;
  int monitor_rows;
  int monitor_columns;
  int sold_freq;
  int refresh_rate;
  int v_scanlines;
  int v_front_porch;
  int v_sync;
  int v_back_porch;
  int d_pixels;
  int h_pixels;
  int h_front_porch;
  int h_sync;
  int h_back_porch;
  int cursor_x;
  int cursor_y;
  ISC isc_data;
};

static struct monitor_data crystal_table[] =
{
{
                                                /* 130.808 Mhz                  */
OPTION_130,                                     /* Option number 1              */
1024,                                           /* rows                         */
1280,                                           /* columns                      */
130,                                            /* 130.8 Mhz                    */
72,                                             /* refresh rate                 */
1024,                                           /* v scanlines                  */
3,                                              /* v front porch                */
3,                                              /* v sync                       */
33,                                             /* v back porch                 */
1280,                                           /* display pixels               */
1280,                                           /* h pixels                     */
32,                                             /* h front porch                */
160,                                            /* h sync                       */
224,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
1,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
0,0,0,0,0,1,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
1,1,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,0,1,0,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 119.84 Mhz                   */
OPTION_119,                                     /* Option number 2              */
1024,                                           /* rows                         */
1280,                                           /* columns                      */
119,                                            /* 119 Mhz                      */
66,                                             /* refresh rate                 */
1024,                                           /* v scanlines                  */
3,                                              /* v front porch                */
3,                                              /* v sync                       */
33,                                             /* v back porch                 */
1280,                                           /* display pixels               */
1280,                                           /* h pixels                     */
32,                                             /* h front porch                */
160,                                            /* h sync                       */
224,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,0,1,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
1,1,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
0,1,0,0,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 108.18 Mhz                   */
OPTION_108,
1024,                                           /* rows                         */
1280,                                           /* columns                      */
108,                                            /* 108 Mhz                      */
60,                                             /* refresh rate                 */
1024,                                           /* v scanlines                  */
3,                                              /* v front porch                */
3,                                              /* v sync                       */
26,                                             /* v back porch                 */
1280,                                           /* display pixels               */
1280,                                           /* h pixels                     */
44,                                             /* h front porch                */
184,                                            /* h sync                       */
200,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */
                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,1,0,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,0,1,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,0,0,0,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 104.00 Mhz                   */
OPTION_104,
900,                                            /* rows                         */
1152,                                           /* columns                      */
104,                                            /* 104 Mhz                      */
72,                                             /* refresh rate                 */
900,                                            /* v scanlines                  */
6,                                              /* v front porch                */
10,                                             /* v sync                       */
44,                                             /* v back porch                 */
1152,                                           /* display pixels               */
1152,                                           /* h pixels                     */
64,                                             /* h front porch                */
112,                                            /* h sync                       */
176,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */
                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,1,0,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,1,1,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
0,1,0,0,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 92.98 Mhz                    */
OPTION_92,
900,                                            /* rows                         */
1152,                                           /* columns                      */
92,                                             /* 92.98 Mhz                    */
66,                                             /* refresh rate                 */
900,                                            /* v scanlines                  */
2,                                              /* v front porch                */
4,                                              /* v sync                       */
31,                                             /* v back porch                 */
1152,                                           /* display pixels               */
1152,                                           /* h pixels                     */
20,                                             /* h front porch                */
132,                                            /* h sync                       */
200,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
0,0,1,1,0,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,0,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,0,1,1,0,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 75.00 Mhz                    */
OPTION_75,
768,                                            /* rows                         */
1024,                                           /* columns                      */
75,                                             /* 74 Mhz                       */
70,                                             /* refresh rate                 */
768,                                            /* v scanlines                  */
3,                                              /* v front porch                */
6,                                              /* v sync                       */
29,                                             /* v back porch                 */
1024,                                           /* display pixels               */
1024,                                           /* h pixels                     */
24,                                             /* h front porch                */
136,                                            /* h sync                       */
144,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,0,0,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,1,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
0,0,1,0,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 74.37 Mhz                    */
OPTION_74,
768,                                            /* rows                         */
1024,                                           /* columns                      */
74,                                             /* 74 Mhz                       */
72,                                             /* refresh rate                 */
768,                                            /* v scanlines                  */
1,                                              /* v front porch                */
6,                                              /* v sync                       */
22,                                             /* v back porch                 */
1024,                                           /* display pixels               */
1024,                                           /* h pixels                     */
16,                                             /* h front porch                */
128,                                            /* h sync                       */
128,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
0,1,1,1,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
1,0,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,1,0,0,0,1,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 69 Mhz DEC 72 Hz             */
OPTION_69,                                      /* Option number 3              */
864,                                            /* rows                         */
1024+PIXEL_BIAS,                                /* columns                      */
69,                                             /* 69.x Mhz                     */
60,                                             /* refresh rate                 */
864,                                            /* v scanlines                  */
0,                                              /* v front porch                */
3,                                              /* v sync                       */
34,                                             /* v back porch                 */
1024,                                           /* display pixels               */
1024+PIXEL_BIAS,                                /* h pixels                     */
12,                                             /* h front porch                */
116,                                            /* h sync                       */
128,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

{
1,0,0,                                          /* 0:1    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,1,1,0,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
1,1,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,0,0,0,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 65 Mhz                       */
OPTION_65,
768,                                            /* rows                         */
1024,                                           /* columns                      */
65,                                             /* 65 Mhz                       */
60,                                             /* refresh rate                 */
768,                                            /* v scanlines                  */
7,                                              /* v front porch                */
9,                                              /* v sync                       */
26,                                             /* v back porch                 */
1024,                                           /* display pixels               */
1024,                                           /* h pixels                     */
56,                                             /* h front porch                */
64,                                             /* h sync                       */
200,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
0,1,0,0,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,0,1,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,0,0,1,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 50 Mhz ergo SVGA             */
OPTION_50,
600,                                            /* rows                         */
800,                                            /* columns                      */
50,                                             /* 50 Mhz                       */
72,                                             /* refresh rate                 */
600,                                            /* v scanlines                  */
31,                                             /* v front porch                */
6,                                              /* v sync                       */
29,                                             /* v back porch                 */
800 ,                                           /* display pixels               */
800,                                            /* h pixels                     */
56,                                             /* h front porch                */
120,                                            /* h sync                       */
64,                                             /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */
                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,0,0,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
1,0,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
0,1,1,1,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 40  Mhz SVGA                 */
OPTION_40,
600,                                            /* rows                         */
800,                                            /* columns                      */
40,                                             /* 36 Mhz                       */
60,                                             /* refresh rate                 */
600,                                            /* v scanlines                  */
1,                                              /* v front porch                */
4,                                              /* v sync                       */
23,                                             /* v back porch                 */
800,                                            /* display pixels               */
800,                                            /* h pixels                     */
40,                                             /* h front porch                */
128,                                            /* h sync                       */
88,                                             /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */
                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
0,1,1,1,0,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
1,0,1,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,0,0,0,0,1,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 31.5 Mhz ergo VGA            */
OPTION_32,
480,                                            /* rows                         */
640,                                            /* columns                      */
32,                                             /* 32 Mhz                       */
72,                                             /* refresh rate                 */
480,                                            /* v scanlines                  */
9,                                              /* v front porch                */
3,                                              /* v sync                       */
28,                                             /* v back porch                 */
640,                                            /* display pixels               */
640,                                            /* h pixels                     */
24,                                             /* h front porch                */
40,                                             /* h sync                       */
128,                                            /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
1,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,1,1,0,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,0,1,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,1,0,0,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                               /* 25.175 Mhz VGA                      */
OPTION_25,                                      /* Option Information           */
480,                                            /* rows                         */
640,                                            /* columns                      */
25,                                             /* 25.175 Mhz                   */
60,                                             /* refresh rate                 */
480,                                            /* v scanlines                  */
10,                                             /* v front porch                */
2,                                              /* v sync                       */
33,                                             /* v back porch                 */
640,                                            /* display pixels               */
640,                                            /* h pixels                     */
16,                                             /* h front porch                */
96,                                             /* h sync                       */
48,                                             /* h back porch                 */
0,                                              /* cursor x placeholder         */
0,                                              /* cursor y placeholder         */
                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
1,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,0,0,0,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
1,0,0,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
0,1,1,1,1,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
 },
{                                                /* 135 Mhz                  */
OPTION_135,                        /* Option number D              */
1024,                              /* rows                         */
1280,                              /* columns                      */
135,                               /* 135 Mhz                      */
75,                                /* refresh rate                 */
1024,                              /* v scanlines                  */
1,                                 /* v front porch                */
3,                                 /* v sync                       */
38,                                /* v back porch                 */
1280,                              /* display pixels               */
1280,                              /* h pixels                     */
16,                                /* h front porch                */
144,                               /* h sync                       */
248,                               /* h back porch                 */
0,                                 /* cursor x placeholder         */
0,                                 /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
1,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
0,0,1,0,1,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,1,1,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
1,0,1,1,0,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
},
{                                                /* 110 Mhz                  */
OPTION_110,                        /* Option number E              */
1024,                              /* rows                         */
1280,                              /* columns                      */
110,                               /* 110 Mhz                      */
60,                                /* refresh rate                 */
1024,                              /* v scanlines                  */
6,                                 /* v front porch                */
7,                                 /* v sync                       */
44,                                /* v back porch                 */
1280,                              /* display pixels               */
1280,                              /* h pixels                     */
19,                                /* h front porch                */
163,                               /* h sync                       */
234,                               /* h back porch                 */
0,                                 /* cursor x placeholder         */
0,                                 /* cursor y placeholder         */

                                                /* ISC serial load information         */

{
1,0,0,                                          /* 0:2    N1 modulus                   */
0,0,                                            /* 3:4    set to zero                  */
0,                                              /*   5    tristates PLL                */
0,                                              /*   6    toggle for ramdac reset      */
0,                                              /*   7 1  to pass reference frequency  */
0,                                              /*   8 1  for N1 and N2 dividers       */
0,                                              /*   9    VRAM shift clock enable      */
0,                                              /*  10    External PLL feedback        */
0,                                              /*  11    phase detect reset           */
0,0,                                            /* 12:13  PLL post scaler bits         */
0,                                              /* 14     aux clock differential       */
0,                                              /* 15     auxen clock mode             */
1,0,0,0,0,0,0,0,                                /* 16:23  N2 modulus                   */
0,0,1,                                          /* 24:26  sets the gain of VCO         */
0,                                              /* 27     bit 28 for N2                */
0,1,                                            /* 28:29  sets gain of phase detector  */
0,                                              /* 30     reserved                     */
0,                                              /* 31     phase detector timing        */
1,1,1,1,0,0,                                    /* 32:37  M counter bits               */
0,                                              /* 38     reserved                     */
0,                                              /* 39     doubles the modulus prescale */
0,0,1,0,                                        /* 40:43  A counter                    */
0,0,0,0,                                        /* 44:47  reserved                     */
0,0,1,1,0,0,0,                                  /* 48:54  Reference divider            */
0,                                              /* 55     reserved                     */
}
}
};
