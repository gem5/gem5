
#ifndef __TSUNAMIREG_H__
#define __TSUNAMIREG_H__

#define ALPHA_K0SEG_BASE  0xfffffc0000000000

// CChip Registers
#define TSDEV_CC_CSR    0x00
#define TSDEV_CC_MTR    0x01
#define TSDEV_CC_MISC   0x02

#define TSDEV_CC_AAR0   0x04
#define TSDEV_CC_AAR1   0x05
#define TSDEV_CC_AAR2   0x06
#define TSDEV_CC_AAR3   0x07
#define TSDEV_CC_DIM0   0x08
#define TSDEV_CC_DIM1   0x09
#define TSDEV_CC_DIR0   0x0A
#define TSDEV_CC_DIR1   0x0B
#define TSDEV_CC_DRIR   0x0C
#define TSDEV_CC_PRBEN  0x0D
#define TSDEV_CC_IIC0   0x0E
#define TSDEV_CC_IIC1   0x0F
#define TSDEV_CC_MPR0   0x10
#define TSDEV_CC_MPR1   0x11
#define TSDEV_CC_MPR2   0x12
#define TSDEV_CC_MPR3   0x13

#define TSDEV_CC_DIM2   0x18
#define TSDEV_CC_DIM3   0x19
#define TSDEV_CC_DIR2   0x1A
#define TSDEV_CC_DIR3   0x1B
#define TSDEV_CC_IIC2   0x1C
#define TSDEV_CC_IIC3   0x1D


// PChip Registers
#define TSDEV_PC_WSBA0      0x00
#define TSDEV_PC_WSBA1      0x01
#define TSDEV_PC_WSBA2      0x02
#define TSDEV_PC_WSBA3      0x03
#define TSDEV_PC_WSM0       0x04
#define TSDEV_PC_WSM1       0x05
#define TSDEV_PC_WSM2       0x06
#define TSDEV_PC_WSM3       0x07
#define TSDEV_PC_TBA0       0x08
#define TSDEV_PC_TBA1       0x09
#define TSDEV_PC_TBA2       0x0A
#define TSDEV_PC_TBA3       0x0B
#define TSDEV_PC_PCTL       0x0C
#define TSDEV_PC_PLAT       0x0D
#define TSDEV_PC_RES        0x0E
#define TSDEV_PC_PERROR     0x0F
#define TSDEV_PC_PERRMASK   0x10
#define TSDEV_PC_PERRSET    0x11
#define TSDEV_PC_TLBIV      0x12
#define TSDEV_PC_TLBIA      0x13
#define TSDEV_PC_PMONCTL    0x14
#define TSDEV_PC_PMONCNT    0x15

#define TSDEV_PC_SPST       0x20


// DChip Registers
#define TSDEV_DC_DSC        0x20
#define TSDEV_DC_STR        0x21
#define TSDEV_DC_DREV       0x22
#define TSDEV_DC_DSC2       0x23

// I/O Ports
#define TSDEV_PIC1_MASK     0x21
#define TSDEV_PIC2_MASK     0xA1
#define TSDEV_DMA1_RESET    0x0D
#define TSDEV_DMA2_RESET    0xDA
#define TSDEV_DMA1_MODE     0x0B
#define TSDEV_DMA2_MODE     0xD6
#define TSDEV_DMA1_MASK     0x0A
#define TSDEV_DMA2_MASK     0xD4
#define TSDEV_TMR_CTL       0x61
#define TSDEV_TMR2_CTL      0x43
#define TSDEV_TMR2_DATA     0x42
#define TSDEV_TMR0_DATA     0x40

#define TSDEV_RTC_ADDR      0x70
#define TSDEV_RTC_DATA      0x71

// RTC defines
#define RTC_SECOND          0	// second of minute [0..59]
#define RTC_SECOND_ALARM    1	// seconds to alarm
#define RTC_MINUTE          2	// minute of hour [0..59]
#define RTC_MINUTE_ALARM    3	// minutes to alarm
#define RTC_HOUR            4	// hour of day [0..23]
#define RTC_HOUR_ALARM      5	// hours to alarm
#define RTC_DAY_OF_WEEK     6	// day of week [1..7]
#define RTC_DAY_OF_MONTH    7	// day of month [1..31]
#define RTC_MONTH           8	// month of year [1..12]
#define RTC_YEAR            9	// year [00..99]
#define RTC_CONTROL_REGISTERA   10	// control register A
#define RTC_CONTROL_REGISTERB   11	// control register B
#define RTC_CONTROL_REGISTERC   12	// control register C
#define RTC_CONTROL_REGISTERD   13	// control register D
#define RTC_REGNUMBER_RTC_CR1   0x6A	// control register 1

#define PCHIP_PCI0_MEMORY       0x10000000000
#define PCHIP_PCI0_IO           0x101FC000000
#define TSUNAMI_PCI0_MEMORY     ALPHA_K0SEG_BASE + PCHIP_PCI0_MEMORY
#define TSUNAMI_PCI0_IO         ALPHA_K0SEG_BASE + PCHIP_PCI0_IO


#endif // __TSUNAMIREG_H__
