// *************************************************************************************************
//
//	Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
//
//
//	  Redistribution and use in source and binary forms, with or without
//	  modification, are permitted provided that the following conditions
//	  are met:
//
//	    Redistributions of source code must retain the above copyright
//	    notice, this list of conditions and the following disclaimer.
//
//	    Redistributions in binary form must reproduce the above copyright
//	    notice, this list of conditions and the following disclaimer in the
//	    documentation and/or other materials provided with the
//	    distribution.
//
//	    Neither the name of Texas Instruments Incorporated nor the names of
//	    its contributors may be used to endorse or promote products derived
//	    from this software without specific prior written permission.
//
//	  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//	  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//	  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//	  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//	  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//	  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//	  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************

#ifndef __DISPLAY_H
#define __DISPLAY_H


// *************************************************************************************************
// Include section

#include "project_defs.h"

#ifdef RAM_BASED_RFBSL

// *************************************************************************************************
// Extern section

// Constants defined in library
extern const u8 lcd_font[];
extern const u8 * segments_lcdmem[];
extern const u8 segments_bitmask[];
extern const u8 itoa_conversion_table[][2];


// *************************************************************************************************
// Global Variable section

// Definitions for line access
#    define LINE1                   (1u)
#    define LINE2                   (2u)

// LCD display modes
#    define SEG_OFF                 (0u)
#    define SEG_ON                  (1u)
#    define SEG_ON_BLINK_ON         (2u)
#    define SEG_ON_BLINK_OFF        (3u)
#    define SEG_OFF_BLINK_OFF       (4u)

// 7-segment character bit assignments
#    define SEG_A                   (BIT4)
#    define SEG_B                   (BIT5)
#    define SEG_C                   (BIT6)
#    define SEG_D                   (BIT7)
#    define SEG_E                   (BIT2)
#    define SEG_F                   (BIT0)
#    define SEG_G                   (BIT1)

// ------------------------------------------
// LCD symbols for easier access
//
// xxx_SEG_xxx          = Seven-segment character (sequence 5-4-3-2-1-0)
// xxx_SYMB_xxx         = Display symbol, e.g. "AM" for ante meridiem
// xxx_UNIT_xxx         = Display unit, e.g. "km/h" for kilometers per hour
// xxx_ICON_xxx         = Display icon, e.g. heart to indicate reception of heart rate data
// xxx_L1_xxx           = Item is part of Line1 information
// xxx_L2_xxx           = Item is part of Line2 information

// Symbols for Line1
#    define LCD_SYMB_PERCENT                        0

#    define LCD_ICON_BEEPER1                        1
#    define LCD_ICON_BEEPER2                        2
#    define LCD_ICON_BEEPER3                        3

// Line1 7-segments
#    define LCD_SEG_L1_3                            4
#    define LCD_SEG_L1_2                            5
#    define LCD_SEG_L1_1                            6
#    define LCD_SEG_L1_0                            7
#    define LCD_SEG_L1_COL                          8
#    define LCD_SEG_L1_DP1                          9
#    define LCD_SEG_L1_DP0                          10

// Line2 7-segments
#    define LCD_SEG_L2_5                            11
#    define LCD_SEG_L2_4                            12
#    define LCD_SEG_L2_3                            13
#    define LCD_SEG_L2_2                            14
#    define LCD_SEG_L2_1                            15
#    define LCD_SEG_L2_0                            16
#    define LCD_SEG_L2_COL1                         17
#    define LCD_SEG_L2_COL0                         18
#    define LCD_SEG_L2_DP                           19


// Line1 7-segment arrays
#    define LCD_SEG_L1_3_0                          70
#    define LCD_SEG_L1_2_0                          71
#    define LCD_SEG_L1_1_0                          72
#    define LCD_SEG_L1_3_1                          73
#    define LCD_SEG_L1_3_2                          74

// Line2 7-segment arrays
#    define LCD_SEG_L2_5_0                          90
#    define LCD_SEG_L2_4_0                          91
#    define LCD_SEG_L2_3_0                          92
#    define LCD_SEG_L2_2_0                          93
#    define LCD_SEG_L2_1_0                          94
#    define LCD_SEG_L2_5_2                          95
#    define LCD_SEG_L2_3_2                          96
#    define LCD_SEG_L2_5_4                          97
#    define LCD_SEG_L2_4_2                          98


// LCD controller memory map
#    define LCD_MEM_1                               ((u8*)0x0A20)
#    define LCD_MEM_2                               ((u8*)0x0A21)
#    define LCD_MEM_3                               ((u8*)0x0A22)
#    define LCD_MEM_4                               ((u8*)0x0A23)
#    define LCD_MEM_5                               ((u8*)0x0A24)
#    define LCD_MEM_6                               ((u8*)0x0A25)
#    define LCD_MEM_7                               ((u8*)0x0A26)
#    define LCD_MEM_8                               ((u8*)0x0A27)
#    define LCD_MEM_9                               ((u8*)0x0A28)
#    define LCD_MEM_10                              ((u8*)0x0A29)
#    define LCD_MEM_11                              ((u8*)0x0A2A)
#    define LCD_MEM_12                              ((u8*)0x0A2B)


// Memory assignment
#    define LCD_SEG_L1_0_MEM                        (LCD_MEM_6)
#    define LCD_SEG_L1_1_MEM                        (LCD_MEM_4)
#    define LCD_SEG_L1_2_MEM                        (LCD_MEM_3)
#    define LCD_SEG_L1_3_MEM                        (LCD_MEM_2)
#    define LCD_SEG_L1_COL_MEM                      (LCD_MEM_1)
#    define LCD_SEG_L1_DP1_MEM                      (LCD_MEM_1)
#    define LCD_SEG_L1_DP0_MEM                      (LCD_MEM_5)
#    define LCD_SEG_L2_0_MEM                        (LCD_MEM_8)
#    define LCD_SEG_L2_1_MEM                        (LCD_MEM_9)
#    define LCD_SEG_L2_2_MEM                        (LCD_MEM_10)
#    define LCD_SEG_L2_3_MEM                        (LCD_MEM_11)
#    define LCD_SEG_L2_4_MEM                        (LCD_MEM_12)
#    define LCD_SEG_L2_5_MEM                        (LCD_MEM_12)
#    define LCD_SEG_L2_COL1_MEM                     (LCD_MEM_1)
#    define LCD_SEG_L2_COL0_MEM                     (LCD_MEM_5)
#    define LCD_SEG_L2_DP_MEM                       (LCD_MEM_9)

#    define LCD_SYMB_PERCENT_MEM            (LCD_MEM_5)
#    define LCD_ICON_BEEPER1_MEM            (LCD_MEM_5)
#    define LCD_ICON_BEEPER2_MEM            (LCD_MEM_6)
#    define LCD_ICON_BEEPER3_MEM            (LCD_MEM_7)

// Bit masks for write access
#    define LCD_SEG_L1_0_MASK                       (BIT2 + BIT1 + BIT0 + BIT7 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L1_1_MASK                       (BIT2 + BIT1 + BIT0 + BIT7 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L1_2_MASK                       (BIT2 + BIT1 + BIT0 + BIT7 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L1_3_MASK                       (BIT2 + BIT1 + BIT0 + BIT7 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L1_COL_MASK                     (BIT5)
#    define LCD_SEG_L1_DP1_MASK                     (BIT6)
#    define LCD_SEG_L1_DP0_MASK                     (BIT2)
#    define LCD_SEG_L2_0_MASK                       (BIT3 + BIT2 + BIT1 + BIT0 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L2_1_MASK                       (BIT3 + BIT2 + BIT1 + BIT0 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L2_2_MASK                       (BIT3 + BIT2 + BIT1 + BIT0 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L2_3_MASK                       (BIT3 + BIT2 + BIT1 + BIT0 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L2_4_MASK                       (BIT3 + BIT2 + BIT1 + BIT0 + BIT6 + BIT5 + BIT4)
#    define LCD_SEG_L2_5_MASK                       (BIT7)
#    define LCD_SEG_L2_COL1_MASK            (BIT4)
#    define LCD_SEG_L2_COL0_MASK            (BIT0)
#    define LCD_SEG_L2_DP_MASK                      (BIT7)

#    define LCD_SYMB_PERCENT_MASK           (BIT4)
#    define LCD_ICON_BEEPER1_MASK           (BIT3)
#    define LCD_ICON_BEEPER2_MASK           (BIT3)
#    define LCD_ICON_BEEPER3_MASK           (BIT3)

#    define LCD_DISPLAY_DONE            (1u)
#    define LCD_DISPLAY_RFBSL           (2u)
#    define LCD_DISPLAY_FAIL            (3u)

// *************************************************************************************************
// API section

// Physical LCD memory write
extern void write_lcd_mem(u8 * lcdmem, u8 bits, u8 bitmask, u8 state);

// Display init / clear
extern void lcd_init(void);
extern void clear_display(void);


// Character / symbol draw functions
extern void display_char(u8 segment, u8 chr, u8 mode);
extern void display_chars(u8 line, u8 * str);
extern void display_symbol(u8 symbol, u8 mode);

// Integer to string conversion
extern u8 *itoa(u32 n, u8 digits, u8 blanks);

// Segment index helper function
extern u8 switch_seg(u8 line, u8 index1, u8 index2);

#endif // RAM_BASED_RFBSL

#endif // __DISPLAY_
