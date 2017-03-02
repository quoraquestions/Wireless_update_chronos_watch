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
// Display functions.
// *************************************************************************************************


// *************************************************************************************************
// Include section

// system
#include "project_defs.h"

#ifdef RAM_BASED_RFBSL
#    include <string.h>

// driver
#    include "display.h"

// *************************************************************************************************
// Prototypes section
void write_lcd_mem(u8 * lcdmem, u8 bits, u8 bitmask, u8 state);
void display_symbol(u8 symbol, u8 mode);
void display_char(u8 segment, u8 chr, u8 mode);
void display_chars(u8 line, u8 * str);


// *************************************************************************************************
// Defines section



// *************************************************************************************************
// Global Variable section

// Global return string for itoa function
u8 itoa_str[4];

// *************************************************************************************************
// @fn          lcd_init
// @brief       Erase LCD memory. Init LCD peripheral.
// @param       none
// @return      none
// *************************************************************************************************
void lcd_init(void)
{
    // Clear entire display memory
    LCDBMEMCTL |= LCDCLRBM + LCDCLRM;

    // LCD_FREQ = ACLK/16/8 = 256Hz
    // Frame frequency = 256Hz/4 = 64Hz, LCD mux 4, LCD on
    LCDBCTL0 = (LCDDIV0 + LCDDIV1 + LCDDIV2 + LCDDIV3) | (LCDPRE0 + LCDPRE1) | LCD4MUX | LCDON;

    // LCB_BLK_FREQ = ACLK/8/4096 = 1Hz
    LCDBBLKCTL = LCDBLKPRE0 | LCDBLKPRE1 | LCDBLKDIV0 | LCDBLKDIV1 | LCDBLKDIV2 | LCDBLKMOD0;

    // I/O to COM outputs
    P5SEL |= (BIT5 | BIT6 | BIT7);
    P5DIR |= (BIT5 | BIT6 | BIT7);

    // Activate LCD output
    LCDBPCTL0 = 0xFFFF;                        // Select LCD segments S0-S15
    LCDBPCTL1 = 0x00FF;                        // Select LCD segments S16-S22

#    ifdef USE_LCD_CHARGE_PUMP
    // Charge pump voltage generated internally, internal bias (V2-V4) generation
    LCDBVCTL = LCDCPEN | VLCD_2_72;
#    endif
}

// *************************************************************************************************
// @fn          clear_display
// @brief       Erase LINE1 and LINE2 segments. Keep icons.
// @param       none
// @return      none
// *************************************************************************************************
void clear_display(void)
{
    display_chars(LINE1, (u8 *)"  ");
    display_chars(LINE2, (u8 *)"      ");
}

// *************************************************************************************************
// @fn          write_segment
// @brief       Write to one or multiple LCD segments
// @param       lcdmem		Pointer to LCD byte memory
//		bits		Segments to address
//		bitmask		Bitmask for particular display item
//		mode		On, off or blink segments
// @return
// *************************************************************************************************
void write_lcd_mem(u8 * lcdmem, u8 bits, u8 bitmask, u8 state)
{
    if (state == SEG_ON)
    {
        // Clear segments before writing
        *lcdmem = (u8)(*lcdmem & ~bitmask);

        // Set visible segments
        *lcdmem = (u8)(*lcdmem | bits);
    }
    else if (state == SEG_OFF)
    {
        // Clear segments
        *lcdmem = (u8)(*lcdmem & ~bitmask);
    }
    else if (state == SEG_ON_BLINK_ON)
    {
        // Clear visible / blink segments before writing
        *lcdmem           = (u8)(*lcdmem & ~bitmask);
        *(lcdmem + 0x20)    = (u8)(*(lcdmem + 0x20) & ~bitmask);

        // Set visible / blink segments
        *lcdmem           = (u8)(*lcdmem | bits);
        *(lcdmem + 0x20)    = (u8)(*(lcdmem + 0x20) | bits);
    }
    else if (state == SEG_ON_BLINK_OFF)
    {
        // Clear visible segments before writing
        *lcdmem = (u8)(*lcdmem & ~bitmask);

        // Set visible segments
        *lcdmem = (u8)(*lcdmem | bits);

        // Clear blink segments
        *(lcdmem + 0x20)    = (u8)(*(lcdmem + 0x20) & ~bitmask);
    }
    else if (state == SEG_OFF_BLINK_OFF)
    {
        // Clear segments
        *lcdmem = (u8)(*lcdmem & ~bitmask);

        // Clear blink segments
        *(lcdmem + 0x20)    = (u8)(*(lcdmem + 0x20) & ~bitmask);
    }
}

// *************************************************************************************************
// @fn          itoa
// @brief       Generic integer to array routine. Converts integer n to string.
//				Default conversion result has leading zeros, e.g. "01"
//				Option to convert leading '0' into whitespace (blanks)
// @param       u32 n		integer to convert
//		u8 digits	number of digits
//		u8 blanks	fill up result string with number of whitespaces instead of leading
// zeros
// @return      u8		string
// *************************************************************************************************
u8 *itoa(u32 n, u8 digits, u8 blanks)
{
    // Preset result string
    memcpy(itoa_str, "00", 2);

    // Return empty string if number of digits is invalid (valid range for digits: 1-7)
    if ((digits == 0) || (digits > 2) || n > 99) return (itoa_str);

    // Numbers 0 .. 99 can be copied from itoa_conversion_table without conversion
    memcpy(itoa_str, itoa_conversion_table[n] + (2 - digits), digits);

    return (itoa_str);
}

// *************************************************************************************************
// @fn          display_symbol
// @brief       Switch symbol on or off on LCD.
// @param       u8 symbol		A valid LCD symbol (index 0..42)
//		u8 state		SEG_ON, SEG_OFF, SEG_BLINK
// @return      none
// *************************************************************************************************
void display_symbol(u8 symbol, u8 mode)
{
    u8 * lcdmem;
    u8 bits;
    u8 bitmask;

    if (symbol <= LCD_SEG_L2_DP)
    {
        // Get LCD memory address for symbol from table
        lcdmem    = (u8 *)segments_lcdmem[symbol];

        // Get bits for symbol from table
        bits      = segments_bitmask[symbol];

        // Bitmask for symbols equals bits
        bitmask = bits;

        // Write LCD memory
        write_lcd_mem(lcdmem, bits, bitmask, mode);
    }
}

// *************************************************************************************************
// @fn          display_char
// @brief       Write to 7-segment characters.
// @param       u8 segment		A valid LCD segment
//		u8 chr			Character to display
//		u8 mode		SEG_ON, SEG_OFF, SEG_BLINK
// @return      none
// *************************************************************************************************
void display_char(u8 segment, u8 chr, u8 mode)
{
    u8 * lcdmem;                // Pointer to LCD memory
    u8 bitmask;                 // Bitmask for character
    u8 bits, bits1;             // Bits to write

    // Write to single 7-segment character
    if ((segment >= LCD_SEG_L1_3) && (segment <= LCD_SEG_L2_DP))
    {
        // Get LCD memory address for segment from table
        lcdmem = (u8 *)segments_lcdmem[segment];

        // Get bitmask for character from table
        bitmask = segments_bitmask[segment];

        // Get bits from font set
        if ((chr >= 0x30) && (chr <= 0x5A))
        {
            // Use font set
            bits = lcd_font[chr - 0x30];
        }
        else if (chr == 0x2D)
        {
            // '-' not in font set
            bits = BIT1;
        }
        else
        {
            // Other characters map to ' ' (blank)
            bits = 0;
        }
        // When addressing LINE2 7-segment characters need to swap high- and low-nibble,
        // because LCD COM/SEG assignment is mirrored against LINE1
        if (segment >= LCD_SEG_L2_5)
        {
            bits1 = ((bits << 4) & 0xF0) | ((bits >> 4) & 0x0F);
            bits = bits1;

            // When addressing LCD_SEG_L2_5, need to convert ASCII '1' and 'L' to 1 bit,
            // because LCD COM/SEG assignment is special for this incomplete character
            if (segment == LCD_SEG_L2_5)
            {
                if ((chr == '1') || (chr == 'L')) bits = BIT7;
            }
        }

        // Physically write to LCD memory
        write_lcd_mem(lcdmem, bits, bitmask, mode);
    }
}

// *************************************************************************************************
// @fn          display_chars
// @brief       Write to consecutive 7-segment characters.
// @param       u8 segments	LCD segment array
//		u8 * str	Pointer to a string
//		u8 mode		SEG_ON, SEG_OFF, SEG_BLINK
// @return      none
// *************************************************************************************************
//void display_chars(u8 segments, u8 * str, u8 mode)
void display_chars(u8 line, u8 * str)
{
    u8 i;
    u8 char_start;                      // Starting point for consecutive write
    u8 length = 0;

    if (line == LINE2)
    {
        char_start = LCD_SEG_L2_5;
        length = 6;
    }
    else
    {
        char_start = LCD_SEG_L1_1;
        length = 2;
    }

    // Write consecutive characters
    for (i = 0; i < length; i++)
    {
        // Use single character routine to write display memory
        display_char(char_start + i, *(str + i), SEG_ON);
    }
}

#endif  // RAM_BASED_RFBSL
