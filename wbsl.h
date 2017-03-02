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
#ifndef WBSL_H_
#define WBSL_H_
#include "project_defs.h"

//Temporal Defines

/*
 * Product = CC430
 * Chip version (VERSION = 0x06)
 * Crystal accuracy = 10 ppm
 * X-tal frequency = 26 MHz
 * RF output power = 0 dBm
 * RX filterbandwidth = 101.562500 kHz
 * Deviation = 19 kHz
 * Datarate = 38.383484 kBaud
 * Modulation = (1) GFSK
 * Manchester enable = (0) Manchester disabled
 * RF Frequency = 914.999 MHz
 * Channel spacing = 199.951172 kHz
 * Channel number = 0
 * Optimization = -
 * Sync mode = (3) 30/32 sync word bits detected
 * Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
 * CRC operation = (1) CRC calculation in TX and CRC check in RX disabled
 * Forward Error Correction = (0) FEC disabled
 * Length configuration = (1) Variable length packets, packet length configured by the first
 * received byte after sync word.
 * Packetlength = 10
 * Preamble count = 0 bytes - sync word is always accepted
 * Append status = 0
 * Address check = (0) No Address check
 * FIFO autoflush = 0
 * Device address = 0
 * GDO0 signals on PA power down signal to control RX/TX switch
 * GDO1 signals on RSSI_VALID
 * GDO2 signal selection = (41) CHIP_RDY
 */

//#include "cc430x613x.cmd"

//Watch ports
#define BUTTONS_IN              (P2IN)
#define BUTTONS_OUT             (P2OUT)
#define BUTTONS_DIR             (P2DIR)
#define BUTTONS_REN             (P2REN)
#define BUTTONS_IE              (P2IE)
#define BUTTONS_IES             (P2IES)
#define BUTTONS_IFG             (P2IFG)
#define BUTTONS_IRQ_VECT2       (PORT2_VECTOR)

// Button ports
#define BUTTON_M1_PIN           (BIT2)
#define BUTTON_M2_PIN           (BIT1)
#define BUTTON_S1_PIN           (BIT4)
#define BUTTON_S2_PIN           (BIT0)
#define BUTTON_BL_PIN           (BIT3)
#define ALL_BUTTONS                             (BUTTON_M1_PIN + BUTTON_M2_PIN + BUTTON_S1_PIN + \
                                                 BUTTON_S2_PIN + BUTTON_BL_PIN)


#define FLASH_RFSBL_ENTRY       (0x1000)
#define RESET_VECTOR_ADDRESS    (0xFFFE)
#define PORT2_VECTOR_ADDRESS    (0xFFE0)

#ifdef RAM_BASED_RFBSL
extern void wbsl_vector_write(void);

#endif

// ---------------------------------------------------------------
// Generic Defines and variables
// Maximum data length
#define WBSL_MAX_PAYLOAD_LENGTH         (253u)
#define WBSL_ED_ADDRESS                 (0xAD)
#define WBSL_CRC_STATUS_OFFSET          (2)
#define DISCOVERY_PAYLOAD_LENGTH        (4u)
#define DISCOVERY_OVERHEAD_LENGTH       (3u)
#define AP_ADDRESS_OFFSET_TX            (1u)
#define ED_ADDRESS_OFFSET_TX            (2u)
#define BATTERY_VOLTAGE_OFFSET          (3u)
#define LINK_ACK_OFFSET                 (3u)
#define AP_ADDRESS_OFFSET_RX            (2u)
#define ED_ADDRESS_OFFSET_RX            (1u)
#define WBSL_OPCODE_OFFSET              (5u)

#define MAX_RXFIFO_SIZE                 (64u)

#define WBSL_PACKET_OVERHEAD            (5u)
#define WBSL_LENGTH_FIELD_SIZE          (1u)
#define PACKET_HEADER_OFFSET            (3u)
//#define IS_ADDRESS                      (0x80)
//#define MSB_PACKET_NUMBER               (0x7F)
//#define LSB_PACKET_NUMBER               (0xFF)

#define INIT_PACKET_OPCODE              (0u)
#define ADDRESS_PACKET_OPCODE           (1u)
#define NORMAL_PACKET_OPCODE            (2u)

#define FIRST_PAYLOAD_BYTE              (WBSL_PACKET_OVERHEAD + WBSL_LENGTH_FIELD_SIZE)

#define WBSL_LAST_FLASH_WORD_ADDRESS   ((u8 *)0xFFFE)

#define WBSL_ACK_PKT_SIZE               (5u)

#define WBSL_LINK_FAIL                   (0u)
#define WBSL_LINK_SUCC                   (1u)

#define WBSL_RECEIVE_FAIL               (0u)
#define WBSL_RECEIVE_SUCC               (1u)

#define WBSL_INIT_ACK                    (0)
#define WBSL_PKT_ACK                     (1u)

#define CRC_STATUS                      (0x80)

#define WBSL_MAXIMUM_RETRIES            (5u)

#define START_USER_FLASH_MEMORY         (0x8000)
#define END_USER_FLASH_MEMORY           (0xFFFE)
#define FLASH_SEGMENT_SIZE              (0x200)
#define INTERRUPT_VECTOR_START_ADDRESS  (0xFFE0)
#define RESET VECTOR_SIZE(0x80)

#define WBSL_STATUS_LINKING             (BIT0)
#define WBSL_STATUS_LINKED              (BIT1)
#define WBSL_STATUS_ERROR               (BIT2)
#define WBSL_TRIGGER_SEND_DATA          (BIT3)
#define WBSL_TRIGGER_RECEIVED_DATA      (BIT4)
#define WBSL_TRIGGER_STOP               (BIT5)
#define WBSL_TRIGGER_SEND_CMD           (BIT6)
#define WBSL_ILLEGAL_MEMORY_ERROR       (BIT7)

// Flag for status information, to see if WBSL is in RX/TX/IDLE mode
extern volatile u8 wbslMode_flag;
#define WBSL_IDLE_MODE                  (BIT0)
#define WBSL_RX_MODE                    (BIT1)
#define WBSL_TX_MODE                    (BIT2)


#define WBSL_RXTX_RECEIVED              (BIT0)
#define WBSL_RXTX_SEND                  (BIT1)

#define WBSL_PACKET_WRITTEN_FAIL        (0u)
#define WBSL_PACKET_WRITTEN_SUCC        (1u)

#define WBSL_OPERATION_FAIL             (0u)
#define WBSL_OPERATION_SUCC             (1u)

#define TIMEOUT_FOR_ACK                 (CONV_MS_TO_TICKS(500)) // 500 milliseconds in ticks

#define LOWER_BOUND_MAIN_MEMORY         (0x8000)
#define UPPER_BOUND_MAIN_MEMORY         (0x10000)

#define LOWER_BOUND_RAM_MEMORY          (0x1D30)
#define UPPER_BOUND_RAM_MEMORY          (0x2AFE)

// SimpliciTI connection states
typedef enum
{
    WBSL_OFF = 0,                                               // Not connected
    WBSL_SEARCHING,                                             // Trying to pair with End Device
    WBSL_ERROR,                                                 // Connection Error
    WBSL_TIMEOUT,                                               // Packet timeout during WBSL
                                                                // transfer
    WBSL_CONNECTED                                              // Bsl Update
} wbsl_mode_t;

// *************************************************************************************************
// Global Variable section
struct RFwbsl
{
    // Different WBSL Modes
    volatile wbsl_mode_t mode;
    //Watch ID
    volatile u32 wId;
    // Timeout until WBSL is automatically stopped
    volatile u16 timeout;
};
//Prototypes
extern void sx_wbsl(u8 line);

#ifdef RAM_BASED_RFBSL
extern void display_wbsl(u8 line, u8 update);

#endif

extern void reset_wbsl(void);
extern void RadioIsr_wbsl(void);

extern void wbsl_resetTimer(void);
extern void wbsl_setTimer(u16 ticks);

extern void init_watchButtons(void);
extern u8 wbsl_writeWord(u16 *addr, u16 data);

extern struct RFwbsl sRFwbsl;

extern volatile u8 wbsl_progress;
extern u8 AP_address;

extern volatile u8 wbsl_flag;
// Flag to signal a packet received to the main program
extern volatile u8 rxtx_flag;


#endif /*WBSL_H_*/
