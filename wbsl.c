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
// WBSL functions.
// *************************************************************************************************

// *************************************************************************************************
// Include section

// system
#include "project_defs.h"
#include <string.h>
#include "intrinsics.h"

#ifdef RAM_BASED_RFBSL
#    include "display.h"
#endif

// driver
#include "radio.h"
#include "rf1a.h"

// logic
#include "wbsl.h"

// *************************************************************************************************
// Global Variable section
struct RFwbsl sRFwbsl = { WBSL_OFF, 0, 0 };

// Rx Buffer to store Received Packages, make sure it starts on an even address so that the payload
// can be written to flash
// with no issues
#pragma data_alignment=2
u8 RxBuffer[258] = {0};

u8 RxBufferLength = 0;

// Pointer to write to RAM/Flash
u8 *ptrMemory = (u8 *) 0x0;

//Address to branch to start RAM Based BSL
u16 start_ram_bsl_address = 0;

//u8 discoveryPayload[4] = {0xDE,0xAD,0xBE,0xEF};
u8 discoveryPayload[4] = {0xBA, 0x5E, 0xBA, 0x11};

/*Length*/   /*Ap Address*/ /*My Address*/ /*Batt Volt*/ /*Discovery Payload*/
u8 discoveryReply[8] = {    7,       0, WBSL_ED_ADDRESS,      0, 0xBA, 0x5E, 0xBA, 0x11};
// flag contains status information, trigger to receive data and trigger to exit WBSL
volatile u8 wbsl_flag = 0;
// RF Mode Flag
volatile u8 wbslMode_flag = 0;
// RX Flag
volatile u8 rxtx_flag = 0;
// ACK Reply Package Format    Size   / Dongle ID /   Watch ID   /  ACK Status  / Package Number /
// Package Numer 2
u8 ackReply[6] =           {    5,    0, WBSL_ED_ADDRESS,      0,       0,     0          };
// How far along is the Update procedure Range: 0 - 100
volatile u8 wbsl_progress = 0;
// This is the Address of the Access point which we receive during the linking phase
u8 AP_address = 0;
// Handle number of ACKs sent for an indivudual packet
u8 wbsl_number_of_retries = 0;
// Variable to see if the Init Packet has been successfully sent
u8 initOk = 0;
// Store the total number of packets to be sent to the Watch
u16 totalPackets = 0;
// Keep track of which packet needs to be sent to the Watch
u16 currentPacket = 0;

#ifndef RAM_BASED_RFBSL
#    pragma location=0x2BE0
__no_init u16 port2_isrAddress;    // Interrupt Vector Address for PORT2 in RAM
#endif

// Function Prototypes for Local Functions
void start_wbsl(void);
void config_radio_wbsl(void);
void stop_radio_wbsl(void);
u8 wbsl_receivePackets(void);
void wbsl_replyAck(u8 status, u16 packetNummer);
void init_watch(void);
u8 wbsl_link(void);

#ifndef RAM_BASED_RFBSL
void init_watchButtons(void);

#endif

#ifdef RAM_BASED_RFBSL
void massEraseFlash(void);
void wbsl_openFlashForWrite(void);
void wbsl_lockFlash(void);
void init_interrupt_vector(void);

#endif

// *************************************************************************************************
// @fn          main
// @brief       Starting point of the RFBSL, calls the needed routines prior to starting
// communication
// @param       none
// @return      u8  exit status
// *************************************************************************************************
int main(){
    WDTCTL = WDTPW + WDTHOLD;        // Stop watchdog timer

#ifndef RAM_BASED_RFBSL
    if (*((u16*)(0xFFFE)) == 0xFFFF) // Check if recovery procedure needs to be started
    {
        port2_isrAddress = 0x1002;   // Move the address of the Port2_ISR to the address of the
                                     // Port2 interrupt vector in RAM
        SYSCTL |= SYSRIVECT;         // Move interrupt vector to RAM
        init_watchButtons();         // Initialize button S2 so that we wait until is pressed to
                                     // continue
        _BIS_SR(LPM4_bits + GIE);    // Wait until Button is pressed to go to ISR
    }
#endif

    //Disable all interrupts
    __disable_interrupt();

    // Initialize clocks and the LCD only in the case of the RAM_BASED_RFBSL
    init_watch();

#ifdef RAM_BASED_RFBSL
    display_wbsl(LINE2, 0);
#endif

    while (1){

#ifdef RAM_BASED_RFBSL
      
#define INCREASED_START_ATTEMPTS (4)
        u8 i = INCREASED_START_ATTEMPTS;
        while(i--)
        {
            // Wait for AP/GUI to be ready to send update image
            wbsl_setTimer(CONV_MS_TO_TICKS(900));
            while (!(TA0CCTL1 & CCIFG)) ;
            wbsl_resetTimer();
        }
#endif

        start_wbsl();

        // If no AP was found during discovery phase, reset the device
        PMMCTL0 = PMMPW | PMMSWBOR; // generate BOR

    }
}

// *************************************************************************************************
// @fn          init_watchButtons
// @brief       Inits the S2 (Down) button so that it triggers the interrupt to wake up in the case
// of
//              of a recovery procedure
// @param       none
// @return      none
// *************************************************************************************************
#ifndef RAM_BASED_RFBSL
void init_watchButtons(void)
{
    // Set button ports to input
    BUTTONS_DIR &= ~BUTTON_S2_PIN;

    // Enable internal pull-downs
    BUTTONS_OUT &= ~BUTTON_S2_PIN;
    BUTTONS_REN |= BUTTON_S2_PIN;

    // IRQ triggers on rising edge
    BUTTONS_IES &= ~BUTTON_S2_PIN;

    // Reset IRQ flags
    BUTTONS_IFG &= ~BUTTON_S2_PIN;

    // Enable button S2 interrupts
    BUTTONS_IE |= BUTTON_S2_PIN;
}

#endif

// *************************************************************************************************
// @fn          wbsl_openFlashForWrite
// @brief       Opens the Flash for writing by setting the appropriate bits
// @param       none
// @return      none
// *************************************************************************************************
#ifdef RAM_BASED_RFBSL
void wbsl_openFlashForWrite()
{
    FCTL3 = FWKEY;                      // Clear Lock bit
    FCTL1 = FWKEY + WRT;                // Set write bit
}

#endif

// *************************************************************************************************
// @fn          wbsl_lockFlash
// @brief       Locks the Flash by setting the appropriate bits
// @param       none
// @return      none
// *************************************************************************************************
#ifdef RAM_BASED_RFBSL
void wbsl_lockFlash(void)
{
    FCTL1 = FWKEY;                      // Clear WRT bit
    FCTL3 = FWKEY + LOCK;               // Set LOCK bit
}

#endif

// *************************************************************************************************
// @fn          wbsl_writeByte
// @brief       writes a byte to Memory and checks if it was correctly written
// @param       u8 *address	Address to which you want to write the byte
//		u8 data	        The data to be written at the given address
// @return      u8		status
//              WBSL_OPERATION_SUCC   Byte successfully written
//              WBSL_OPERATION_FAIL   Write Check failed
// *************************************************************************************************
u8 wbsl_writeByte(u8 *address, u8 data)
{
    u8 ret_status = WBSL_OPERATION_SUCC;

#ifdef RAM_BASED_RFBSL
    while (FCTL3 & BUSY) ;
#endif
    *address = data;
#ifdef RAM_BASED_RFBSL
    while (FCTL3 & BUSY) ;
#endif
    // Check if write was successfull
    if (*address != data)
    {
        ret_status = WBSL_OPERATION_FAIL;
    }
    return ret_status;
}

u8 wbsl_writeWord(u16 *addr, u16 data)
{
    u8 ret_status = WBSL_OPERATION_SUCC;

#ifdef RAM_BASED_RFBSL
    while (FCTL3 & BUSY) ;
#endif
    *addr = data;
#ifdef RAM_BASED_RFBSL
    while (FCTL3 & BUSY) ;
#endif
    if (*addr != data)
    {
        ret_status = WBSL_OPERATION_FAIL;
    }
    return ret_status;
}

// *************************************************************************************************
// @fn          wbsl_writeBytes
// @brief       writes a byte to Memory and checks if it was correctly written
//              the function will write in word mode if possible
// @param       u8 *address	Address to which you want to write the byte
//              u8 size         The number of bytes to be written
//		u8 data	        The data to be written at the given address (must be even aligned)
// @return      u8		status
//              WBSL_OPERATION_SUCC   Bytes successfully written
//              WBSL_OPERATION_FAIL   Write Check failed
// *************************************************************************************************
u8 wbsl_writeBytes(u8 *startAddr, u8 size,  u8 *data)
{
    //u8 *i;
    u32 i;
    u8 ret_status = WBSL_OPERATION_SUCC;

    // Make sure that the comparison doesn't overflow at the End of the 16 bit address, make the
    // comparison in 32bit
    for (i = (u32)startAddr; i < ((u32)startAddr) + ((u32)size); i++)
    {
#ifdef RAM_BASED_RFBSL
        if ((((u16)startAddr) & 0x01) || i == (u32)startAddr + (u32)size - 1)
#endif
        {
            ret_status = wbsl_writeByte((u8 *)((u16)i), *data);
            data += 1;
        }
#ifdef RAM_BASED_RFBSL
        else
        {
            ret_status = wbsl_writeWord((u16 *)((u16)i), *((u16 *)(data)));
            data += 2;
            i++;
        }
#endif
        if (ret_status != WBSL_OPERATION_SUCC)
        {
            return ret_status;
        } // if

    }     // for
    return ret_status;

}

// *************************************************************************************************
// @fn          wbsl_writePacket
// @brief       writes the packet which is in the global RxBuffer to the address pointed by
//              the global ptrMemory address
// @param       none
// @return      u8		status
//              WBSL_OPERATION_SUCC   Packet successfully written
//              WBSL_OPERATION_FAIL   Write Check failed for some byte of the packet
// *************************************************************************************************
u8 wbsl_writePacket(void){

    u16 packetSize;
    u8 payloadStart;
    u8 ret_status = WBSL_OPERATION_SUCC;

    packetSize = RxBuffer[0] - WBSL_PACKET_OVERHEAD;
    payloadStart = FIRST_PAYLOAD_BYTE;

    // Write to flash the packet
    if (RxBuffer[WBSL_OPCODE_OFFSET] == ADDRESS_PACKET_OPCODE)
    {
        ptrMemory = (u8 *)((RxBuffer[FIRST_PAYLOAD_BYTE] << 8) + RxBuffer[FIRST_PAYLOAD_BYTE + 1]);
        packetSize -= 2;   // Substract the address length
        payloadStart += 2; // Move Offset to compensate for the Address Length

#ifndef RAM_BASED_RFBSL    // This is only for the flash based BSL, for the RAM Based the address
                           // should be written to flash
        if (ptrMemory == (u8 *)(0xFFFE))
        {
            start_ram_bsl_address = RxBuffer[payloadStart] + (RxBuffer[payloadStart + 1] << 8);
            packetSize -= 2;
        }
#endif
    }

#ifdef RAM_BASED_RFBSL
    wbsl_openFlashForWrite();
#endif
    // Only if there is something to write
    if (packetSize)
    {
#ifdef RAM_BASED_RFBSL
        // Check memory write boundaries Main Memory 0x8000 - 0xFFFF
        if ((u16)ptrMemory < LOWER_BOUND_MAIN_MEMORY || ((u32)ptrMemory + (u32)packetSize) >
            (u32)UPPER_BOUND_MAIN_MEMORY)
#else
        // Check memory write boundaries Main Memory 0x1D30 - 0x2AFE   (Depends on how you set up
        // the linker file for the flash based RFBSL)
        if ((u16)ptrMemory < LOWER_BOUND_RAM_MEMORY || ((u32)ptrMemory + (u32)packetSize) >
            (u32)UPPER_BOUND_RAM_MEMORY)
#endif
        {
            ret_status = WBSL_OPERATION_FAIL;
        }
        else
        {
            ret_status = wbsl_writeBytes(ptrMemory, packetSize, &RxBuffer[payloadStart]);
        }

        if (ret_status == WBSL_OPERATION_SUCC)
        {
            ptrMemory += packetSize;
        }
    }

#ifdef RAM_BASED_RFBSL
    wbsl_lockFlash();
#endif

    return ret_status;
}

// *************************************************************************************************
// @fn          start_wbsl
// @brief       This is the main body of the RFBSL, this functions calls all the auxiliary functions
//              of the RFBSL and handles some of the state flags
// @param       none
// @return      none
// *************************************************************************************************
void start_wbsl(void)
{
#ifdef RAM_BASED_RFBSL
#define INCREASED_LINK_ATTEMPTS (10)
    u8 max_link_attempts;
    max_link_attempts = INCREASED_LINK_ATTEMPTS;

    while (max_link_attempts >= 1 && !initOk)
    {
#endif
    //Reset all WBSL Variables
    reset_wbsl();

    /* Select Interrupt edge for PA_PD and SYNC signal:
     * Interrupt Edge select register: 0 == Interrupt on Low to High transition.
     * SYNC word Detected
     */
    RF1AIES = 0;

#ifdef RAM_BASED_RFBSL
    // Clear LINE1 Display
    clear_display();
    display_wbsl(LINE1, 0);

    // Turn on beeper icon to show activity
    display_symbol(LCD_ICON_BEEPER1, SEG_ON_BLINK_ON);
    display_symbol(LCD_ICON_BEEPER2, SEG_ON_BLINK_ON);
    display_symbol(LCD_ICON_BEEPER3, SEG_ON_BLINK_ON);

#endif // RAM_BASED_RFBSL

    // Prepare radio for RF communication
    radio_reset();

#ifndef RAM_BASED_RFBSL
    // It only needs to be configured when the Flash Based RFBSL is running since when the RAM RFBSL
    // runs, the radio registers are already configured
    config_radio_wbsl();
#endif

#ifdef RAM_BASED_RFBSL
    display_symbol(LCD_SYMB_PERCENT, SEG_ON);
#endif
    // Set WBSL smode
    sRFwbsl.mode = WBSL_SEARCHING;
    wbsl_flag = WBSL_STATUS_LINKING;

    // Try to link once only to save watch battery
    if (wbsl_link() == WBSL_LINK_SUCC)
    {
        wbsl_flag = WBSL_STATUS_LINKED;

#ifdef RAM_BASED_RFBSL
        // Turn off blinking on beeper icon to show that the device is linked
        display_symbol(LCD_ICON_BEEPER1, SEG_ON_BLINK_OFF);
        display_symbol(LCD_ICON_BEEPER2, SEG_ON_BLINK_OFF);
        display_symbol(LCD_ICON_BEEPER3, SEG_ON_BLINK_OFF);

        //Erase all memory main memory
        massEraseFlash();

#endif  // RAM_BASED_RFBSL

        while (currentPacket < totalPackets || !initOk)
        {
            if (wbsl_receivePackets() == WBSL_RECEIVE_SUCC)
            {
#ifdef RAM_BASED_RFBSL
                if (wbsl_progress < 100)
                {
                    display_chars(LINE1, itoa(wbsl_progress, 2, 0));
                }
                else
                {
                    display_chars(LINE1, "  ");
                    display_symbol(LCD_SYMB_PERCENT, SEG_OFF);
                    display_chars(LINE2, (u8 *)"  DONE");
                }
#endif
            }

            // If image has been completely received or the RFBSL was triggered to stop
            if (wbsl_flag == WBSL_TRIGGER_STOP || (currentPacket >= totalPackets && initOk))
            {
                break;
            }

        }
    }


#ifdef RAM_BASED_RFBSL
    // Clear icons
    display_symbol(LCD_ICON_BEEPER1, SEG_OFF_BLINK_OFF);
    display_symbol(LCD_ICON_BEEPER2, SEG_OFF_BLINK_OFF);
    display_symbol(LCD_ICON_BEEPER3, SEG_OFF_BLINK_OFF);

    // Powerdown radio
    radio_reset();

    // Clear the LCD segments
    display_symbol(LCD_SYMB_PERCENT, SEG_OFF);

#endif  // RAM_BASED_RFBSL

    // Set WBSL state to OFF
    sRFwbsl.mode = WBSL_OFF;


    if (currentPacket >= totalPackets && initOk)
    {
#ifdef RAM_BASED_RFBSL
        // Reset the device so that the application which was just downloaded starts from the RESET
        // VECTOR
        PMMCTL0 = PMMPW | PMMSWBOR;  // generate BOR
#else    //Only for the Flash Based RFBSL, jump to the address at which the RAM_RFBSL starts
        asm (" mov.w &start_ram_bsl_address,PC");
#endif
    }
#ifdef RAM_BASED_RFBSL
    else if (max_link_attempts < 1)
    {
        display_chars(LINE1, (u8 *)"  ");
        display_chars(LINE2, (u8 *)"  FAIL");
        break;
    }

    // Link Failed, decrement link attempts to keep track of how many times we've tried
    max_link_attempts--;
}

#endif
}

// *************************************************************************************************
// @fn          transmitPacket
// @brief       This functions handles the transmission of a packet by filling the
//              TXFIFO and Strobing the Radio with TX, it uses CCA if is defined in
//              the project, if not, it forces the transmit
// @param       u8 *buffer	Buffer containing the bytes to be transmitted
//		u8 length       Length of the buffer to be transmitted
// @return      none
// *************************************************************************************************
void transmitPacket(u8 *buffer, u8 length)
{
#ifdef CCA
    u8 ccaRetries;
    u8 x;
#endif
    // Turn off receiver
    ReceiveOff();
    // Write the Buffer to the Transmit FIFO
    WriteBurstReg(RF_TXFIFOWR, buffer, length);

#ifdef CCA
    ccaRetries = 4;

    while (1){
        /* Radio must be in RX mode for CCA to happen.
         * Otherwise it will transmit without CCA happening.
         * We don't use the RxModeOn because it turns the RX interrupt
         * which we don't need
         */
        Strobe(RF_SRX);

        //Wait for valid RSSI
        while (!(RF1AIN & BV(1))) ;

        /*
         *  Clear the PA_PD pin interrupt flag.  This flag, not the interrupt itself,
         *  is used to capture the transition that indicates a transmit was started.
         *  The pin level cannot be used to indicate transmit success as timing may
         *  prevent the transition from being detected.  The interrupt latch captures
         *  the event regardless of timing.
         */
        RF1AIFG &= ~BV(0);

        Strobe(RF_STX);      // Strobe STX    to initiate transfer

        /*   Found out that we need a delay of atleast 25 us on CC1100 to see
         * the PA_PD signal change. Hence keeping the same for CC430
         */
        wbsl_setTimer(25);
        while (!(TA0CCTL1 & CCIFG)) ;
        wbsl_resetTimer();

        if (RF1AIFG & BV(0))
        {
            /* ------------------------------------------------------------------
             *    Clear Channel Assessment passed.
             *   ----------------------------------
             */

            //Clear the PA_PD pin interrupt flag.
            RF1AIFG &= ~BV(0);

            /* wait for transmit to complete */
            while (!(RF1AIN & BV(0))) ;
            // Transmit done, break of loop
            break;
        }
        else
        {
            /* ------------------------------------------------------------------
             *    Clear Channel Assessment failed.
             *   ----------------------------------
             */
            // Turn off radio to save power
            do {
                x = Strobe(RF_SIDLE);
            } while (x & 0x70);
            /* Wait for XOSC to be stable and radio in IDLE state */

            // Flush receive FIFO of residual Data
            Strobe(RF_SFRX);

            if (ccaRetries != 0)
            {
                /* delay for a number of us */
                wbsl_setTimer(15);
                while (!(TA0CCTL1 & CCIFG)) ;
                wbsl_resetTimer();

                /* decrement CCA retries before loop continues */
                ccaRetries--;
            }
            else /* No CCA retries are left, abort */
            {
                /* set return value for failed transmit and break */
                return;
            }
        }
    }
#else            // Forced TX

    Strobe(RF_STX);

    while (!(RF1AIFG & BIT9)) ;  //Wait until packet has been sent

    // Clear the interrupt flag
    RF1AIFG &= ~BIT9;
#endif // CCA

    // Flush transmit FIFO, Radio is already in IDLE state due to Register configuration
    Strobe(RF_SFTX);
}

// *************************************************************************************************
// @fn          wbsl_link
// @brief       Tries to link with a AP by sending a discovery packet and waiting for
//              a discovery reply or TIMEOUT_FOR_ACK time
// @param       none
// @return      u8 link_Status
//              WBSL_LINK_SUCC   The watch found an AP and they're both linked
//              WBSL_LINK_FAIL   The watch didn't find any AP which responded to
//                               the discovery packet
// *************************************************************************************************
u8 wbsl_link(void){
    u8 crcOk = 0;
    u8 retStatus = WBSL_LINK_FAIL;

    //u8 temp_PKTCTRL1;
    // Already Set
    //discoveryReply[AP_ADDRESS_OFFSET_TX] = 0x00;   // Broadcast so that any AP listening receives
    // the package

    transmitPacket((u8*)discoveryReply, sizeof discoveryReply);

    // Activate RX Mode to wait for ACK of pairing from the AP
    ReceiveOn();

    // Wait for a packet to be received or timeout
    wbsl_setTimer(TIMEOUT_FOR_ACK);

    // Go to LPM3 and wait for either the interrupt from the timer or the RX Interrupt
    //_BIS_SR(LPM3_bits);

    while (!(TA0CCTL1 & CCIFG) && rxtx_flag != WBSL_RXTX_RECEIVED)
    {
        if (RF1AIFG & BIT9)   // Check for the SYNC WORD detected flag
            RadioIsr_wbsl();  // Call the ISR to retrieve the received packet

        //Here we should sleep to wait for either Timeout or Radio
        //_BIS_SR(LPM0_bits);
    }

    // Reset the timer so that next time it starts from fresh state.
    wbsl_resetTimer();

    // Turn radio off during the decoding of the package
    ReceiveOff();

    if (rxtx_flag == WBSL_RXTX_RECEIVED)
    {
        // Clear RX Flag
        rxtx_flag = 0;

        crcOk = RxBuffer[RxBuffer[0] + WBSL_CRC_STATUS_OFFSET] & CRC_STATUS; //CRC status is
                                                                             // appended at the end
                                                                             // of RXFIFO

        if (                                                                 //RxBuffer[ED_ADDRESS_OFFSET_RX]!=
                                                                             // WBSL_ED_ADDRESS ||
            RxBuffer[LINK_ACK_OFFSET] != WBSL_LINK_SUCC      ||
            !crcOk)                                                          // Check the package is
                                                                             // from the AP, the
                                                                             // Address check is
                                                                             // done in hardware
        {
            return retStatus;
        }
        else
        {
            //Save AP_address
            AP_address = RxBuffer[AP_ADDRESS_OFFSET_RX];

            // Set the Link as succesfull so the actual communication can start
            retStatus = WBSL_LINK_SUCC;
        }
    }
    return retStatus;
}

// *************************************************************************************************
// @fn          wbsl_replyAck
// @brief       Replies with an ACK or NACK to the AP
// @param       u8 status          ACK or NACK
//              u16 packetNummer   The packet number for which the ACK is
// @return      none
// *************************************************************************************************
void wbsl_replyAck(u8 status, u16 packetNummer){
    // Delay to let AP Dongle be in RX Mode
    // Set the timeout
    wbsl_setTimer(CONV_US_TO_TICKS(300));
    // Wait for a timeout
    while (!(TA0CCTL1 & CCIFG)) ;
    // Reset the timer
    wbsl_resetTimer();
    //Fill the ACK Reply Buffer
    ackReply[AP_ADDRESS_OFFSET_TX] = AP_address;
    //ackReply[ED_ADDRESS_OFFSET_TX] = WBSL_ED_ADDRESS;  // This is put on declaration
    ackReply[ED_ADDRESS_OFFSET_TX + 1] = status;

    ackReply[ED_ADDRESS_OFFSET_TX + 2] = (packetNummer >> 8) & 0x7F;
    ackReply[ED_ADDRESS_OFFSET_TX + 3] = packetNummer;

    transmitPacket((u8*)ackReply, sizeof(ackReply));

}

// *************************************************************************************************
// @fn          wbsl_receivePackets
// @brief       This functions puts the radio in RX Mode to listen for new packages
//              and waits for TIMEOUT, if packet is received, check for CRC,
//              if packet was sent from AP and addressed to us, if everything
//              checks out, it calls the write packet function
// @param       none
// @return      u8 status
//              WBSL_ERROR      No packet was received or error during writing
//              WBSL_SUCC       Packet was received and written to memory
// *************************************************************************************************
u8 wbsl_receivePackets(void){
    u8 crcOk = 0;
    u16 tempCurrentPacket = 0;
    u8 retStatus = WBSL_LINK_FAIL;

    // Increment the number of retries to send the ACK
    wbsl_number_of_retries++;

    // Check if too many retries for one packet have been already made
    if (wbsl_number_of_retries >= WBSL_MAXIMUM_RETRIES)
    {
        // Trigger the stop of the WBSL Update procedure
        wbsl_flag = WBSL_TRIGGER_STOP;
        return retStatus;
    }
    // Activate RX Mode to listen to new packets
    ReceiveOn();

    // Set the timeout so we don't get stuck waiting for a packet
    wbsl_setTimer(TIMEOUT_FOR_ACK);

    // Wait for a packet to be received or timeout
    while (!(TA0CCTL1 & CCIFG) && rxtx_flag != WBSL_RXTX_RECEIVED)
    {
        // Check the SYNC Word detected flag
        if (RF1AIFG & BIT9)
            RadioIsr_wbsl();

        //Here we should sleep to wait for either Timeout or Radio
        //_BIS_SR(LPM0_bits);
    }

    // Reset the timer so that next time it starts from fresh state.
    wbsl_resetTimer();
    // Turn radio off during the decoding of the package
    ReceiveOff();

    // If the was no packet received, then a TIMEOUT happened, we need to send
    // an ack for previous pkt, but just in the case it was a regular packet and not a init Packet
    if (rxtx_flag != WBSL_RXTX_RECEIVED && initOk)
    {
        //Reply Positive ACK for previous packet, in case AP didn't receive last ACK
        wbsl_replyAck(WBSL_LINK_SUCC, currentPacket - 1);
    }
    else if (rxtx_flag == WBSL_RXTX_RECEIVED)
    {
        // Clear RX Flag
        rxtx_flag = 0;

        crcOk = RxBuffer[RxBuffer[0] + WBSL_CRC_STATUS_OFFSET] & CRC_STATUS; //CRC status is
                                                                             // appended at the end
                                                                             // of RXFIFO

        if (                                                                 //RxBuffer[ED_ADDRESS_OFFSET_RX]
                                                                             // == WBSL_ED_ADDRESS
                                                                             // &&  // Address check
                                                                             // is now done in
                                                                             // hardware
            RxBuffer[AP_ADDRESS_OFFSET_RX] == AP_address)                    // Check that the
                                                                             // packet comes from
                                                                             // the AP we're linked
                                                                             // to
        {
            // If we're not initialized for communication, this should be an Init packet, treat it
            // as such
            if (!initOk)
            {
                if (crcOk)
                {
                    // Reset the retry counter for ACKs
                    wbsl_number_of_retries = 0;
                    // Total number of packets to be sent during the update process
                    totalPackets = RxBuffer[3] + (RxBuffer[4] << 8);

                    //Reply Positive ACK
                    wbsl_replyAck(WBSL_LINK_SUCC, 0xFFFF);

                    initOk = 1;
                }
                else
                {
                    // CRC ERROR Reply Negative ACK
                    wbsl_replyAck(WBSL_LINK_FAIL, 0xFFFF);
                }
            }
            else
            {
                // Delay to let AP Dongle be in RX Mode
                // Set the timeout
                //wbsl_setTimer(CONV_US_TO_TICKS(500));
                // Wait for a packet to be received or timeout
                //while(!(TA0CCTL1 & CCIFG));
                // Reset the timer
                //wbsl_resetTimer();

                if (crcOk)
                {
                    tempCurrentPacket = RxBuffer[4] + (RxBuffer[3] << 8 & 0xFF00);                    //
                                                                                                      // Keep
                                                                                                      // track
                                                                                                      // of
                                                                                                      // which
                                                                                                      // packet
                                                                                                      // is
                                                                                                      // being
                                                                                                      // sent

                    //Check that we are receiving the next packet needed
                    if (tempCurrentPacket == currentPacket)
                    {
                        if (wbsl_writePacket() == WBSL_OPERATION_SUCC)
                        {
                            // Reset the retry counter for ACKs
                            wbsl_number_of_retries = 0;
                            //Reply Positive ACK
                            wbsl_replyAck(WBSL_LINK_SUCC, currentPacket);
                            //Increment the currentPacket to keep track of which packets we have
                            // received
                            currentPacket++;
                            wbsl_progress = (currentPacket * 100) / totalPackets;
                            retStatus = WBSL_LINK_SUCC;
                        }
                        else
                        {
                            // Write Error ask AP to send package again.
                            wbsl_replyAck(WBSL_LINK_FAIL, currentPacket);
                        }
                    }
                    else
                    {
                        //There is an error on the flow of packets DEAL with it
                        //Reply Positive ACK for previous packet, in case AP didn't receive last ACK
                        wbsl_replyAck(WBSL_LINK_SUCC, currentPacket - 1);
                    }
                }
                else
                {
                    // CRC Error reply with a NACK
                    wbsl_replyAck(WBSL_LINK_FAIL, currentPacket);
                }
            }
        }
    }

    return retStatus;
}

// *************************************************************************************************
// @fn          RadioIsr_wbsl
// @brief       Retrieves the packet waiting on the RXFIFO of the Radio
// @param       none
// @return      none
// *************************************************************************************************
void RadioIsr_wbsl(void)
{
    u8 rxBytes;
    u8 tL;
    u8 *tmpRxBuffer;
    u16 coreIntSource = RF1AIV;

    // Clear RX Interrupt Flag
    RF1AIFG &= ~BIT9;

    // We should only be here in RX mode, not in TX mode, nor if RX mode was turned on during CCA
    if (wbslMode_flag != WBSL_RX_MODE)
    {
        return;
    }

    // Clean Buffer to help protect against spurios frames
    memset(RxBuffer, 0x00, sizeof(RxBuffer));

    // Use a pointer to move through the Buffer
    tmpRxBuffer = RxBuffer;

    // Read the number of bytes ready on the FIFO
    rxBytes = ReadSingleReg(RXBYTES);

    do {
        tL = rxBytes;
        rxBytes = ReadSingleReg(RXBYTES);
    } while (tL != rxBytes);                 // Due to a chip bug, the RXBYTES has to read the same
                                             // value twice for it to be correct

    if (rxBytes == 0)                        // Check if the RX FIFO is empty, this may happen if
                                             // address check is enabled and the FIFO is flushed
                                             // when address doesn't match
    {
        return;
    }

    *tmpRxBuffer++ = ReadSingleReg(RXFIFO);
    RxBufferLength = *(tmpRxBuffer - 1) + 2; // Add 2 for the status bytes which are appended by the
                                             // Radio


    // Check if number of bytes in Fifo exceed the FIFO Size, if so, assume FIFO overflow due to
    // something
    // gone wrong with radio, and the only way to fix it, is to force IDLE mode and then back to RX
    // Mode
    if (rxBytes > MAX_RXFIFO_SIZE)
    {
        ReceiveOff();
        ReceiveOn();
        return;
    }
    // else: everything matches continue

    //Copy Rest of packet
    while (RxBufferLength > 1){

        rxBytes = ReadSingleReg(RXBYTES);

        do {
            tL = rxBytes;
            rxBytes = ReadSingleReg(RXBYTES);
        } while (tL != rxBytes); // Due to a chip bug, the RXBYTES has to read the same value twice
                                 // for it to be correct

        while (rxBytes > 1)
        {
            *tmpRxBuffer++ = ReadSingleReg(RXFIFO);
            RxBufferLength--; rxBytes--;
        }
    }
    *tmpRxBuffer++ = ReadSingleReg(RXFIFO);

    // Signal main program that packet has been received and ready in RxBuffer
    rxtx_flag = WBSL_RXTX_RECEIVED;
}

// *************************************************************************************************
// @fn          massEraseFlash
// @brief       Erases the main memory flash of the watch
// @param       none
// @return      none
// *************************************************************************************************
#ifdef RAM_BASED_RFBSL
void massEraseFlash(void)
{
    volatile char *Flash_ptr;                           //Flash pointer

    while (FCTL3 & BUSY) ;
    FCTL3 = FWKEY;
    while (FCTL3 & BUSY) ;
    Flash_ptr = (char *)INTERRUPT_VECTOR_START_ADDRESS; //Interrupt Vector Start
    FCTL1 = FWKEY + MERAS + ERASE;                      // Set Mass Erase Bits
    *Flash_ptr = 0;
    while (FCTL3 & BUSY) ;
    FCTL3 = FWKEY +  LOCK;
}

#endif // RAM_BASED_RFBSL

// *************************************************************************************************
// @fn          init_watch
// @brief       Inits basic options in the watch and depending of the type of RFBSL
//              being compiled it choses which components to initialize
// @param       none
// @return      none
// *************************************************************************************************
void init_watch(void){


    // Initialize Clocks
    // ---------------------------------------------------------------------
    // Enable 32kHz ACLK
    P5SEL |= 0x03;                     // Select XIN, XOUT on P5.0 and P5.1
    UCSCTL6 &= ~(XT1OFF + XT1DRIVE_3); // XT1 On, Lowest drive strength
    UCSCTL6 |= XCAP_3;                 // Internal load cap

    UCSCTL3 = SELA__XT1CLK;            // Select XT1 as FLL reference
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;
    // ---------------------------------------------------------------------
    // Configure CPU clock for 12MHz
    _BIS_SR(SCG0);                     // Disable the FLL control loop
    UCSCTL0 = 0x0000;                  // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;               // Select suitable range
    UCSCTL2 = FLLD_1 + 0x16E;          // Set DCO Multiplier
    _BIC_SR(SCG0);                     // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 8 MHz / 32,768 Hz = 250000 = MCLK cycles for DCO to settle
    __delay_cycles(250000);

    // Loop until XT1 & DCO stabilizes, use do-while to insure that
    // body is executed at least once
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;             // Clear fault flags
    } while ((SFRIFG1 & OFIFG));

#ifdef RAM_BASED_RFBSL
    lcd_init();
#endif

}

// *************************************************************************************************
// @fn          display_wbsl
// @brief       WBSL display routine.
// @param       u8 line			LINE2
//	        u8 update		DISPLAY_LINE_UPDATE_FULL
// @return      none
// *************************************************************************************************
#ifdef RAM_BASED_RFBSL
void display_wbsl(u8 line, u8 update)
{
    display_chars(LINE2, (u8 *)" RFBSL");
}

#endif

// *************************************************************************************************
// @fn          reset_wbsl
// @brief       Reset WBSL data.
// @param       none
// @return      none
// *************************************************************************************************
void reset_wbsl(void)
{
    // No connection
    sRFwbsl.mode = WBSL_OFF;

    //Reset the progress variables
    wbsl_progress = 0;
    currentPacket = 0;
    totalPackets = 0;

    // Reset the number of retries to send the ACK
    wbsl_number_of_retries = 0;

    // Clear init flag to send the total number of packets on the first transmission packet
    initOk = 0;
    RF1AIFG = 0;   // Clear Radio Flags
}

// *************************************************************************************************
// @fn          wbsl_resetTimer
// @brief       Resets the timer TA0 and stops it
// @param       none
// @return      none
// *************************************************************************************************
void wbsl_resetTimer(void){

    // Reset IRQ flag
    TA0CCTL1 &= ~CCIFG;

    // Stop Timer0
    TA0CTL &= ~(MC1 + MC0);

    // Set Timer0 count register to 0x0000
    TA0R = 0;
}

// *************************************************************************************************
// @fn          wbsl_setTimer
// @brief       Set the timer for the Packet timeouts the timeout param is in ticks 1 tick = 1 /
// 32768 sec
// @param       u16 ticks   Value in ticks (1 tick = 1 / 32768 secs) that wants the timeout to run
// must
//                          be less than 32768 if greater than 32768 it is set to 32767
// @return      none
// *************************************************************************************************
void wbsl_setTimer(u16 ticks)
{

    if (ticks > 32767)
        ticks = 32767;

    // Update CCR
    TA0CCR1 = ticks;

    // Reset IRQ flag
    TA0CCTL1 &= ~CCIFG;

    // Clear and start timer now
    // Continuous mode: Count to 0xFFFF and restart from 0 again - 1sec timing will be generated by
    // ISR
    TA0CTL   |= TASSEL0 + MC1 + TACLR;

}

