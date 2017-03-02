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

#ifndef RADIO_H_
#define RADIO_H_

#define SMARTRF_RADIO_CC430

#define SMARTRF_SETTING_FSCTRL1    0x0C //
#define SMARTRF_SETTING_FSCTRL0    0x00 //


#ifdef ISM_EU
/*   #define SMARTRF_SETTING_FREQ2      0x21 // 868 Mhz
   #define SMARTRF_SETTING_FREQ1      0x62 //
   #define SMARTRF_SETTING_FREQ0      0x76 //
   #define SMARTRF_SETTING_CHANNR     0    //
   #define WBSL_SETTING_PATABLE       0x8C //
*/
   #define SMARTRF_SETTING_FREQ2      0x21 // 869.525 Mhz
   #define SMARTRF_SETTING_FREQ1      0x71 //
   #define SMARTRF_SETTING_FREQ0      0x7A //
   #define SMARTRF_SETTING_CHANNR     0    //
   #define WBSL_SETTING_PATABLE       0xC0 //

#else
  #ifdef ISM_US  
/*   #define SMARTRF_SETTING_FREQ2      0x23 // 915 Mhz
   #define SMARTRF_SETTING_FREQ1      0x31 //
   #define SMARTRF_SETTING_FREQ0      0x3B //
   #define SMARTRF_SETTING_CHANNR     20   //
   #define WBSL_SETTING_PATABLE       0x8B //
*/
   #define SMARTRF_SETTING_FREQ2      0x22 // 902 Mhz --> 906 MHz (CHANNR = 20)
   #define SMARTRF_SETTING_FREQ1      0xB1 //
   #define SMARTRF_SETTING_FREQ0      0x3B //
   #define SMARTRF_SETTING_CHANNR     20   //
   #define WBSL_SETTING_PATABLE       0xC0 //

  #else
    #ifdef ISM_LF
	   // 433.92MHz
	    #define SMARTRF_SETTING_FREQ2      0x10
	    #define SMARTRF_SETTING_FREQ1      0xB0
	    #define SMARTRF_SETTING_FREQ0      0x71
            #define SMARTRF_SETTING_CHANNR     0 
            #define WBSL_SETTING_PATABLE       0xC0

      #else
      #ifdef RAM_BASED_RFBSL  // We don't need these values on the RAM_BASED_RFBSL, but need to define STUBS to make it compile
        #define SMARTRF_SETTING_FREQ2      0x00 //
        #define SMARTRF_SETTING_FREQ1      0x00 //
        #define SMARTRF_SETTING_FREQ0      0x00 //
        #define SMARTRF_SETTING_CHANNR     0x00 //
        #define WBSL_SETTING_PATABLE       0x00 //
      #else
        #error "Wrong ISM band specified (valid are ISM_LF, ISM_EU and ISM_US)"
      #endif // RAM_BASED_RFBSL
    #endif // ISM_LF
  #endif // ISM_US
#endif // ISM_EU


#define SMARTRF_SETTING_MDMCFG4    0x2D //
#define SMARTRF_SETTING_MDMCFG3    0x3B //
#define SMARTRF_SETTING_MDMCFG2    0x13 //
#define SMARTRF_SETTING_MDMCFG1    0x22 //
#define SMARTRF_SETTING_MDMCFG0    0xF8 //



#define SMARTRF_SETTING_DEVIATN    0x62 //
#define SMARTRF_SETTING_FREND1     0xB6 // 
#define SMARTRF_SETTING_FREND0     0x10 //
#define SMARTRF_SETTING_MCSM0      0x18 // 
#define SMARTRF_SETTING_MCSM1      0x3C // 
#define SMARTRF_SETTING_MCSM2      0x07 //
#define SMARTRF_SETTING_WOREVT1    0x87 //
#define SMARTRF_SETTING_WOREVT0    0x6B //  
#define SMARTRF_SETTING_WORCTL     0xF8  /* Note: WOR not supported on CC430 */   
#define SMARTRF_SETTING_FOCCFG     0x1D //
#define SMARTRF_SETTING_BSCFG      0x1C //
#define SMARTRF_SETTING_AGCCTRL2   0xC7 //
#define SMARTRF_SETTING_AGCCTRL1   0x00 //
#define SMARTRF_SETTING_AGCCTRL0   0xB0 //
#define SMARTRF_SETTING_FSCAL3     0xEA //
#define SMARTRF_SETTING_FSCAL2     0x2A //
#define SMARTRF_SETTING_FSCAL1     0x00 //
#define SMARTRF_SETTING_FSCAL0     0x1F //
#define SMARTRF_SETTING_FSTEST     0x59 //
#define SMARTRF_SETTING_TEST2      0x88 //
#define SMARTRF_SETTING_TEST1      0x31 //
#define SMARTRF_SETTING_TEST0      0x09 //
#define SMARTRF_SETTING_PTEST      0x7F //
#define SMARTRF_SETTING_AGCTEST    0x88 //
#define SMARTRF_SETTING_FIFOTHR    0x07 //
#define SMARTRF_SETTING_IOCFG2     0x29 //
#define SMARTRF_SETTING_IOCFG1     0x1E //
#define SMARTRF_SETTING_IOCFG0     0x1B //
#define SMARTRF_SETTING_PKTCTRL1   0x05 // Address Check and Append Status
#define SMARTRF_SETTING_PKTCTRL0   0x45 //
#define SMARTRF_SETTING_ADDR       WBSL_ED_ADDRESS //
#define SMARTRF_SETTING_PKTLEN     0xFE //


extern void radio_reset(void);
extern void radio_sxoff(void);
extern void open_radio(void);
extern void close_radio(void);


void config_radio_wbsl(void);
void stop_radio_wbsl(void);
void WriteSmartRFReg(const u8 SmartRFSetting[][2], u8 size);
void ReceiveOn(void);
void ReceiveOff(void);

#endif /*RADIO_H_*/
