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
// Radio core access functions. Taken from TI reference code for CC430.
// *************************************************************************************************

// system
#include "project_defs.h"

// driver
#include "rf1a.h"
#include "radio.h"

// logic
#include "wbsl.h"


//RF Test Values
const u8 RF1A_REG_SMARTRF_SETTING[][2] =
{
    /* internal radio configuration */
    {  IOCFG0,    SMARTRF_SETTING_IOCFG0    }, /* Configure GDO_0 to output PA_PD signal (low during
                                                *TX, high otherwise). */
    {  IOCFG1,    SMARTRF_SETTING_IOCFG1    }, /* Configure GDO_1 to output RSSI_VALID signal (high
                                                *when RSSI is valid, low otherwise). */
    {  IOCFG2,    SMARTRF_SETTING_IOCFG2    },
    {  MCSM2,     SMARTRF_SETTING_MCSM2     },
    {  MCSM1,     SMARTRF_SETTING_MCSM1     }, /* CCA mode, RX_OFF_MODE and TX_OFF_MODE */
    {  MCSM0,     SMARTRF_SETTING_MCSM0     }, /* AUTO_CAL and XOSC state in sleep */
    {  SYNC1,     0xD3                      },
    {  SYNC0,     0x91                      },
    {  PKTLEN,    SMARTRF_SETTING_PKTLEN    },
    {  PKTCTRL1,  SMARTRF_SETTING_PKTCTRL1  },
    {  PKTCTRL0,  SMARTRF_SETTING_PKTCTRL0  },
    {  ADDR,      SMARTRF_SETTING_ADDR      },
    {  FIFOTHR,   SMARTRF_SETTING_FIFOTHR   },
    {  WOREVT1,   SMARTRF_SETTING_WOREVT1   },
    {  WOREVT0,   SMARTRF_SETTING_WOREVT0   },
    {  WORCTRL,    SMARTRF_SETTING_WORCTL    },
    /* imported SmartRF radio configuration */
    {  CHANNR,    SMARTRF_SETTING_CHANNR    },
    {  FSCTRL1,   SMARTRF_SETTING_FSCTRL1   },
    {  FSCTRL0,   SMARTRF_SETTING_FSCTRL0   },
    {  FREQ2,     SMARTRF_SETTING_FREQ2     },
    {  FREQ1,     SMARTRF_SETTING_FREQ1     },
    {  FREQ0,     SMARTRF_SETTING_FREQ0     },
    {  MDMCFG4,   SMARTRF_SETTING_MDMCFG4   },
    {  MDMCFG3,   SMARTRF_SETTING_MDMCFG3   },
    {  MDMCFG2,   SMARTRF_SETTING_MDMCFG2   },
    {  MDMCFG1,   SMARTRF_SETTING_MDMCFG1   },
    {  MDMCFG0,   SMARTRF_SETTING_MDMCFG0   },
    {  DEVIATN,   SMARTRF_SETTING_DEVIATN   },

    {  FOCCFG,    SMARTRF_SETTING_FOCCFG    },
    {  BSCFG,     SMARTRF_SETTING_BSCFG     },
    {  AGCCTRL2,  SMARTRF_SETTING_AGCCTRL2  },
    {  AGCCTRL1,  SMARTRF_SETTING_AGCCTRL1  },
    {  AGCCTRL0,  SMARTRF_SETTING_AGCCTRL0  },
    {  FREND1,    SMARTRF_SETTING_FREND1    },
    {  FREND0,    SMARTRF_SETTING_FREND0    },
    {  FSCAL3,    SMARTRF_SETTING_FSCAL3    },
    {  FSCAL2,    SMARTRF_SETTING_FSCAL2    },
    {  FSCAL1,    SMARTRF_SETTING_FSCAL1    },
    {  FSCAL0,    SMARTRF_SETTING_FSCAL0    },
    {  AGCTEST,   SMARTRF_SETTING_AGCTEST   },
    {  PTEST,     SMARTRF_SETTING_PTEST     },
    {  FSTEST,    SMARTRF_SETTING_FSTEST    },
    {  TEST2,     SMARTRF_SETTING_TEST2     },
    {  TEST1,     SMARTRF_SETTING_TEST1     },
    {  TEST0,     SMARTRF_SETTING_TEST0     },
};
// *************************************************************************************************
// @fn          radio_reset
// @brief       Reset radio core.
// @param       none
// @return      none
// *************************************************************************************************
void radio_reset(void)
{
    volatile u16 i;
    u8 x;

    // Clear any radio pending interrupt
    RF1AIFG = 0;

    // Reset radio core
    Strobe(RF_SRES);
    // Wait before checking IDLE
    for (i = 0; i < 100; i++) ;

    do {
        x = Strobe(RF_SIDLE);
    } while ((x & 0x70) != 0x00);

    // Clear radio error register
    RF1AIFERR = 0;
}

// *************************************************************************************************
// @fn          WriteSmartRFReg
// @brief       Writes all the Radio Settings to the Radio module
// @param       none
// @return      none
// *************************************************************************************************
void WriteSmartRFReg(const u8 SmartRFSetting[][2], u8 size)
{
    u8 i;

    for (i = 0; i < (size); i++)
        WriteSingleReg(SmartRFSetting[i][0], SmartRFSetting[i][1]);
}

// *************************************************************************************************
// @fn          config_radio_wbsl
// @brief       Configure the RF Module to start the WBSL Process
// @param       none
// @return      none
// *************************************************************************************************
void config_radio_wbsl(void)
{
    WritePATable(WBSL_SETTING_PATABLE);
    WriteSmartRFReg(RF1A_REG_SMARTRF_SETTING, sizeof(RF1A_REG_SMARTRF_SETTING) / 2);
}

// *************************************************************************************************
// @fn          ReceiveOn
// @brief       Put Radio in RX Mode
// @param       none
// @return      none
// *************************************************************************************************
void ReceiveOn(void)
{
    RF1AIFG = 0; //&= ~BIT9;                         // Clear a pending interrupt
    // Update Mode Flag
    wbslMode_flag = WBSL_RX_MODE;
    Strobe(RF_SRX);
}

// *************************************************************************************************
// @fn          ReceiveOff
// @brief       Put Radio in Idle Mode
// @param       none
// @return      none
// *************************************************************************************************
void ReceiveOff(void)
{
    u8 x;

    // Update Mode Flag
    wbslMode_flag = WBSL_IDLE_MODE;
    do {
        x = Strobe(RF_SIDLE);
    } while (x & 0x70);
    /* Wait for XOSC to be stable and radio in IDLE state */

    Strobe(RF_SFRX);      /* Flush the receive FIFO of any residual data */

    RF1AIFG = 0;          // Clear pending IFG
}

