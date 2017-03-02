// *************************************************************************************************
//
// Actual revision: $Revision: $
// Revision label:  $Name: $
// Revision state:  $State: $
//
// *************************************************************************************************
// Radio core access functions. Taken from TI reference code for CC430.
// *************************************************************************************************

// *************************************************************************************************
// Prototype section
u8 Strobe(u8 strobe);
u8 ReadSingleReg(u8 addr);
void WriteSingleReg(u8 addr, u8 value);
void ReadBurstReg(u8 addr, u8 *buffer, u8 count);
void WriteBurstReg(u8 addr, u8 *buffer, u8 count);
void ResetRadioCore(void);
void WritePATable(u8 value);
void WaitForXT2(void);
void Transmit(u8 *buffer, u8 length);
