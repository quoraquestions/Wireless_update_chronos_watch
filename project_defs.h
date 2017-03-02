#ifndef PROJECT_DEFS_H_
#define PROJECT_DEFS_H_

#include <cc430x613x.h>

#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long int

// Conversion from usec to ACLK timer ticks
#define CONV_US_TO_TICKS(usec)                          (((usec) * 32768) / 1000000)

// Conversion from msec to ACLK timer ticks
#define CONV_MS_TO_TICKS(msec)                          (((msec) * 32768) / 1000)

#define st(x)                                           do { x } while (__LINE__ == -1)
#define ENTER_CRITICAL_SECTION(x)                       st( \
        x = __get_interrupt_state(); __disable_interrupt(); )
#define EXIT_CRITICAL_SECTION(x)                        __set_interrupt_state(x)

#define BV(x)                                           (1 << (x))

// Use CCA to sense the traffic before transmitting
#define CCA


// This Macro is defined of undefined using the active configuration of CCS, if not implemented this
// way define it here
//#define RAM_BASED_RFBSL

// Testing
//#define TESTING_INTERRUPT_CATCHER


#endif /*PROJECT_DEFS_H_*/
