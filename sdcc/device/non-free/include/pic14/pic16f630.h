/*
 * This declarations of the PIC16F630 MCU.
 *
 * This file is part of the GNU PIC library for SDCC, originally
 * created by Molnar Karoly <molnarkaroly@users.sf.net> 2014.
 *
 * This file is generated automatically by the cinc2h.pl, 2014-09-15 14:38:38 UTC.
 *
 * SDCC is licensed under the GNU Public license (GPL) v2. Note that
 * this license covers the code to the compiler and other executables,
 * but explicitly does not cover any code or objects generated by sdcc.
 *
 * For pic device libraries and header files which are derived from
 * Microchip header (.inc) and linker script (.lkr) files Microchip
 * requires that "The header files should state that they are only to be
 * used with authentic Microchip devices" which makes them incompatible
 * with the GPL. Pic device libraries and header files are located at
 * non-free/lib and non-free/include directories respectively.
 * Sdcc should be run with the --use-non-free command line option in
 * order to include non-free header files and libraries.
 *
 * See http://sdcc.sourceforge.net/ for the latest information on sdcc.
 */

#ifndef __PIC16F630_H__
#define __PIC16F630_H__

//==============================================================================
//
//	Register Addresses
//
//==============================================================================

#ifndef NO_ADDR_DEFINES

#define INDF_ADDR               0x0000
#define TMR0_ADDR               0x0001
#define PCL_ADDR                0x0002
#define STATUS_ADDR             0x0003
#define FSR_ADDR                0x0004
#define PORTA_ADDR              0x0005
#define PORTC_ADDR              0x0007
#define PCLATH_ADDR             0x000A
#define INTCON_ADDR             0x000B
#define PIR1_ADDR               0x000C
#define TMR1_ADDR               0x000E
#define TMR1L_ADDR              0x000E
#define TMR1H_ADDR              0x000F
#define T1CON_ADDR              0x0010
#define CMCON_ADDR              0x0019
#define OPTION_REG_ADDR         0x0081
#define TRISA_ADDR              0x0085
#define TRISC_ADDR              0x0087
#define PIE1_ADDR               0x008C
#define PCON_ADDR               0x008E
#define OSCCAL_ADDR             0x0090
#define WPU_ADDR                0x0095
#define WPUA_ADDR               0x0095
#define IOC_ADDR                0x0096
#define IOCA_ADDR               0x0096
#define VRCON_ADDR              0x0099
#define EEDAT_ADDR              0x009A
#define EEDATA_ADDR             0x009A
#define EEADR_ADDR              0x009B
#define EECON1_ADDR             0x009C
#define EECON2_ADDR             0x009D

#endif // #ifndef NO_ADDR_DEFINES

//==============================================================================
//
//	Register Definitions
//
//==============================================================================

extern __at(0x0000) __sfr INDF;
extern __at(0x0001) __sfr TMR0;
extern __at(0x0002) __sfr PCL;

//==============================================================================
//        STATUS Bits

extern __at(0x0003) __sfr STATUS;

typedef union
  {
  struct
    {
    unsigned C                  : 1;
    unsigned DC                 : 1;
    unsigned Z                  : 1;
    unsigned NOT_PD             : 1;
    unsigned NOT_TO             : 1;
    unsigned RP0                : 1;
    unsigned RP1                : 1;
    unsigned IRP                : 1;
    };

  struct
    {
    unsigned                    : 5;
    unsigned RP                 : 2;
    unsigned                    : 1;
    };
  } __STATUSbits_t;

extern __at(0x0003) volatile __STATUSbits_t STATUSbits;

#define _C                      0x01
#define _DC                     0x02
#define _Z                      0x04
#define _NOT_PD                 0x08
#define _NOT_TO                 0x10
#define _RP0                    0x20
#define _RP1                    0x40
#define _IRP                    0x80

//==============================================================================

extern __at(0x0004) __sfr FSR;

//==============================================================================
//        PORTA Bits

extern __at(0x0005) __sfr PORTA;

typedef union
  {
  struct
    {
    unsigned RA0                : 1;
    unsigned RA1                : 1;
    unsigned RA2                : 1;
    unsigned RA3                : 1;
    unsigned RA4                : 1;
    unsigned RA5                : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned RA                 : 6;
    unsigned                    : 2;
    };
  } __PORTAbits_t;

extern __at(0x0005) volatile __PORTAbits_t PORTAbits;

#define _RA0                    0x01
#define _RA1                    0x02
#define _RA2                    0x04
#define _RA3                    0x08
#define _RA4                    0x10
#define _RA5                    0x20

//==============================================================================


//==============================================================================
//        PORTC Bits

extern __at(0x0007) __sfr PORTC;

typedef union
  {
  struct
    {
    unsigned RC0                : 1;
    unsigned RC1                : 1;
    unsigned RC2                : 1;
    unsigned RC3                : 1;
    unsigned RC4                : 1;
    unsigned RC5                : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned RC                 : 6;
    unsigned                    : 2;
    };
  } __PORTCbits_t;

extern __at(0x0007) volatile __PORTCbits_t PORTCbits;

#define _RC0                    0x01
#define _RC1                    0x02
#define _RC2                    0x04
#define _RC3                    0x08
#define _RC4                    0x10
#define _RC5                    0x20

//==============================================================================

extern __at(0x000A) __sfr PCLATH;

//==============================================================================
//        INTCON Bits

extern __at(0x000B) __sfr INTCON;

typedef union
  {
  struct
    {
    unsigned RAIF               : 1;
    unsigned INTF               : 1;
    unsigned T0IF               : 1;
    unsigned RAIE               : 1;
    unsigned INTE               : 1;
    unsigned T0IE               : 1;
    unsigned PEIE               : 1;
    unsigned GIE                : 1;
    };

  struct
    {
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned TMR0IF             : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned TMR0IE             : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };
  } __INTCONbits_t;

extern __at(0x000B) volatile __INTCONbits_t INTCONbits;

#define _RAIF                   0x01
#define _INTF                   0x02
#define _T0IF                   0x04
#define _TMR0IF                 0x04
#define _RAIE                   0x08
#define _INTE                   0x10
#define _T0IE                   0x20
#define _TMR0IE                 0x20
#define _PEIE                   0x40
#define _GIE                    0x80

//==============================================================================


//==============================================================================
//        PIR1 Bits

extern __at(0x000C) __sfr PIR1;

typedef union
  {
  struct
    {
    unsigned TMR1IF             : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned CMIF               : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned EEIF               : 1;
    };

  struct
    {
    unsigned T1IF               : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };
  } __PIR1bits_t;

extern __at(0x000C) volatile __PIR1bits_t PIR1bits;

#define _TMR1IF                 0x01
#define _T1IF                   0x01
#define _CMIF                   0x08
#define _EEIF                   0x80

//==============================================================================

extern __at(0x000E) __sfr TMR1;
extern __at(0x000E) __sfr TMR1L;
extern __at(0x000F) __sfr TMR1H;

//==============================================================================
//        T1CON Bits

extern __at(0x0010) __sfr T1CON;

typedef union
  {
  struct
    {
    unsigned TMR1ON             : 1;
    unsigned TMR1CS             : 1;
    unsigned NOT_T1SYNC         : 1;
    unsigned T1OSCEN            : 1;
    unsigned T1CKPS0            : 1;
    unsigned T1CKPS1            : 1;
    unsigned TMR1GE             : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned                    : 4;
    unsigned T1CKPS             : 2;
    unsigned                    : 2;
    };
  } __T1CONbits_t;

extern __at(0x0010) volatile __T1CONbits_t T1CONbits;

#define _TMR1ON                 0x01
#define _TMR1CS                 0x02
#define _NOT_T1SYNC             0x04
#define _T1OSCEN                0x08
#define _T1CKPS0                0x10
#define _T1CKPS1                0x20
#define _TMR1GE                 0x40

//==============================================================================


//==============================================================================
//        CMCON Bits

extern __at(0x0019) __sfr CMCON;

typedef union
  {
  struct
    {
    unsigned CM0                : 1;
    unsigned CM1                : 1;
    unsigned CM2                : 1;
    unsigned CIS                : 1;
    unsigned CINV               : 1;
    unsigned                    : 1;
    unsigned COUT               : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned CM                 : 3;
    unsigned                    : 5;
    };
  } __CMCONbits_t;

extern __at(0x0019) volatile __CMCONbits_t CMCONbits;

#define _CM0                    0x01
#define _CM1                    0x02
#define _CM2                    0x04
#define _CIS                    0x08
#define _CINV                   0x10
#define _COUT                   0x40

//==============================================================================


//==============================================================================
//        OPTION_REG Bits

extern __at(0x0081) __sfr OPTION_REG;

typedef union
  {
  struct
    {
    unsigned PS0                : 1;
    unsigned PS1                : 1;
    unsigned PS2                : 1;
    unsigned PSA                : 1;
    unsigned T0SE               : 1;
    unsigned T0CS               : 1;
    unsigned INTEDG             : 1;
    unsigned NOT_RAPU           : 1;
    };

  struct
    {
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned NOT_GPPU           : 1;
    };

  struct
    {
    unsigned PS                 : 3;
    unsigned                    : 5;
    };
  } __OPTION_REGbits_t;

extern __at(0x0081) volatile __OPTION_REGbits_t OPTION_REGbits;

#define _PS0                    0x01
#define _PS1                    0x02
#define _PS2                    0x04
#define _PSA                    0x08
#define _T0SE                   0x10
#define _T0CS                   0x20
#define _INTEDG                 0x40
#define _NOT_RAPU               0x80
#define _NOT_GPPU               0x80

//==============================================================================


//==============================================================================
//        TRISA Bits

extern __at(0x0085) __sfr TRISA;

typedef union
  {
  struct
    {
    unsigned TRISA0             : 1;
    unsigned TRISA1             : 1;
    unsigned TRISA2             : 1;
    unsigned TRISA3             : 1;
    unsigned TRISA4             : 1;
    unsigned TRISA5             : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned TRISA              : 6;
    unsigned                    : 2;
    };
  } __TRISAbits_t;

extern __at(0x0085) volatile __TRISAbits_t TRISAbits;

#define _TRISA0                 0x01
#define _TRISA1                 0x02
#define _TRISA2                 0x04
#define _TRISA3                 0x08
#define _TRISA4                 0x10
#define _TRISA5                 0x20

//==============================================================================


//==============================================================================
//        TRISC Bits

extern __at(0x0087) __sfr TRISC;

typedef union
  {
  struct
    {
    unsigned TRISC0             : 1;
    unsigned TRISC1             : 1;
    unsigned TRISC2             : 1;
    unsigned TRISC3             : 1;
    unsigned TRISC4             : 1;
    unsigned TRISC5             : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned TRISC              : 6;
    unsigned                    : 2;
    };
  } __TRISCbits_t;

extern __at(0x0087) volatile __TRISCbits_t TRISCbits;

#define _TRISC0                 0x01
#define _TRISC1                 0x02
#define _TRISC2                 0x04
#define _TRISC3                 0x08
#define _TRISC4                 0x10
#define _TRISC5                 0x20

//==============================================================================


//==============================================================================
//        PIE1 Bits

extern __at(0x008C) __sfr PIE1;

typedef union
  {
  struct
    {
    unsigned TMR1IE             : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned CMIE               : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned EEIE               : 1;
    };

  struct
    {
    unsigned T1IE               : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };
  } __PIE1bits_t;

extern __at(0x008C) volatile __PIE1bits_t PIE1bits;

#define _TMR1IE                 0x01
#define _T1IE                   0x01
#define _CMIE                   0x08
#define _EEIE                   0x80

//==============================================================================


//==============================================================================
//        PCON Bits

extern __at(0x008E) __sfr PCON;

typedef union
  {
  struct
    {
    unsigned NOT_BOR            : 1;
    unsigned NOT_POR            : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned NOT_BOD            : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };
  } __PCONbits_t;

extern __at(0x008E) volatile __PCONbits_t PCONbits;

#define _NOT_BOR                0x01
#define _NOT_BOD                0x01
#define _NOT_POR                0x02

//==============================================================================


//==============================================================================
//        OSCCAL Bits

extern __at(0x0090) __sfr OSCCAL;

typedef union
  {
  struct
    {
    unsigned                    : 1;
    unsigned                    : 1;
    unsigned CAL0               : 1;
    unsigned CAL1               : 1;
    unsigned CAL2               : 1;
    unsigned CAL3               : 1;
    unsigned CAL4               : 1;
    unsigned CAL5               : 1;
    };

  struct
    {
    unsigned                    : 2;
    unsigned CAL                : 6;
    };
  } __OSCCALbits_t;

extern __at(0x0090) volatile __OSCCALbits_t OSCCALbits;

#define _CAL0                   0x04
#define _CAL1                   0x08
#define _CAL2                   0x10
#define _CAL3                   0x20
#define _CAL4                   0x40
#define _CAL5                   0x80

//==============================================================================


//==============================================================================
//        WPU Bits

extern __at(0x0095) __sfr WPU;

typedef struct
  {
  unsigned WPUA0                : 1;
  unsigned WPUA1                : 1;
  unsigned WPUA2                : 1;
  unsigned                      : 1;
  unsigned WPUA4                : 1;
  unsigned WPUA5                : 1;
  unsigned                      : 1;
  unsigned                      : 1;
  } __WPUbits_t;

extern __at(0x0095) volatile __WPUbits_t WPUbits;

#define _WPUA0                  0x01
#define _WPUA1                  0x02
#define _WPUA2                  0x04
#define _WPUA4                  0x10
#define _WPUA5                  0x20

//==============================================================================


//==============================================================================
//        WPUA Bits

extern __at(0x0095) __sfr WPUA;

typedef struct
  {
  unsigned WPUA0                : 1;
  unsigned WPUA1                : 1;
  unsigned WPUA2                : 1;
  unsigned                      : 1;
  unsigned WPUA4                : 1;
  unsigned WPUA5                : 1;
  unsigned                      : 1;
  unsigned                      : 1;
  } __WPUAbits_t;

extern __at(0x0095) volatile __WPUAbits_t WPUAbits;

#define _WPUA_WPUA0             0x01
#define _WPUA_WPUA1             0x02
#define _WPUA_WPUA2             0x04
#define _WPUA_WPUA4             0x10
#define _WPUA_WPUA5             0x20

//==============================================================================


//==============================================================================
//        IOC Bits

extern __at(0x0096) __sfr IOC;

typedef union
  {
  struct
    {
    unsigned IOCA0              : 1;
    unsigned IOCA1              : 1;
    unsigned IOCA2              : 1;
    unsigned IOCA3              : 1;
    unsigned IOCA4              : 1;
    unsigned IOCA5              : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned IOCA               : 6;
    unsigned                    : 2;
    };
  } __IOCbits_t;

extern __at(0x0096) volatile __IOCbits_t IOCbits;

#define _IOCA0                  0x01
#define _IOCA1                  0x02
#define _IOCA2                  0x04
#define _IOCA3                  0x08
#define _IOCA4                  0x10
#define _IOCA5                  0x20

//==============================================================================


//==============================================================================
//        IOCA Bits

extern __at(0x0096) __sfr IOCA;

typedef union
  {
  struct
    {
    unsigned IOCA0              : 1;
    unsigned IOCA1              : 1;
    unsigned IOCA2              : 1;
    unsigned IOCA3              : 1;
    unsigned IOCA4              : 1;
    unsigned IOCA5              : 1;
    unsigned                    : 1;
    unsigned                    : 1;
    };

  struct
    {
    unsigned IOCA               : 6;
    unsigned                    : 2;
    };
  } __IOCAbits_t;

extern __at(0x0096) volatile __IOCAbits_t IOCAbits;

#define _IOCA_IOCA0             0x01
#define _IOCA_IOCA1             0x02
#define _IOCA_IOCA2             0x04
#define _IOCA_IOCA3             0x08
#define _IOCA_IOCA4             0x10
#define _IOCA_IOCA5             0x20

//==============================================================================


//==============================================================================
//        VRCON Bits

extern __at(0x0099) __sfr VRCON;

typedef union
  {
  struct
    {
    unsigned VR0                : 1;
    unsigned VR1                : 1;
    unsigned VR2                : 1;
    unsigned VR3                : 1;
    unsigned                    : 1;
    unsigned VRR                : 1;
    unsigned                    : 1;
    unsigned VREN               : 1;
    };

  struct
    {
    unsigned VR                 : 4;
    unsigned                    : 4;
    };
  } __VRCONbits_t;

extern __at(0x0099) volatile __VRCONbits_t VRCONbits;

#define _VR0                    0x01
#define _VR1                    0x02
#define _VR2                    0x04
#define _VR3                    0x08
#define _VRR                    0x20
#define _VREN                   0x80

//==============================================================================

extern __at(0x009A) __sfr EEDAT;
extern __at(0x009A) __sfr EEDATA;
extern __at(0x009B) __sfr EEADR;

//==============================================================================
//        EECON1 Bits

extern __at(0x009C) __sfr EECON1;

typedef struct
  {
  unsigned RD                   : 1;
  unsigned WR                   : 1;
  unsigned WREN                 : 1;
  unsigned WRERR                : 1;
  unsigned                      : 1;
  unsigned                      : 1;
  unsigned                      : 1;
  unsigned                      : 1;
  } __EECON1bits_t;

extern __at(0x009C) volatile __EECON1bits_t EECON1bits;

#define _RD                     0x01
#define _WR                     0x02
#define _WREN                   0x04
#define _WRERR                  0x08

//==============================================================================

extern __at(0x009D) __sfr EECON2;

//==============================================================================
//
//        Configuration Bits
//
//==============================================================================

#define _CONFIG                 0x2007

//----------------------------- CONFIG Options -------------------------------

#define _FOSC_LP                0x3FF8  // LP oscillator: Low power crystal on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN.
#define _LP_OSC                 0x3FF8  // LP oscillator: Low power crystal on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN.
#define _FOSC_XT                0x3FF9  // XT oscillator: Crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN.
#define _XT_OSC                 0x3FF9  // XT oscillator: Crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN.
#define _FOSC_HS                0x3FFA  // HS oscillator: High speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN.
#define _HS_OSC                 0x3FFA  // HS oscillator: High speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN.
#define _FOSC_EC                0x3FFB  // EC: I/O function on RA4/OSC2/CLKOUT pin, CLKIN on RA5/OSC1/CLKIN.
#define _EC_OSC                 0x3FFB  // EC: I/O function on RA4/OSC2/CLKOUT pin, CLKIN on RA5/OSC1/CLKIN.
#define _FOSC_INTRCIO           0x3FFC  // INTOSC oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN.
#define _INTRC_OSC_NOCLKOUT     0x3FFC  // INTOSC oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN.
#define _FOSC_INTRCCLK          0x3FFD  // INTOSC oscillator: CLKOUT function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN.
#define _INTRC_OSC_CLKOUT       0x3FFD  // INTOSC oscillator: CLKOUT function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN.
#define _FOSC_EXTRCIO           0x3FFE  // RC oscillator: I/O function on RA4/OSC2/CLKOUT pin, RC on RA5/OSC1/CLKIN.
#define _EXTRC_OSC_NOCLKOUT     0x3FFE  // RC oscillator: I/O function on RA4/OSC2/CLKOUT pin, RC on RA5/OSC1/CLKIN.
#define _FOSC_EXTRCCLK          0x3FFF  // RC oscillator: CLKOUT function on RA4/OSC2/CLKOUT pin, RC on RA5/OSC1/CLKIN.
#define _EXTRC_OSC_CLKOUT       0x3FFF  // RC oscillator: CLKOUT function on RA4/OSC2/CLKOUT pin, RC on RA5/OSC1/CLKIN.
#define _WDTE_OFF               0x3FF7  // WDT disabled.
#define _WDT_OFF                0x3FF7  // WDT disabled.
#define _WDTE_ON                0x3FFF  // WDT enabled.
#define _WDT_ON                 0x3FFF  // WDT enabled.
#define _PWRTE_ON               0x3FEF  // PWRT enabled.
#define _PWRTE_OFF              0x3FFF  // PWRT disabled.
#define _MCLRE_OFF              0x3FDF  // RA3/MCLR pin function is digital I/O, MCLR internally tied to VDD.
#define _MCLRE_ON               0x3FFF  // RA3/MCLR pin function is MCLR.
#define _BOREN_OFF              0x3FBF  // BOD disabled.
#define _BODEN_OFF              0x3FBF  // BOD disabled.
#define _BOREN_ON               0x3FFF  // BOD enabled.
#define _BODEN                  0x3FFF  // BOD enabled.
#define _CP_ON                  0x3F7F  // Program Memory code protection is enabled.
#define _CP                     0x3F7F  // Program Memory code protection is enabled.
#define _CP_OFF                 0x3FFF  // Program Memory code protection is disabled.
#define _CPD_ON                 0x3EFF  // Data memory code protection is enabled.
#define _CPD                    0x3EFF  // Data memory code protection is enabled.
#define _CPD_OFF                0x3FFF  // Data memory code protection is disabled.

//==============================================================================

#define _DEVID1                 0x2006

#define _IDLOC0                 0x2000
#define _IDLOC1                 0x2001
#define _IDLOC2                 0x2002
#define _IDLOC3                 0x2003

//==============================================================================

#ifndef NO_BIT_DEFINES

#define CM0                     CMCONbits.CM0                   // bit 0
#define CM1                     CMCONbits.CM1                   // bit 1
#define CM2                     CMCONbits.CM2                   // bit 2
#define CIS                     CMCONbits.CIS                   // bit 3
#define CINV                    CMCONbits.CINV                  // bit 4
#define COUT                    CMCONbits.COUT                  // bit 6

#define RD                      EECON1bits.RD                   // bit 0
#define WR                      EECON1bits.WR                   // bit 1
#define WREN                    EECON1bits.WREN                 // bit 2
#define WRERR                   EECON1bits.WRERR                // bit 3

#define RAIF                    INTCONbits.RAIF                 // bit 0
#define INTF                    INTCONbits.INTF                 // bit 1
#define T0IF                    INTCONbits.T0IF                 // bit 2, shadows bit in INTCONbits
#define TMR0IF                  INTCONbits.TMR0IF               // bit 2, shadows bit in INTCONbits
#define RAIE                    INTCONbits.RAIE                 // bit 3
#define INTE                    INTCONbits.INTE                 // bit 4
#define T0IE                    INTCONbits.T0IE                 // bit 5, shadows bit in INTCONbits
#define TMR0IE                  INTCONbits.TMR0IE               // bit 5, shadows bit in INTCONbits
#define PEIE                    INTCONbits.PEIE                 // bit 6
#define GIE                     INTCONbits.GIE                  // bit 7

#define IOCA0                   IOCbits.IOCA0                   // bit 0
#define IOCA1                   IOCbits.IOCA1                   // bit 1
#define IOCA2                   IOCbits.IOCA2                   // bit 2
#define IOCA3                   IOCbits.IOCA3                   // bit 3
#define IOCA4                   IOCbits.IOCA4                   // bit 4
#define IOCA5                   IOCbits.IOCA5                   // bit 5

#define PS0                     OPTION_REGbits.PS0              // bit 0
#define PS1                     OPTION_REGbits.PS1              // bit 1
#define PS2                     OPTION_REGbits.PS2              // bit 2
#define PSA                     OPTION_REGbits.PSA              // bit 3
#define T0SE                    OPTION_REGbits.T0SE             // bit 4
#define T0CS                    OPTION_REGbits.T0CS             // bit 5
#define INTEDG                  OPTION_REGbits.INTEDG           // bit 6
#define NOT_RAPU                OPTION_REGbits.NOT_RAPU         // bit 7, shadows bit in OPTION_REGbits
#define NOT_GPPU                OPTION_REGbits.NOT_GPPU         // bit 7, shadows bit in OPTION_REGbits

#define CAL0                    OSCCALbits.CAL0                 // bit 2
#define CAL1                    OSCCALbits.CAL1                 // bit 3
#define CAL2                    OSCCALbits.CAL2                 // bit 4
#define CAL3                    OSCCALbits.CAL3                 // bit 5
#define CAL4                    OSCCALbits.CAL4                 // bit 6
#define CAL5                    OSCCALbits.CAL5                 // bit 7

#define NOT_BOR                 PCONbits.NOT_BOR                // bit 0, shadows bit in PCONbits
#define NOT_BOD                 PCONbits.NOT_BOD                // bit 0, shadows bit in PCONbits
#define NOT_POR                 PCONbits.NOT_POR                // bit 1

#define TMR1IE                  PIE1bits.TMR1IE                 // bit 0, shadows bit in PIE1bits
#define T1IE                    PIE1bits.T1IE                   // bit 0, shadows bit in PIE1bits
#define CMIE                    PIE1bits.CMIE                   // bit 3
#define EEIE                    PIE1bits.EEIE                   // bit 7

#define TMR1IF                  PIR1bits.TMR1IF                 // bit 0, shadows bit in PIR1bits
#define T1IF                    PIR1bits.T1IF                   // bit 0, shadows bit in PIR1bits
#define CMIF                    PIR1bits.CMIF                   // bit 3
#define EEIF                    PIR1bits.EEIF                   // bit 7

#define RA0                     PORTAbits.RA0                   // bit 0
#define RA1                     PORTAbits.RA1                   // bit 1
#define RA2                     PORTAbits.RA2                   // bit 2
#define RA3                     PORTAbits.RA3                   // bit 3
#define RA4                     PORTAbits.RA4                   // bit 4
#define RA5                     PORTAbits.RA5                   // bit 5

#define RC0                     PORTCbits.RC0                   // bit 0
#define RC1                     PORTCbits.RC1                   // bit 1
#define RC2                     PORTCbits.RC2                   // bit 2
#define RC3                     PORTCbits.RC3                   // bit 3
#define RC4                     PORTCbits.RC4                   // bit 4
#define RC5                     PORTCbits.RC5                   // bit 5

#define C                       STATUSbits.C                    // bit 0
#define DC                      STATUSbits.DC                   // bit 1
#define Z                       STATUSbits.Z                    // bit 2
#define NOT_PD                  STATUSbits.NOT_PD               // bit 3
#define NOT_TO                  STATUSbits.NOT_TO               // bit 4
#define RP0                     STATUSbits.RP0                  // bit 5
#define RP1                     STATUSbits.RP1                  // bit 6
#define IRP                     STATUSbits.IRP                  // bit 7

#define TMR1ON                  T1CONbits.TMR1ON                // bit 0
#define TMR1CS                  T1CONbits.TMR1CS                // bit 1
#define NOT_T1SYNC              T1CONbits.NOT_T1SYNC            // bit 2
#define T1OSCEN                 T1CONbits.T1OSCEN               // bit 3
#define T1CKPS0                 T1CONbits.T1CKPS0               // bit 4
#define T1CKPS1                 T1CONbits.T1CKPS1               // bit 5
#define TMR1GE                  T1CONbits.TMR1GE                // bit 6

#define TRISA0                  TRISAbits.TRISA0                // bit 0
#define TRISA1                  TRISAbits.TRISA1                // bit 1
#define TRISA2                  TRISAbits.TRISA2                // bit 2
#define TRISA3                  TRISAbits.TRISA3                // bit 3
#define TRISA4                  TRISAbits.TRISA4                // bit 4
#define TRISA5                  TRISAbits.TRISA5                // bit 5

#define TRISC0                  TRISCbits.TRISC0                // bit 0
#define TRISC1                  TRISCbits.TRISC1                // bit 1
#define TRISC2                  TRISCbits.TRISC2                // bit 2
#define TRISC3                  TRISCbits.TRISC3                // bit 3
#define TRISC4                  TRISCbits.TRISC4                // bit 4
#define TRISC5                  TRISCbits.TRISC5                // bit 5

#define VR0                     VRCONbits.VR0                   // bit 0
#define VR1                     VRCONbits.VR1                   // bit 1
#define VR2                     VRCONbits.VR2                   // bit 2
#define VR3                     VRCONbits.VR3                   // bit 3
#define VRR                     VRCONbits.VRR                   // bit 5
#define VREN                    VRCONbits.VREN                  // bit 7

#define WPUA0                   WPUbits.WPUA0                   // bit 0
#define WPUA1                   WPUbits.WPUA1                   // bit 1
#define WPUA2                   WPUbits.WPUA2                   // bit 2
#define WPUA4                   WPUbits.WPUA4                   // bit 4
#define WPUA5                   WPUbits.WPUA5                   // bit 5

#endif // #ifndef NO_BIT_DEFINES

#endif // #ifndef __PIC16F630_H__
