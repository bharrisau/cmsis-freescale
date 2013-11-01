/**************************************************************************//**
 * @file     sim.h
 * @brief    System integration module header file for Kinetis devices
 * @version  V0.1
 * @date     30. October 2013
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2013 Ben Harris

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/
#ifndef __FREESC_SIM_H
#define __FREESC_SIM_H

struct SIM_SOPT1_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0 : 18;
      __IO uint32_t OSC32KSEL : 2;
           uint32_t RESERVED1 : 9;
      __IO uint32_t USBVSTBY  : 1;
      __IO uint32_t USBSSTBY  : 1;
      __IO uint32_t USBREGEN  : 1;
    };
  };
};

struct SIM_SOPTCFG_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0 : 24;
      __IO uint32_t URWE      : 1;
      __IO uint32_t UVSWE     : 1;
      __IO uint32_t USSWE     : 1;
           uint32_t RESERVED1 : 5;
    };
  };
};

struct SIM_SOPT2_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0 : 4;
      __IO uint32_t RTCCLKOUTSEL      : 1;
      __IO enum {
        SIM_CLKOUTSEL_BUS       = 2,
        SIM_CLKOUTSEL_LPO       = 3,
        SIM_CLKOUTSEL_MCGIRCLK  = 4,
        SIM_CLKOUTSEL_OSCERCLK  = 6
      } CLKOUTSEL     : 3;
           uint32_t RESERVED1 : 8;
      __IO uint32_t PLLFLLSEL      : 1;
           uint32_t RESERVED2 : 1;
      __IO uint32_t USBSRC      : 1;
           uint32_t RESERVED3 : 5;
      __IO enum {
        SIM_TPMSRC_DISABLE      = 0,
        SIM_TPMSRC_MCGFLLPLLE   = 1,
        SIM_TPMSRC_OSCERCLK     = 2,
        SIM_TPMSRC_MCGIRCLK     = 3
      } TPMSRC      : 2;
      __IO enum {
        SIM_UART0SRC_DISABLE    = 0,
        SIM_UART0SRC_MCGFLLPLL  = 1,
        SIM_UART0SRC_OSCERCLK   = 2,
        SIM_UART0SRC_MCGIRCLK   = 3
      } UART0SRC     : 2;
           uint32_t RESERVED4 : 4;
    };
  };
};

struct SIM_SOPT4_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0   : 18;
      __IO uint32_t TPM1CH0SRC  : 1;
           uint32_t RESERVED1   : 1;
      __IO uint32_t TPM2CH0SRC  : 1;
           uint32_t RESERVED2   : 3;
      __IO uint32_t TPM0CLKSEL  : 1;
      __IO uint32_t TPM1CLKSEL  : 1;
      __IO uint32_t TPM2CLKSEL  : 1;
           uint32_t RESERVED3   : 5;
    };
  };
};

struct SIM_SOPT5_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO enum {
        SIM_UART0TXSRC_NORMAL   = 0,
        SIM_UART0TXSRC_TPM1     = 1,
        SIM_UART0TXSRC_TPM2     = 2
      } UART0TXSRC              : 2;
      __IO uint32_t UART0RXSRC  : 1;
           uint32_t RESERVED0   : 1;
      __IO enum {
        SIM_UART1TXSRC_NORMAL   = 0,
        SIM_UART1TXSRC_TPM1     = 1,
        SIM_UART1TXSRC_TPM2     = 2
      } UART1TXSRC              : 2;
      __IO uint32_t UART1RXSRC  : 1;
           uint32_t RESERVED1   : 9;
      __IO uint32_t UART0ODE    : 1;
      __IO uint32_t UART1ODE    : 1;
      __IO uint32_t UART2ODE    : 1;
           uint32_t RESERVED2   : 13;
    };
  };
};

struct SIM_SOPT7_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO enum {
        SIM_ADC0TRGSEL_EXTRG    = 0,
        SIM_ADC0TRGSEL_CMP0     = 1,
        SIM_ADC0TRGSEL_PIT0     = 4,
        SIM_ADC0TRGSEL_PIT1     = 5,
        SIM_ADC0TRGSEL_TPM0     = 8,
        SIM_ADC0TRGSEL_TPM1     = 9,
        SIM_ADC0TRGSEL_TPM2     = 10,
        SIM_ADC0TRGSEL_RTC_ALRM = 12,
        SIM_ADC0TRGSEL_RTC_SEC  = 13,
        SIM_ADC0TRGSEL_LPTMR0   = 14
      } ADC0TRGSEL                : 4;
      __IO uint32_t ADC0PRETRGSEL : 1;
           uint32_t RESERVED0     : 2;
      __IO uint32_t ADC0ALTTRGEN  : 1;
           uint32_t RESERVED1     : 24;
    };
  };
};

struct SIM_SDID_REG {
  union {
    __I  uint32_t             all;
    struct {
      __I  uint32_t PINID       : 4;
           uint32_t RESERVED0   : 3;
      __I  uint32_t DIEID       : 5;
      __I  uint32_t REVID       : 4;
      __I  uint32_t SRAMSIZE    : 4;
      __I  uint32_t SERIESID    : 4;
      __I  uint32_t SUBFAMID    : 4;
      __I  uint32_t FAMID       : 4;

    };
  };
};

struct SIM_SCGC4_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0   : 6;
      __IO uint32_t I2C0        : 1;
      __IO uint32_t I2C1        : 1;
           uint32_t RESERVED1   : 2;
      __IO uint32_t UART0       : 1;
      __IO uint32_t UART1       : 1;
      __IO uint32_t UART2       : 1;
           uint32_t RESERVED2   : 5;
      __IO uint32_t USBOTG      : 1;
      __IO uint32_t CMP         : 1;
           uint32_t RESERVED3   : 2;
      __IO uint32_t SPI0        : 1;
      __IO uint32_t SPI1        : 1;
           uint32_t RESERVED4   : 8;
    };
  };
};

struct SIM_SCGC5_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO uint32_t LPTMR       : 1;
           uint32_t RESERVED0   : 4;
      __IO uint32_t TSI         : 1;
           uint32_t RESERVED1   : 3;
      __IO uint32_t PORTA       : 1;
      __IO uint32_t PORTB       : 1;
      __IO uint32_t PORTC       : 1;
      __IO uint32_t PORTD       : 1;
      __IO uint32_t PORTE       : 1;
           uint32_t RESERVED2   : 18;
    };
  };
};

struct SIM_SCGC6_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO uint32_t FTF         : 1;
      __IO uint32_t DMAMUX      : 1;
           uint32_t RESERVED0   : 21;
      __IO uint32_t PIT         : 1;
      __IO uint32_t TPM0        : 1;
      __IO uint32_t TPM1        : 1;
      __IO uint32_t TPM2        : 1;
      __IO uint32_t ADC0        : 1;
           uint32_t RESERVED1   : 1;
      __IO uint32_t RTC         : 1;
           uint32_t RESERVED2   : 1;
      __IO uint32_t DAC0        : 1;
    };
  };
};

struct SIM_SCGC7_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0   : 8;
      __IO uint32_t DMA         : 1;
           uint32_t RESERVED1   : 23;
    };
  };
};

struct SIM_CLKDIV1_REG {
  union {
    __IO uint32_t             all;
    struct {
           uint32_t RESERVED0   : 16;
      __IO uint32_t OUTDIV4     : 3;
           uint32_t RESERVED1   : 9;
      __IO uint32_t OUTDIV1     : 4;
    };
  };
};

struct SIM_FCFG1_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO uint32_t FLASHDIS    : 1;
      __IO uint32_t FLASHDOZE   : 1;
           uint32_t RESERVED0   : 22;
      __I  uint32_t PFSIZE      : 4;
           uint32_t RESERVED1   : 4;
    };
  };
};

struct SIM_FCFG2_REG {
  union {
    __I  uint32_t             all;
    struct {
           uint32_t RESERVED0   : 24;
      __I  uint32_t MAXADDR0    : 7;
           uint32_t RESERVED1   : 1;
    };
  };
};

struct SIM_COPC_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO uint32_t COPW        : 1;
      __IO uint32_t COPCLKS     : 1;
      __IO uint32_t COPT        : 2;
           uint32_t RESERVED0   : 28;
    };
  };
};

struct SIM_SRVCOP_REG {
  union {
    __O  uint32_t             all;
    struct {
      __O  uint32_t SRVCOP      : 8;
           uint32_t RESERVED0   : 24;
    };
  };
};

typedef struct {
       struct SIM_SOPT1_REG SOPT1;      /*!< Offset: 0x0000   System Options Register 1 */
       struct SIM_SOPTCFG_REG SOPTCFG;  /*!< Offset: 0x0004   SOPT1 Configuration Register */
} SIMLP_TypeDef;

typedef struct {
       uint32_t RESERVED0;
       struct SIM_SOPT2_REG SOPT2;      /*!< Offset: 0x0004   System Options Register 2 */
       uint32_t RESERVED1;
       struct SIM_SOPT4_REG SOPT4;      /*!< Offset: 0x000C   System Options Register 4 */
       struct SIM_SOPT5_REG SOPT5;      /*!< Offset: 0x0010   System Options Register 5 */
       uint32_t RESERVED2;
       struct SIM_SOPT7_REG SOPT7;      /*!< Offset: 0x0018   System Options Register 7 */
       uint32_t RESERVED3[2];
       struct SIM_SDID_REG SDID;        /*!< Offset: 0x0024   System Device Identification Register */
       uint32_t RESERVED4[3];
       struct SIM_SCGC4_REG SCGC4;      /*!< Offset: 0x0034   System Clock Gating Control Register 4 */
       struct SIM_SCGC5_REG SCGC5;      /*!< Offset: 0x0038   System Clock Gating Control Register 5 */
       struct SIM_SCGC6_REG SCGC6;      /*!< Offset: 0x003C   System Clock Gating Control Register 6 */
       struct SIM_SCGC7_REG SCGC7;      /*!< Offset: 0x0040   System Clock Gating Control Register 7 */
       struct SIM_CLKDIV1_REG CLKDIV1;  /*!< Offset: 0x0044   System Clock Divider Register 1 */
       uint32_t RESERVED5;
       struct SIM_FCFG1_REG FCFG1;      /*!< Offset: 0x004C   Flash Configuration Register 1 */
       struct SIM_FCFG2_REG FCFG2;       /*!< Offset: 0x0050   Flash Configuration Register 2 */
       uint32_t RESERVED6;
  __I  uint32_t UIDMH;                  /*!< Offset: 0x0058   Unique Identification Register Mid-High */
  __I  uint32_t UIDML;                  /*!< Offset: 0x005C   Unique Identification Register Mid-Low */
  __I  uint32_t UIDL;                   /*!< Offset: 0x0060   Unique Identification Register Low */
       uint32_t RESERVED7[39];
       struct SIM_COPC_REG COPC;        /*!< Offset: 0x0100   COP Control Register */
       struct SIM_SRVCOP_REG SRVCOP;    /*!< Offset: 0x0104   Service COP Register */
} SIM_TypeDef;

#endif