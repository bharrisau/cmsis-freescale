/**************************************************************************//**
 * @file     mcg.h
 * @brief    Multipurpose clock generator header file for Kinetis devices
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

struct MCG_C1_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t IREFSTEN  : 1;
      __IO uint8_t IRCLKEN   : 1;
      __IO uint8_t IREFS     : 1;
      __IO uint8_t FRDIV     : 3;
      __IO enum {
        MCG_CLKS_FLLPLL = 0,
        MCG_CLKS_INT    = 1,
        MCG_CLKS_EXT    = 2
      } CLKS            : 2;
    };
  };
};

struct MCG_C2_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t IRCS      : 1;
      __IO uint8_t LP        : 1;
      __IO uint8_t EREFS0    : 1;
      __IO uint8_t HGO0      : 1;
      __IO enum {
        MCG_RANGE0_LOW    = 0,
        MCG_RANGE0_HIGH   = 1,
        MCG_RANGE0_VHIGH  = 2
      } RANGE0          : 2;
           uint8_t RESERVED  : 1;
      __IO uint8_t LOCRE0    : 1;
    };
  };
};

struct MCG_C4_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t SCFTRIM   : 1;
      __IO uint8_t FCTRIM    : 4;
      __IO enum {
        MCG_DRST_DRS_LOW      = 0,
        MCG_DRST_DRS_MID      = 1,
        MCG_DRST_DRS_MIDHIGH  = 2,
        MCG_DRST_DRS_HIGH     = 3
      } DRST_DRS        : 2;
      __IO uint8_t DMX32     : 1;
    };
  };
};

struct MCG_C5_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t PRDIV0    : 5;
      __IO uint8_t PLLSTEN0  : 1;
      __IO uint8_t PLLCLKEN0 : 1;
           uint8_t RESERVED  : 1;
    };
  };
};

struct MCG_C6_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t VDIV0     : 5;
      __IO uint8_t CME0      : 1;
      __IO uint8_t PLLS      : 1;
      __IO uint8_t LOLIE0    : 1;
    };
  };
};

struct MCG_S_REG {
  union {
    __I  uint8_t             all;
    struct {
      __I  uint8_t IRCST     : 1;
      __I  uint8_t OSCINIT0  : 1;
      __I  enum {
        MCG_CLKST_FLL     = 0,
        MCG_CLKST_INT     = 1,
        MCG_CLKST_EXT     = 2,
        MCG_CLKST_PLL     = 3
      } CLKST           : 2;
      __I  uint8_t IREFST    : 1;
      __I  uint8_t PLLST     : 1;
      __I  uint8_t LOCK0     : 1;
      __I  uint8_t LOLS      : 1;
    };
  };
};

struct MCG_SC_REG {
  union {
    __IO uint8_t             all;
    struct {
      __I  uint8_t LOCS0     : 1;
      __IO enum {
        MCG_FCRDIV_1      = 0,
        MCG_FCRDIV_2      = 1,
        MCG_FCRDIV_4      = 2,
        MCG_FCRDIV_8      = 3,
        MCG_FCRDIV_16     = 4,
        MCG_FCRDIV_32     = 5,
        MCG_FCRDIV_64     = 6,
        MCG_FCRDIV_128    = 7
      } FCRDIV          : 3;
      __IO uint8_t FLTPRSRV  : 1;
      __I  uint8_t ATMF      : 1;
      __IO uint8_t ATMS      : 1;
      __IO uint8_t ATME      : 1;
    };
  };
};

struct MCG_C8_REG {
  union {
    __IO uint8_t             all;
    struct {
           uint8_t RESERVED0 : 6;
      __IO uint8_t LOLRE     : 1;
           uint8_t RESERVED1 : 1;
    };
  };
};

typedef struct {
       struct MCG_C1_REG C1;    /*!< Offset: 0x0000   MCG Control 1 Register */
       struct MCG_C2_REG C2;    /*!< Offset: 0x0001   MCG Control 2 Register */
  __IO uint8_t C3;              /*!< Offset: 0x0002   MCG Control 3 Register */
       struct MCG_C4_REG C4;    /*!< Offset: 0x0003   MCG Control 4 Register */
       struct MCG_C5_REG C5;    /*!< Offset: 0x0004   MCG Control 5 Register */
       struct MCG_C6_REG C6;    /*!< Offset: 0x0005   MCG Control 6 Register */
       struct MCG_S_REG S;      /*!< Offset: 0x0006   MCG Status Register    */
       uint8_t RESERVED0;
       struct MCG_SC_REG SC     /*!< Offset: 0x0008   MCG Status and Control Register */
       uint8_t RESERVED1;
  __IO uint8_t ATCVH;           /*!< Offset: 0x000A   MCG Auto Trim Compare Value High Register */
  __IO uint8_t ATCVL;           /*!< Offset: 0x000B   MCG Auto Trim Compare Value Low Register */
  __IO uint8_t C7;              /*!< Offset: 0x000C   MCG Control 7 Register */
       struct MCG_C8_REG C8;    /*!< Offset: 0x000D   MCG Control 8 Register */
  __IO uint8_t C9;              /*!< Offset: 0x000E   MCG Control 9 Register */
  __IO uint8_t C10;             /*!< Offset: 0x000F   MCG Control 10 Register */
} MCG_TypeDef