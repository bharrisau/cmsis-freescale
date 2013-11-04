/**************************************************************************//**
 * @file     analog.h
 * @brief    Analog header file for Kinetis devices
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
#ifndef __FREESC_ANALOG_H
#define __FREESC_ANALOG_H

/* ADC Registers */

struct ADC_SC_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO enum {
        ADC_ADCH_DADP0          = 0,
        ADC_ADCH_DADP1          = 1,
        ADC_ADCH_DADP2          = 2,
        ADC_ADCH_DADP3          = 3,
        ADC_ADCH_AD4            = 4,
        ADC_ADCH_AD5            = 5,
        ADC_ADCH_AD6            = 6,
        ADC_ADCH_AD7            = 7,
        ADC_ADCH_AD8            = 8,
        ADC_ADCH_AD9            = 9,
        ADC_ADCH_AD10           = 10,
        ADC_ADCH_AD11           = 11,
        ADC_ADCH_AD12           = 12,
        ADC_ADCH_AD13           = 13,
        ADC_ADCH_AD14           = 14,
        ADC_ADCH_AD15           = 15,
        ADC_ADCH_AD16           = 16,
        ADC_ADCH_AD17           = 17,
        ADC_ADCH_AD18           = 18,
        ADC_ADCH_AD19           = 19,
        ADC_ADCH_AD20           = 20,
        ADC_ADCH_AD21           = 21,
        ADC_ADCH_AD22           = 22,
        ADC_ADCH_AD23           = 23,
        ADC_ADCH_TEMP           = 26,
        ADC_ADCH_BANDGAP        = 27,
        ADC_ADCH_VREFSH         = 29,
        ADC_ADCH_VREFSL         = 30,

        ADC_ADCH_DIFF_DAD0      = 0,
        ADC_ADCH_DIFF_DAD1      = 1,
        ADC_ADCH_DIFF_DAD2      = 2,
        ADC_ADCH_DIFF_DAD3      = 3,
        ADC_ADCH_DIFF_TEMP      = 26,
        ADC_ADCH_DIFF_BANDGAP   = 27,
        ADC_ADCH_DIFF_VREFSH    = 29
      } uint32_t ADCH         : 5;
      __IO uint32_t DIFF      : 1;
      __IO uint32_t AIEN      : 1;
      __I  uint32_t COCO      : 1;
           uint32_t RESERVED0 : 24;
    };
  };
};

struct ADC_CFG1_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO enum {
        ADC_ADICLK_BUS        = 0,
        ADC_ADICLK_BUS2       = 1,
        ADC_ADICLK_ALTCLK     = 2,
        ADC_ADICLK_ADACK      = 3
      } ADICLK                : 2;
      __IO enum {
        ADC_MODE_8            = 0,
        ADC_MODE_12           = 1,
        ADC_MODE_10           = 2,
        ADC_MODE_16           = 3,
        ADC_MODE_DIFF_9       = 0,
        ADC_MODE_DIFF_13      = 1,
        ADC_MODE_DIFF_11      = 2,
        ADC_MODE_DIFF_16      = 3
      } MODE                  : 2;
      __IO uint32_t ADLSMP    : 1;
      __IO enum {
        ADC_ADIV_1            = 0,
        ADC_ADIV_2            = 1,
        ADC_ADIV_4            = 2,
        ADC_ADIV_8            = 3
      } ADIV                  : 2;
      __IO uint32_t ADLPC     : 1;
           uint32_t RESERVED0 : 24;
    };
  };
};

struct ADC_CFG2_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO enum {
        ADC_ADLSTS_24         = 0,
        ADC_ADLSTS_16         = 1,
        ADC_ADLSTS_10         = 2,
        ADC_ADLSTS_6          = 3
      } ADLSTS                : 2;
      __IO uint32_t ADHSC     : 1;
      __IO uint32_t ADACKEN   : 1;
      __IO uint32_t MUXSEL    : 1;
           uint32_t RESERVED0 : 27;
    };
  };
};

struct ADC_SC2_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO enum {
        ADC_REFSEL_VREF       = 0,
        ADC_REFSEL_VALT       = 1
      } REFSEL                : 2;
      __IO uint32_t DMAEN     : 1;
      __IO uint32_t ACREN     : 1;
      __IO uint32_t ACFGT     : 1;
      __IO uint32_t ACFE      : 1;
      __IO uint32_t ADTRG     : 1;
      __I  uint32_t ADACT     : 1;
           uint32_t RESERVED0 : 24;
    };
  };
};

struct ADC_SC3_REG {
  union {
    __IO uint32_t             all;
    struct {
      __IO enum {
        ADC_AVGS_4      = 0,
        ADC_AVGS_8      = 1,
        ADC_AVGS_16     = 2,
        ADC_AVGS_32     = 3
      } AVGS                    : 2;
      __IO uint32_t AVGE        : 1;
      __IO uint32_t ADCO        : 1;
           uint32_t RESERVED0   : 2;
      __I  uint32_t CALF        : 1;
      __IO uint32_t CAL         : 1;
           uint32_t RESERVED1   : 24;
    };
  };
};

/* CMP Registers */

struct CMP_CR0_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t HYSTCTR      : 2;
           uint8_t RESERVED0    : 2;
      __IO uint8_t FILTER_CNT   : 3;
           uint8_t RESERVED0    : 1;
    };
  };
};

struct CMP_CR1_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t EN           : 1;
      __IO uint8_t OPE          : 1;
      __IO uint8_t COS          : 1;
      __IO uint8_t INV          : 1;
      __IO uint8_t PMODE        : 1;
      __IO uint8_t TRIGM        : 1;
      __IO uint8_t WE           : 1;
      __IO uint8_t SE           : 1;
    };
  };
};

struct CMP_SCR_REG {
  union {
    __IO uint8_t             all;
    struct {
      __I  uint8_t COUT         : 1;
      __IO uint8_t CFF          : 1;
      __IO uint8_t CFR          : 1;
      __IO uint8_t IEF          : 1;
      __IO uint8_t IER          : 1;
           uint8_t RESERVED0    : 1;
      __IO uint8_t DMAEN        : 1;
           uint8_t RESERVED1    : 1;
    };
  };
};

struct CMP_DACCR_REG {
  union {
    __IO uint8_t             all;
    struct {
      __I  uint8_t VOSEL        : 6;    /* DACO = (Vin/64) * (VOSEL[5:0] + 1) */
      __IO uint8_t VRSEL        : 1;
      __IO uint8_t DACEN        : 1;
    };
  };
};

struct CMP_MUXCR_REG {
  union {
    __IO uint8_t             all;
    struct {
      __I  uint8_t MSEL         : 3;
      __IO uint8_t PSEL         : 3;
           uint8_t RESERVED0    : 1;
      __IO uint8_t PSTM         : 1;
    };
  };
};

/* DAC Registers */

struct DAC_SR_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t DACBFRPBF    : 1;
      __IO uint8_t DACBFRPTF    : 1;
           uint8_t RESERVED0    : 6;
    };
  };
};

struct DAC_C0_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t DACBBIEN     : 1;
      __IO uint8_t DACBTIEN     : 1;
           uint8_t RESERVED0    : 1;
      __IO uint8_t LPEN         : 1;
      __O  uint8_t DACSWTRG     : 1;
      __IO uint8_t DACTRGSEL    : 1;
      __IO uint8_t DACRFS       : 1;
      __IO uint8_t DACEN        : 1;
    };
  };
};

struct DAC_C1_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t DACBFEN      : 1;
           uint8_t RESERVED0    : 1;
      __IO uint8_t DACBFMD      : 1;
           uint8_t RESERVED1    : 4;
      __IO uint8_t DMAEN        : 1;
    };
  };
};

struct DAC_C2_REG {
  union {
    __IO uint8_t             all;
    struct {
      __IO uint8_t DACBFUP      : 1;
           uint8_t RESERVED0    : 3;
      __IO uint8_t DACBFRP      : 1;
           uint8_t RESERVED1    : 3;
    };
  };
};

/* Main definitions */

typedef struct {
       struct ADC_SC_REG SC1[2];        /*!< Offset: 0x0000   ADC Status and Control Registers 1 */
       struct ADC_CFG1_REG CFG1;        /*!< Offset: 0x0008   ADC Configuration Register 1 */
       struct ADC_CFG2_REG CFG1;        /*!< Offset: 0x000C   ADC Configuration Register 2 */
  __I  uint32_t R[2];                   /*!< Offset: 0x0010   ADC Data Result Register */
  __IO uint32_t CV[2];                  /*!< Offset: 0x0018   Compare Value Registers */
       struct ADC_SC2_REG SC2;          /*!< Offset: 0x0020   Status and Control Register 2 */
       struct ADC_SC3_REG SC3;          /*!< Offset: 0x0024   Status and Control Register 3 */
  __IO uint32_t OFS;                    /*!< Offset: 0x0028   ADC Offset Correction Register */
  __IO uint32_t PG;                     /*!< Offset: 0x002C   ADC Plus-Side Gain Register */
  __IO uint32_t MG;                     /*!< Offset: 0x0030   ADC Minus-Side Gain Register */
  __IO uint32_t CLP[7];                 /*!< Offset: 0x0034   ADC Plus-Side General Calibration Value Register */
  __IO uint32_t CLM[7];                 /*!< Offset: 0x0058   ADC Minus-Side General Calibration Value Register */
} ADC_TypeDef;

typedef struct {
       struct CMP_CR0_REG CR0;          /*!< Offset: 0x0000   CMP Control Register 0 */
       struct CMP_CR1_REG CR1;          /*!< Offset: 0x0001   CMP Control Register 1 */
  __IO uint8_t FPR;                     /*!< Offset: 0x0002   CMP Filter Period Register */
       struct CMP_SCR_REG SCR;          /*!< Offset: 0x0003   CMP Status and Control Register */
       struct CMP_DACCR_REG DACCR;      /*!< Offset: 0x0004   DAC Control Register */
       struct CMP_MUXCR_REG MUXCR;      /*!< Offset: 0x0005   MUX Control Register */
} CMP_TypeDef;

typedef struct {
  __IO uint8_t DAT0L;                   /*!< Offset: 0x0000   DAC Data Low Register */
  __IO uint8_t DAT0H;                   /*!< Offset: 0x0001   DAC Data High Register */
  __IO uint8_t DAT1L;                   /*!< Offset: 0x0002   DAC Data Low Register */
  __IO uint8_t DAT1H;                   /*!< Offset: 0x0003   DAC Data High Register */
       uint8_t RESERVED0[28];
       struct DAC_SR_REG SR;            /*!< Offset: 0x0020   DAC Status Register */
       struct DAC_C0_REG C0;            /*!< Offset: 0x0021   DAC Control Register */
       struct DAC_C1_REG C1;            /*!< Offset: 0x0022   DAC Control Register 1 */
       struct DAC_C2_REG C2;            /*!< Offset: 0x0023   DAC Control Register 2 */
} DAC_TypeDef;

#endif