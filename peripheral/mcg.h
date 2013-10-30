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

typedef struct {
  __IO uint8_t C1;             /*!< Offset: 0x0000   MCG Control 1 Register */
  __IO uint8_t C2;             /*!< Offset: 0x0001   MCG Control 2 Register */
  __IO uint8_t C3;             /*!< Offset: 0x0002   MCG Control 3 Register */
  __IO uint8_t C4;             /*!< Offset: 0x0003   MCG Control 4 Register */
  __IO uint8_t C5;             /*!< Offset: 0x0004   MCG Control 5 Register */
  __IO uint8_t C6;             /*!< Offset: 0x0005   MCG Control 6 Register */
  __I  uint8_t S;              /*!< Offset: 0x0006   MCG Status Register    */
       uint8_t RESERVED0;
  __IO uint8_t SC              /*!< Offset: 0x0008   MCG Status and Control Register                 */
       uint8_t RESERVED1;
  __IO uint8_t ATCVH;          /*!< Offset: 0x000A   MCG Auto Trim Compare Value High Register */
  __IO uint8_t ATCVL;          /*!< Offset: 0x000B   MCG Auto Trim Compare Value Low Register */
  __IO uint8_t C7;             /*!< Offset: 0x000C   MCG Control 7 Register */
  __IO uint8_t C8;             /*!< Offset: 0x000D   MCG Control 8 Register */
  __IO uint8_t C9;             /*!< Offset: 0x000E   MCG Control 9 Register */
  __IO uint8_t C10;            /*!< Offset: 0x000F   MCG Control 10 Register */
} MCG_TypeDef