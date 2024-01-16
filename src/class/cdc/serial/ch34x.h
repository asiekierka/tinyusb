/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Heiko Kuester
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _CH34X_H_
#define _CH34X_H_

// set line_coding @ enumeration
#ifdef CFG_TUH_CDC_LINE_CODING_ON_ENUM
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM_CH34X CFG_TUH_CDC_LINE_CODING_ON_ENUM
#else // this default is necessary to work properly
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM_CH34X { 9600, CDC_LINE_CONDING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }
#endif

// USB requests
#define CH34X_REQ_READ_VERSION 0x5F // dec  95
#define CH34X_REQ_WRITE_REG    0x9A // dec 154
#define CH34X_REQ_READ_REG     0x95 // dec 149
#define CH34X_REQ_SERIAL_INIT  0xA1 // dec 161
#define CH34X_REQ_MODEM_CTRL   0xA4 // dev 164

// modem control bits
#define CH34X_BIT_RTS ( 1 << 6 )
#define CH34X_BIT_DTR ( 1 << 5 )

// line control bits
#define CH34X_LCR_ENABLE_RX    0x80
#define CH34X_LCR_ENABLE_TX    0x40
#define CH34X_LCR_MARK_SPACE   0x20
#define CH34X_LCR_PAR_EVEN     0x10
#define CH34X_LCR_ENABLE_PAR   0x08
#define CH34X_LCR_STOP_BITS_2  0x04
#define CH34X_LCR_CS8          0x03
#define CH34X_LCR_CS7          0x02
#define CH34X_LCR_CS6          0x01
#define CH34X_LCR_CS5          0x00

#endif /* _CH34X_H_ */
