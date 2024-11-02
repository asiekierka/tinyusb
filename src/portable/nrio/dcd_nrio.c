/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 * Copyright (c) 2024 Adrian "asie" Siekierka
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

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_NRIO

#include <nds.h>
#include "device/dcd.h"

/**
 * NRIO register defines.
 *
 * The NRIO (DS-Writer etc.) is an ISP1581-compatible chip exposed via the
 * Slot-2 (GBA cartridge) bus interface, albeit without the ability to use
 * DMA.
 */

#define NRIO_REG(index) GBA_BUS[(index) * 0x10000]

#define NRIO_DEVEN_ENABLE  BIT(7)
#define NRIO_DEVEN_ADDR(n) (n)
#define NRIO_DEVEN     NRIO_REG(0x00)

#define NRIO_MODE_CLKAON   BIT(7)
#define NRIO_MODE_SNDRSU   BIT(6)
#define NRIO_MODE_GOSUSP   BIT(5)
#define NRIO_MODE_SFRESET  BIT(4)
#define NRIO_MODE_GLINTENA BIT(3)
#define NRIO_MODE_WKUPCS   BIT(2)
#define NRIO_MODE_PWROFF   BIT(1)
#define NRIO_MODE_SOFTCT   BIT(0)
#define NRIO_MODE      NRIO_REG(0x0C)

#define NRIO_INT_CFG_POL_LOW       (0)
#define NRIO_INT_CFG_POL_HIGH      BIT(0)
#define NRIO_INT_CFG_SIG_LEVEL     (0)
#define NRIO_INT_CFG_SIG_PULSED    BIT(1)
#define NRIO_INT_CFG_DDBGMODOUT(n) ((n) << 2)
#define NRIO_INT_CFG_DDBGMODIN(n)  ((n) << 4)
#define NRIO_INT_CFG_CDBGMOD(n)    ((n) << 6)
#define NRIO_INT_CFG   NRIO_REG(0x10)

#define NRIO_INTL_BRESET   BIT(0)
#define NRIO_INTL_SOF      BIT(1)
#define NRIO_INTL_PSOF     BIT(2)
#define NRIO_INTL_SUSP     BIT(3)
#define NRIO_INTL_RESUME   BIT(4)
#define NRIO_INTL_HS_STAT  BIT(5)
#define NRIO_INTL_DMA      BIT(6)
#define NRIO_INTL_EP0SETUP BIT(8)
#define NRIO_INTL_EP0RX    BIT(10)
#define NRIO_INTL_EP0TX    BIT(11)
#define NRIO_INTL_EP1RX    BIT(12)
#define NRIO_INTL_EP1TX    BIT(13)
#define NRIO_INTL_EP2RX    BIT(14)
#define NRIO_INTL_EP2TX    BIT(15)
#define NRIO_INTL_ALL      0xFD7F
#define NRIO_INTL_ALL_NON_EDPT 0x017F
#define NRIO_INTH_EP3RX    BIT(0)
#define NRIO_INTH_EP3TX    BIT(1)
#define NRIO_INTH_EP4RX    BIT(2)
#define NRIO_INTH_EP4TX    BIT(3)
#define NRIO_INTH_EP5RX    BIT(4)
#define NRIO_INTH_EP5TX    BIT(5)
#define NRIO_INTH_EP6RX    BIT(6)
#define NRIO_INTH_EP6TX    BIT(7)
#define NRIO_INTH_EP7RX    BIT(8)
#define NRIO_INTH_EP7TX    BIT(9)
#define NRIO_INTH_ALL      0x03FF
#define NRIO_INTH_ALL_NON_EDPT 0x0000
#define NRIO_INT_ENL   NRIO_REG(0x14)
#define NRIO_INT_ENH   NRIO_REG(0x16)
#define NRIO_INT_STL   NRIO_REG(0x18)
#define NRIO_INT_STH   NRIO_REG(0x1A)

#define NRIO_EP_IDX_OUT       (0)
#define NRIO_EP_IDX_IN        BIT(0)
#define NRIO_EP_IDX_IDX(n)    ((n) << 1)
#define NRIO_EP_IDX_EP0_DATA  (0)
#define NRIO_EP_IDX_EP0_SETUP BIT(5)
#define NRIO_EP_IDX    NRIO_REG(0x2C)

#define NRIO_EP_CFG_STALL         BIT(0)
#define NRIO_EP_CFG_STATUS        BIT(1)
#define NRIO_EP_CFG_VENDP         BIT(3)
#define NRIO_EP_CFG_CLBUF         BIT(4)
#define NRIO_EP_CFG    NRIO_REG(0x28)

#define NRIO_EP_DATA   NRIO_REG(0x20)
#define NRIO_EP_DATA32 (*((vu32*) &NRIO_REG(0x20)))
#define NRIO_EP_BUFLEN NRIO_REG(0x1C)

#define NRIO_EP_PKT_FIFO_LEN(n) (n)
#define NRIO_EP_PKT_NTRANS(n)   (((n) - 1) << 11)
#define NRIO_EP_PKT    NRIO_REG(0x04)

#define NRIO_EP_TYPE_CTRL    (0)
#define NRIO_EP_TYPE_ISO     (1)
#define NRIO_EP_TYPE_BULK    (2)
#define NRIO_EP_TYPE_INT     (3)
#define NRIO_EP_TYPE_DBLBUF  BIT(2)
#define NRIO_EP_TYPE_ENABLE  BIT(3)
#define NRIO_EP_TYPE_NOEMPKT BIT(4)
#define NRIO_EP_TYPE   NRIO_REG(0x08)

#define NRIO_EP_SHORT_OUT(n) ((n) << 8)
#define NRIO_EP_SHORT_OUT0   NRIO_EP_SHORT_OUT(0)
#define NRIO_EP_SHORT_OUT1   NRIO_EP_SHORT_OUT(1)
#define NRIO_EP_SHORT_OUT2   NRIO_EP_SHORT_OUT(2)
#define NRIO_EP_SHORT_OUT3   NRIO_EP_SHORT_OUT(3)
#define NRIO_EP_SHORT_OUT4   NRIO_EP_SHORT_OUT(4)
#define NRIO_EP_SHORT_OUT5   NRIO_EP_SHORT_OUT(5)
#define NRIO_EP_SHORT_OUT6   NRIO_EP_SHORT_OUT(6)
#define NRIO_EP_SHORT_OUT7   NRIO_EP_SHORT_OUT(7)
#define NRIO_EP_SHORT  NRIO_REG(0x24)

#define NRIO_CHIP_ID_ISP1581 0x1581
#define NRIO_CHIP_ID   NRIO_REG(0x70)
#define NRIO_CHIP_REV  NRIO_REG(0x72)

#define NRIO_FRN_MICROSOF(n) ((n) << 11)
#define NRIO_FRN_SOF(n)      (n)
#define NRIO_FRN       NRIO_REG(0x74)
#define NRIO_SCRATCH   NRIO_REG(0x78)

#define NRIO_UNLOCK_CODE 0xAA37
#define NRIO_UNLOCK    NRIO_REG(0x7C)

#define NRIO_TEST_FORCEHS BIT(7)
#define NRIO_TEST_PHYTEST BIT(6)
#define NRIO_TEST_LPBK    BIT(5)
#define NRIO_TEST_FORCEFS BIT(4)
#define NRIO_TEST_PRBS    BIT(3)
#define NRIO_TEST_KSTATE  BIT(2)
#define NRIO_TEST_JSTATE  BIT(1)
#define NRIO_TEST_SE0_NAK BIT(0)
#define NRIO_TEST      NRIO_REG(0x84)

/**
 * Helper macros, defines, etc.
 */

// Convert a TinyUSB ep_addr to an NRIO index.
#define tu_edpt_nrio_idx(e) ((((e) << 1) & 0xFE) | ((e) >> 7))
// Convert an NRIO index to a TinyUSB ep_addr.
#define tu_edpt_from_nrio_idx(e) ((((e) << 7) & 0x80) | ((e) >> 1))

// Default interrupt enable masks.
// SOF is enabled on demand; PSOF and DMA are not used.
// TODO: Enable EPxTX/EPxRX on demand.
#define NRIQ_INTL_DEFAULT (NRIO_INTL_ALL & ~(NRIO_INTL_DMA | NRIO_INTL_PSOF | NRIO_INTL_SOF))
#define NRIQ_INTH_DEFAULT NRIO_INTH_ALL

// Device driver state.
DTCM_BSS
static struct {
  uint16_t *rx_buffer[TUP_DCD_ENDPOINT_MAX];
  uint16_t buffer_length[TUP_DCD_ENDPOINT_MAX * 2];
  bool irq_enabled;
  uint16_t xfer_mask;
} _dcd;

#define BUFFER_LENGTH_NONE 0xFFFF

// Two endpoint 0 descriptor definition for unified dcd_edpt_open()
static const tusb_desc_endpoint_t ep0OUT_desc =
{
  .bLength          = sizeof(tusb_desc_endpoint_t),
  .bDescriptorType  = TUSB_DESC_ENDPOINT,

  .bEndpointAddress = 0x00,
  .bmAttributes     = { .xfer = TUSB_XFER_CONTROL },
  .wMaxPacketSize   = CFG_TUD_ENDPOINT0_SIZE,
  .bInterval        = 0
};

static const tusb_desc_endpoint_t ep0IN_desc =
{
  .bLength          = sizeof(tusb_desc_endpoint_t),
  .bDescriptorType  = TUSB_DESC_ENDPOINT,

  .bEndpointAddress = 0x80,
  .bmAttributes     = { .xfer = TUSB_XFER_CONTROL },
  .wMaxPacketSize   = CFG_TUD_ENDPOINT0_SIZE,
  .bInterval        = 0
};

TU_ATTR_FAST_FUNC
void dcd_int_handler (uint8_t rhport);

// Use software IRQ disable. Seems to slightly improve USB performance,
// but feels less clean.
// #define USE_SOFTWARE_IRQ_DISABLE

#define USE_ASM_TXRX

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

extern void dcd_nrio_tx_bytes(const void *buffer, uint32_t len);
extern void dcd_nrio_rx_bytes(void *buffer, uint32_t len);

/**
 * @brief Post-bus reset initialization.
 */
static void dcd_bus_init (void) {
  // Reinitialize _dcd.
  int oldIME = enterCriticalSection();
  memset(_dcd.buffer_length, 0xFF, sizeof(_dcd.buffer_length));
  _dcd.xfer_mask = 0;
  leaveCriticalSection(oldIME);

  // Configure interrupts
  NRIO_INT_CFG = NRIO_INT_CFG_CDBGMOD(1) | NRIO_INT_CFG_DDBGMODIN(1) | NRIO_INT_CFG_DDBGMODOUT(1)
    | NRIO_INT_CFG_SIG_LEVEL | NRIO_INT_CFG_POL_HIGH;

  NRIO_INT_ENL = NRIQ_INTL_DEFAULT;
  NRIO_INT_ENH = NRIQ_INTH_DEFAULT;

  // Configure control endpoints
  dcd_edpt_open(0, &ep0OUT_desc);
  dcd_edpt_open(0, &ep0IN_desc);

  // Enable device
  NRIO_DEVEN = NRIO_DEVEN_ENABLE;
}

static void nrio_reset (void) {
  // Reset USB controller
  NRIO_MODE = NRIO_MODE_SFRESET;
  swiDelay(8378); // 1ms
  NRIO_MODE = 0;
  swiDelay(8378); // 1ms

  NRIO_MODE = NRIO_MODE_GLINTENA | NRIO_MODE_CLKAON;
}

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  // Configure GBA cartridge bus
#ifdef ARM9
  sysSetCartOwner(BUS_OWNER_ARM9);
#else
  sysSetCartOwner(BUS_OWNER_ARM7);
#endif
  REG_EXMEMCNT = (REG_EXMEMCNT & ~0x1F) | EXMEMCNT_ROM_TIME1_6_CYCLES
    | EXMEMCNT_ROM_TIME2_4_CYCLES | EXMEMCNT_SRAM_TIME_6_CYCLES;
  irqSet(IRQ_CART, (VoidFn) dcd_int_handler);
#ifdef USE_SOFTWARE_IRQ_DISABLE
  _dcd.irq_enabled = false;
  irqEnable(IRQ_CART);
#endif

  nrio_reset();
  dcd_bus_init();
  dcd_connect(rhport);
  return true;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport) {
  (void) rhport;

#ifdef USE_SOFTWARE_IRQ_DISABLE
  _dcd.irq_enabled = true;
#else
  int oldIME = enterCriticalSection();
  irqEnable(IRQ_CART);
  dcd_int_handler(0);
  leaveCriticalSection(oldIME);
#endif
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport) {
  (void) rhport;

#ifdef USE_SOFTWARE_IRQ_DISABLE
  _dcd.irq_enabled = false;
#else
  irqDisable(IRQ_CART);
#endif
}

/**
 * @brief Attempt handling a TX/RX event.
 * 
 * @param nrio_addr NRIO-format address.
 * @param ep_addr TinyUSB-format address.
 * @param in_isr True if executed in IRQ handler.
 * @return bool Whether or not the event was processed.
 */
TU_ATTR_FAST_FUNC
static bool dcd_xfer_handle (uint32_t nrio_addr, uint32_t ep_addr, bool in_isr) {
  uint16_t expected_length = _dcd.buffer_length[nrio_addr];

  if (expected_length == BUFFER_LENGTH_NONE)
    return false;

  NRIO_EP_IDX = nrio_addr;
  int num = tu_edpt_number(ep_addr);

  if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
    // Handle Host->Device transfers
    uint16_t received_bytes = NRIO_EP_BUFLEN;
    if (!received_bytes)
      return false;
    if (expected_length > received_bytes)
      expected_length = received_bytes;

    // Copy data from NRIO to buffer
#ifdef USE_ASM_TXRX
    dcd_nrio_rx_bytes(_dcd.rx_buffer[num], expected_length);
#else
    uint16_t *buf = (uint16_t*) _dcd.rx_buffer[num];    
    for (int i = 0; i < expected_length >> 1; i++)
      buf[i] = NRIO_EP_DATA;
    if (expected_length & 1)
      ((uint8_t*) buf)[expected_length - 1] = NRIO_EP_DATA;
#endif
  }

  // Signal end of transfer
  _dcd.buffer_length[nrio_addr] = BUFFER_LENGTH_NONE;
  dcd_event_xfer_complete(0, ep_addr, expected_length, XFER_RESULT_SUCCESS, in_isr);
  return true;
}

TU_ATTR_FAST_FUNC
void dcd_int_handler (uint8_t rhport) {
#ifdef USE_SOFTWARE_IRQ_DISABLE
  if (!_dcd.irq_enabled)
    return;
#endif

  // EPxTX/EPxRX interrupt flags.
  uint32_t xfers = 0;
  uint32_t _setup_packet[2];

  uint16_t mask;
  mask = NRIO_INT_STL;
  NRIO_INT_STL = mask;
  if (mask) {
    if (mask & NRIO_INTL_BRESET) {
      NRIO_INT_STL = 0xFFFF;
      NRIO_INT_STH = 0xFFFF;
      dcd_bus_init();
      dcd_event_bus_reset(0, TUSB_SPEED_HIGH, true);
      return;
    } else {
      xfers |= (mask >> 10);

      if (mask & NRIO_INTL_SOF) {
        dcd_event_sof(0, NRIO_FRN, true);
      }
      if (mask & NRIO_INTL_SUSP) {
        NRIO_MODE &= ~NRIO_MODE_CLKAON;
        dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
      }
      if (mask & NRIO_INTL_RESUME) {
        NRIO_MODE |= NRIO_MODE_CLKAON;
        dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
        NRIO_UNLOCK = NRIO_UNLOCK_CODE;
      }
      if (mask & NRIO_INTL_EP0SETUP) {
        NRIO_EP_IDX = NRIO_EP_IDX_EP0_SETUP;
        _setup_packet[0] = NRIO_EP_DATA32;
        _setup_packet[1] = NRIO_EP_DATA32;
        dcd_event_setup_received(0, (uint8_t*) _setup_packet, true);
      }
    }
  }

  mask = NRIO_INT_STH;
  NRIO_INT_STH = mask;
  xfers |= (mask << 6) & 0xFFC0;

  // Apply an additional, polling-centric set of transfer flags.
  // As the NRIO does not have proper interrupts, ensuring reliable response
  // to TX/RX events is a bit of a challenge.
  xfers |= _dcd.xfer_mask;
  // Handle TX/RX interrupts.
  while (xfers) {
    uint32_t index = __builtin_clz(xfers) ^ 0x1F;
    if (dcd_xfer_handle(index, tu_edpt_from_nrio_idx(index), true)) {
      _dcd.xfer_mask &= ~(1 << index);
    } else {
      _dcd.xfer_mask |= (1 << index);
    }
    xfers &= ~(1 << index);
  }
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr) {
  NRIO_DEVEN = NRIO_DEVEN_ENABLE | NRIO_DEVEN_ADDR(dev_addr & 0x7F);
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport) {
  (void) rhport;

  NRIO_MODE |= NRIO_MODE_SNDRSU;
  NRIO_MODE &= ~NRIO_MODE_SNDRSU;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport) {
  (void) rhport;
  NRIO_MODE |= NRIO_MODE_SOFTCT;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
  NRIO_MODE &= ~NRIO_MODE_SOFTCT;
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void) rhport;
  (void) en;

  if (en) {
    NRIO_INT_ENL |= NRIO_INTL_SOF;
  } else {
    NRIO_INT_ENL &= ~NRIO_INTL_SOF;
  }
}

#if CFG_TUD_TEST_MODE
void dcd_enter_test_mode(uint8_t rhport, tusb_feature_test_mode_t test_selector) {
  switch (test_selector) {
    case TUSB_FEATURE_TEST_J:
      NRIO_TEST = NRIO_TEST_JSTATE;
      break;
    case TUSB_FEATURE_TEST_K:
      NRIO_TEST = NRIO_TEST_KSTATE;
      break;
    case TUSB_FEATURE_TEST_SE0_NAK:
      NRIO_TEST = NRIO_TEST_SE0_NAK;
      break;
  }
}
#endif

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc) {
  (void) rhport;
  uint32_t idx = tu_edpt_nrio_idx(ep_desc->bEndpointAddress);

  NRIO_EP_IDX = idx;
  // Enable double-buffering for Host->Device transfers.
  NRIO_EP_TYPE = ep_desc->bmAttributes.xfer | NRIO_EP_TYPE_NOEMPKT
    | (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_OUT ? NRIO_EP_TYPE_DBLBUF : 0);
  NRIO_EP_PKT = ep_desc->wMaxPacketSize;
  NRIO_EP_CFG = NRIO_EP_CFG_CLBUF;
  NRIO_EP_IDX = idx;
  NRIO_EP_CFG = NRIO_EP_CFG_CLBUF;
  NRIO_EP_TYPE |= NRIO_EP_TYPE_ENABLE;

  return true;
}

void dcd_edpt_close_all (uint8_t rhport) {
  (void) rhport;

  for (int i = 15; i >= 2; i--) {
    NRIO_EP_IDX = i;
    NRIO_EP_TYPE = 0;
  }
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes) {
  (void) rhport;
  (void) ep_addr;
  (void) buffer;
  (void) total_bytes;

#ifdef ARM9
  sassert(total_bytes <= 512, "USBD Xfer too large: %d > 512", total_bytes);
#endif

  uint32_t idx = tu_edpt_nrio_idx(ep_addr);
  uint32_t num = tu_edpt_number(ep_addr);

  if (_dcd.buffer_length[idx] != BUFFER_LENGTH_NONE)
    return false;

  uint32_t *buffer32 = (uint32_t*) buffer;
  if (!total_bytes) {
    // Handle zero-length packets
    NRIO_EP_CFG = NRIO_EP_CFG_STATUS;
    while (NRIO_EP_CFG & NRIO_EP_CFG_STATUS)
      dcd_event_xfer_complete(0, ep_addr, 0, XFER_RESULT_SUCCESS, false);
  } else if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
    // Device to Host - queue data on FIFO, wait for TX interrupt
    NRIO_EP_IDX = idx;
    NRIO_EP_BUFLEN = total_bytes;

#ifdef USE_ASM_TXRX
    dcd_nrio_tx_bytes(buffer32, total_bytes);
#else
    for (int i = 0; i < (total_bytes + 3) >> 2; i++)
      NRIO_EP_DATA32 = buffer32[i];
#endif

    _dcd.buffer_length[idx] = total_bytes;
  } else {
    // Host to Device - poll BUFLEN, then write to buffer
    int oldIME = enterCriticalSection();
    _dcd.rx_buffer[num] = (uint16_t*) buffer32;
    _dcd.xfer_mask |= (1 << idx);
    _dcd.buffer_length[idx] = total_bytes;
    leaveCriticalSection(oldIME);
  }

  return true;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
/* bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes) {
  (void) rhport;
  (void) ep_addr;
  (void) ff;
  (void) total_bytes;

  // TODO
  return false;
} */

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  NRIO_EP_IDX = tu_edpt_nrio_idx(ep_addr);
  NRIO_EP_CFG |= NRIO_EP_CFG_STALL;
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  NRIO_EP_IDX = tu_edpt_nrio_idx(ep_addr);
  NRIO_EP_CFG &= ~NRIO_EP_CFG_STALL;

  // Reset data toggle bit
  NRIO_EP_TYPE &= ~NRIO_EP_TYPE_ENABLE;
  NRIO_EP_TYPE |= NRIO_EP_TYPE_ENABLE;
}

#endif
