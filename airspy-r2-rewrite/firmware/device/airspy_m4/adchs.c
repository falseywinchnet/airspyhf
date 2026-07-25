/*
 * Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
 * Copyright 2015 Ian Gilmour <ian@sdrsharp.com>
 *
 * This file is part of AirSpy.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <airspy_core.h>
#include <stream_contract.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/cm3/memorymap.h>
#include <libopencm3/cm3/scs.h>
#include "adchs.h"

#define RESET_CTRL1_ADCHS_SHIFT (28)
#define RESET_CTRL1_ADCHS (1 << RESET_CTRL1_ADCHS_SHIFT)
#define STATUS0_CLEAR (0x7D)
#define STATUS0_CLEAR_MASK (0x7F)
#define STATUS1_CLEAR_MASK (0x1FFFFFFF)

typedef struct
{
  uint32_t src_addr;
  uint32_t dst_addr;
  uint32_t next_lli;
  uint32_t control;
} t_gpdma_lli;

typedef struct
{
  uint8_t data[ADCHS_DMA_NB_BUFFER][ADCHS_DATA_TRANSFER_SIZE_BYTE];
} t_adchs_buffer;

t_adchs_buffer* adchs_data_buffer_filled = (t_adchs_buffer*)ADCHS_DATA_BUFFER;

/* Allocate aligned buffer on 16bytes for DMA LLI */
t_gpdma_lli adchs_dma_lli[ADCHS_DMA_RING_NUM_LLI]
  __attribute__ ((aligned(16)));
static t_gpdma_lli gpdma_probe_lli __attribute__ ((aligned(16)));

int GPDMA_memory_copy_probe(
  const uint32_t source_addr,
  const uint32_t destination_addr,
  const uint32_t length,
  uint32_t* const cycles,
  uint32_t* const error_status)
{
  enum {
    GPDMA_CHANNEL = 7,
    GPDMA_CHANNEL_MASK = 1u << GPDMA_CHANNEL,
    GPDMA_ENABLE = 1u << 0,
    GPDMA_ACTIVE = 1u << 17,
    GPDMA_HALT = 1u << 18,
    GPDMA_TIMEOUT_CYCLES = 20u * 1000u * 1000u
  };

  if ((source_addr & 3u) != 0
    || (destination_addr & 3u) != 0
    || length != 16u * 1024u
    || (LPC_GPDMA->C7CONFIG & GPDMA_ENABLE) != 0)
  {
    return -1;
  }

  const uint32_t words_per_half = length / 2u / sizeof(uint32_t);
  const uint32_t control =
      words_per_half
    | (0x2u << 12) /* source burst: 8 words */
    | (0x2u << 15) /* destination burst: 8 words */
    | (0x2u << 18) /* source width: 32 bits */
    | (0x2u << 21) /* destination width: 32 bits */
    | (0x0u << 24) /* source through AHB master 0 */
    | (0x1u << 25) /* destination through AHB master 1 */
    | (0x1u << 26) /* increment source */
    | (0x1u << 27);/* increment destination */

  gpdma_probe_lli.src_addr = source_addr + length / 2u;
  gpdma_probe_lli.dst_addr = destination_addr + length / 2u;
  gpdma_probe_lli.next_lli = 0;
  gpdma_probe_lli.control = control;

  LPC_GPDMA->INTTCCLEAR = GPDMA_CHANNEL_MASK;
  LPC_GPDMA->INTERRCLR = GPDMA_CHANNEL_MASK;
  LPC_GPDMA->CONFIG = 1;
  while ((LPC_GPDMA->CONFIG & 1u) == 0);

  airspy_stream_publish_barrier();
  LPC_GPDMA->C7SRCADDR = source_addr;
  LPC_GPDMA->C7DESTADDR = destination_addr;
  LPC_GPDMA->C7LLI = (uint32_t)&gpdma_probe_lli;
  LPC_GPDMA->C7CONTROL = control;

  const uint32_t started = SCS_DWT_CYCCNT;
  LPC_GPDMA->C7CONFIG = GPDMA_ENABLE;
  while ((LPC_GPDMA->C7CONFIG & GPDMA_ENABLE) != 0
    && (SCS_DWT_CYCCNT - started) < GPDMA_TIMEOUT_CYCLES);

  *cycles = SCS_DWT_CYCCNT - started;
  *error_status = LPC_GPDMA->RAWINTERRSTAT & GPDMA_CHANNEL_MASK;

  if ((LPC_GPDMA->C7CONFIG & GPDMA_ENABLE) != 0)
  {
    LPC_GPDMA->C7CONFIG |= GPDMA_HALT;
    while ((LPC_GPDMA->C7CONFIG & GPDMA_ACTIVE) != 0
      && (SCS_DWT_CYCCNT - started) < (GPDMA_TIMEOUT_CYCLES * 2u));
    LPC_GPDMA->C7CONFIG &= ~GPDMA_ENABLE;
    return -2;
  }

  LPC_GPDMA->INTTCCLEAR = GPDMA_CHANNEL_MASK;
  LPC_GPDMA->INTERRCLR = GPDMA_CHANNEL_MASK;
  return *error_status == 0 ? 0 : -3;
}

void ADCHS_DMA_init_stop(void)
{
  /* clear all interrupts on channel 0 */
  LPC_GPDMA->INTTCCLEAR = 0x01;
  LPC_GPDMA->INTERRCLR = 0x01;

  /* Setup the DMAMUX */
  CREG_DMAMUX &= ~(0x3<<(ADCHS_DMA_WRITE*2));
  CREG_DMAMUX |= 0x3<<(ADCHS_DMA_WRITE*2);
  CREG_DMAMUX &= ~(0x3<<(ADCHS_DMA_READ*2));
  CREG_DMAMUX |= 0x3<<(ADCHS_DMA_READ*2);

  LPC_GPDMA->CONFIG = 0x01; /* Enable DMA channels, little endian */
  while ( !(LPC_GPDMA->CONFIG & 0x01) );

  /* Disable Channel before to change settings */
  LPC_GPDMA->CONFIG = 0x00; /* Disable DMA channels, little endian */
}

void ADCHS_DMA_init(uint32_t dest_addr, uint8_t packed)
{
  uint32_t nb_dma_transfer;
  int i;

  ADCHS_DMA_init_stop();

  /* Configure DMA LLI in Round Rubin */
  // The size of the transfer is in multiples of 32bit copies (hence the /4)
  // and must be even multiples of ADC_FIFO_LEVEL.
  nb_dma_transfer = ADCHS_DATA_BUFFER_SIZE_BYTE / (ADC_FIFO_LEVEL * ADCHS_DMA_NUM_LLI);
  nb_dma_transfer = (nb_dma_transfer * ADC_FIFO_LEVEL) / 4;

  for(i=0; i<ADCHS_DMA_NUM_LLI; i++)
  {
    adchs_dma_lli[i].src_addr = ADCHS_DMA_READ_SRC;
    adchs_dma_lli[i].dst_addr = ((uint32_t)dest_addr) + (nb_dma_transfer*4*i);
    /* Modulo with round rubin last LLI point to First in infinite loop */
    adchs_dma_lli[i].next_lli = (uint32_t)(&adchs_dma_lli[(i+1)%ADCHS_DMA_NUM_LLI]);

    adchs_dma_lli[i].control = ( (nb_dma_transfer) << 0) |
                               (0x2 << 12)  |
                               (0x2 << 15)  |
                               (0x2 << 18)  |
                               (0x2 << 21)  |
                               (0x1 << 24)  |
                               (0x1 << 25)  |
                               (0x0 << 26)  |
                               (0x1 << 27)  |
                               (0x0UL << 31);
  }

  if(packed)
  {
    for(i=0; i<ADCHS_DMA_NUM_LLI; i++)
    {
      adchs_dma_lli[i].control |= (0x1UL << 31);
    }
  }
  else
  {
    adchs_dma_lli[(ADCHS_DMA_NUM_LLI/2)-1].control |= (0x1UL << 31);
    adchs_dma_lli[i-1].control |= (0x1UL << 31);
  }

  LPC_GPDMA->C0SRCADDR = adchs_dma_lli[0].src_addr;
  LPC_GPDMA->C0DESTADDR = adchs_dma_lli[0].dst_addr;
  LPC_GPDMA->C0CONTROL = adchs_dma_lli[0].control;
  LPC_GPDMA->C0LLI     = (uint32_t)(&adchs_dma_lli[1]); // must be pointing to the second LLI as the first is used when initializing
  LPC_GPDMA->C0CONFIG =  (0x1)        |          // Enable bit
                         (ADCHS_DMA_READ << 1) |
                         (0x0 << 6)   |
                         (0x2 << 11)  |
                         (0x1 << 14)  |
                         (0x1 << 15);

  LPC_GPDMA->CONFIG = 0x01; /* Enable DMA channels, little endian */
}

void ADCHS_DMA_init_ring(
  const uint32_t destination_addresses[AIRSPY_STREAM_BUFFER_COUNT])
{
  enum {
    HALF_BUFFER_BYTES = AIRSPY_STREAM_BUFFER_BYTES / 2,
    WORDS_PER_HALF = HALF_BUFFER_BYTES / sizeof(uint32_t)
  };

  ADCHS_DMA_init_stop();
  for (uint32_t index = 0; index < ADCHS_DMA_RING_NUM_LLI; ++index)
  {
    const uint32_t buffer_index = index / 2u;
    const uint32_t buffer_offset = (index & 1u) * HALF_BUFFER_BYTES;
    adchs_dma_lli[index].src_addr = ADCHS_DMA_READ_SRC;
    adchs_dma_lli[index].dst_addr =
      destination_addresses[buffer_index] + buffer_offset;
    adchs_dma_lli[index].next_lli = (uint32_t)&adchs_dma_lli[
      (index + 1u) % ADCHS_DMA_RING_NUM_LLI];
    adchs_dma_lli[index].control =
        WORDS_PER_HALF
      | (0x2u << 12) /* source burst: 8 words */
      | (0x2u << 15) /* destination burst: 8 words */
      | (0x2u << 18) /* source width: 32 bits */
      | (0x2u << 21) /* destination width: 32 bits */
      | (0x1u << 24) /* source through AHB master 1 */
      | (0x1u << 25) /* destination through AHB master 1 */
      | (0x0u << 26) /* fixed FIFO source */
      | (0x1u << 27) /* increment destination */
      | ((index & 1u) << 31); /* interrupt only on full-bank boundary */
  }

  LPC_GPDMA->C0SRCADDR = adchs_dma_lli[0].src_addr;
  LPC_GPDMA->C0DESTADDR = adchs_dma_lli[0].dst_addr;
  LPC_GPDMA->C0CONTROL = adchs_dma_lli[0].control;
  LPC_GPDMA->C0LLI = (uint32_t)&adchs_dma_lli[1];
  LPC_GPDMA->C0CONFIG =
      0x1u
    | (ADCHS_DMA_READ << 1)
    | (0x2u << 11)
    | (0x1u << 14)
    | (0x1u << 15);
  LPC_GPDMA->CONFIG = 0x01;
}

void ADCHS_DMA_steer_ring_slot(
  const uint32_t slot_index,
  const uint32_t destination_address)
{
  enum { HALF_BUFFER_BYTES = AIRSPY_STREAM_BUFFER_BYTES / 2 };
  const uint32_t first = slot_index * 2u;

  /*
   * These are descriptors in SRAM, not the live C0LLI channel register.
   * UM10503 19.6.16/19.6.19 says the controller fetches the next LLI only
   * when the current packet completes. The caller rewrites a bank two slots
   * ahead, leaving one complete 8 KiB packet before either word is fetched.
   * Never write LPC_GPDMA->C0LLI while channel 0 is enabled.
   */
  adchs_dma_lli[first].dst_addr = destination_address;
  adchs_dma_lli[first + 1u].dst_addr =
    destination_address + HALF_BUFFER_BYTES;
  airspy_stream_publish_barrier();
}

void ADCHS_init_stop(void)
{
  /* Reset ADCHS using RGU */
  RESET_CTRL1 = RESET_CTRL1_ADCHS;
  /* Wait end of Reset */
  while( (RESET_ACTIVE_STATUS1 & RESET_CTRL1_ADCHS) != RESET_CTRL1_ADCHS );

  LPC_ADCHS->CLR_EN0 = STATUS0_CLEAR_MASK;
  LPC_ADCHS->CLR_STAT0 = STATUS0_CLEAR_MASK;

  while(LPC_ADCHS->STATUS0 & STATUS0_CLEAR);

  LPC_ADCHS->CLR_EN1 = STATUS1_CLEAR_MASK;
  LPC_ADCHS->CLR_STAT1 = STATUS1_CLEAR_MASK;
  while(LPC_ADCHS->STATUS1);

  LPC_ADCHS->POWER_DOWN = (0<<0);
  LPC_ADCHS->FLUSH = 1;
  LPC_ADCHS->FIFO_CFG = ADC_FIFO_LEVEL<<1 | 0x1;
}

void ADCHS_deinit(void)
{
  ADCHS_DMA_init_stop();
  ADCHS_init_stop();
  LPC_ADCHS->POWER_CONTROL = 0x0;
}

/* Initialized ADCHS for freq between 0 to less than 30MSPS */
void ADCHS_init(void)
{
  ADCHS_init_stop();

  LPC_ADCHS->CONFIG =
  ( 0x1 << 0 ) |
  ( 0x0 << 2 ) |
  ( 0x0  << 4 ) |
  ( 0x0  << 5 ) |
  ( 0x90 << 6 );

  /* Configure and Enable ADCHS for fADC less than 30MS/s */
  LPC_ADCHS->POWER_CONTROL =
  0 |
  (0x1 << 4)   |
  (0x1 << 10)  |
  (0 << 16)    |
  (1 << 17)    |
  (1 << 18);
  LPC_ADCHS->ADC_SPEED = 0x0;

  LPC_ADCHS->FLUSH = 1;
  /*
   * UM10503 requires at least one CPU cycle before observing fill level.
   * FIFO_STS level zero is ambiguous (empty or 16 words), so wait on the
   * explicit STATUS0 FIFO_EMPTY indication instead.
   */
  __asm volatile ("nop");
  while ((LPC_ADCHS->STATUS0 & STAT0_FIFO_EMPTY) == 0);

  /* Configure Threshold A & B to default value (even if not used) */
  LPC_ADCHS->THR_A = 0x00FFF000;
  LPC_ADCHS->THR_B = 0x00FFF000;

  /* Configure Interrupt 0 & 1 Enable register to default value (even if not used) */
  LPC_ADCHS->CLR_EN0   = STATUS0_CLEAR_MASK;
  LPC_ADCHS->CLR_STAT0 = STATUS0_CLEAR_MASK;
  LPC_ADCHS->CLR_EN1   = STATUS1_CLEAR_MASK;
  LPC_ADCHS->CLR_STAT1 = STATUS1_CLEAR_MASK;
  LPC_ADCHS->SET_EN0   = STAT0_FIFO_OVERFLOW;

}

void ADCHS_desc_init(uint8_t chan_num)
{
  LPC_ADCHS->DSCR_STS =
    (1<<0) |
    (0<<1);

  LPC_ADCHS->DESCRIPTOR_1[0] =
  (chan_num << 0) |
  (0 << 3)        |
  (0 << 4)        |
  (0 << 5)        |
  (0x2 << 6)      |
  (1 << 8)        |
  (0 << 22)       |
  (1 << 24)       |
  (0x1U << 31);

  LPC_ADCHS->DESCRIPTOR_0[0] =
  (chan_num << 0) |
  (0 << 3)        |
  (0 << 4)        |
  (0 << 5)        |
  (0x1 << 6)      |
  (0 << 8)        |
  (0 << 22)       |
  (1 << 24)       |
  (0x1U << 31);

}
