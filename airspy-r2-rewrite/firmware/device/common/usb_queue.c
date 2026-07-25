/*
 * Copyright 2012 Jared Boone
 * Copyright 2013 Ben Gamari
 * Copyright 2014-2015 Benjamin Vernoux <bvernoux@airspy.com>
 *
 * This file is part of AirSpy (based on HackRF project).
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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

#include <libopencm3/cm3/cortex.h>

/*
__ldrex() & __strex() are not compatible with M0 so use lock compatible with both M0 & M3/M4
#include <libopencm3/cm3/sync.h>
*/

#include "usb.h"
#include "usb_queue.h"

usb_queue_t* endpoint_queues[12] = {};

#define USB_ENDPOINT_INDEX(endpoint_address) (((endpoint_address & 0xF) * 2) + ((endpoint_address >> 7) & 1))

static uint32_t interrupt_save(void)
{
  uint32_t primask;
  __asm volatile (
    "mrs %0, primask\n"
    "cpsid i"
    : "=r"(primask)
    :
    : "memory");
  return primask;
}

static void interrupt_restore(const uint32_t primask)
{
  if ((primask & 1u) == 0)
  {
    __asm volatile ("cpsie i" ::: "memory");
  }
}

static usb_queue_t* endpoint_queue(const usb_endpoint_t* const endpoint)
{
  uint32_t index = USB_ENDPOINT_INDEX(endpoint->address);
  if (endpoint_queues[index] == NULL)
  {
    usb_fatal_error_reset();
  }
  return endpoint_queues[index];
}

void usb_queue_init(usb_queue_t* const queue)
{
  uint32_t index = USB_ENDPOINT_INDEX(queue->endpoint->address);
  if (endpoint_queues[index] != NULL)
  {
    usb_fatal_error_reset();
  }
  endpoint_queues[index] = queue;

  usb_transfer_t* t = queue->free_transfers;
  for (unsigned int i=0; i < queue->pool_size - 1; i++, t++) {
          t->next = t+1;
          t->queue = queue;
  }
  t->next = NULL;
  t->queue = queue;
  queue->free_tail = t;
  queue->active = NULL;
  queue->active_tail = NULL;
}

/* Allocate a transfer */
static usb_transfer_t* allocate_transfer(
        usb_queue_t* const queue
) {
/*
  bool aborted;
*/
  usb_transfer_t* transfer;

/*
  do {
          transfer = (void *) __ldrex((uint32_t *) &queue->free_transfers);
          aborted = __strex((uint32_t) transfer->next, (uint32_t *) &queue->free_transfers);
  } while (aborted);
*/
  const uint32_t primask = interrupt_save();
  transfer = queue->free_transfers;
  if (transfer == NULL)
  {
    interrupt_restore(primask);
    return NULL;
  }
  queue->free_transfers = transfer->next;
  if (queue->free_transfers == NULL)
  {
    queue->free_tail = NULL;
  }
  transfer->next = NULL;
  interrupt_restore(primask);

  return transfer;
}

/* Place a transfer in the free list */
static void free_transfer(usb_transfer_t* const transfer)
{
        usb_queue_t* const queue = transfer->queue;
/*
  bool aborted;
  do {
          transfer->next = (void *) __ldrex((uint32_t *) &queue->free_transfers);
          aborted = __strex((uint32_t) transfer, (uint32_t *) &queue->free_transfers);
  } while (aborted);
*/
  const uint32_t primask = interrupt_save();
  transfer->next = NULL;
  if (queue->free_tail != NULL)
  {
    queue->free_tail->next = transfer;
  } else {
    queue->free_transfers = transfer;
  }
  queue->free_tail = transfer;
  interrupt_restore(primask);
}

/* Place a transfer in the free list (nolock or disable IRQ) */
static void free_transfer_nolock(usb_transfer_t* const transfer)
{
  usb_queue_t* const queue = transfer->queue;
/*
  bool aborted;
  do {
          transfer->next = (void *) __ldrex((uint32_t *) &queue->free_transfers);
          aborted = __strex((uint32_t) transfer, (uint32_t *) &queue->free_transfers);
  } while (aborted);
*/
  transfer->next = NULL;
  if (queue->free_tail != NULL)
  {
    queue->free_tail->next = transfer;
  } else {
    queue->free_transfers = transfer;
  }
  queue->free_tail = transfer;
}

/* Add a transfer to the end of an endpoint's queue. Returns the old
 * tail or NULL is the queue was empty
 */
static usb_transfer_t* endpoint_queue_transfer(usb_transfer_t* const transfer)
{
  usb_queue_t* const queue = transfer->queue;
  transfer->next = NULL;
  usb_transfer_t* const previous_tail = queue->active_tail;
  if (previous_tail != NULL)
  {
      previous_tail->next = transfer;
  } else {
      queue->active = transfer;
  }
  queue->active_tail = transfer;
  return previous_tail;
}

static void usb_queue_flush_queue(usb_queue_t* const queue)
{
  const uint32_t primask = interrupt_save();
  while (queue->active)
  {
    usb_transfer_t* transfer = queue->active;
    queue->active = transfer->next;
    if (transfer->retired != NULL)
    {
      transfer->retired(
        transfer->retired_context,
        transfer->tag,
        0,
        USB_TRANSFER_RETIREMENT_CANCELLED);
    }
    free_transfer_nolock(transfer);
  }
  queue->active_tail = NULL;
  interrupt_restore(primask);
}

void usb_queue_flush_endpoint(const usb_endpoint_t* const endpoint)
{
  usb_queue_flush_queue(endpoint_queue(endpoint));
}

int usb_transfer_schedule(
  const usb_endpoint_t* const endpoint,
  void* const data,
  const uint32_t maximum_length)
{
  return usb_transfer_schedule_tagged(
    endpoint, data, maximum_length, NULL, NULL, 0);
}

int usb_transfer_schedule_tagged(
  const usb_endpoint_t* const endpoint,
  void* const data,
  const uint32_t maximum_length,
  usb_transfer_retired_fn const retired,
  void* const retired_context,
  const uint32_t tag)
{
  /*
   * A ChipIdea dTD has a 15-bit byte count and five 4 KiB page pointers.
   * Reject malformed requests instead of truncating the token or making DMA
   * walk beyond the descriptor's addressable window.
   */
  const uint32_t page_offset = (uint32_t)data & 0xfffu;
  const uint32_t addressable_bytes = (5u * 0x1000u) - page_offset;
  if ((maximum_length != 0 && data == NULL)
    || maximum_length > 0x7fffu
    || maximum_length > addressable_bytes)
  {
    return -2;
  }

  usb_queue_t* const queue = endpoint_queue(endpoint);
  usb_transfer_t* const transfer = allocate_transfer(queue);
  if (transfer == NULL) return -1;
  usb_transfer_descriptor_t* const td = &transfer->td;

  // Configure the transfer descriptor
  td->next_dtd_pointer = USB_TD_NEXT_DTD_POINTER_TERMINATE;
  td->total_bytes =
      USB_TD_DTD_TOKEN_TOTAL_BYTES(maximum_length)
    | USB_TD_DTD_TOKEN_IOC
    | USB_TD_DTD_TOKEN_MULTO(0)
    | USB_TD_DTD_TOKEN_STATUS_ACTIVE;

  /*
   * The controller advances buffer_pointer_page[0] in the dTD overlay while
   * executing a transfer. Restore it on every submission. Page bases 1..4
   * are not part of that current-offset update and may be retained while a
   * transfer object remains paired with the same buffer.
   */
  td->buffer_pointer_page[0] = (uint32_t)data;
  if (transfer->prepared_data != data
    || transfer->prepared_length != maximum_length)
  {
    td->buffer_pointer_page[1] = ((uint32_t)data + 0x1000) & 0xfffff000;
    td->buffer_pointer_page[2] = ((uint32_t)data + 0x2000) & 0xfffff000;
    td->buffer_pointer_page[3] = ((uint32_t)data + 0x3000) & 0xfffff000;
    td->buffer_pointer_page[4] = ((uint32_t)data + 0x4000) & 0xfffff000;
    transfer->prepared_data = data;
    transfer->prepared_length = maximum_length;
  }

  // Fill in transfer fields
  transfer->maximum_length = maximum_length;
  transfer->retired = retired;
  transfer->retired_context = retired_context;
  transfer->tag = tag;

  const uint32_t primask = interrupt_save();
  usb_transfer_t* tail = endpoint_queue_transfer(transfer);
  if (tail == NULL)
  {
    // The queue is currently empty, we need to re-prime
    usb_endpoint_schedule_wait(queue->endpoint, &transfer->td);
  } else {
    // The queue is currently running, try to append
    usb_endpoint_schedule_append(queue->endpoint, &tail->td, &transfer->td);
  }
  interrupt_restore(primask);
  return 0;
}

int usb_transfer_schedule_block(
  const usb_endpoint_t* const endpoint,
  void* const data,
  const uint32_t maximum_length)
{
  /*
   * Waiting for a descriptor can deadlock inside the USB ISR and stalls M0
   * progress outside it. Report exhaustion immediately. EP0 stalls until a
   * new SETUP abort boundary; non-control callers drop this piece of work and
   * remain live.
   */
  const int result =
    usb_transfer_schedule(endpoint, data, maximum_length);
  if (result != 0 && (endpoint->address & 0x0fu) == 0)
  {
    usb_endpoint_stall(endpoint);
  }
  return result;
}

int usb_transfer_schedule_ack(const usb_endpoint_t* const endpoint)
{
  return usb_transfer_schedule_block(endpoint, 0, 0);
}

/* Called when an endpoint might have completed a transfer */
void usb_queue_transfer_complete(usb_endpoint_t* const endpoint)
{
  usb_queue_t* const queue = endpoint_queue(endpoint);
  usb_transfer_t* transfer = queue->active;
  uint32_t retired = 0;
  uint32_t errors = 0;

  while (transfer != NULL)
  {
    const uint32_t token = transfer->td.total_bytes;
    const uint8_t status = token;
    usb_transfer_retirement_status_t retirement_status =
      USB_TRANSFER_RETIREMENT_COMPLETE;

    // Check for failures
    if (status & USB_TD_DTD_TOKEN_STATUS_HALTED)
    {
      retirement_status = USB_TRANSFER_RETIREMENT_HALTED;
    }
    else if (status & USB_TD_DTD_TOKEN_STATUS_BUFFER_ERROR)
    {
      retirement_status = USB_TRANSFER_RETIREMENT_BUFFER_ERROR;
    }
    else if (status & USB_TD_DTD_TOKEN_STATUS_TRANSACTION_ERROR)
    {
      retirement_status = USB_TRANSFER_RETIREMENT_TRANSACTION_ERROR;
    }

    if (retirement_status != USB_TRANSFER_RETIREMENT_COMPLETE)
    {
      errors++;
    }

    // Still not finished
    if (status & USB_TD_DTD_TOKEN_STATUS_ACTIVE)
            break;

    // Advance the head.
    queue->active = transfer->next;
    usb_transfer_t* next = transfer->next;
    if (next == NULL)
    {
      queue->active_tail = NULL;
    }

    const uint32_t remaining =
      (token & USB_TD_DTD_TOKEN_TOTAL_BYTES_MASK)
      >> USB_TD_DTD_TOKEN_TOTAL_BYTES_SHIFT;
    const uint32_t actual_length =
      remaining <= transfer->maximum_length
      ? transfer->maximum_length - remaining
      : 0;
    if (transfer->retired != NULL)
    {
      transfer->retired(
        transfer->retired_context,
        transfer->tag,
        actual_length,
        retirement_status);
    }

    // Free transfer
    free_transfer(transfer);
    transfer = next;
    retired++;
  }

  /*
   * A halted/error dTD stops a non-control endpoint's hardware queue. Leaving
   * later ACTIVE descriptors linked would strand every buffer indefinitely.
   * Cancel and flush the remainder; the next producer submission will prime
   * an empty queue and usb_endpoint_prime() clears the queue-head halt state.
   */
  if (errors != 0 && (endpoint->address & 0x0fu) != 0)
  {
    usb_endpoint_flush(endpoint);
  }

  if (endpoint->address == 0x81)
  {
#ifdef AIRSPY_USB_QUEUE_TELEMETRY
    usb_atdtw_telemetry_record_completion(retired, errors);
#endif
  }
}
