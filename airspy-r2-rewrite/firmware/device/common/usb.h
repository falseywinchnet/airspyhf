/*
 * Copyright 2012 Jared Boone
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

#ifndef __USB_H__
#define __USB_H__

// TODO: Refactor to support high performance operations without having to
// expose usb_transfer_descriptor_t. Or usb_endpoint_prime(). Or, or, or...
#include <libopencm3/lpc43xx/usb.h>

#include "usb_type.h"

typedef struct {
  uint32_t append_calls;
  uint32_t prime_in_progress_exits;
  uint32_t tripwire_samples;
  uint32_t tripwire_retries;
  uint32_t endpoint_active_exits;
  uint32_t endpoint_reprime_exits;
  uint32_t completion_callbacks;
  uint32_t descriptors_retired;
  uint32_t maximum_retired_per_callback;
  uint32_t descriptor_errors;
} usb_atdtw_telemetry_t;

typedef void (*usb_suspend_changed_fn)(const bool suspended);
typedef void (*usb_fatal_error_fn)(void);
typedef void (*usb_bus_event_fn)(const uint32_t status);

void usb_set_suspend_changed_cb(usb_suspend_changed_fn const callback);
void usb_set_fatal_error_cb(usb_fatal_error_fn const callback);
void usb_set_bus_event_cb(usb_bus_event_fn const callback);
void usb_fatal_error_reset(void);

void usb_atdtw_telemetry_reset(void);
void usb_atdtw_telemetry_snapshot(usb_atdtw_telemetry_t* const destination);
void usb_atdtw_telemetry_record_completion(
  const uint32_t retired,
  const uint32_t errors);

void usb_peripheral_reset();

void usb_device_init(
  const uint_fast8_t device_ordinal,
  usb_device_t* const device
);

void usb_run(
  usb_device_t* const device
);

void usb_run_tasks(
  const usb_device_t* const device
);

usb_speed_t usb_speed(
  const usb_device_t* const device
);

void usb_set_address_immediate(
  const usb_device_t* const device,
  const uint_fast8_t address
);

void usb_set_address_deferred(
  const usb_device_t* const device,
  const uint_fast8_t address
);

void usb_endpoint_init(
  const usb_endpoint_t* const endpoint
);

void usb_endpoint_resume(
  const usb_endpoint_t* const endpoint
);

void usb_endpoint_stall(
  const usb_endpoint_t* const endpoint
);

void usb_endpoint_disable(
  const usb_endpoint_t* const endpoint
);

void usb_endpoint_flush(
  const usb_endpoint_t* const endpoint
);

bool usb_endpoint_is_ready(
  const usb_endpoint_t* const endpoint
);

void usb_endpoint_prime(
  const usb_endpoint_t* const endpoint,
  usb_transfer_descriptor_t* const first_td
);

void usb_endpoint_schedule_wait(
  const usb_endpoint_t* const endpoint,
  usb_transfer_descriptor_t* const td
);

void usb_endpoint_schedule_append(
  const usb_endpoint_t* const endpoint,
  usb_transfer_descriptor_t* const tail_td,
  usb_transfer_descriptor_t* const new_td
);

#endif//__USB_H__
