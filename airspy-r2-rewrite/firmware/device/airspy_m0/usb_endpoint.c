/*
 * Copyright 2012 Jared Boone
 * Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
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
#include "usb_endpoint.h"
#include <usb_request.h>
#include "usb_device.h"
#include <stream_contract.h>

usb_endpoint_t usb_endpoint_control_out = {
  .address = 0x00,
  .device = &usb_device,
  .in = &usb_endpoint_control_in,
  .out = &usb_endpoint_control_out,
  .setup_complete = usb_setup_complete,
  .transfer_complete = usb_control_out_complete,
};
USB_DEFINE_QUEUE(usb_endpoint_control_out, 4);

usb_endpoint_t usb_endpoint_control_in = {
  .address = 0x80,
  .device = &usb_device,
  .in = &usb_endpoint_control_in,
  .out = &usb_endpoint_control_out,
  .setup_complete = 0,
  .transfer_complete = usb_control_in_complete,
};
static USB_DEFINE_QUEUE(usb_endpoint_control_in, 4);

usb_endpoint_t usb_endpoint_bulk_in = {
  .address = 0x81,
  .device = &usb_device,
  .in = &usb_endpoint_bulk_in,
  .out = 0,
  .setup_complete = 0,
  .transfer_complete = usb_queue_transfer_complete,
};
struct _usb_transfer_t
usb_endpoint_bulk_in_transfers[AIRSPY_STREAM_BUFFER_COUNT]
  ATTR_ALIGNED(64) ATTR_SECTION(".usb_dma_metadata");
struct _usb_queue_t usb_endpoint_bulk_in_queue = {
  .endpoint = &usb_endpoint_bulk_in,
  .free_transfers = usb_endpoint_bulk_in_transfers,
  .pool_size = AIRSPY_STREAM_BUFFER_COUNT
};

usb_endpoint_t usb_endpoint_bulk_out = {
  .address = 0x02,
  .device = &usb_device,
  .in = 0,
  .out = &usb_endpoint_bulk_out,
  .setup_complete = 0,
  .transfer_complete = usb_queue_transfer_complete,
};
static USB_DEFINE_QUEUE(usb_endpoint_bulk_out, 1);
