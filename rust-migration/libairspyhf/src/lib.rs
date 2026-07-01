/*
Copyright (c) 2016-2023, Youssef Touil <youssef@airspy.com>
Copyright (c) 2018, Leif Asbrink <leif@sm5bsz.com>
Copyright (C) 2024, Joshuah Rainstar <joshuah.rainstar@gmail.com>
Contributions to this work were provided by OpenAI Codex, an artificial general intelligence.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of Airspy HF+ nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//! Rust port of the AirspyHF+ driver.
//!
//! The crate exposes the same C ABI as the reference `libairspyhf` so existing
//! applications link against it unchanged, while the implementation is safe(ish)
//! Rust. The module layout mirrors the C++ sources:
//!
//! | Rust module     | C++ source            |
//! |-----------------|-----------------------|
//! | [`commands`]    | `airspyhf_commands.h` |
//! | [`device`]      | `airspyhf.cpp`        |
//! | [`iqbalancer`]  | `iqbalancer.cpp`      |
//! | [`ffi`]         | exported `airspyhf.h` |
//!
//! The shared C types and version constants live here in the crate root.
//!
//! # Safety posture
//!
//! `unsafe` is confined to the FFI surface ([`ffi`]) and the USB/thread glue in
//! [`device`]; the DSP core ([`iqbalancer`]) and [`commands`] are safe Rust. Every
//! `unsafe` operation is written in an explicit block with a `// SAFETY:` note
//! (`unsafe_op_in_unsafe_fn` is forbidden crate-wide), and no Rust panic can
//! unwind across the C ABI (every entry point is panic-guarded). See
//! `VERIFICATION.md` for the recorded evidence chain (tests, Miri, Loom, Kani).

#![forbid(unsafe_op_in_unsafe_fn)]
#![deny(unused_must_use)]

use std::ffi::c_void;

pub mod commands;
pub mod device;
pub mod ffi;
pub mod iqbalancer;

pub use device::AirspyHfDevice;
pub use ffi::*;
pub use iqbalancer::IqBalancer;

pub const AIRSPYHF_VER_MAJOR: u32 = 1;
pub const AIRSPYHF_VER_MINOR: u32 = 8;
pub const AIRSPYHF_VER_REVISION: u32 = 0;

/// Number of complex samples delivered to the callback per block.
pub const SAMPLES_TO_TRANSFER: i32 = 1024 * 4;

/// `airspyhf_complex_float_t`.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AirspyhfComplexFloat {
    pub re: f32,
    pub im: f32,
}

/// `airspyhf_lib_version_t`.
#[repr(C)]
pub struct AirspyhfLibVersion {
    pub major_version: u32,
    pub minor_version: u32,
    pub revision: u32,
}

/// `airspyhf_read_partid_serialno_t`.
#[repr(C)]
pub struct AirspyhfReadPartIdSerialNo {
    pub part_id: u32,
    pub serial_no: [u32; 4],
}

/// `airspyhf_transfer_t` handed to the streaming callback.
#[repr(C)]
pub struct AirspyhfTransfer {
    pub device: *mut AirspyHfDevice,
    pub ctx: *mut c_void,
    pub samples: *mut AirspyhfComplexFloat,
    pub sample_count: i32,
    pub dropped_samples: u64,
}

/// `airspyhf_sample_block_cb_fn`.
pub type AirspyhfSampleBlockCbFn = Option<unsafe extern "C" fn(*mut AirspyhfTransfer) -> i32>;
