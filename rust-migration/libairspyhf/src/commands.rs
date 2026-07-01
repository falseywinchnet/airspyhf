/*
Copyright (c) 2016-2023, Youssef Touil <youssef@airspy.com>
Copyright (C) 2024, Joshuah Rainstar <joshuah.rainstar@gmail.com>

Rust port of `airspyhf_commands.h`: the vendor control-transfer request codes
understood by the AirspyHF+ firmware, plus a couple of protocol constants that
travel with them.
*/

//! USB vendor request codes (mirrors `airspyhf_commands.h`).

/// Receiver mode payload values for [`AIRSPYHF_RECEIVER_MODE`].
pub const RECEIVER_MODE_OFF: u16 = 0;
pub const RECEIVER_MODE_ON: u16 = 1;

pub const AIRSPYHF_RECEIVER_MODE: u8 = 1;
pub const AIRSPYHF_SET_FREQ: u8 = 2;
pub const AIRSPYHF_GET_SAMPLERATES: u8 = 3;
pub const AIRSPYHF_SET_SAMPLERATE: u8 = 4;
pub const AIRSPYHF_CONFIG_READ: u8 = 5;
pub const AIRSPYHF_CONFIG_WRITE: u8 = 6;
pub const AIRSPYHF_GET_SERIALNO_BOARDID: u8 = 7;
pub const AIRSPYHF_SET_USER_OUTPUT: u8 = 8;
pub const AIRSPYHF_GET_VERSION_STRING: u8 = 9;
pub const AIRSPYHF_SET_AGC: u8 = 10;
pub const AIRSPYHF_SET_AGC_THRESHOLD: u8 = 11;
pub const AIRSPYHF_SET_ATT: u8 = 12;
pub const AIRSPYHF_SET_LNA: u8 = 13;
pub const AIRSPYHF_GET_SAMPLERATE_ARCHITECTURES: u8 = 14;
pub const AIRSPYHF_GET_FILTER_GAIN: u8 = 15;
pub const AIRSPYHF_GET_FREQ_DELTA: u8 = 16;
pub const AIRSPYHF_SET_VCTCXO_CALIBRATION: u8 = 17;
pub const AIRSPYHF_SET_FRONTEND_OPTIONS: u8 = 18;
pub const AIRSPYHF_GET_ATT_STEPS: u8 = 19;
pub const AIRSPYHF_GET_BIAS_TEE_COUNT: u8 = 20;
pub const AIRSPYHF_GET_BIAS_TEE_NAME: u8 = 21;
pub const AIRSPYHF_SET_BIAS_TEE: u8 = 22;

/// Magic number stamped into the on-device flash calibration record.
pub const CALIBRATION_MAGIC: u32 = 0xA5CA71B0;
/// Maximum length of the firmware version / bias-tee name strings.
pub const MAX_VERSION_STRING_SIZE: usize = 255;
