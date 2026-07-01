// Port of tools/src/airspyhf_calibrate.c: read (and optionally set + flash) the
// receiver's VCTCXO ppb calibration.

use std::process::ExitCode;

use airspyhf::{
    airspyhf_board_partid_serialno_read, airspyhf_flash_configuration, airspyhf_get_calibration,
    airspyhf_list_devices, airspyhf_open_sn, airspyhf_set_calibration, AirspyhfDeviceHandle,
    AirspyhfReadPartIdSerialNo,
};

const AIRSPYHF_SUCCESS: i32 = 0;

fn usage() {
    println!("Usage:");
    println!("\t-s <serial number>: open receiver with specified 64-bit serial number (required).");
    println!("\t-c <new ppb>: set receiver with specified new calibration value (optional, signed decimal).");
}

fn parse_hex_serial(s: &str) -> Option<u64> {
    let hex = s.trim_start_matches("0x").trim_start_matches("0X");
    u64::from_str_radix(hex, 16).ok()
}

fn print_receiver_data(dev: AirspyhfDeviceHandle) {
    unsafe {
        let mut ids = AirspyhfReadPartIdSerialNo {
            part_id: 0,
            serial_no: [0; 4],
        };
        if airspyhf_board_partid_serialno_read(dev, &mut ids) == AIRSPYHF_SUCCESS {
            println!("S/N: 0x{:08X}{:08X}", ids.serial_no[0], ids.serial_no[1]);
        } else {
            eprintln!("airspyhf_board_partid_serialno_read() failed");
        }
        let mut ppb: i32 = 0;
        if airspyhf_get_calibration(dev, &mut ppb) == AIRSPYHF_SUCCESS {
            println!("Calibration = {ppb}");
        } else {
            eprintln!("airspyhf_get_calibration() failed");
        }
    }
}

fn write_new_calibration_value(dev: AirspyhfDeviceHandle, new_ppb: i32) {
    unsafe {
        if airspyhf_set_calibration(dev, new_ppb) == AIRSPYHF_SUCCESS {
            println!("New Calibration = {new_ppb}");
        } else {
            eprintln!("airspyhf_set_calibration() failed");
        }
        if airspyhf_flash_configuration(dev) == AIRSPYHF_SUCCESS {
            println!("Flash Calibration successfully done");
        } else {
            eprintln!("airspyhf_flash_calibration() failed");
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let mut opts = getopts::Options::new();
    opts.optopt("s", "", "serial number", "SN");
    opts.optopt("c", "", "new calibration ppb", "PPB");
    opts.optflag("h", "help", "show this help");

    let matches = match opts.parse(&args[1..]) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("{e}");
            usage();
            return ExitCode::FAILURE;
        }
    };
    if matches.opt_present("h") {
        usage();
        return ExitCode::FAILURE;
    }

    let serial = match matches.opt_str("s").as_deref().map(parse_hex_serial) {
        Some(Some(sn)) => sn,
        Some(None) => {
            eprintln!("argument error: bad serial number");
            usage();
            return ExitCode::FAILURE;
        }
        None => {
            usage();
            return ExitCode::FAILURE;
        }
    };

    let new_ppb = match matches.opt_str("c") {
        Some(c) => match c.parse::<i32>() {
            Ok(v) => Some(v),
            Err(_) => {
                eprintln!("argument error: '-c {c}'");
                usage();
                return ExitCode::FAILURE;
            }
        },
        None => None,
    };

    let ndev = unsafe { airspyhf_list_devices(std::ptr::null_mut(), 0) };
    if ndev <= 0 {
        eprintln!("No devices attached.");
        return ExitCode::FAILURE;
    }

    let mut dev: AirspyhfDeviceHandle = std::ptr::null_mut();
    if unsafe { airspyhf_open_sn(&mut dev, serial) } == AIRSPYHF_SUCCESS {
        print_receiver_data(dev);
        if let Some(v) = new_ppb {
            write_new_calibration_value(dev, v);
        }
        unsafe { airspyhf::airspyhf_close(dev) };
        ExitCode::SUCCESS
    } else {
        eprintln!("Unable to open device with S/N 0x{serial:016X}");
        ExitCode::FAILURE
    }
}
