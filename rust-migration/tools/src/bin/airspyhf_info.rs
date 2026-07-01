// Port of tools/src/airspyhf_info.c: enumerate AirspyHF+ receivers and print
// their serial number, part id, firmware version and supported sample rates.

use std::ffi::CStr;
use std::os::raw::c_char;
use std::process::ExitCode;

use airspyhf::{
    airspyhf_board_partid_serialno_read, airspyhf_close, airspyhf_get_samplerates,
    airspyhf_lib_version, airspyhf_list_devices, airspyhf_open_sn, airspyhf_version_string_read,
    AirspyhfDeviceHandle, AirspyhfLibVersion, AirspyhfReadPartIdSerialNo,
};

const AIRSPYHF_SUCCESS: i32 = 0;

fn usage(prog: &str, opts: &getopts::Options) {
    let brief = format!(
        "Usage: {prog} [-s <serial>]\n\
         \t-s serial number: open receiver with specified 64 bits serial number."
    );
    print!("{}", opts.usage(&brief));
}

/// Print all receiver data available from the library, then close the device.
fn print_receiver_data(dev: AirspyhfDeviceHandle) {
    unsafe {
        let mut ids = AirspyhfReadPartIdSerialNo {
            part_id: 0,
            serial_no: [0; 4],
        };
        if airspyhf_board_partid_serialno_read(dev, &mut ids) == AIRSPYHF_SUCCESS {
            println!("S/N: 0x{:08X}{:08X}", ids.serial_no[0], ids.serial_no[1]);
            println!("Part ID: 0x{:08X}", ids.part_id);
        } else {
            eprintln!("airspyhf_board_partid_serialno_read() failed");
        }

        let mut vstr = [0u8; 255];
        if airspyhf_version_string_read(dev, vstr.as_mut_ptr() as *mut c_char, vstr.len() as u8)
            == AIRSPYHF_SUCCESS
        {
            let s = CStr::from_ptr(vstr.as_ptr() as *const c_char).to_string_lossy();
            println!("Firmware Version: {s}");
        } else {
            eprintln!("airspyhf_version_string_read() failed");
        }

        let mut nsrates: u32 = 0;
        if airspyhf_get_samplerates(dev, &mut nsrates, 0) == AIRSPYHF_SUCCESS && nsrates > 0 {
            let mut rates = vec![0u32; nsrates as usize];
            print!(
                "Available sample rate{}",
                if nsrates > 1 { "s:" } else { ":" }
            );
            if airspyhf_get_samplerates(dev, rates.as_mut_ptr(), nsrates) == AIRSPYHF_SUCCESS {
                for r in &rates {
                    print!(" {} kS/s", r / 1000);
                }
                println!();
            }
            println!();
        } else {
            eprintln!("airspyhf_get_samplerates() failed");
        }

        if airspyhf_close(dev) != AIRSPYHF_SUCCESS {
            eprintln!("airspyhf_close() board failed");
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let mut opts = getopts::Options::new();
    opts.optopt(
        "s",
        "",
        "open receiver with specified 64-bit serial number",
        "SN",
    );
    opts.optflag("h", "help", "show this help");

    let matches = match opts.parse(&args[1..]) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("{e}");
            usage(&args[0], &opts);
            return ExitCode::FAILURE;
        }
    };
    if matches.opt_present("h") {
        usage(&args[0], &opts);
        return ExitCode::FAILURE;
    }

    let serial = match matches.opt_str("s") {
        Some(s) => match parse_hex_serial(&s) {
            Some(sn) => {
                println!(
                    "Receiver serial number to be opened: 0x{:08X}{:08X}",
                    (sn >> 32) as u32,
                    (sn & 0xFFFF_FFFF) as u32
                );
                Some(sn)
            }
            None => {
                eprintln!("argument error: '-s {s}'");
                usage(&args[0], &opts);
                return ExitCode::FAILURE;
            }
        },
        None => None,
    };

    // Print the library version.
    let mut libv = AirspyhfLibVersion {
        major_version: 0,
        minor_version: 0,
        revision: 0,
    };
    unsafe { airspyhf_lib_version(&mut libv) };
    println!(
        "AirSpy HF library version: {}.{}.{}",
        libv.major_version, libv.minor_version, libv.revision
    );

    let ndev = unsafe { airspyhf_list_devices(std::ptr::null_mut(), 0) };
    if ndev <= 0 {
        eprintln!("No devices attached.");
        return ExitCode::FAILURE;
    }
    eprintln!();

    if let Some(sn) = serial {
        let mut dev: AirspyhfDeviceHandle = std::ptr::null_mut();
        if unsafe { airspyhf_open_sn(&mut dev, sn) } == AIRSPYHF_SUCCESS {
            print_receiver_data(dev);
            ExitCode::SUCCESS
        } else {
            eprintln!("Unable to open device with S/N 0x{sn:016X}");
            ExitCode::FAILURE
        }
    } else {
        let mut serials = vec![0u64; ndev as usize];
        if unsafe { airspyhf_list_devices(serials.as_mut_ptr(), ndev) } > 0 {
            for (n, &sn) in serials.iter().enumerate() {
                let mut dev: AirspyhfDeviceHandle = std::ptr::null_mut();
                if unsafe { airspyhf_open_sn(&mut dev, sn) } == AIRSPYHF_SUCCESS {
                    print_receiver_data(dev);
                } else {
                    eprintln!("airspyhf_open() receiver #{} failed", n + 1);
                    break;
                }
            }
        }
        ExitCode::SUCCESS
    }
}

/// Parse a `0x`-prefixed (or bare) hexadecimal 64-bit serial number.
fn parse_hex_serial(s: &str) -> Option<u64> {
    let hex = s.trim_start_matches("0x").trim_start_matches("0X");
    u64::from_str_radix(hex, 16).ok()
}
