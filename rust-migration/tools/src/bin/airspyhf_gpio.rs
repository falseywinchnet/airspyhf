// Port of tools/src/airspyhf_gpio.c: set the four user-output GPIO pins.

use std::process::ExitCode;

use airspyhf::{
    airspyhf_close, airspyhf_lib_version, airspyhf_list_devices, airspyhf_open_sn,
    airspyhf_set_user_output, AirspyhfDeviceHandle, AirspyhfLibVersion,
};

const AIRSPYHF_SUCCESS: i32 = 0;

fn usage() {
    eprintln!(
        "Usage:\n\
         \t-s serial number: open receiver with specified 64 bits serial number.\n\
         \t-0 on|off\n\
         \t-1 on|off\n\
         \t-2 on|off\n\
         \t-3 on|off"
    );
}

fn parse_hex_serial(s: &str) -> Option<u64> {
    let hex = s.trim_start_matches("0x").trim_start_matches("0X");
    u64::from_str_radix(hex, 16).ok()
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let mut opts = getopts::Options::new();
    opts.optopt("s", "", "serial number", "SN");
    for pin in 0..4 {
        opts.optopt(&pin.to_string(), "", "GPIO on|off", "STATE");
    }
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

    // Default all pins LOW; override from -0..-3.
    let mut states = [0u32; 4];
    for (pin, state) in states.iter_mut().enumerate() {
        if let Some(val) = matches.opt_str(&pin.to_string()) {
            *state = match val.as_str() {
                "on" => 1,
                "off" => 0,
                other => {
                    eprintln!("Bad GPIO status: {other}.");
                    usage();
                    return ExitCode::FAILURE;
                }
            };
        }
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
                usage();
                return ExitCode::FAILURE;
            }
        },
        None => None,
    };

    let mut libv = AirspyhfLibVersion {
        major_version: 0,
        minor_version: 0,
        revision: 0,
    };
    unsafe { airspyhf_lib_version(&mut libv) };
    eprintln!(
        "AirSpy HF library version: {}.{}.{}",
        libv.major_version, libv.minor_version, libv.revision
    );

    let ndev = unsafe { airspyhf_list_devices(std::ptr::null_mut(), 0) };
    if ndev <= 0 {
        eprintln!("No devices attached.");
        return ExitCode::FAILURE;
    }

    // Resolve which device to drive. Mirroring the C tool, without -s it opens
    // the last enumerated device.
    let mut dev: AirspyhfDeviceHandle = std::ptr::null_mut();
    match serial {
        Some(sn) => {
            if unsafe { airspyhf_open_sn(&mut dev, sn) } != AIRSPYHF_SUCCESS {
                eprintln!("Unable to open device with S/N 0x{sn:016X}");
                return ExitCode::FAILURE;
            }
        }
        None => {
            let mut serials = vec![0u64; ndev as usize];
            if unsafe { airspyhf_list_devices(serials.as_mut_ptr(), ndev) } > 0 {
                for (n, &sn) in serials.iter().enumerate() {
                    if unsafe { airspyhf_open_sn(&mut dev, sn) } != AIRSPYHF_SUCCESS {
                        eprintln!("airspyhf_open() receiver #{} failed", n + 1);
                        return ExitCode::FAILURE;
                    }
                }
            }
        }
    }

    for (n, &state) in states.iter().enumerate() {
        if unsafe { airspyhf_set_user_output(dev, n as u32, state) } != AIRSPYHF_SUCCESS {
            eprintln!("Error setting GPIO #{n}");
            unsafe { airspyhf_close(dev) };
            return ExitCode::FAILURE;
        }
        eprintln!("GPIO #{n}: {state}");
    }

    unsafe { airspyhf_close(dev) };
    ExitCode::SUCCESS
}
