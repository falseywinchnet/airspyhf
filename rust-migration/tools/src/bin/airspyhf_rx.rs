// Port of tools/src/airspyhf_rx.c: stream IQ samples to a file (raw or WAV) or
// to stdout, with optional sample limit, AGC/LNA/attenuator control and a live
// throughput readout.

use std::ffi::c_void;
use std::fs::{File, OpenOptions};
use std::io::{self, BufWriter, Seek, SeekFrom, Write};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use airspyhf::{
    airspyhf_board_partid_serialno_read, airspyhf_close, airspyhf_get_samplerates,
    airspyhf_is_streaming, airspyhf_open, airspyhf_open_sn, airspyhf_set_freq, airspyhf_set_hf_agc,
    airspyhf_set_hf_agc_threshold, airspyhf_set_hf_att, airspyhf_set_hf_lna,
    airspyhf_set_samplerate, airspyhf_start, airspyhf_stop, AirspyhfDeviceHandle,
    AirspyhfReadPartIdSerialNo, AirspyhfTransfer,
};

const AIRSPYHF_SUCCESS: i32 = 0;
const DEFAULT_FREQ_HZ: u32 = 7_100_000;
const FD_BUFFER_SIZE: usize = 16 * 1024;
const WAV_HEADER_LEN: u64 = 44;

/// State shared between the callback thread and main (lock-free).
struct Shared {
    do_exit: AtomicBool,
    avg_rate_bits: AtomicU32,
}

/// Context passed to the streaming callback via the `ctx` pointer.
struct RxContext {
    writer: Box<dyn Write + Send>,
    limit_num_samples: bool,
    bytes_to_xfer: u64,
    got_first: bool,
    t_start: Instant,
    time_start: Instant,
    buffer_count: u32,
    sample_count_acc: u32,
    average_rate: f32,
    global_average_rate: f32,
    rate_samples: u32,
    shared: Arc<Shared>,
}

extern "C" fn rx_callback(transfer: *mut AirspyhfTransfer) -> i32 {
    let t = unsafe { &mut *transfer };
    if t.ctx.is_null() {
        return -1;
    }
    let ctx = unsafe { &mut *(t.ctx as *mut RxContext) };

    // #samples * float size * (I + Q)
    let mut bytes_to_write = (t.sample_count as usize) * 4 * 2;
    let now = Instant::now();

    if !ctx.got_first {
        ctx.t_start = now;
        ctx.time_start = now;
        ctx.got_first = true;
    } else {
        ctx.buffer_count += 1;
        ctx.sample_count_acc += t.sample_count as u32;
        if ctx.buffer_count == 50 {
            let dt = now.duration_since(ctx.time_start).as_secs_f32();
            let rate = if dt > 0.0 {
                ctx.sample_count_acc as f32 / dt
            } else {
                0.0
            };
            ctx.average_rate += 0.2 * (rate - ctx.average_rate);
            ctx.global_average_rate += ctx.average_rate;
            ctx.rate_samples += 1;
            ctx.time_start = now;
            ctx.sample_count_acc = 0;
            ctx.buffer_count = 0;
            ctx.shared
                .avg_rate_bits
                .store(ctx.average_rate.to_bits(), Ordering::Relaxed);
        }
    }

    if ctx.limit_num_samples {
        if bytes_to_write as u64 >= ctx.bytes_to_xfer {
            bytes_to_write = ctx.bytes_to_xfer as usize;
        }
        ctx.bytes_to_xfer -= bytes_to_write as u64;
    }

    let mut write_ok = true;
    if !t.samples.is_null() && bytes_to_write > 0 {
        let buf = unsafe { std::slice::from_raw_parts(t.samples as *const u8, bytes_to_write) };
        write_ok = ctx.writer.write_all(buf).is_ok();
    }

    if !write_ok || (ctx.limit_num_samples && ctx.bytes_to_xfer == 0) {
        -1
    } else {
        0
    }
}

fn usage() {
    eprint!(
        "airspyhf_rx\n\
Usage:\n\
\t-r <filename>\t\tReceive data into the file; use 'stdout' to emit on standard output\n\
\t-s <serial number>\tOpen device with specified 64bits serial number\n\
\t-f <frequency>\t\tSet frequency in MHz between 9 kHz - 31 MHz or 60 - 260 MHz\n\
\t-a <sample_rate>\tSet sample rate\n\
\t-n <#samples>\t\tNumber of samples to transfer (default is unlimited)\n\
\t-d\t\t\tVerbose mode\n\
\t-w\t\t\tReceive data into file with WAV header and automatic name (SDR# compatible)\n\
\t-g on|off\t\tHF AGC on / off\n\
\t-l high|low\t\tHF AGC threshold high / low (when AGC On)\n\
\t-t <value>\t\tHF attenuator value 0..8 (each step increases 6 dB the attenuation)\n\
\t-m on|off\t\ton to activate LNA (preamplifier): +6 dB gain - compensated in digital.\n\
\t-z\t\t\tDo not attempt to use manual AGC/LNA commands\n"
    );
}

fn parse_u64(s: &str) -> Option<u64> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else if let Some(bin) = s.strip_prefix("0b").or_else(|| s.strip_prefix("0B")) {
        u64::from_str_radix(bin, 2).ok()
    } else {
        s.parse().ok()
    }
}

fn build_wav_header(sample_rate: u32, data_len: u64) -> [u8; WAV_HEADER_LEN as usize] {
    let mut h = [0u8; WAV_HEADER_LEN as usize];
    let file_len = data_len + WAV_HEADER_LEN;
    h[0..4].copy_from_slice(b"RIFF");
    h[4..8].copy_from_slice(&((file_len - 8) as u32).to_le_bytes());
    h[8..12].copy_from_slice(b"WAVE");
    h[12..16].copy_from_slice(b"fmt ");
    h[16..20].copy_from_slice(&16u32.to_le_bytes());
    h[20..22].copy_from_slice(&3u16.to_le_bytes()); // Float32
    h[22..24].copy_from_slice(&2u16.to_le_bytes()); // I + Q
    h[24..28].copy_from_slice(&sample_rate.to_le_bytes());
    h[28..32].copy_from_slice(&(sample_rate * 4).to_le_bytes());
    h[32..34].copy_from_slice(&8u16.to_le_bytes()); // block align = 2 * 32/8
    h[34..36].copy_from_slice(&32u16.to_le_bytes()); // bits per sample
    h[36..40].copy_from_slice(b"data");
    h[40..44].copy_from_slice(&(data_len as u32).to_le_bytes());
    h
}

struct Args {
    path: Option<String>,
    receive_wav: bool,
    serial: Option<u64>,
    freq_hz: Option<u32>,
    sample_rate: u32,
    samples_to_xfer: Option<u64>,
    verbose: bool,
    hf_agc: bool,
    hf_agc_threshold: bool,
    hf_att: u8,
    hf_lna: bool,
    no_manual: bool,
}

fn parse_args() -> Result<Args, ()> {
    let argv: Vec<String> = std::env::args().collect();
    let mut opts = getopts::Options::new();
    opts.optopt("r", "", "output file (or 'stdout')", "FILE");
    opts.optflag("w", "", "WAV output with auto name");
    opts.optopt("s", "", "serial number", "SN");
    opts.optopt("f", "", "frequency MHz", "FREQ");
    opts.optopt("a", "", "sample rate", "RATE");
    opts.optopt("n", "", "number of samples", "N");
    opts.optopt("g", "", "HF AGC on|off", "STATE");
    opts.optopt("l", "", "AGC threshold high|low", "LEVEL");
    opts.optopt("t", "", "HF attenuator 0..8", "VAL");
    opts.optopt("m", "", "LNA on|off", "STATE");
    opts.optflag("d", "", "verbose");
    opts.optflag("z", "", "do not use manual AGC/LNA commands");
    opts.optflag("h", "help", "show this help");

    let m = opts.parse(&argv[1..]).map_err(|e| {
        eprintln!("{e}");
        usage();
    })?;
    if m.opt_present("h") {
        usage();
        return Err(());
    }

    let freq_hz = match m.opt_str("f") {
        Some(f) => match f.parse::<f64>() {
            Ok(mhz) => Some((mhz * 1_000_000.0) as u32),
            Err(_) => {
                eprintln!("argument error: '-f {f}'");
                usage();
                return Err(());
            }
        },
        None => None,
    };
    let sample_rate = match m.opt_str("a") {
        Some(a) => parse_u64(&a).map(|v| v as u32).ok_or_else(|| {
            eprintln!("argument error: '-a {a}'");
            usage();
        })?,
        None => 768_000,
    };
    let serial = match m.opt_str("s") {
        Some(s) => Some(parse_u64(&s).ok_or_else(|| {
            eprintln!("argument error: '-s {s}'");
            usage();
        })?),
        None => None,
    };
    let samples_to_xfer = match m.opt_str("n") {
        Some(n) => Some(parse_u64(&n).ok_or_else(|| {
            eprintln!("argument error: '-n {n}'");
            usage();
        })?),
        None => None,
    };
    let hf_att = match m.opt_str("t") {
        Some(t) => {
            let v: i32 = t.parse().map_err(|_| {
                eprintln!("argument error: '-t {t}'");
                usage();
            })?;
            if !(0..=8).contains(&v) {
                eprintln!("Bad HF attenuator value.");
                usage();
                return Err(());
            }
            v as u8
        }
        None => 0,
    };

    Ok(Args {
        path: m.opt_str("r"),
        receive_wav: m.opt_present("w"),
        serial,
        freq_hz,
        sample_rate,
        samples_to_xfer,
        verbose: m.opt_present("d"),
        hf_agc: m.opt_str("g").as_deref() != Some("off"),
        hf_agc_threshold: m.opt_str("l").as_deref() == Some("high"),
        hf_att,
        hf_lna: m.opt_str("m").as_deref() == Some("on"),
        no_manual: m.opt_present("z"),
    })
}

fn run() -> Result<(), ()> {
    let args = parse_args()?;

    // Resolve frequency (default 7.1 MHz) and validate the HF/VHF ranges.
    let freq_hz = args.freq_hz.unwrap_or(DEFAULT_FREQ_HZ);
    if args.freq_hz.is_some()
        && !((9_000..=31_000_000).contains(&freq_hz)
            || (60_000_000..=260_000_000).contains(&freq_hz))
    {
        eprintln!("argument error: frequency {freq_hz} Hz out of range");
        usage();
        return Err(());
    }

    // Determine output path (auto-named for -w).
    let (path, is_wav) = if args.receive_wav {
        let name = format!("AirSpy_{}Z_{}kHz_IQ.wav", timestamp(), freq_hz / 1000);
        eprintln!("Receive wav file: [{name}]");
        (name, true)
    } else {
        match args.path.clone() {
            Some(p) => (p, false),
            None => {
                eprintln!("error: you shall specify at least -r <with filename> or -w option");
                usage();
                return Err(());
            }
        }
    };

    // Open the device.
    let mut device: AirspyhfDeviceHandle = std::ptr::null_mut();
    let opened = match args.serial {
        Some(sn) => unsafe { airspyhf_open_sn(&mut device, sn) },
        None => unsafe { airspyhf_open(&mut device) },
    };
    if opened != AIRSPYHF_SUCCESS {
        eprintln!("airspyhf_open() failed");
        return Err(());
    }

    // Validate + set sample rate.
    let mut nsrates: u32 = 0;
    unsafe { airspyhf_get_samplerates(device, &mut nsrates, 0) };
    let mut rates = vec![0u32; nsrates as usize];
    unsafe { airspyhf_get_samplerates(device, rates.as_mut_ptr(), nsrates) };
    let wav_sample_per_sec = if rates.contains(&args.sample_rate) {
        args.sample_rate
    } else {
        eprintln!(
            "argument error: unsupported sample rate: {}",
            args.sample_rate
        );
        unsafe { airspyhf_close(device) };
        return Err(());
    };
    if unsafe { airspyhf_set_samplerate(device, wav_sample_per_sec) } != AIRSPYHF_SUCCESS {
        eprintln!("airspyhf_set_samplerate() failed: {wav_sample_per_sec}");
        unsafe { airspyhf_close(device) };
        return Err(());
    }
    if args.verbose {
        eprintln!("{} MS/s IQ", wav_sample_per_sec as f32 * 0.000_001);
    }

    // Print receiver serial number.
    let mut ids = AirspyhfReadPartIdSerialNo {
        part_id: 0,
        serial_no: [0; 4],
    };
    if unsafe { airspyhf_board_partid_serialno_read(device, &mut ids) } != AIRSPYHF_SUCCESS {
        eprintln!("airspyhf_board_partid_serialno_read() failed");
        unsafe { airspyhf_close(device) };
        return Err(());
    }
    eprintln!(
        "Device Serial Number: 0x{:08X}{:08X}",
        ids.serial_no[0], ids.serial_no[1]
    );

    // Manual gain commands.
    if !args.no_manual && apply_manual_commands(device, &args).is_err() {
        unsafe { airspyhf_close(device) };
        return Err(());
    }

    // Open the output sink.
    let writer: Box<dyn Write + Send> = if path == "stdout" {
        Box::new(BufWriter::with_capacity(FD_BUFFER_SIZE, io::stdout()))
    } else {
        match File::create(&path) {
            Ok(mut f) => {
                if is_wav {
                    // Placeholder header; patched on exit.
                    if f.write_all(&build_wav_header(wav_sample_per_sec, 0))
                        .is_err()
                    {
                        eprintln!("failed writing wav header");
                        unsafe { airspyhf_close(device) };
                        return Err(());
                    }
                }
                Box::new(BufWriter::with_capacity(FD_BUFFER_SIZE, f))
            }
            Err(e) => {
                eprintln!("{path}: {e}");
                unsafe { airspyhf_close(device) };
                return Err(());
            }
        }
    };

    let shared = Arc::new(Shared {
        do_exit: AtomicBool::new(false),
        avg_rate_bits: AtomicU32::new((wav_sample_per_sec as f32).to_bits()),
    });
    let now = Instant::now();
    let mut ctx = Box::new(RxContext {
        writer,
        limit_num_samples: args.samples_to_xfer.is_some(),
        bytes_to_xfer: args.samples_to_xfer.unwrap_or(0) * 8,
        got_first: false,
        t_start: now,
        time_start: now,
        buffer_count: 0,
        sample_count_acc: 0,
        average_rate: wav_sample_per_sec as f32,
        global_average_rate: 0.0,
        rate_samples: 0,
        shared: shared.clone(),
    });
    let ctx_ptr = &mut *ctx as *mut RxContext as *mut c_void;

    // Ctrl-C / termination handling.
    {
        let shared = shared.clone();
        let _ = ctrlc::set_handler(move || {
            eprintln!("Caught signal, stopping...");
            shared.do_exit.store(true, Ordering::SeqCst);
        });
    }

    if unsafe { airspyhf_start(device, Some(rx_callback), ctx_ptr) } != AIRSPYHF_SUCCESS {
        eprintln!("airspyhf_start() failed.");
        unsafe { airspyhf_close(device) };
        return Err(());
    }
    if unsafe { airspyhf_set_freq(device, freq_hz) } != AIRSPYHF_SUCCESS {
        eprintln!("airspyhf_set_freq() failed.");
        unsafe { airspyhf_close(device) };
        return Err(());
    }

    eprintln!("Stop with Ctrl-C");
    std::thread::sleep(Duration::from_secs(1));
    while unsafe { airspyhf_is_streaming(device) } == 1 && !shared.do_exit.load(Ordering::SeqCst) {
        if args.verbose {
            let rate = f32::from_bits(shared.avg_rate_bits.load(Ordering::Relaxed)) * 1e-6;
            eprintln!("Streaming at {rate:5.3} MS/s");
        }
        if args.samples_to_xfer.is_some() && ctx_bytes_remaining(&ctx) == 0 {
            shared.do_exit.store(true, Ordering::SeqCst);
        } else {
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    if shared.do_exit.load(Ordering::SeqCst) {
        eprintln!("\nUser cancel, exiting...");
    } else {
        eprintln!("\nExiting...");
    }

    // Stop streaming (joins the worker threads, so ctx is ours again).
    unsafe { airspyhf_stop(device) };

    let time_diff = ctx.t_start.elapsed().as_secs_f32();
    eprintln!("Total time: {time_diff:.4} s");
    if ctx.rate_samples > 0 {
        eprintln!(
            "Average speed {:.4} MS/s IQ",
            ctx.global_average_rate * 1e-6 / ctx.rate_samples as f32
        );
    }

    unsafe { airspyhf_close(device) };

    // Flush and patch the WAV header now that the size is known.
    let _ = ctx.writer.flush();
    drop(ctx); // ensure the BufWriter is fully flushed/closed
    if is_wav && path != "stdout" {
        patch_wav(&path, wav_sample_per_sec);
    }
    eprintln!("done");
    Ok(())
}

fn ctx_bytes_remaining(ctx: &RxContext) -> u64 {
    ctx.bytes_to_xfer
}

fn apply_manual_commands(device: AirspyhfDeviceHandle, args: &Args) -> Result<(), ()> {
    unsafe {
        if airspyhf_set_hf_agc(device, args.hf_agc as u8) == AIRSPYHF_SUCCESS {
            eprintln!("HF AGC {}", if args.hf_agc { "ON" } else { "OFF" });
        } else {
            eprintln!("airspyhf_set_hf_agc failed.");
            return Err(());
        }
        if args.hf_agc {
            if airspyhf_set_hf_agc_threshold(device, args.hf_agc_threshold as u8)
                == AIRSPYHF_SUCCESS
            {
                eprintln!(
                    "HF AGC threshold {}",
                    if args.hf_agc_threshold { "High" } else { "Low" }
                );
            } else {
                eprintln!("airspyhf_set_agc_threshold() failed.");
                return Err(());
            }
        } else {
            eprintln!("HF AGC threshold ignored as AGC is disabled.");
            if airspyhf_set_hf_att(device, args.hf_att) == AIRSPYHF_SUCCESS {
                eprintln!("HF Attenuator value: -{} dB", args.hf_att * 6);
            } else {
                eprintln!(
                    "airspyhf_set_hf_att() failed: offending value: {}",
                    args.hf_att
                );
                return Err(());
            }
        }
        if airspyhf_set_hf_lna(device, args.hf_lna as u8) == AIRSPYHF_SUCCESS {
            eprintln!("HF LNA {}", if args.hf_lna { "ON" } else { "OFF" });
        } else {
            eprintln!("airspyhf_set_hf_lna() failed.");
            return Err(());
        }
    }
    Ok(())
}

fn patch_wav(path: &str, sample_rate: u32) {
    if let Ok(mut f) = OpenOptions::new().read(true).write(true).open(path) {
        if let Ok(len) = f.metadata().map(|m| m.len()) {
            let data_len = len.saturating_sub(WAV_HEADER_LEN);
            let header = build_wav_header(sample_rate, data_len);
            let _ = f.seek(SeekFrom::Start(0));
            let _ = f.write_all(&header);
        }
    }
}

fn timestamp() -> String {
    // Local wall-clock time formatted as YYYYmmdd_HHMMSS, matching the C tool's
    // strftime output. Uses only std by converting from the Unix epoch.
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let (y, mo, d, h, mi, s) = civil_from_unix(secs as i64);
    format!("{y:04}{mo:02}{d:02}_{h:02}{mi:02}{s:02}")
}

/// Convert a Unix timestamp (UTC) to (year, month, day, hour, min, sec).
fn civil_from_unix(secs: i64) -> (i64, u32, u32, u32, u32, u32) {
    let days = secs.div_euclid(86_400);
    let rem = secs.rem_euclid(86_400);
    let (h, mi, s) = (
        (rem / 3600) as u32,
        ((rem % 3600) / 60) as u32,
        (rem % 60) as u32,
    );
    // Howard Hinnant's civil-from-days algorithm.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d, h, mi, s)
}

fn main() -> ExitCode {
    // Argument/usage errors print usage at their own site (matching the C
    // tool's exit_usage); runtime failures print a specific message only.
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(()) => ExitCode::FAILURE,
    }
}
