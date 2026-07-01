// Port of tools/src/airspyhf_lib_version.c: print the library version.

use airspyhf::AirspyhfLibVersion;

fn main() {
    let mut v = AirspyhfLibVersion {
        major_version: 0,
        minor_version: 0,
        revision: 0,
    };
    unsafe { airspyhf::airspyhf_lib_version(&mut v) };
    println!(
        "AirSpy HF library version: {}.{}.{}",
        v.major_version, v.minor_version, v.revision
    );
}
