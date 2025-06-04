# Building the Rust driver on Windows

This repository ships with a `.cargo/config.toml` that targets Linux by default.
To compile the Rust port on Windows you need the MSVC toolchain and to override
the target triple.

1. Install the Windows target:

   ```shell
   rustup target add x86_64-pc-windows-msvc
   ```

2. Either edit `rust-migration/.cargo/config.toml` and set
   `target = "x86_64-pc-windows-msvc"` or pass the target on the command line:

   ```shell
   cargo build --release --target x86_64-pc-windows-msvc
   ```

3. Ensure that Visual Studio (or Build Tools) with C++ support is installed so
   that the MSVC linker is available.

The resulting DLL will appear under
`rust-migration/target/x86_64-pc-windows-msvc/release/`.
