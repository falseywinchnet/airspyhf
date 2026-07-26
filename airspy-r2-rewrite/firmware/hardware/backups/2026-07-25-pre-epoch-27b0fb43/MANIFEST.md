# Pre-epoch qualified backup

This directory preserves the source and linked artifacts immediately before
the halt-free transport-epoch experiment.

- Control binary:
  `device/airspy_rom_to_ram/airspy_rom_to_ram.bin`
- Size: 22,828 bytes
- SHA-256:
  `27b0fb43ab42cc45a43ba049dca488171485fa62e62ba818ef75fa0b20634b5e`
- The same bytes are archived as
  `hardware/images/airspy-ring-pre-epoch-qualified-27b0fb43-release.bin`.
- This exact control image was written and read back byte-for-byte on Mini
  `35AC63DC2D6ABB4F` on 2026-07-25.

The backup includes the M4 source, M0 source, shared stream contract, ELF, map,
listing, and binary. It intentionally does not include toolchain binaries.
