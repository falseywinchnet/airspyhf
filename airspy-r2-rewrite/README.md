# Airspy R2 / Mini Rewrite Program

This directory is a coordinated rewrite of the Airspy One firmware and host
driver. It is deliberately **not** an optimization branch yet.

The first deliverable is a readable, testable description of the behavior that
already exists. Only after that description agrees with real hardware traces
may an optimization replace an implementation. The optimized implementation
must continue to pass the same trace and sample-vector tests.

This work is for the Airspy R2 and Airspy Mini (`1d50:60a1`). It is not an
Airspy HF firmware project.

## The three projects

- [`contract/`](contract/README.md) owns the boundary between device and host.
  Legacy behavior is frozen as contract v1. Experimental behavior is opt-in
  contract v2 and later.
- [`firmware/`](firmware/README.md) first models buffer ownership, DMA, memory,
  and USB scheduling in readable C. Target code remains C until equivalence is
  demonstrated.
- [`driver/`](driver/README.md) first becomes a readable C++ implementation
  behind the existing C ABI. Optimized host kernels come second. A Rust/nusb
  implementation comes only after C++ parity.

Neither implementation owns the contract. A change that crosses USB must first
be proposed in `contract/`, with legacy fallback, test vectors, and an explicit
version.

## Sequential gates

1. **Inventory** — name current behavior, including awkward behavior.
2. **Readable parity** — reorganize without deliberately changing results.
3. **Measured baseline** — capture throughput, latency, CPU, heat, and loss.
4. **Local optimization** — change internals without changing the wire.
5. **Contract experiment** — add an opt-in wire feature with raw fallback.
6. **Joint target** — select firmware and driver changes from both audits.
7. **Rust parity** — reproduce the proven C++ driver using nusb.

A stage does not graduate on “it seems to work.” Its exit criteria are listed
in [`PROGRAM.md`](PROGRAM.md).

## Repository layout

```text
contract/         versioned device/host boundary and golden vectors
firmware/
  device/         current LPC4370 firmware source
  model/          host-buildable ownership and queue models
  hardware/       qualified images, recovery material, and host tools
  docs/           audits and hardware qualification records
driver/
  current/        current field-compatible libairspy baseline
  model/          readable C++ lifecycle/stream scaffold
  docs/           driver gates and nusb lessons
  rust/           deliberately deferred Rust-port gate
```

The firmware and driver are independently buildable from their own directories:

```sh
make -C firmware
make -C driver
```

Those commands build both the current implementation and that project's
host-side executable model. See [`firmware/BUILDING.md`](firmware/BUILDING.md)
and [`driver/BUILDING.md`](driver/BUILDING.md) for prerequisites.

The current implementation handoffs are:

- [`firmware/NEXT_ADVANCEMENT.md`](firmware/NEXT_ADVANCEMENT.md)
- [`driver/NEXT_ADVANCEMENT.md`](driver/NEXT_ADVANCEMENT.md)

These are authoritative for the immediate next work. They record not only the
chosen mechanisms, but why they were selected, what legacy behavior remains,
what is explicitly deferred, and the exit criteria.

## Build all executable models

The current scaffold has host-buildable contract, firmware ownership and linked
USB-queue models, plus driver lifecycle and stream-accounting models. They are
intentionally small: these are the invariants against which target-specific
code will be rewritten. The queue model supports both the stock two-buffer /
one-dTD shape and injected deeper configurations; it does not claim that an
unverified LPC43xx SRAM address is suitable for another physical buffer.

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

The untouched upstream repositories remain under
`../airspy-r2-research/upstream/` as parity oracles. Canonical development
happens here. Exact imported revisions are recorded beside each current source
tree.
