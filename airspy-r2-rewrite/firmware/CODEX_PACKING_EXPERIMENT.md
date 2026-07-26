# Experiment: in-place 12-bit packing on the ten-bank ring

Status: speculative. This is an alternate path we did not plan to ship. If it
works it is worth a lot; if it does not, nothing depends on it.

## Why bother

The R2 at 10 MS/s needs 40 MB/s. The Mini at 6 MS/s needs 24 MB/s. Two high
speed devices behind any hub share one 480 Mbit/s bus, whose bulk ceiling is
53.248 MB/s (13 x 512-byte transactions per 125 us microframe, with MULT forced
to 00 for bulk by UM10503 Table 513). 40 + 24 = 64 MB/s, so the pair is
oversubscribed by about 20% and the R2 loses banks continuously.

No amount of device buffering fixes that. Elasticity absorbs stalls that end; a
continuous deficit never ends. The only lossless fix is to send fewer bytes.

The ADC is 12-bit and we ship 16 bits per sample. Packing removes four bits of
nothing per sample:

| | unpacked | packed |
|---|---|---|
| R2 @ 10 MS/s | 40 MB/s | 30 MB/s |
| Mini @ 6 MS/s | 24 MB/s | 18 MB/s |
| pair on one bus | 64 — over ceiling | 48 — fits |

That is the whole thesis. Two radios that cannot currently share a bus would be
able to.

## Why this attempt differs from the existing packing path

Stock packing works and we do not use it, because it is slow and because
enabling it drops us to `AIRSPY_STREAM_MODE_LEGACY` (the contiguous two-bank
path) at `airspy_m4/airspy_m4.c:756`. That trade — lose the entire ten-bank ring
to gain 25% bandwidth — is not worth taking.

Three things make this attempt different:

1. **The M4 is idle.** Measured ISR duty is 0.37%. The core is available.
2. **The AHB matrix is multilayer, so contention is per slave, not global.** The
   packer works on a bank that, by the ring's existing alternation invariant, is
   on a different slave port from the one the ADC is currently writing. The two
   should proceed concurrently without arbitrating against each other.
3. **Packing in place needs no new memory.** 4 words in, 3 words out, so the
   destination cursor always trails the source cursor within the same bank.

Point 2 is the load-bearing claim and it is unproven. Phase 0 tests it before
any packer is written.

---

## Phase 0 — answer the FIFO margin question first

**Do this before writing any packing code.** If it fails, stop; nothing below
matters.

The ADCHS FIFO grants 0.8 us between DMA request and overflow. We have spent
considerable effort removing bus competitors from that path. This proposal adds
the largest competitor yet: a continuous M4 read/write loop at 40 MB/s read plus
30 MB/s write. The claim is that slave separation makes this free. Test the
claim directly, with no packer involved.

Build a null mover: a loop that reads the just-completed bank word by word and
writes it back, doing no transform. Same access pattern, same volume, same
timing as a real packer, zero useful work. Run at 10 MS/s on the R2 with
`DMA_ISR_DEBUG` enabled so `adc_fifo_overflow_count` is live.

Run four configurations and report overflow counts for each over a fixed
interval (ten minutes is plenty):

1. baseline, no mover
2. mover reading and writing the completed bank in place
3. mover reading the completed bank, writing to a bank on a *different* slave
4. mover deliberately reading and writing on the *same slave the ADC is
   currently writing* — the control that should fail

Configuration 4 existing to fail is the point: if it does not produce more
overflows than 1, then slave placement is not the mechanism and the whole
premise is wrong regardless of what 2 and 3 show.

**Questions this must answer:**

- Does a continuous M4 bus master perturb the FIFO margin at all?
- If it does, is the effect sensitive to slave placement, or is it uniform?
- Does M4 instruction fetch matter? The packer's own code is in SRAM and its
  fetch traffic crosses the matrix with no slave discipline. Try running the
  mover from a different SRAM region than the banks and see if it changes
  anything.
- What is the worst-case delay added to `dma_isr` entry? The mover must be
  preemptible; measure the ISR's own latency with the mover running.

Report the numbers, not a verdict. If configurations 2 and 3 add no overflows
over ten minutes at 10 MS/s, proceed. If they add any, stop and report.

---

## Data format

### Input — what the DMA writes

Each 32-bit word in a bank is one ADCHS FIFO read. UM10503 Table 1136:

```
bits 11:0    SAMPLE     first converted sample  (12-bit)
bits 14:12   CHAN_ID    0  (CHANNEL_ID_EN is clear in our CONFIG)
bit  15      EMPTY      0  in normal operation
bits 27:16   SAMPLE2    second converted sample (12-bit)
bits 30:28   CHAN_ID2   0
bit  31      EMPTY2     0
```

So `W = (S_odd << 16) | S_even`, both 12-bit, and **both nibbles at 15:12 and
31:28 are structurally zero** in our configuration. The low half is the
*earlier* sample. You may rely on the zero nibbles; do not rely on them if
anyone ever sets `CHANNEL_ID_EN`.

### Output — what the host already expects

This is the part most likely to be got wrong, so it is specified exactly.
`unpack_samples()` in `driver/current/libairspy/src/airspy.c:363` defines the
contract, and it is a **big-endian bitstream**: samples are laid MSB-first into
a 96-bit field spanning three 32-bit words. Eight samples s0..s7 become three
words w0, w1, w2:

```
w0 = (s0 << 20) | (s1 << 8)  | (s2 >> 4)
w1 = ((s2 & 0xF) << 28) | (s3 << 16) | (s4 << 4) | (s5 >> 8)
w2 = ((s5 & 0xFF) << 24) | (s6 << 12) | s7
```

Verify against the host unpacker before writing anything else. Any deviation
produces a stream that decodes to noise, and it will not be obvious which side
is wrong.

### Sizes

- bank: 16384 bytes = 4096 words = 8192 samples = exactly 1024 groups of 8
  samples, so **no group ever straddles a bank boundary**
- packed bank: 12288 bytes, which is 24 x 512, so ZLT=1 still suppresses any
  zero-length packet
- host buffer under packing: `6144 * 24` = 147456 bytes = exactly 12 packed
  banks. Already correct in `airspy.c:2057`; no host change is required.

---

## The algorithm

Per group: read 4 words, write 3 words, advance source by 4 and destination
by 3. 1024 groups per bank.

```c
/*
 * Both nibbles above each 12-bit sample are structurally zero (CHANNEL_ID_EN
 * clear, EMPTY clear), so the samples can be extracted without masking:
 * the low half of the word is already the sample, and the high half shifted
 * down by 16 is already the sample.
 */
const uint32_t w0 = src[0], w1 = src[1], w2 = src[2], w3 = src[3];

const uint32_t s0 = w0 & 0xFFFFu, s1 = w0 >> 16;
const uint32_t s2 = w1 & 0xFFFFu, s3 = w1 >> 16;
const uint32_t s4 = w2 & 0xFFFFu, s5 = w2 >> 16;
const uint32_t s6 = w3 & 0xFFFFu, s7 = w3 >> 16;

dst[0] = (s0 << 20) | (s1 << 8)  | (s2 >> 4);
dst[1] = (s2 << 28) | (s3 << 16) | (s4 << 4) | (s5 >> 8);
dst[2] = (s5 << 24) | (s6 << 12) |  s7;

src += 4;
dst += 3;
```

Note that `(s2 << 28)` and `(s5 << 24)` discard the high bits naturally, so the
`& 0xF` and `& 0xFF` in the host-side formula are unnecessary on this side.

**Why this should be cheap on this core.** Every shift folds into the barrel
shifter, so `ORR Rd, Rn, Rm, LSL #20` is a single instruction. The loads and
stores should become `LDM`/`STM`. If GCC does not produce that, `UBFX` and `BFI`
are available on ARMv7-M and express the field moves directly — that is the
instruction pair stock's portable C never gets. Do not hand-write assembly until
you have looked at what GCC emits; check for `UBFX`, `BFI`, `LDM`, `STM`, and
shifted-operand `ORR`, and count instructions per group.

Target is roughly 20 instructions per group of 8 samples.

### In-place safety

After k groups, source offset is 4k words and destination offset is 3k words.
The destination therefore trails the source for all k > 0, and at k = 0 the
three writes happen after all four reads. **Packing a bank into itself is safe**
and needs no second buffer. The bank holds 12288 valid bytes afterwards and the
remaining 4096 bytes are stale.

---

## Where it runs

Not in the ISR. The packer runs in the M4's WFE-woken main context on the
just-completed bank, and the bank is granted to USB only after packing finishes.

That inserts one bank of latency into the pipeline and imposes a hard deadline:
packing must complete within one bank interval or the ring falls behind
permanently.

**Budget at 10 MS/s:** a bank is 16384 bytes at 40 MB/s, so the interval is
**409.6 us**, which at 204 MHz is about **83,500 cycles** for 8192 samples —
roughly **10 cycles per sample**. At an estimated 2.5 instructions per sample the
packer should need about 3 cycles per sample, leaving roughly 3x margin. Report
the measured figure; if it is above 6 cycles per sample the margin is too thin
to ship.

### Scratch and resumption

The packer must never delay `dma_isr`. It will be interrupted mid-bank, so it
must be resumable. Keep in a small scratch structure:

- source and destination cursors
- the index of the bank being packed
- a generation stamp for that bank

On resume, verify the generation still matches before continuing. If the bank
was reclaimed underneath the packer — which should be impossible if the grant is
deferred correctly, but check anyway — abandon the work and count it rather than
writing into a bank someone else now owns.

No partial-group state is needed: a group is 4 words in and 3 out, entirely
within one iteration, and groups never straddle banks.

### Placement rule

The packer must operate on a bank whose slave port differs from the one the ADC
DMA is currently writing. The ring's existing alternation invariant already
guarantees consecutive banks differ, and the just-completed bank is one behind
the one now filling — so this should hold by construction. **Assert it rather
than assuming it**, and count violations. If the assertion ever fires, the
placement argument is void and Phase 0's conclusion does not transfer.

---

## Required firmware change outside the packer

Packing currently forces `AIRSPY_STREAM_MODE_LEGACY` at `airspy_m4.c:756`. The
whole proposal assumes packing runs *on* the ten-bank ring, so that coupling has
to be broken. The comment there reads like inherited structure from the original
two-bank design rather than a constraint anyone established — check whether
anything else actually depends on legacy mode when `use_packing` is set.

---

## Open questions

1. Does the M4's instruction fetch traffic perturb the FIFO margin independently
   of the data accesses? Phase 0 configuration variations should isolate this.
2. What happens at a sample-rate change or a stop while a bank is mid-pack?
   Define it explicitly — the safe answer is to abandon the bank and count it.
3. Does deferring the grant by one bank interact badly with the reserve rule?
   The ring now has one more bank in a non-grantable state; confirm the
   available-bank floor still holds.
4. Is the 24% M4 duty compatible with `pll1_park()`? Parking only happens when
   the ADC is stopped, so it should be irrelevant, but confirm.

## Abort criteria

Stop and report rather than working around any of these:

- Phase 0 shows any FIFO overflow increase attributable to the mover
- measured packing cost exceeds 6 cycles per sample
- the slave-placement assertion fires
- output does not decode bit-identically against the host unpacker

A negative result here is a perfectly good outcome. We are not currently
shipping packing and have no plan that depends on it.
