# GPDMA/SRAM hardware constraints for the ten-bank ring

Status: reference notes
Sources: UM10503 Rev 1.4 (LPC43xx user manual), ES_LPC43x0 Rev 7.2 (errata)

Facts relevant to the existing ten-bank ring and to the rolling-mode work in
`firmware/NEXT_ADVANCEMENT.md`. No design proposal here.

## Rewriting a future LLI is safe; rewriting the channel register is not

UM10503 19.6.16 and 19.6.19: the channel's SRCADDR, DESTADDR, CLLI and CONTROL
registers are updated

> "By following the linked list when a complete packet of data has been
> transferred."

The controller holds only the *current* LLI and fetches the next one **at packet
completion**. It does not prefetch during a packet. An in-memory LLI descriptor
two or more hops ahead of the active one therefore cannot be under the
controller's eye and may be rewritten.

This validates the approach `NEXT_ADVANCEMENT.md` already specifies — "change a
sufficiently future GPDMA LLI link, never the active LLI" — and quantifies
"sufficiently future" as **at least two descriptor hops**, with a margin of one
full packet time:

```text
8 KiB packet at 10 MSPS (40 MB/s)  = 204.8 us
8 KiB packet at  6 MSPS (24 MB/s)  = 341.3 us
```

**The trap.** UM10503 19.6.18 says of the *channel* LLI register at
`0x4000 2108`:

> "Programming this register when the DMA channel is enabled may have
> unpredictable side effects."

That prohibits writing `LPC_GPDMA->C0LLI` while the channel is enabled. It does
not apply to the descriptor array in memory. The distinction is the entire
safety argument for future-LLI switching and is worth a comment at the write
site.

## Two LLIs per bank is forced by hardware

UM10503 Table 290: `TRANSFERSIZE` is bits 11:0 — twelve bits, maximum 4095
transfers. At 32-bit transfer width that is 16,380 bytes, four bytes short of a
16 KiB bank. A bank cannot be described by one LLI.

The existing twenty descriptors for ten banks is the minimum legal encoding, not
a design choice, and any future change to bank size has to respect the 4095
transfer ceiling.

## The SRAM alternation requirement is arbitration, not errata

ES_LPC43x0 Rev 7.2 contains **no GPDMA erratum**. The observed requirement is
explained by UM10503 3.6:

> "When two or more bus masters try to access the same slave, a round robin
> arbitration scheme is used; each master takes turns accessing the slave in
> circular order. The access length is determined by the burst access length of
> the master. For the CPU, the burst size is 1, for GPDMA, the burst size can be
> up to 8."

The AHB matrix exposes the memories as separate slave ports:

```text
0x1000 0000   128 kB local SRAM
0x1008 0000    72 kB local SRAM
0x2000 0000    32 kB AHB SRAM
0x2000 8000    16 kB AHB SRAM   (+16 kB ETB at 0x2000 C000)
```

GPDMA, USB0, M4 and M0 are all independent masters on this matrix. Two
long-burst masters landing on one slave port share it round-robin.

This corroborates the V5/V5b → V5c result: two consecutive 16 KiB destinations
in local SRAM1 produced one repeatable ADC FIFO overflow per revolution;
alternating slaves removed it. The requirement stands as observed:

> Consecutive GPDMA destinations must not lie on the same AHB slave port.

The five-and-five split is incidental — it is how the available local-SRAM1 and
other-slave banks balanced, not a required ratio.

## LLI descriptor placement is itself a bus transaction

UM10503 Table 288 bit 0 (`LM`) selects which AHB master fetches the next LLI, so
the descriptor fetch contends like any other access. Descriptors sitting on a
slave port that also carries capture data or USB traffic can queue behind an
8-beat GPDMA burst, at the packet boundary — the one instant where delay costs
FIFO margin. The array is 320 bytes.

Worth confirming against the current link map; no change implied if it is
already clear.

## Threshold corroboration

The reported 649,470-cycle service gap that overflowed the ADC FIFO is about 7.9
bank intervals against the measured 81,883–82,798 cycles per bank. The
two-ahead protection at `airspy_m4.c:693` trips when the bank two ahead is not
free, so the usable depth is `n - 2` = 8 banks, not the nominal 10. The observed
event landed on that threshold.

Of the two banks unavailable for backlog, only one is a cost of the lookahead.
Bank `i+1` is being filled and would be unavailable under any design. The
lookahead itself costs the single bank held free at `i+2`: 0.41 ms of 4.096 ms,
about ten percent.

That horizon is not a tunable threshold. The controller fetches the next
descriptor at packet completion, the same instant the terminal-count interrupt
fires, so the ISR cannot beat the fetch for `i+1` and the earliest influenceable
bank is structurally `i+2`.

What could shorten it is interrupt granularity rather than a threshold. With two
8 KiB descriptors per bank, setting terminal count on both would give a decision
point every 8 KiB and halve the steering horizon, recovering roughly 0.2 ms at
10 MSPS at the cost of doubling the DMA ISR rate to 4882 Hz. That is a separate
question from steering and should not be bundled with it; the ISR cost would
need measuring against `AIRSPY_STREAM_WORK_TELEMETRY` first.
