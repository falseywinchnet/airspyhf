#!/usr/bin/env python3
"""Exhaustive check of the steering safety invariant.

Claim: if the non-SUBMITTED bank set always spans at least two distinct AHB
slave groups, then the steering pop at every bank boundary can always find a
bank in a group different from the one already committed -- so GPDMA never
halts and the alternation invariant is never violated.

The concrete state space (10 banks x 4 states x filling x prev) is ~42M, but
steering only ever consults GROUP-level information, so the per-group
non-submitted counts are a sound abstraction. That space is tiny.
"""
from collections import deque
from itertools import product

# measured from airspy_m4.c:182-191, classified by UM10503 AHB slave port
SIZES = (5, 3, 1, 1)          # S1 128k local, S2 72k local, S3 32k AHB, S4 16k AHB
NAMES = ("S1", "S2", "S3", "S4")
G = len(SIZES)


def spans_two_groups(c):
    return sum(1 for x in c if x > 0) >= 2


def steer_targets(c, prev):
    """Groups we may steer into: different slave than the committed next bank,
    and holding at least one non-SUBMITTED bank."""
    return [g for g in range(G) if g != prev and c[g] > 0]


def successors(c, prev, enforce_reserve):
    out = []
    # 1. steering decision at a bank boundary
    for g in steer_targets(c, prev):
        out.append((c, g))
    # 2. M0 submits a READY bank of group g to USB.
    #    The bank currently being filled is in group `prev` and is not READY,
    #    so submitting from `prev` needs one more than the others.
    for g in range(G):
        need = 2 if g == prev else 1
        if c[g] >= need:
            nc = list(c)
            nc[g] -= 1
            nc = tuple(nc)
            if enforce_reserve and not spans_two_groups(nc):
                continue          # reserve rule refuses this submission
            out.append((nc, prev))
    # 3. USB retires a dTD, returning a bank to group g
    for g in range(G):
        if c[g] < SIZES[g]:
            nc = list(c)
            nc[g] += 1
            out.append((tuple(nc), prev))
    return out


def explore(enforce_reserve):
    starts = [(SIZES, p) for p in range(G)]
    seen, q, bad = set(starts), deque(starts), []
    while q:
        c, prev = q.popleft()
        if not steer_targets(c, prev):
            bad.append((c, prev))
            continue
        for s in successors(c, prev, enforce_reserve):
            if s not in seen:
                seen.add(s)
                q.append(s)
    return seen, bad


for enforce in (True, False):
    seen, bad = explore(enforce)
    tag = "WITH reserve rule" if enforce else "WITHOUT reserve rule"
    print(f"{tag}: {len(seen)} reachable states, {len(bad)} stuck states")
    for c, prev in bad[:4]:
        desc = ", ".join(f"{NAMES[i]}={c[i]}" for i in range(G))
        print(f"    STUCK: non-submitted [{desc}], next bank committed to "
              f"{NAMES[prev]} -> no legal steering target")
    print()

# max submitted depth still permitted by the rule
seen, _ = explore(True)
best = max(seen, key=lambda s: sum(SIZES) - sum(s[0]))
print(f"deepest reachable submission under the rule: "
      f"{sum(SIZES) - sum(best[0])} of {sum(SIZES)} banks submitted "
      f"(non-submitted {best[0]})")
