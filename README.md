# grblHAL-glowforge

A [grblHAL](https://github.com/grblHAL) driver for the **stock Glowforge
(Basic/Plus/Pro) control board**: the factory NXP i.MX6 SOM running Linux.
Part of the **ForgeFIRM** project, which replaces the cloud-dependent
factory firmware with an open, locally-controlled image — no hardware
modification.

The unmodified grblHAL core (git submodule at `src/grbl`) runs as a Linux
userspace process. Steps are not fired from a GPIO ISR: the driver streams
**pulse bytes** (one byte per machine tick) into the factory kernel module's
SDMA + EPIT playback engine (`glowforge.ko`, `/dev/glowforge`), the same
jitter-free hardware step generator the factory firmware used — but fed
live from grblHAL's planner instead of a cloud-generated file.

## Architecture

- **grbl protocol thread** — parser/planner/protocol loop; Grbl 1.1
  protocol over raw TCP (`-p 23`, LightBurn/UGS/cncjs-compatible) or stdio.
- **stepper producer thread** — replaces a hardware step timer: runs the
  core's stepper interrupt callback against a virtual step clock
  (1000 × machine tick), wall-clock paced, and maps each step event onto
  the pulse-byte grid.
- **shipper thread** (`SCHED_FIFO`) — writes due bytes to `/dev/glowforge`
  with a bounded queue (default 200 ms = feed-hold latency), and owns the
  kernel run/stop/streaming/underrun state machine plus the factory's PIC
  run/hold stepper-current scheme.

Machine constants (steps/mm, max rates, accelerations) are measured from
the factory machine and its pulse streams — see `src/boards/glowforge.h`
for sources.

**Laser control** (`src/glowforge_laser.c`): grblHAL laser mode (M3/M4,
`$32`) maps spindle power onto the pulse stream's power bytes and fire
bits. The first laser-on of a job requires the **operator's physical
button press** (the kernel laser latch stays locked until then, and
relocks on disarm/alarm/reset); fire only ever rides motion segments of
laser blocks, and an armed underrun fails safe. The hardware safety
AND-chain remains authoritative regardless.

**Safety inputs** (`src/glowforge_switches.c`): the lid switches and the
remote-interlock loop drive the core's safety-door signal, so opening the
lid mid-job parks it in the door state and closing it resumes — matching
what the hardware chain does to the beam. The e-stop bit is a sense line
that rests active and drops during any stepper motion on this board, so it
gates nothing unless `estop_halts_motion` is set in the machine settings
(for machines retrofitted with a real e-stop circuit).

**Cooling** is enforced in-process but owned by the forgectrl cooling
engine: the driver reports job state, gates fire and issues hold/resume
from the engine's published verdict (a missing or stale verdict reads as
fire-blocked), and carries a compiled-in fallback fan write for the case
where the engine is provably absent while the laser is armed. The
contract is forgectrl `docs/SERVICES.md`.

Under the ForgeFIRM image the driver runs as a **supervised child of
forgectrl** and receives `/dev/glowforge` as a broker-inherited fd
(`GF_PULSE_FD`) — handovers such as the `$H` homing session then never
close the device or cycle the 40 V motor rail. Standalone (no
`GF_PULSE_FD`), it opens the device itself and every takeover runs a
deliberate rail-off settle (`rail_settle_s`).

## Building

```sh
cmake -B build && cmake --build build      # host (null-sink mode for testing)
```

Cross-compile for the board with your i.MX6 toolchain (the ForgeFIRM
project builds it via the Yocto SDK; see `forgefirm/scripts/bench/`).

## Running (on the board)

```sh
GFSINK=/dev/glowforge grblHAL_glowforge -p 23 -e /data/EEPROM.DAT
```

Environment: `GFSINK` (pulse device; unset = null-sink test mode),
`GFSINK_RATE` (machine tick, default 28160 Hz — the factory's own
travel-move tick), `GFSINK_DEPTH_MS` (queue depth, default 200).

## Lineage & license

Derived from the [grblHAL Simulator](https://github.com/grblHAL/Simulator)
(platform layer, stream/NVS shape). GPL-3.0-or-later; see
`COPYING`. grblHAL core © Terje Io and contributors; Simulator platform
code © Jens Geisler, Adam Shelly; Glowforge driver © Scott Wiederhold.
