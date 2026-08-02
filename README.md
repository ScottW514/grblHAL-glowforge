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

**Laser fire is hard-locked out** (the kernel laser latch is forced locked
and the laser bit is never emitted). Laser control is a later, safety-gated
milestone; the hardware safety chain remains authoritative regardless.

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
