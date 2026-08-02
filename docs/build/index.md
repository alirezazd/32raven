# Build the prototype

The from-scratch path: a pile of parts to an aircraft that arms.

Work through the pages in order — each assumes the previous one passed, and the link at the
foot of every page takes you to the next. The arc is: choose and buy the parts, mount and wire
the two boards, add the sensors and the power system, then bring the whole thing up on a bench
before it ever leaves the ground.

## What you need to be able to do

- **Solder.** A 0.1-inch header and tinned stranded wire covers most of it — everything is
  through-hole or module-level, with no PCB to fabricate. The exception is one or two joints
  onto surface-mount pads, for a signal that reaches no header. Those are described pad by
  pad; a fine tip, flux and some magnification are enough, but practise on scrap first if you
  have never soldered to an SMD pad.
- **Use a multimeter** for continuity and DC voltage. One measurement in this guide is
  load-bearing: the battery divider, before it ever reaches an ADC pin.
- **Find your way around Linux and a terminal.** Everything here is driven from a shell —
  `git` and submodules, `make`, a Kconfig menu, flashing over USB and serial. You do not need
  C++ or any knowledge of the toolchain internals, but you should be comfortable running
  commands, reading what they print back, and sorting out a path or a device permission when
  something does not work the first time.

No prior flight-controller experience is assumed, and no oscilloscope is required.

## Flying it is on you

This guide covers building the machine. It does not cover operating one lawfully, and that
part is not optional: registration, weight thresholds, and where you are allowed to fly all
vary by country and change over time. Find out what applies where you are before the first
flight, not after.

## Reference build

Every pin in these pages is the value checked into `config/32raven.config` — one specific
build, not a suggestion. Most are Kconfig choices, so wiring your board differently is a
`make 32raven-menuconfig` change.
