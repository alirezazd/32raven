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

!!! warning

    **Surface-mount pads do not tolerate a dwelling iron.** A chip component is a couple of
    millimetres long and held down by two small pads. It will not take the heat a header pin
    shrugs off: too long on the joint and you cook the part, or the pad lifts off the laminate
    and takes the net with it.

    Wherever these joints come up, the technique is the same:

    - **Tin the wire first**, away from the board. The joint should be a touch, not a
      construction.
    - **Flux the pad.** It is what makes the solder take immediately instead of you waiting on
      it with the iron down.
    - **One or two seconds of contact.** If it does not take, lift off and let the part cool
      completely before trying again. Repeated short attempts are far safer than one long one.
    - **Do not push.** Pressure is what shears a pad off, and a chip component needs none.

## Flying it is on you

This guide covers building the machine. It does not cover operating one lawfully.

!!! danger

    **Flying this and transmitting from it are separately regulated, and both are on you.**

    Registration, remote ID, weight and altitude thresholds, distance from people and from
    controlled airspace, line of sight — all of it varies by country and changes over time.
    An uncertified aircraft assembled from parts is not exempt from any of it.

    The radio side is the half people miss. An RC link and a telemetry radio are transmitters:
    which bands you may use, at what power, and whether you need a licence to use them at all
    are national rules. A module sold openly online is not evidence that operating it where
    you live is lawful.

    The penalties are real — fines, confiscation of the aircraft, and prosecution where a
    flight endangers people or other aircraft. Nothing in this handbook is legal advice and
    none of it transfers that liability to anyone but you. Find out what applies where you
    are before the first flight, not after.

## Reference build

Every pin in these pages is the value checked into `config/32raven.config` — one specific
build, not a suggestion. Most are Kconfig choices, so wiring your board differently is a
`make 32raven-menuconfig` change.
