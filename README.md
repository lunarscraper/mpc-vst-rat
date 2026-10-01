# mpc-vst-rat

RAT: the ProCo RAT distortion pedal as a VST2 **insert effect** for the Akai MPC OS plugin host
(Force, MPC Live/One/X/Key). Modelled from the circuit (`vst/rat_core.h`), in the spirit of
[dm-Rat](https://github.com/davemollen/dm-Rat) (Rust, GPL-3), whose code is not used here.

- **DISTORTION**: the LM308 gain stage, 100 k log pot into 47 R + 2.2 uF || 560 R + 4.7 uF - up to
  ~66 dB, with the op-amp's gain-bandwidth limit (the RAT's dark top at full gain) and slew rate
- **FILTER**: 1.5 k + 100 k pot into 3.3 nF, ~32 kHz (left) down to ~475 Hz (right)
- **VOLUME**
- **DIODES**: RAT (1N914 silicon), TURBO (LEDs, louder and more open), GE (germanium, softer),
  OP-AMP (no diodes: only the op-amp's own clipping) - levels matched
- **RUETZ**: the classic mod, the 560 R leg removed: less bass into the clipper, tighter
- **INPUT** (-12..+12 dB into the pedal, for synths that are hotter than a guitar), **MIX** (dry/wet)

Stereo (one circuit per channel), 4x oversampled. `vst/test.sh` runs the offline test (clean at
DISTORTION 0, THD rising with the knob, FILTER darkening, diode levels, Ruetz bass cut, MIX 0
bit-exact dry, stereo independence, project restore) and a benchmark (~1 % of an x86 core).

The same circuit model is RiffBox's PEDAL stage (`mpc-vst-riffbox`).

"ProCo RAT" names the pedal whose circuit this models; no affiliation. Build/deploy workflow:
see `sd88me/mpc-vst-plugins`' `docs/PORTING.md`. License: MIT.

## Hinweis

Entwickelt mit Unterstützung von Claude (Anthropic)
