# Testing Observations (informal)

*Owen Murphy — 2026-07-13*

Preliminary, informal runs taken while tuning open- vs. closed-loop jog speed. **These are NOT
formal test data** — see the caveat at the bottom. They exist to inform where to *start and end the
acceleration sweep* in [`TESTING_PLAN.md`](TESTING_PLAN.md) once formal testing begins, and to record
the vibration behavior seen at speed.

All runs in **1/4 microstep** mode (`ACTIVE_MICROSTEP = MS_QUARTER`, 800 steps/rev → 25 steps/mm, so
step/s ÷ 25 = mm/s).

## Runs

| # | Mode | Speed (step/s) | Speed (mm/s) | Accel (step/s²) | Observation |
|---|------|----------------|--------------|-----------------|-------------|
| 1 | CL | 6000 | 240 | 1500 | Consistent and repeatable. Hard vibration at high speed & center-span of the rail; **reduced with improved manual belt tension.** |
| 2 | CL | 8000 | 320 | 20000 | Major vibrations; desync / stall most likely. |
| 3 | OL | 6000 | 240 | 10000 | Still vibrates, also at midspan / high speed. No noticeable step-loss errors. |
| 4 | OL | 10000 | 400 | 12000 | Still vibrates; **independent of being lifted off the table.** Eventually drove the motor to decouple and hit a hard limit switch. |
| 5 | OL | 6000 | 240 | up to 21000 | **After belt-tensioning screws added.** Automated ramp (`Mode::TESTING`) to 21k step/s²: **no noticeable step-loss errors and minimal vibration** — surpasses the previous OL error range (run 4 decoupled at 12k). Belt tension appears to be the dominant lever. |

Key cross-cutting signals:
- Vibration is **worst at midspan and at higher speed**, in both control modes.
- **Belt tension reduces it** (run 1), and appears to be the **dominant lever**: after dedicated
  belt-tensioning screws were added, OL held 21k step/s² with minimal vibration and no step loss
  (run 5) vs. decoupling at 12k before (run 4). Strong support for hypothesis #1 (belt compliance).
- It is **independent of table coupling** (run 4, lifted off table) — the source is inside the
  rail/belt/motor system, not the mounting surface.
- CL vs. OL does not change the vibration character — this is a **mechanical/motor** phenomenon, not a
  control-law one.

## Why the vibration at speed / midspan (working hypotheses)

The midspan-worst + tension-helps + table-independent signature points strongly at the **belt and the
stepper**, not at the rail beam's stiffness. Ranked by how well they fit the evidence:

**1. Belt compliance is lowest at midspan → lowest natural frequency (most likely).**
The two belt runs act as two springs in parallel between the carriage and the fixed ends:
`k = EA(1/L₁ + 1/L₂)`. By AM–HM this sum is **minimized when L₁ = L₂**, i.e. at midspan. Lower
stiffness → lower carriage-on-belt natural frequency `f = (1/2π)·√(k/m)` and larger oscillation
amplitude. Raising belt tension raises `k` (and adds damping), which raises `f` and shrinks the
amplitude — exactly what run 1 showed. This is the classic "midpoint is the worst spot" of a belt
drive and fits every data point here.

**2. Transverse belt resonance (belt "whip").**
The free belt span behaves like a plucked string: `f = (1/2L)·√(T/μ)`. The free span is longest at
midspan, so its resonant frequency is lowest and easiest for carriage motion / motor torque ripple to
excite. Tension `T` raises the frequency and damps it — again consistent with the tension result.
Often audible as a buzz distinct from a structural rumble.

**3. Stepper mid-band resonance / torque-dip instability (drives the desync/stall).**
Steppers have a well-known instability region where the rotor oscillates about each commanded
position; near it, available torque collapses and the motor can lose sync. It is **speed-dependent**,
which matches the "only at speed" observation and especially run 2's likely stall. In 1/4 stepping the
current waveform is still fairly coarse, so torque ripple is a strong excitation source. Microstepping
finer (1/8, 1/16) smooths the waveform and is one of the most effective anti-resonance changes
available. Run 4's decouple-and-runaway at 400 mm/s is most likely pull-out torque exceeded (possibly
resonance-assisted) rather than pure resonance.

**4. Rail-beam bending mode (the case the composite-rail idea targets).**
A beam's first bending mode has its antinode at midspan too, so "worst at midspan" alone doesn't rule
it out. But the fact that **belt tension** helps and the effect is **table-independent** argues the
dominant compliant element is the *belt*, not the *rail beam*. Stiffening the rail only helps if a
beam mode is actually being excited.

### On the composite-rail (superglue interface) idea

Honest read: if the dominant compliance is the belt (which the evidence favors), **stiffening the rail
structure will give little return** — you'd be reinforcing the stiff member while the soft member (the
belt) sets the resonance. Worth confirming *which* compliance dominates before investing in a
composite rail:

- Push/wiggle the carriage by hand at midspan with the motor holding: does the *belt* visibly deflect
  (belt-dominated) or does the *rail* flex (beam-dominated)?
- Does the vibration frequency track **step rate** (→ motor/electrical resonance, position-independent
  once you correct for it) or **position** (worst at midspan → belt/structure)?
- Sweep speed slowly and note whether there are narrow bad *bands* (classic stepper resonance) vs. a
  monotonic "worse with speed" (compliance + excitation growing with velocity).

### Cheaper / higher-ROI mitigations to try before a rail rebuild

- **Belt tension** — already shown to help; set it properly and repeatably (it's currently manual).
- **Finer microstepping** (1/8 or 1/16) — smooths torque ripple, the biggest single anti-resonance
  lever, and it's a one-line `ACTIVE_MICROSTEP` change (watch the higher step rate the driver must
  sustain at speed).
- **A better driver** (e.g. TMC2209 with stealthChop/spreadCycle) — stealthChop is very effective at
  killing mid-band resonance; a strong candidate for the "hardware upgrade" already being considered.
- **Avoid the resonant speed band** in the motion profile once it's characterized.
- **Reduce moving mass**, **shorten the free belt span**, or a **wider/stiffer belt** — all raise the
  belt-limited natural frequency.
- **Jerk/S-curve limiting** — note `FastAccelStepper` does **linear** acceleration only (no jerk
  limit), so acceleration steps are impulsive; that impulse excites exactly these modes. A driver/
  planner with S-curve profiling would soften the excitation, but that's a larger change.

## Caveat (read this on reread)

These runs are in **1/4 step mode and should not be used as testing data.** Their only purpose is to
inform where to begin and end the acceleration ranges when formal testing (per
[`TESTING_PLAN.md`](TESTING_PLAN.md)) is done later. A **hardware upgrade is being considered before
more extensive testing** — revisit the microstepping/driver choices above in that light.
