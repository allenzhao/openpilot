# Mazda Torque Interceptor

Support for the MoreTorque Torque Interceptor (TI, the original "TI1") on gen1 Mazdas, for the
cars whose EPS firmware steers only above 45 kph. The code is in
`opendbc/car/mazda/{values,mazdacan,carcontroller,carstate}.py`, the panda's side in
`opendbc/safety/modes/mazda.h` and `panda/board/main.c`, and this tree's side in
`openpilot/sunnypilot/selfdrive/car/{interfaces,card_ext,car_specific}.py`,
`openpilot/sunnypilot/selfdrive/selfdrived/events_zp.py` and
`openpilot/sunnypilot/selfdrive/controls/controls_lateral_zp.py`.

**Status: not driven, and not yet run on a device.** Everything below is built from the
interceptor's published documentation, its author's openpilot fork (MoreTore/openpilot) and
pdscomp's port of it to an earlier zoompilot (pdscomp/MorePilot, `zoom-more`). It is tested on
a PC: the real car interface against the compiled panda safety model, the firmware compiled
for the panda's processor, no car, no interceptor and no comma device. "Not verified" at the
end lists what only those can answer.

## What it is and where it sits

The TI goes between the steering column's torque sensor and the EPS (`TORQUE_IN`,
`TORQUE_OUT`) and adds a bias to the signal the EPS reads. The EPS then steers with its own
power assist, as it would for a driver's hand, at any speed: the legacy firmware's LKAS floor
(off going down through 45 kph, back on going up through 52) and its hands-off lockout apply
to the LKAS request on CAN, not to the torque sensor. In any state but run, and with no power,
a relay connects the sensor straight to the EPS (bypass) and the car steers as stock.

Its CAN side is daisy-chained into the cable between the OBD-II port and the comma harness
(the "Comma Power" link), so it lives on the OBD-II CAN pair. The panda reaches that pair as
**bus 1**, and only while it multiplexes CAN2 onto the OBD pins, which it otherwise does just
for the fingerprint. `MAZDA_PARAM_TI` in the safety param keeps it there
(`panda/board/main.c`, `set_safety_mode`).

The TI cannot be fingerprinted: it says nothing until it has found its keyed command stream.
The driver declares it with **Settings, Vehicle, Torque Interceptor**
(`TorqueInterceptorEnabled`), between drives. The toggle sets `MazdaFlags.TORQUE_INTERCEPTOR`
(64) and `MazdaSafetyFlags.TORQUE_INTERCEPTOR` (8). It is not in the sunnylink backup and
cannot be written remotely: it changes the panda's safety param.

## Protocol

From the TI's published safety and communication documentation.

**Command, `CAM_LKAS2` 0x249, bus 1, 100 Hz.** Bytes 0-1: the request plus 2048 in the low
12 bits. Bytes 2-3: the same again, in place of a checksum. Bytes 4-6: the discovery key
`C4 61 CE`. Byte 7: the option, `0x60` for normal run. `0x61` is a cold reset that clears
latched violations and `0x62` forces bypass; neither is ever sent and the panda refuses both.
The TI scans 0x200-0x2FF for the key, locks onto that address until its next reset and
answers one address up.

**Feedback, `TI_FEEDBACK` 0x24A, bus 1, 50 Hz.** Byte 0: driver torque plus 127, measured at
the sensor, before the bias. Byte 1: the same again. Byte 2: firmware version (0x01 and 0x10
have been seen). Byte 3: state. Byte 4: violation number. Byte 5: logic error code. Byte 6:
ramp-down, on firmware above version 1: the TI is taking its bias to zero by itself and
ignores the request until it reads zero. Byte 7 is unused (0x30 in captures).

| State | | |
|---|---|---|
| 0 | discover | booted, scanning for the key; bypass |
| 1 | off | start-up or recovery delay, or a latched violation; bypass |
| 2 | driver override | the driver is steering harder than it can pass; bypass |
| 3 | run | applying its bias |

What the TI does with the request:

- It clips it at 600 either way.
- A request that does not change, anywhere but 0 and 600, is `VIOL_LKAS_STUK`: off for 30 s.
- No command for more than 100 ms is `VIOL_CAN_TMO`: off until 2 s after the stream is back.
- A request that changes faster than it allows, or an unsafe value, makes it ramp its bias to
  zero and wait for a zero request.

Violations by class: class 1 clears with a power cycle (or the reset key), class 2 by itself
30 s after the cause is gone, class 3 after 2 s. The violation byte reads in hex as the code
the green LED flashes, which is the number in the alert on the device. Log tools print the
byte in decimal, the last column:

| Code | Name | Class | | In logs |
|---|---|---|---|---|
| 11 | LKAS_STUK | 2 | request did not change, off 0 and 600 | 17 |
| 12 | CAN_TMO | 3 | no command for over 100 ms | 18 |
| 13 | LKAS_ERR | 2 | request out of the window | 19 |
| 14 | LOW_VOLT | 3 | CPU voltage too low | 20 |
| 15 | TMR_ERR | 1 | periodic timer interrupt missing | 21 |
| 16 | MAIN_ERR | 1 | main loop too slow | 22 |
| 17, 18 | TORQUE_H, TORQUE_L | 2 | computed output outside its window | 23, 24 |
| 19 | WDT_ERR | 1 | watchdog | 25 |
| 21 | CPU_REG | 1 | pin or peripheral configuration | 33 |
| 22 | HIGH_TMP | 3 | CPU above 65 C | 34 |
| 23 | DATA_MEM | 1 | EEPROM CRC | 35 |
| 25 | TK_TRNSF | 2 | measured output off the computed one | 37 |
| 26 | ADC_STUK | 1 | ADC channel failed | 38 |
| 31 | HIN_ERR | 2 | sensor input out of its window | 49 |
| 32, 33 | TK_NLOAD, TK_WLOAD | 2 | feedback output out of its window | 50, 51 |
| 34 | CAN_ERR | 3 | any other CAN error | 52 |
| 35 | STCK_ERR | 1 | stack | 53 |
| 36, 37 | PANIC_MON, PANIC | 1 | panic circuit | 54, 55 |
| 41 | LOGIC | 1 | illegal internal state | 65 |
| 43 | CLK_STOP | 1 | oscillator stopped | 67 |

12 during the first seconds of a drive is ordinary: it is how the TI sits before the stream
starts.

## What the port sends

Two commands every 10 ms, from the same controller torque `u`:

- **The EPS's, 0x243**: `u` x 1200, clipped to the measured ceiling, limited against the
  driver, as without the TI. Two things differ on a TI car. `u` stops at the TI's rail
  (below), so the command tops out at 800 counts wherever the ceiling is higher: under about
  13 m/s, which on legacy firmware is only 12.5 to 13 m/s and on steer-to-zero firmware is
  1148 counts without the toggle. And the share it yields to an opposing driver runs over 15
  to 30 counts of the TI's torque reading, not 15 to 50 of the EPS's. On legacy firmware the
  command is zero while that EPS takes no request (below its floor, during its lockout), as
  it always was, and ramps from zero when it does again.
- **The TI's, 0x249**: `u` x 900, clipped to 600, at 6 counts per frame up and 15 down, limited
  against the driver with an allowance of 15 and a multiplier of 40. The clip, the rates and
  the driver limit are the TI author's numbers. The 900 is pdscomp's pairing on legacy
  firmware, an 800 count EPS command beside `u` x 600, on this fork's 1200 scale. The
  author's own fork sends the TI the same count as the EPS command, clipped to 600, a third
  more per unit of controller torque: `STEER_SCALE` 1200 is the second value to try if the TI
  feels weak, and the rail moves with it.

The TI is the actuator the controller reports (`actuatorsOutput.torque` = its command / 900),
because it is the one that acts at every speed. The lateral stack reads a rail and a slew of
its own for it (`CarControllerParams.STEER_RAIL_SCHEDULE`, `STEER_SLEW_SCHEDULE`, through
`opendbc/sunnypilot/car/lateral_tune.py`): the rail is 600/900 flat, so a saturated controller
puts exactly 600 on the wire, the one value besides 0 the TI lets a request rest on.

The TI's own rules, as the controller keeps them:

- **Nothing rests.** A request that has not moved for 25 frames is moved one count toward
  zero and comes back on the next frame. A steady hand against the command pins the driver
  limit, a quiet road pins the controller's output; either would be a stuck request.
- **Nothing below 1 m/s.** A stopped car holds its command still. On legacy firmware lateral
  is off below it too (`minSteerSpeed` 1.0). On steer-to-zero firmware `minSteerSpeed` stays
  0: lateral runs from 0.3 m/s as it does without the TI, the EPS command alone carries it,
  and the reported actuator reads zero there.
- **Every frame, engaged or not**, zero when not steering.
- **A release walks down at 15 a frame** when openpilot lets go while the panda still lets it
  steer: a software pause, the MADS button, slowing through 1 m/s. A cut is a tug at low
  speed. A brake, a cruise cancel or main off reaches the panda first: the frame on its way is
  refused and the next is a zero. That is a cut, as on every openpilot port and in the
  author's fork.
- **Zero at once when the TI is not fit**, and the ramp restarts from zero when it is again:
  the panda's reference is at zero by then too. The panda hears an unfit frame a cycle before
  the controller does, so the one command on its way is refused.
- **A refused frame is answered with a zero** before the ramp restarts. The panda reports each
  0x249 it refuses (src 193); a zero it admits whatever the state (bar a relay malfunction,
  when it sends nothing at all), so even a run of refusals leaves the TI a stream.

Fit means: feedback heard within 70 ms (two lost frames and a cycle of jitter), both torque
bytes equal, a nonzero version, state run, no violation, no error, no ramp-down.

Driver torque is the TI's reading (the EPS's is the biased signal), in carstate and in the
panda. `steeringPressed` is above 6 for five frames, or above 20 at once. When the TI goes
silent its last reading stands for 1.2 s: until its own watchdogs have put it in bypass the
EPS still reports the bias, and that read as a press would soften the alarm for this very
fault. After that, and with a TI never heard, the EPS's own reading and threshold are used.

The cluster's hands-on-wheel frame mirrors the steer-required alert at any speed, not only
above the EPS's floor.

What the stock port warns of and the TI steers through is off on a TI car: the low-speed
alert, the car's LKAS setting, the camera's fault as a permanent steer fault. Above the EPS's
floor that is not free. A camera fault, or the car's lane keep switched off, takes the EPS's
share of the steering for the rest of the drive and the TI carries on alone, with nothing
shown.

## Faults and what the driver sees

- Not fit while moving for 0.3 s: `steerFaultTemporary`. Lateral drops and upstream's
  take-control alert runs, as for any steer fault. With a hand on the wheel upstream softens
  that alert to a banner or drops it, and the TI reads a resting hand as one.
- A shorter dropout costs the TI's command for those frames and its ramp back from zero
  (0.9 s from 540 counts), not the driver's lateral. Unfit frames add up until the TI has been
  fit for 0.5 s, so dropouts that keep coming are a fault too, and the fault is held 0.5 s
  after the last of them.
- Driver override is not a fault while the driver holds the wheel, or for a second after.
- At a stop a TI that answers is not counted against: it is sent nothing there. What was
  counted while the car moved is kept, so a latch is still a fault on the first frame the
  car moves again.
- Not fit for 2 s in all while moving, or silent for 2 s at any speed once the command stream
  has run for 8 s: **Torque Interceptor Not Ready**, engaged or not, with one chime and what
  the TI last said. MADS takes no lateral entry while it shows. It stays through a stop and
  clears 0.5 s after the TI is fit:

| Second line | |
|---|---|
| No signal: check its cables and OBD power | no feedback at all. Unplugged, unpowered, or the panda firmware is not this branch's and bus 1 is not on the OBD pins |
| Fault code NN | its violation number, table above |
| Error code NN | its logic error byte |
| Starting up | off or driver override with no violation |
| Not taking requests | run, but ramping down or malformed |

- A TI silent for more than a second also fails the panda's RX checks. That is **Controls
  Mismatch**: an immediate disable of cruise as well as steering when engaged (stock cruise is
  canceled), and a refused entry otherwise. It is on purpose: the panda limits both commands
  against the driver torque the TI reports and has no other source for it. Turn the toggle off
  to drive without the interceptor.

The named alert never disables anything itself. A zoompilot event that soft-disables would
drop MADS lateral in one frame with no alert while cruise is engaged.

`carStateSP.zoompilot.torqueInterceptor` logs heard, ready, state, violation, error, version
and notReady every frame.

## The panda

- 0x249 is admitted on bus 1 only, 8 bytes, with clear high nibbles, request equal to its copy
  and key `C4 61 CE 60`.
- Torque needs openpilot steering (the same test as 0x243, MADS included) and a fit TI heard
  within 100 ms. The controller's 70 ms is inside that, so one that has stopped hearing the
  TI stops before it is refused. Staleness latches until the next fit frame and is kept by
  every frame the mode hears, so the microsecond timer's wrap cannot revive an old one.
- Limits 600, 6 up, 15 down, 192 per 250 ms, allowance 15, multiplier 40, on a history of its
  own: `steer_torque_cmd_checks`' globals belong to 0x243. The down rate must not exceed the
  controller's, or every driver-override winddown is refused.
- `TI_FEEDBACK` is an RX check on bus 1. Its quality flag is whether the frame is well formed
  (bytes 0 and 1 equal, version nonzero, state at most 3). Off and driver override are valid
  frames that carry no permission; failing the flag on them would be a controls mismatch at
  every stop. A malformed frame takes MADS lateral as well as `controls_allowed`: it is the
  driver torque both commands are limited against.
- Driver torque is sampled from `TI_FEEDBACK`, not 0x240. For the second or two between a TI
  going silent and the RX check's lag, 0x243 is limited against its last readings, the same
  tolerance every car's driver torque message has.
- 0x249 is never forwarded between the car and the camera.
- Firmware: if nothing acknowledges bus 1 (the TI's CAN pair open with power still there) the
  frames for it would queue until the host could write to no bus at all, in about 2.5 s. The
  8 Hz tick drops that queue once more than 50 frames wait in it (`panda/board/main.c`).

Without `MAZDA_PARAM_TI` none of this is reachable and the mode is as it was, with one
exception that is not about the TI: while the panda refuses 0x243 because lateral is off, it
now also restarts that command's ramp. Under MADS with cruise engaged the rate checks used to
follow a host's request through the refusals, and the first frame after lateral rose could be
a step.

## Lateral tune

- Tune scale stays 1.5 and the offline tune is the car's own. The speed-binned seeds are not
  used: they were learned on the EPS command alone, most behind steer-to-zero firmware. A TI
  car learns from the default bins under seed version 64, so neither plant restores the
  other's cache. torqued's own cache is cleared when the toggle changes
  (`_reset_torque_caches_on_actuator_change`).
- The proportional ladder is held to 4.0 on TI cars
  (`TORQUE_INTERCEPTOR_LOW_SPEED_KP_CAP`), on every tune. It climbs to 250 at 1 m/s, where no
  Mazda EPS ever delivered; pdscomp's CX-8 owner logged 150 to 220 command reversals a minute
  under 30 kph with it, and 4.0 was the knee of a replay sweep on that car's drives. From
  10 m/s up it is unchanged.

## Constants

| Name | Value | From |
|---|---|---|
| `STEER_MAX` | 600 | the TI's clip (its documentation) |
| `STEER_DELTA_UP` / `DOWN` | 6 / 15 | TI author's fork |
| `STEER_DRIVER_ALLOWANCE` / `MULTIPLIER` | 15 / 40 | TI author's fork |
| `STEER_SCALE` | 900 | 600 x tune scale 1.5: pdscomp's 800:600 on legacy firmware |
| `STEER_HELD_FRAMES` | 25 | chosen; the TI's stuck time is not published |
| `MIN_SPEED` | 1.0 m/s | pdscomp: 5 of 9 latches entered at a stop |
| `STEER_THRESHOLD`, `STEER_OVERRIDE_NOW` | 6, 20 | TI author; pdscomp (noise peaks 10 to 16) |
| `FEEDBACK_TIMEOUT` | 70 ms | two lost 50 Hz frames and a cycle of jitter; panda 100 ms |
| `FAULT_T` | 0.3 s | chosen, over the 242 ms feedback pauses pdscomp logged |
| `REFIT_T` | 0.5 s | chosen |
| `BYPASS_T` | 1.2 s | the TI's documented worst case to bypass, 1.09 s |
| `NOT_READY_ALERT_T`, `BOOT_T` | 2 s, 8 s | chosen; the TI is in run about 5 s after the first frame |
| `DRIVER_OVER_RELEASE_T` | 1 s | chosen |
| low-speed KP cap | 4.0 | pdscomp's replay sweep, one CX-8 |
| bus 1 backlog | 50 frames | chosen; the host's writes stop at 246 |

## Installing on a comma 3X

The branch is source: the device builds it at first boot. The comma installer's short URL
always clones a repository named `openpilot`, so a fork named `zoompilot` is installed over
SSH, onto a device that already runs openpilot (it keeps `/data/continue.sh`, which starts
`/data/openpilot`):

```
ssh comma@<device ip>
cd /data && rm -rf openpilot
git clone --depth 1 --recurse-submodules -b zoompilot-ti https://github.com/<you>/zoompilot.git openpilot
sudo reboot
```

The first boot should compile for some minutes and then flash the panda with the firmware
built from this branch's `panda` and `opendbc_repo`; it is that firmware that puts bus 1 on
the OBD pins. If the screen says openpilot failed to build, it did not. With Quickboot Mode
on, delete `/data/openpilot/prebuilt` after updating the branch, or nothing rebuilds.

The toggle appears in Settings, Vehicle once the car is known: after one drive, or after
picking the car there. Until it is on the TI stays in bypass and the car drives as stock.

## First drive

1. Parked, engine on, toggle on. `python tools/scripts/car/can_printer.py --bus 1` should show
   0x24A at 50 Hz with byte 3 reaching 03 within a few seconds: the TI only answers a stream
   it has found. Our own 0x249 shows at 100 Hz under `--bus 129` (what the panda queued for
   bus 1), and under `--bus 193` only what it refused. No 0x24A: the alert's first line of
   the table above.
2. `python tools/scripts/dump.py carStateSP --values carStateSP.zoompilot.torqueInterceptor`
   shows the decoded state, in decimal: version 16 is 0x10, violation 18 is code 12. Note the
   version. `pandaStates` should show `safetyParam` 10 (11 under openpilot longitudinal).
3. Hands off and then turning the wheel by hand, compare `carState.steeringTorque` (the TI's
   reading) with `STEER_TORQUE_SENSOR` in 0x240 on bus 0. If the two are not in the same
   units, the driver thresholds here (6, 15, 20, 30) need rescaling. With the TI steering,
   note what 0x240 reads hands off: that is the bias.
4. Engage on an empty road above 60 kph first, where the EPS's own LKAS also acts and a stock
   zoompilot drive is the reference, then below 45 kph. On steer-to-zero firmware compare
   tight low-speed turns with the toggle off: the EPS command stops at 800 counts with it on.
5. After the drive look at `torqueInterceptor.violation` over time. A 17 (code 11) is a
   request that rested somewhere.
6. If the learned lat accel factor sits pinned at its limit after a few drives, the
   interceptor's gain is outside what self-tune may learn: turn on Less Restrict Settings for
   Self-Tune (Settings, Steering, Torque).
7. Listen for the cluster's hands-on-wheel chime below 45 kph on legacy firmware: the frame
   was never sent there before.

## Not verified

- Nothing has been driven. No frame here has been seen by a TI.
- Nothing has run on a device: this branch has not been cloned, built or booted on a comma
  3X, the firmware has been compiled and never flashed, and bus 1 on the OBD pins has not
  been observed.
- Whether the TI's torque reading is in the EPS sensor's units. The two forks' pressed
  thresholds (6 against 15) suggest it may not be.
- What the legacy EPS's 0x240 reads while the TI injects, and whether that firmware minds a
  nonzero 0x243 at the moment its block lifts.
- The TI's gain per count against the EPS's own LKAS, by speed, and so which scale is right:
  900 here and in pdscomp's port, 1200 by the author's fork. Above the floor both act and
  below it only the TI does, so the plant steps at 45/52 kph, inside one speed bin.
- How long a request may rest before `VIOL_LKAS_STUK`, and whether one count counts as a
  change. pdscomp's cutouts (byte 0x11, about 30 s) followed 574-count plateaus and constant
  commands at a stop; that the cause was the stuck rule, and that the byte is the LED code in
  hex, is this port's reading of two captures.
- When driver override happens on an ordinary push, and how long the TI takes to return to
  run. Longer than a second after the wheel is let go and each hard override ends in a short
  steer fault.
- The firmware version of any given unit, and byte 6 on version 1. Any nonzero version is
  accepted, and ramp-down is read only above version 1, as the author's fork does. A
  version 1 unit ramping its bias down gives no sign of it and waits for a zero request.
- What the TI does in its 229 to 242 ms feedback pauses, and how often it ramps down.
- The low-speed gain. 4.0 is one car's replay sweep on a different chassis.
- Whether the car's OBD-II CAN pair is the same bus the camera sits on. If it is, 0x249 and
  0x24A are on the vehicle bus too, which the forks that drive TIs have always done, and a TI
  still in discover scans the vehicle's own 0x200-0x2FF frames for its key: the panda does
  not look for the key in anything but 0x249.
- The bus 1 backlog bound in the firmware: modeled on the host build of the queue code, not
  run on a panda with an open bus.
- comma's servers. Upstream's device flag `ignore_uploads` ("Uploads ignored due to a
  potential detected safety policy violation") exists; whether a TI trips it is not known.
  Nothing here hides the interceptor from the logs.

## Tried and rejected

- **Reporting the EPS command, or the larger of the two.** Below the floor the EPS command is
  zero while the TI steers: the learner and the steer-limit classifier would see torque with
  no command behind it.
- **Commanding the EPS below its floor**, as both other forks do. It delivers nothing there,
  and the 2022 firmware raised `LKAS_FAULT` in a similar state. Zero costs no torque.
- **pdscomp's 12/25 panda backstop over a 6/15 controller.** A panda down rate above the
  controller's refuses every driver-override winddown.
- **Letting the panda admit a shrinking request after its permission falls**, to make the
  release a ramp on a brake as well. It is torque after the driver has braked, the one thing
  the panda refuses unconditionally.
- **Refusing 0x243 once the TI's torque reading is 100 ms old.** The controller keeps the EPS
  command until the steer fault at 0.3 s; the panda would cut holes in the EPS's stream.
- **Clearing the unfit count on the first fit frame.** Unfit 180 ms of every 220 never
  faulted and never alerted while the command averaged 3 counts of 450.
- **Reading the EPS's torque sensor the moment the TI goes quiet.** It is the TI's bias until
  the TI is in bypass.
- **A firmware version whitelist (0x01, 0x10).** A unit on any other firmware would fail the
  RX check and take cruise with it, with nothing to learn the byte from but a log.
- **Failing the RX quality flag on off or a violation.** A controls mismatch at every stop
  (pdscomp's first port).
- **Fingerprinting the TI.** It is silent until spoken to.
- **Hiding the toggle from the logs, or switching backends when it is on.** pdscomp's fork
  does both; no source was found for the claim behind it.
