# Mazda Torque Interceptor

Support for the MoreTorque Torque Interceptor (TI, the original "TI1") on gen1 Mazdas, for the
cars whose EPS firmware steers only above 45 kph. The code is in
`opendbc/car/mazda/{values,mazdacan,carcontroller,carstate}.py`, the panda's side in
`opendbc/safety/modes/mazda.h` and `panda/board/main.c`, and this tree's side in
`openpilot/sunnypilot/selfdrive/car/{interfaces,card_ext,car_specific}.py`,
`openpilot/sunnypilot/selfdrive/selfdrived/events_zp.py` and
`openpilot/sunnypilot/selfdrive/controls/controls_lateral_zp.py`.

**Status: not driven.** Everything below is built from the interceptor's published
documentation, its author's openpilot fork (MoreTore/openpilot) and pdscomp's port of it to
an earlier zoompilot (pdscomp/MorePilot, `zoom-more`), and is tested against the compiled
panda safety model and the real car interface on a bench with no car and no interceptor.
"Not verified" at the end lists what only a car can answer.

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
- A rise faster than it allows makes it ramp its bias to zero and wait for a zero request.

Violations by class: class 1 clears with a power cycle (or the reset key), class 2 by itself
30 s after the cause is gone, class 3 after 2 s. The violation byte reads in hex as the code
the green LED flashes, and is the number in the alert on the device:

| Code | Name | Class | |
|---|---|---|---|
| 11 | LKAS_STUK | 2 | request did not change, off 0 and 600 |
| 12 | CAN_TMO | 3 | no command for over 100 ms |
| 13 | LKAS_ERR | 2 | request out of the window |
| 14 | LOW_VOLT | 3 | CPU voltage too low |
| 15 | TMR_ERR | 1 | periodic timer interrupt missing |
| 16 | MAIN_ERR | 1 | main loop too slow |
| 17, 18 | TORQUE_H, TORQUE_L | 2 | computed output outside its window |
| 19 | WDT_ERR | 1 | watchdog |
| 21 | CPU_REG | 1 | pin or peripheral configuration |
| 22 | HIGH_TMP | 3 | CPU above 65 C |
| 23 | DATA_MEM | 1 | EEPROM CRC |
| 25 | TK_TRNSF | 2 | measured output off the computed one |
| 26 | ADC_STUK | 1 | ADC channel failed |
| 31 | HIN_ERR | 2 | sensor input out of its window |
| 32, 33 | TK_NLOAD, TK_WLOAD | 2 | feedback output out of its window |
| 34 | CAN_ERR | 3 | any other CAN error |
| 35 | STCK_ERR | 1 | stack |
| 36, 37 | PANIC_MON, PANIC | 1 | panic circuit |
| 41 | LOGIC | 1 | illegal internal state |
| 43 | CLK_STOP | 1 | oscillator stopped |

12 during the first seconds of a drive is ordinary: it is how the TI sits before the stream
starts.

## What the port sends

Two commands every 10 ms, from the same controller torque `u`:

- **The EPS's, 0x243, as before**: `u` x 1200, clipped to the measured ceiling, limited
  against the driver. On legacy firmware it is zero while that EPS takes no request (below its
  floor, during its lockout), as it always was, and ramps from zero when it does again. On the
  steer-to-zero firmware it is untouched.
- **The TI's, 0x249**: `u` x 900, clipped to 600, at 6 counts per frame up and 15 down, limited
  against the driver with an allowance of 15 and a multiplier of 40. Those are the TI author's
  numbers. 900 keeps his 800:600 ratio between the two commands on this fork's 1200 scale.

The TI is the actuator the controller reports (`actuatorsOutput.torque` = its command / 900),
because it is the one that acts at every speed. The lateral stack reads a rail and a slew of
its own for it (`CarControllerParams.STEER_RAIL_SCHEDULE`, `STEER_SLEW_SCHEDULE`, through
`opendbc/sunnypilot/car/lateral_tune.py`): the rail is 600/900 flat, so a saturated controller
puts exactly 600 on the wire, the one value besides 0 the TI lets a request rest on.

The TI's own rules, as the controller keeps them:

- **Nothing rests.** A request that has not moved for 25 frames is moved one count toward
  zero and comes back on the next frame. A steady hand against the command pins the driver
  limit, a quiet road pins the controller's output; either would be a stuck request.
- **Nothing below 1 m/s**, and lateral is off below it (`minSteerSpeed`). A stopped car holds
  its command still.
- **Every frame, engaged or not**, zero when not steering. A release walks down at 15 a frame
  instead of cutting: a cut is a tug at low speed.
- **Zero at once when the TI is not fit**, and the ramp restarts from zero when it is again:
  the panda's reference is at zero by then too.
- **A refused frame is answered with a zero** before the ramp restarts. The panda reports each
  0x249 it refuses (src 193); it always admits a zero, so even a run of refusals leaves the TI
  a stream.

Fit means: feedback heard within 60 ms, both torque bytes equal, a nonzero version, state run,
no violation, no error, no ramp-down.

Driver torque is the TI's reading (the EPS's is the biased signal), in carstate and in the
panda. `steeringPressed` is above 6 for five frames, or above 20 at once. With the TI silent
the EPS's own reading and threshold are used.

What the stock port warns of and the TI makes moot is off on a TI car: the low-speed alert,
the car's LKAS setting, the camera's fault as a permanent steer fault.

## Faults and what the driver sees

- Not fit while moving for 0.2 s: `steerFaultTemporary`. Lateral drops and upstream's
  take-control alert runs, as for any steer fault. A shorter dropout costs only the TI's
  torque for those frames.
- Driver override is not a fault while the driver holds the wheel, or for a second after.
- Not fit for 2 s while moving, or silent for 2 s at any speed once the command stream has
  run for 8 s: **Torque Interceptor Not Ready**, engaged or not, with what the TI last said:

| Second line | |
|---|---|
| No signal: check its cables and OBD power | no feedback at all. Unplugged, unpowered, or the panda firmware is not this branch's and bus 1 is not on the OBD pins |
| Fault code NN | its violation number, table above |
| Error code NN | its logic error byte |
| Starting up | off or driver override with no violation |
| Not taking requests | run, but ramping down or malformed |

- A silent TI also fails the panda's RX checks, which is **Controls Mismatch** on an engage
  attempt, for cruise as well as steering. That is on purpose: the panda limits both commands
  against the driver torque the TI reports and has no other source for it. Turn the toggle off
  to drive without the interceptor.

The alert never disables anything itself. A zoompilot event that soft-disables would drop
MADS lateral in one frame with no alert while cruise is engaged.

`carStateSP.zoompilot.torqueInterceptor` logs heard, ready, state, violation, error, version
and notReady every frame.

## The panda

- 0x249 is admitted on bus 1 only, 8 bytes, with clear high nibbles, request equal to its copy
  and key `C4 61 CE 60`.
- Torque needs openpilot steering (the same test as 0x243, MADS included) and a fit TI heard
  within 100 ms. The controller's 60 ms is inside that, so it stops before it is refused.
- Limits 600, 6 up, 15 down, 192 per 250 ms, allowance 15, multiplier 40, on a history of its
  own: `steer_torque_cmd_checks`' globals belong to 0x243. The down rate must not exceed the
  controller's, or every driver-override winddown is refused.
- `TI_FEEDBACK` is an RX check on bus 1. Its quality flag is whether the frame is well formed
  (bytes 0 and 1 equal, version nonzero, state at most 3). Off and driver override are valid
  frames that carry no permission; failing the flag on them would be a controls mismatch at
  every stop.
- Driver torque is sampled from `TI_FEEDBACK`, not 0x240.
- 0x249 is never forwarded between the car and the camera.

Without `MAZDA_PARAM_TI` none of this is reachable and the mode is as it was.

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
| `STEER_SCALE` | 900 | 600 x tune scale 1.5: the author's 800:600 |
| `STEER_HELD_FRAMES` | 25 | chosen; the TI's stuck time is not published |
| `MIN_SPEED` | 1.0 m/s | pdscomp: 5 of 9 latches entered at a stop |
| `STEER_THRESHOLD`, `STEER_OVERRIDE_NOW` | 6, 20 | TI author; pdscomp (noise peaks 10 to 16) |
| `FEEDBACK_TIMEOUT` | 60 ms | three 50 Hz periods; panda 100 ms |
| `FAULT_T` | 0.2 s | chosen |
| `NOT_READY_ALERT_T`, `BOOT_T` | 2 s, 8 s | chosen; the TI is in run about 5 s after the first frame |
| `DRIVER_OVER_RELEASE_T` | 1 s | chosen |
| low-speed KP cap | 4.0 | pdscomp's replay sweep, one CX-8 |

## Installing on a comma 3X

The branch builds on the device. The comma installer's short URL always clones a repository
named `openpilot`, so a fork named `zoompilot` is installed over SSH:

```
ssh comma@<device ip>
cd /data && rm -rf openpilot
git clone --depth 1 --recurse-submodules -b zoompilot-ti https://github.com/<you>/zoompilot.git openpilot
sudo reboot
```

The first boot compiles for some minutes and flashes the panda firmware built from this
branch's `panda` and `opendbc_repo`; it is that firmware that puts bus 1 on the OBD pins.
With Quickboot Mode on, delete `/data/openpilot/prebuilt` after updating the branch, or
nothing rebuilds.

The toggle appears in Settings, Vehicle once the car is known: after one drive, or after
picking the car there. Until it is on the TI stays in bypass and the car drives as stock.

## First drive

1. Parked, engine on, toggle on. `python tools/scripts/car/can_printer.py --bus 1` should show
   0x249 at 100 Hz and 0x24A at 50 Hz, with byte 3 of 0x24A reaching 03 within a few seconds.
   No 0x24A: the alert's first line of the table above.
2. `python tools/scripts/dump.py carStateSP --values zoompilot.torqueInterceptor` shows the
   decoded state. Note the version byte.
3. Hands off and then turning the wheel by hand, compare `carState.steeringTorque` (the TI's
   reading) with `STEER_TORQUE_SENSOR` in 0x240 on bus 0. If the two are not in the same
   units, the driver thresholds here (6, 15, 20, 30) need rescaling.
4. Engage on an empty road above 60 kph first, where the EPS's own LKAS also acts and a stock
   zoompilot drive is the reference, then below 45 kph.
5. After the drive look at `torqueInterceptor.violation` over time. An 11 is a request that
   rested somewhere.
6. If the learned lat accel factor sits pinned at its limit after a few drives, the
   interceptor's gain is outside what self-tune may learn: turn on Less Restrict Settings for
   Self-Tune (Settings, Steering, Torque).

## Not verified

- Nothing has been driven. No frame here has been seen by a TI.
- Whether the TI's torque reading is in the EPS sensor's units. The DBC ranges and the
  thresholds of the two forks suggest it may not be.
- What the legacy EPS's 0x240 reads while the TI injects, and whether that firmware minds a
  nonzero 0x243 at the moment its block lifts.
- The TI's gain per count against the EPS's own LKAS, by speed. Above the floor both act and
  below it only the TI does, so the plant steps at 45/52 kph, inside one speed bin.
- How long a request may rest before `VIOL_LKAS_STUK`, and whether one count counts as a
  change. pdscomp's cutouts (code 11, about 30 s) followed 574-count plateaus and constant
  commands at a stop; that the cause was the stuck rule is this port's reading of them.
- When driver override happens on an ordinary push, and how long it lasts.
- The firmware version of any given unit, and byte 6 on version 1. Any nonzero version is
  accepted, and ramp-down is read only above version 1, as the author's fork does.
- The low-speed gain. 4.0 is one car's replay sweep on a different chassis.
- Whether the car's OBD-II CAN pair is the same bus the camera sits on. If it is, 0x249 and
  0x24A are on the vehicle bus too, which the forks that drive TIs have always done.
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
- **A firmware version whitelist (0x01, 0x10).** A unit on any other firmware would fail the
  RX check and take cruise with it, with nothing to learn the byte from but a log.
- **Failing the RX quality flag on off or a violation.** A controls mismatch at every stop
  (pdscomp's first port).
- **Fingerprinting the TI.** It is silent until spoken to.
- **Hiding the toggle from the logs, or switching backends when it is on.** pdscomp's fork
  does both; no source was found for the claim behind it.
